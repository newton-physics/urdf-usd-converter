# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib
import re
from collections import Counter
from dataclasses import dataclass
from urllib.parse import unquote

import numpy as np
import usdex.core
from pxr import Gf, Tf, Usd, UsdGeom, UsdShade, Vt

from . import gltf
from .data import ConversionData
from .material import store_mesh_material_reference
from .material_data import MaterialData
from .numpy import convert_vec2f_array, convert_vec3f_array

__all__ = ["convert_gltf"]

# Image formats that OpenUSD reads natively. Embedded images are identified by the MIME type.
_MIME_SUFFIXES = {"image/png": ".png", "image/jpeg": ".jpg"}
_IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg"}


@dataclass
class _MergedGeometry:
    """
    All the triangles of a glTF file merged into one mesh. This does not depend on USD.
    """

    points: np.ndarray  # (N, 3) float32. The world transform is baked.
    normals: np.ndarray | None  # (N, 3) float32 per vertex.
    uvs: np.ndarray | None  # (N, 2) float32 per vertex. V is flipped for USD.
    # COLOR_0 as the display color. glTF vertex colors are linear, the same as USD display colors, so they are used as they are.
    # Only the geometry of the primitives with COLOR_0 has them, and every vertex of it has a color.
    display_colors: np.ndarray | None  # (N, 3) float32 per vertex.
    display_opacities: np.ndarray | None  # (N,) float32 per vertex. Only when a primitive has an RGBA COLOR_0. The opacity of RGB is 1.
    face_vertex_indices: np.ndarray  # (F * 3,) int32. Every face is a triangle.
    # Face index arrays keyed by glTF material index (None: no material), in the order first used.
    # This is only used when there are two or more entries.
    subsets: dict[int | None, np.ndarray]
    double_sided: bool


@dataclass
class _VertexBlock:
    """
    Vertices of one accessor set under one world matrix. Primitives sharing it also share these vertices.
    """

    offset: int
    count: int


class _Warner:
    """
    Emits a warning for the file once per message.
    """

    def __init__(self, input_path: pathlib.Path):
        self._input_path = input_path
        self._messages: set[str] = set()

    def __call__(self, message: str):
        if message not in self._messages:
            self._messages.add(message)
            Tf.Warn(f'{message} in file: "{self._input_path}"')


def convert_gltf(prim: Usd.Prim, input_path: pathlib.Path, data: ConversionData) -> Usd.Prim | None:
    """
    Convert a glTF or GLB file to a USD mesh.

    URDF treats the whole file as one geometry, so every mesh node in the default scene is merged.
    Node transforms are baked into the points, so no UsdGeomXform is authored for the nodes.

    The primitives with COLOR_0 are an exception. Their vertex colors are authored as the display color,
    which a bound material would hide, so they do not have a material and are merged into a separate mesh.
    Only when a file has both kinds of primitives, the two meshes are children of the given prim (as for an OBJ with
    several shapes). Otherwise the given prim is the single mesh.

    The glTF coordinates are the URDF mesh frame as they are. glTF +Y up is not rotated to +Z up,
    because glTF meshes for URDF (e.g. dex-urdf) are authored in the link frame, and URDF tools such as
    URDF Visualizer do not rotate them. A Y-up glTF is rotated with the <origin> of the URDF visual.
    """
    try:
        model = gltf.load(input_path)
    except gltf.GltfError as e:
        Tf.Warn(f'Invalid input_path: "{input_path}" could not be parsed. {e}')
        return None

    for message in model.warnings:
        Tf.Warn(f'{message} in file: "{input_path}"')

    warn = _Warner(input_path)
    geometry = _merge_geometry(model, input_path, warn)
    color_geometry = _merge_geometry(model, input_path, warn, vertex_colors=True)
    if geometry is None and color_geometry is None:
        Tf.Warn(f'Invalid input_path: "{input_path}" contains no meshes')
        return None

    if geometry is not None and color_geometry is not None:
        # Two meshes below the prim. The names are made unique within the prim.
        stem = input_path.stem
        names = [stem, f"{stem}_vertexColor"]
        safe_names = data.name_cache.getPrimNames(prim, names)
        for geometry_part, safe_name, name in zip((geometry, color_geometry), safe_names, names):
            if not _define_mesh(prim, safe_name, name, geometry_part, model, input_path, data, warn):
                return None
        return prim

    # Same naming as a DAE with a single geometry. The prim itself becomes the mesh.
    safe_name = prim.GetName()
    name = data.mesh_cache.get_name_from_safe_name(safe_name)
    if not _define_mesh(prim.GetParent(), safe_name, name, geometry or color_geometry, model, input_path, data, warn):
        return None
    return prim


def _define_mesh(
    parent: Usd.Prim,
    safe_name: str,
    name: str | None,
    geometry: _MergedGeometry,
    model: gltf.Gltf,
    input_path: pathlib.Path,
    data: ConversionData,
    warn: _Warner,
) -> UsdGeom.Mesh | None:
    """
    Define one mesh below the parent from the merged geometry, with its subsets, and store its materials.
    """
    normals = None
    if geometry.normals is not None:
        normals = usdex.core.Vec3fPrimvarData(UsdGeom.Tokens.vertex, convert_vec3f_array(geometry.normals))
        normals.index()  # remove duplicates

    uvs = None
    if geometry.uvs is not None:
        uvs = usdex.core.Vec2fPrimvarData(UsdGeom.Tokens.vertex, convert_vec2f_array(geometry.uvs))
        uvs.index()  # remove duplicates

    # The vertex colors are a display fallback. A bound material does not read them.
    display_color = None
    if geometry.display_colors is not None:
        display_color = usdex.core.Vec3fPrimvarData(UsdGeom.Tokens.vertex, convert_vec3f_array(geometry.display_colors))
        display_color.index()  # remove duplicates

    display_opacity = None
    if geometry.display_opacities is not None:
        display_opacity = usdex.core.FloatPrimvarData(UsdGeom.Tokens.vertex, Vt.FloatArray.FromNumpy(geometry.display_opacities))
        display_opacity.index()  # remove duplicates

    face_count = len(geometry.face_vertex_indices) // 3

    usd_mesh = usdex.core.definePolyMesh(
        parent,
        safe_name,
        faceVertexCounts=Vt.IntArray.FromNumpy(np.full(face_count, 3, dtype=np.int32)),
        faceVertexIndices=Vt.IntArray.FromNumpy(geometry.face_vertex_indices),
        points=convert_vec3f_array(geometry.points),
        normals=normals,
        uvs=uvs,
        displayColor=display_color,
        displayOpacity=display_opacity,
    )
    if not usd_mesh:
        Tf.Warn(f'Failed to convert mesh "{parent.GetPath().AppendChild(safe_name)}" from {input_path}')
        return None

    if name and name != safe_name:
        usdex.core.setDisplayName(usd_mesh.GetPrim(), name)

    if geometry.double_sided:
        usd_mesh.GetDoubleSidedAttr().Set(True)

    _define_subsets(usd_mesh, geometry)

    # Material binding is done on the Material layer, so only the material data and the references are stored here.
    _store_materials(model, geometry, usd_mesh.GetPrim().GetName(), input_path, data, warn)

    return usd_mesh


def _define_subsets(usd_mesh: UsdGeom.Mesh, geometry: _MergedGeometry):
    """
    Create a subset for each material when two or more are used. With a single material, no subset is needed.
    Material binding is done on the Material layer, so no binding is done at this stage.
    """
    if len(geometry.subsets) < 2:
        return

    stage = usd_mesh.GetPrim().GetStage()
    for i, face_indices in enumerate(geometry.subsets.values()):
        subset_name = f"GeomSubset_{(i+1):03d}"
        geom_subset = UsdGeom.Subset.Define(stage, usd_mesh.GetPath().AppendChild(subset_name))
        geom_subset.GetIndicesAttr().Set(Vt.IntArray.FromNumpy(face_indices))
        geom_subset.GetElementTypeAttr().Set(UsdGeom.Tokens.face)
        geom_subset.GetFamilyNameAttr().Set(UsdShade.Tokens.materialBind)
        geom_subset.SetFamilyType(usd_mesh, UsdShade.Tokens.materialBind, UsdGeom.Tokens.partition)


def _merge_geometry(
    model: gltf.Gltf,
    input_path: pathlib.Path,
    warn: _Warner | None = None,
    vertex_colors: bool = False,
) -> _MergedGeometry | None:
    """
    Walk the default scene and merge the triangle primitives into one geometry.

    The primitives are split by COLOR_0. With vertex_colors=False, only the primitives without COLOR_0 are merged,
    with their materials. With vertex_colors=True, only the primitives with COLOR_0 are merged. Their vertex colors
    are authored as the display color, and their materials are ignored with a warning.
    """
    if warn is None:
        warn = _Warner(input_path)

    if model.scene is None or model.scene >= len(model.scenes):
        warn("The file has no scene")
        return None

    point_blocks: list[np.ndarray] = []
    normal_blocks: list[np.ndarray | None] = []
    uv_blocks: list[np.ndarray | None] = []
    color_blocks: list[np.ndarray] = []  # (N, 4) RGBA. The alpha is 1 for an RGB COLOR_0. Only with vertex_colors.
    has_color_alpha = False  # Whether a primitive has an RGBA COLOR_0.
    double_sided = False  # Whether a material of a merged primitive is double sided, even if the material is ignored.
    index_chunks: list[np.ndarray] = []
    # (material index, face count) in face order.
    face_runs: list[tuple[int | None, int]] = []

    # Primitives sharing the same accessors under the same world matrix reuse the transformed vertices.
    block_cache: dict[tuple, _VertexBlock] = {}
    vertex_total = 0

    # The UV transform of each material (KHR_texture_transform), baked into the UVs of its primitives.
    uv_transforms: dict[int, np.ndarray | None] = {}

    # A node is visited at most once, which also protects from cyclic references.
    visited: set[int] = set()
    stack: list[tuple[int, np.ndarray]] = [(index, np.identity(4)) for index in reversed(model.scenes[model.scene].nodes)]
    while stack:
        node_index, parent_matrix = stack.pop()
        if node_index in visited or node_index < 0 or node_index >= len(model.nodes):
            warn(f"Node {node_index} is referenced more than once or does not exist. It is skipped")
            continue
        visited.add(node_index)

        node = model.nodes[node_index]
        world_matrix = parent_matrix @ gltf.local_matrix(node)
        stack.extend((child_index, world_matrix) for child_index in reversed(node.children))

        if node.mesh is None:
            continue
        if node.mesh < 0 or node.mesh >= len(model.meshes):
            warn(f"Node {node_index} references a mesh that does not exist. It is skipped")
            continue

        matrix3 = world_matrix[:3, :3]
        determinant = np.linalg.det(matrix3)
        if determinant == 0.0 or not np.isfinite(determinant):
            warn("A node has a singular transform. Its meshes are skipped")
            continue
        # With a negative determinant the winding order is reversed by the transform, so flip it back.
        flip_winding = determinant < 0.0
        matrix_key = world_matrix.tobytes()

        for primitive in model.meshes[node.mesh].primitives:
            if primitive.mode not in gltf.TRIANGLE_MODES:
                # 0 (POINTS) to 3 (LINE_STRIP) are not geometry for URDF. The loader has already warned about these.
                continue
            positions = primitive.attributes.get("POSITION")
            if positions is None or positions.ndim != 2 or positions.shape[1] != 3 or len(positions) == 0:
                warn("A primitive has no POSITION. It is skipped")
                continue

            colors = _checked_color_attribute(primitive.attributes.get("COLOR_0"), len(positions))
            if (colors is not None) != vertex_colors:
                continue

            triangles = _triangle_list(primitive.mode, primitive.indices, len(positions))
            if len(triangles) == 0:
                continue
            if triangles.max() >= len(positions):
                warn("A primitive has a vertex index out of range. It is skipped")
                continue
            if flip_winding:
                triangles = triangles[:, [0, 2, 1]]

            normals = _checked_attribute(primitive.attributes.get("NORMAL"), len(positions), 3)
            uvs = _checked_attribute(primitive.attributes.get("TEXCOORD_0"), len(positions), 2)
            if any(name.startswith(("TEXCOORD_", "COLOR_")) and not name.endswith("_0") for name in primitive.attributes):
                warn("Only TEXCOORD_0 and COLOR_0 are supported. The other sets are ignored")

            material = primitive.material
            if material is not None and not 0 <= material < len(model.materials):
                warn(f"A primitive references material {material} that does not exist. It is treated as having no material")
                material = None

            if material is not None and model.materials[material].double_sided:
                double_sided = True
            if vertex_colors:
                # A bound material would hide the display color, so the primitive has no material. It has no texture either.
                if material is not None:
                    warn(
                        f"{_material_label(model.materials[material], material)} is ignored for a primitive with COLOR_0. "
                        "The vertex colors are used as the display color instead"
                    )
                material = None
                uvs = None

            uv_transform = None
            if uvs is not None and material is not None:
                if material not in uv_transforms:
                    uv_transforms[material] = _material_uv_transform(model.materials[material], material, warn)
                uv_transform = uv_transforms[material]

            # The UV transform is part of the key, because primitives with different transforms cannot share their UVs.
            key = (id(positions), id(normals), id(uvs), id(colors), matrix_key, None if uv_transform is None else uv_transform.tobytes())
            block = block_cache.get(key)
            if block is None:
                block = _VertexBlock(offset=vertex_total, count=len(positions))
                block_cache[key] = block
                vertex_total += block.count

                point_blocks.append((positions @ matrix3.T + world_matrix[:3, 3]).astype(np.float32))
                normal_blocks.append(_transform_normals(normals, matrix3) if normals is not None else None)
                # The transform is applied in the glTF UV space (top-left origin), then V is flipped for USD.
                uv_blocks.append(_flip_v(_apply_uv_transform(uvs, uv_transform)) if uvs is not None else None)
                if colors is not None:
                    color_blocks.append(_rgba(colors))
                    has_color_alpha = has_color_alpha or colors.shape[1] == 4

            index_chunks.append((triangles + block.offset).astype(np.int32).reshape(-1))
            face_runs.append((material, len(triangles)))

    if not index_chunks:
        return None

    points = np.concatenate(point_blocks)
    face_vertex_indices = np.concatenate(index_chunks)

    # Remove the vertices that no face uses.
    used = np.zeros(len(points), dtype=bool)
    used[face_vertex_indices] = True
    if not used.all():
        remap = (np.cumsum(used) - 1).astype(np.int32)
        face_vertex_indices = remap[face_vertex_indices]
    else:
        remap = None

    def select(array: np.ndarray) -> np.ndarray:
        return np.ascontiguousarray(array if remap is None else array[used], dtype=np.float32)

    merged_normals = None
    if all(block is not None for block in normal_blocks):
        merged_normals = select(np.concatenate(normal_blocks))
    elif any(block is not None for block in normal_blocks):
        warn("Only some primitives have NORMAL. The normals are ignored")

    merged_uvs = None
    if any(block is not None for block in uv_blocks):
        # Vertices of primitives without TEXCOORD_0 get (0, 0).
        filled = [
            block if block is not None else np.zeros((len(block_points), 2), dtype=np.float32) for block, block_points in zip(uv_blocks, point_blocks)
        ]
        merged_uvs = select(np.concatenate(filled))

    merged_colors = None
    merged_opacities = None
    if vertex_colors:
        # Every vertex has a COLOR_0 here. The opacity of an RGB COLOR_0 is 1.
        rgba = select(np.concatenate(color_blocks))
        merged_colors = np.ascontiguousarray(rgba[:, :3])
        if has_color_alpha:
            merged_opacities = np.ascontiguousarray(rgba[:, 3])

    subsets: dict[int | None, list[np.ndarray]] = {}
    face_start = 0
    for material, face_count in face_runs:
        subsets.setdefault(material, []).append(np.arange(face_start, face_start + face_count, dtype=np.int32))
        face_start += face_count

    return _MergedGeometry(
        points=select(points),
        normals=merged_normals,
        uvs=merged_uvs,
        display_colors=merged_colors,
        display_opacities=merged_opacities,
        face_vertex_indices=np.ascontiguousarray(face_vertex_indices, dtype=np.int32),
        subsets={material: np.concatenate(runs) for material, runs in subsets.items()},
        double_sided=double_sided,
    )


def _triangle_list(mode: int, indices: np.ndarray | None, vertex_count: int) -> np.ndarray:
    """
    Return (F, 3) vertex indices as an independent triangle list.
    Without indices, the vertex order is the topology.
    """
    order = np.arange(vertex_count, dtype=np.int64) if indices is None else indices.astype(np.int64, copy=False)

    if mode == gltf.TRIANGLES:
        # Ignore a trailing partial triangle.
        return order[: len(order) // 3 * 3].reshape(-1, 3)

    if len(order) < 3:
        return np.zeros((0, 3), dtype=np.int64)

    if mode == gltf.TRIANGLE_STRIP:
        # Triangle i is (v[i], v[i + 1 + i % 2], v[i + 2 - i % 2]); odd triangles keep the winding consistent.
        i = np.arange(len(order) - 2)
        odd = i % 2
        return np.stack((order[i], order[i + 1 + odd], order[i + 2 - odd]), axis=1)

    # TRIANGLE_FAN: triangle i is (v[i + 1], v[i + 2], v[0]).
    return np.stack((order[1:-1], order[2:], np.full(len(order) - 2, order[0])), axis=1)


def _checked_attribute(values: np.ndarray | None, vertex_count: int, components: int) -> np.ndarray | None:
    if values is None or values.ndim != 2 or values.shape != (vertex_count, components):
        return None
    return values


def _checked_color_attribute(values: np.ndarray | None, vertex_count: int) -> np.ndarray | None:
    """
    COLOR_0 is RGB (N, 3) or RGBA (N, 4). Anything else is ignored.
    """
    if values is None or values.ndim != 2 or values.shape[0] != vertex_count or values.shape[1] not in (3, 4):
        return None
    return values


def _rgba(colors: np.ndarray) -> np.ndarray:
    """
    Return (N, 4) float32 RGBA. The alpha of an RGB color is 1.
    """
    if colors.shape[1] == 4:
        return colors.astype(np.float32, copy=False)
    return np.concatenate((colors, np.ones((len(colors), 1), dtype=colors.dtype)), axis=1).astype(np.float32, copy=False)


def _transform_normals(normals: np.ndarray, matrix3: np.ndarray) -> np.ndarray:
    """
    Transform normals with the inverse transpose of the upper 3x3 matrix, then normalize.
    This stays correct under non-uniform scale.
    """
    transformed = normals.astype(np.float64, copy=False) @ np.linalg.inv(matrix3)
    length = np.linalg.norm(transformed, axis=1, keepdims=True)
    np.divide(transformed, length, out=transformed, where=length > 0.0)
    return transformed.astype(np.float32)


def _flip_v(uvs: np.ndarray) -> np.ndarray:
    """
    glTF has the origin at the top left, USD at the bottom left: v_usd = 1 - v_gltf.
    """
    flipped = np.array(uvs, dtype=np.float32, copy=True)
    flipped[:, 1] = 1.0 - flipped[:, 1]
    return flipped


def _texture_transform_matrix(transform: gltf.TextureTransform | None) -> np.ndarray:
    """
    KHR_texture_transform as a 3x3 matrix on the glTF UV (top-left origin): translation * rotation * scale.
    """
    if transform is None:
        return np.identity(3)
    cos, sin = np.cos(transform.rotation), np.sin(transform.rotation)
    translation = np.array([[1.0, 0.0, transform.offset[0]], [0.0, 1.0, transform.offset[1]], [0.0, 0.0, 1.0]])
    rotation = np.array([[cos, sin, 0.0], [-sin, cos, 0.0], [0.0, 0.0, 1.0]])
    scale = np.diag([transform.scale[0], transform.scale[1], 1.0])
    return translation @ rotation @ scale


def _material_label(material: gltf.Material, index: int) -> str:
    return f'Material "{material.name}"' if material.name else f"Material #{index}"


def _material_texture_refs(material: gltf.Material) -> list[gltf.TextureRef]:
    refs = (
        material.base_color_texture,
        material.metallic_roughness_texture,
        material.normal_texture,
        material.occlusion_texture,
        material.emissive_texture,
    )
    return [ref for ref in refs if ref is not None]


def _material_uv_transform(material: gltf.Material, index: int, warn: _Warner) -> np.ndarray | None:
    """
    Get the UV transform that is baked into the st primvar, or None when it is the identity.

    There is only one st primvar per mesh, so the transform can only be baked when every texture of the material
    uses the same one. If they differ, the first texture (base color first) wins and the others are not transformed correctly.
    """
    refs = _material_texture_refs(material)
    if not refs:
        return None

    label = _material_label(material, index)
    if any(ref.tex_coord != 0 for ref in refs):
        warn(f"{label} has a texture that uses TEXCOORD_{max(ref.tex_coord for ref in refs)}. Only TEXCOORD_0 is supported, so TEXCOORD_0 is used")

    matrices = [_texture_transform_matrix(ref.transform) for ref in refs]
    if any(not np.allclose(matrix, matrices[0]) for matrix in matrices[1:]):
        warn(f"{label} has textures with different KHR_texture_transform. The transform of the first texture is used for all")

    return None if np.allclose(matrices[0], np.identity(3)) else matrices[0]


def _apply_uv_transform(uvs: np.ndarray, transform: np.ndarray | None) -> np.ndarray:
    if transform is None:
        return uvs
    return (uvs.astype(np.float64, copy=False) @ transform[:2, :2].T + transform[:2, 2]).astype(np.float32)


def _material_identifiers(model: gltf.Gltf) -> list[str]:
    """
    Get a name for each material that is unique in the file. A material name that is empty or shared gets its index.
    """
    counts = Counter(material.name for material in model.materials if material.name)
    taken = {name for name, count in counts.items() if count == 1}

    identifiers = []
    for index, material in enumerate(model.materials):
        if material.name and counts[material.name] == 1:
            identifiers.append(material.name)
            continue
        identifier = f"{material.name or 'Material'}_{index}"
        while identifier in taken:
            identifier += "_"
        taken.add(identifier)
        identifiers.append(identifier)
    return identifiers


def _store_materials(
    model: gltf.Gltf,
    geometry: _MergedGeometry,
    mesh_name: str,
    input_path: pathlib.Path,
    data: ConversionData,
    warn: _Warner,
):
    """
    Store the material data of the materials that the mesh uses, and which material each subset (or the mesh) references.
    """
    if all(material is None for material in geometry.subsets):
        return

    identifiers = _material_identifiers(model)
    resolver = _TextureResolver(model, input_path, data, warn)
    for index in geometry.subsets:
        if index is not None:
            data.material_data_list.append(_create_material_data(model.materials[index], index, identifiers[index], input_path, resolver, warn))

    # The order follows the subsets. A face without a material has None.
    material_names = [identifiers[index] if index is not None else None for index in geometry.subsets]
    store_mesh_material_reference(input_path, mesh_name, material_names, data)


class _TextureResolver:
    """
    Get the image file that a glTF texture refers to.

    An external image is the file next to the glTF. An embedded image has no file, so its bytes are stored in
    ConversionData.embedded_textures under a virtual path below the glTF file, and written out with the other textures.
    """

    def __init__(self, model: gltf.Gltf, input_path: pathlib.Path, data: ConversionData, warn: _Warner):
        self._model = model
        self._input_path = input_path
        self._data = data
        self._warn = warn
        self._image_paths: dict[int, pathlib.Path | None] = {}

    def path(self, ref: gltf.TextureRef | None, label: str) -> pathlib.Path | None:
        if ref is None:
            return None

        if not 0 <= ref.index < len(self._model.textures):
            self._warn(f"{label} references texture {ref.index} that does not exist. The texture is ignored")
            return None
        texture = self._model.textures[ref.index]
        if texture.source is None or not 0 <= texture.source < len(self._model.images):
            self._warn(f"{label} references a texture without a valid image. The texture is ignored")
            return None

        if texture.sampler is not None and 0 <= texture.sampler < len(self._model.samplers):
            sampler = self._model.samplers[texture.sampler]
            # REPEAT is the default and the only wrap mode that is authored.
            if sampler.wrap_s != gltf.REPEAT or sampler.wrap_t != gltf.REPEAT:
                self._warn(f"{label} has a texture with a wrap mode other than REPEAT. REPEAT is used")

        return self._image_path(texture.source, label)

    def _image_path(self, index: int, label: str) -> pathlib.Path | None:
        if index not in self._image_paths:
            self._image_paths[index] = self._resolve_image(index, label)
        return self._image_paths[index]

    def _resolve_image(self, index: int, label: str) -> pathlib.Path | None:
        image = self._model.images[index]

        if image.data is not None:
            suffix = _MIME_SUFFIXES.get((image.mime_type or "").lower())
            if suffix is None:
                self._warn(
                    f"{label} has an image of an unsupported format ({image.mime_type}). Only PNG and JPEG are supported. The texture is ignored"
                )
                return None
            # Name the file after the glTF file and the image. The path never exists on disk, so it cannot collide with a real file.
            image_name = re.sub(r"[^\w.\-]", "_", image.name) if image.name else f"image{index}"
            file_stem = f"{self._input_path.stem}_{image_name}"
            path = self._input_path / f"{file_stem}{suffix}"
            if path in self._data.embedded_textures:
                path = self._input_path / f"{file_stem}_{index}{suffix}"
            self._data.embedded_textures[path] = image.data
            return path

        if image.uri is None:
            return None

        # The loader has already warned about an external URL, an absolute path, and a missing file.
        uri = unquote(image.uri)
        if "://" in uri or pathlib.PurePosixPath(uri).is_absolute() or pathlib.PureWindowsPath(uri).is_absolute():
            return None
        path = self._input_path.parent / uri
        if path.suffix.lower() not in _IMAGE_SUFFIXES:
            self._warn(f"{label} has an image of an unsupported format ({path.suffix}). Only PNG and JPEG are supported. The texture is ignored")
            return None
        return path


def _factor(values: np.ndarray, size: int, default: float) -> np.ndarray:
    """
    Pad or cut a factor to the expected number of components.
    """
    result = np.full(size, default, dtype=np.float64)
    count = min(size, len(values))
    result[:count] = values[:count]
    return result


def _create_material_data(
    material: gltf.Material,
    index: int,
    identifier: str,
    input_path: pathlib.Path,
    resolver: _TextureResolver,
    warn: _Warner,
) -> MaterialData:
    """
    Convert a glTF material (core pbrMetallicRoughness) to the material data shared with OBJ and DAE.

    A texture is used as it is. The factor that glTF multiplies with it is not applied, and a warning is given instead.
    The alpha of the base color factor is the exception. It is used as the scalar opacity.
    A texture whose factor is 0 would be black in glTF, so it is not used.
    """
    label = _material_label(material, index)

    material_data = MaterialData()
    material_data.mesh_file_path = input_path
    material_data.name = identifier
    material_data.material_name = material.name
    # glTF factors are linear. The textures are read with their own color space.
    material_data.linear_color = True

    # Base color and alpha.
    base_color = _factor(material.base_color_factor, 4, 1.0)
    color = np.clip(base_color[:3], 0.0, 1.0)
    alpha = float(np.clip(base_color[3], 0.0, 1.0))
    material_data.diffuse_color = Gf.Vec3f(*(float(c) for c in color))

    alpha_mode = material.alpha_mode
    if alpha_mode not in (gltf.OPAQUE, gltf.MASK, gltf.BLEND):
        warn(f'{label} has an unknown alphaMode "{alpha_mode}". OPAQUE is used')
        alpha_mode = gltf.OPAQUE
    # The alpha is ignored for OPAQUE.
    material_data.opacity = 1.0 if alpha_mode == gltf.OPAQUE else alpha
    if alpha_mode == gltf.MASK:
        material_data.opacity_threshold = max(0.0, material.alpha_cutoff)

    base_color_path = resolver.path(material.base_color_texture, label)
    if base_color_path is not None and not color.any():
        # glTF multiplies the texture by black.
        base_color_path = None
    if base_color_path is not None:
        material_data.diffuse_texture_path = base_color_path
        # glTF multiplies the alpha of the texture by the alpha of the factor, but the two cannot be combined here.
        # An alpha of the factor below 1 is used as the scalar opacity (the alpha of the texture is not used).
        # Otherwise the alpha of the texture is the opacity. For OPAQUE the alpha is ignored.
        material_data.opacity_from_diffuse_texture = alpha_mode != gltf.OPAQUE and bool(np.isclose(alpha, 1.0))
        if not np.allclose(color, 1.0):
            warn(f"{label} has a baseColorFactor color that is not applied to its baseColorTexture")

    # Metallic and roughness. The texture packs roughness in G and metallic in B.
    material_data.metallic = float(np.clip(material.metallic_factor, 0.0, 1.0))
    material_data.roughness = float(np.clip(material.roughness_factor, 0.0, 1.0))

    metallic_roughness_path = resolver.path(material.metallic_roughness_texture, label)
    occlusion_path = resolver.path(material.occlusion_texture, label)
    if metallic_roughness_path is not None and material_data.metallic == 0.0 and material_data.roughness == 0.0:
        metallic_roughness_path = None
    if metallic_roughness_path is not None:
        material_data.orm_texture_path = metallic_roughness_path
        # The occlusion is only supported when it shares the image with the metallic roughness (an ORM texture).
        material_data.occlusion_in_orm_texture = occlusion_path == metallic_roughness_path
        if not np.isclose(material_data.metallic, 1.0) or not np.isclose(material_data.roughness, 1.0):
            warn(f"{label} has a metallicFactor or roughnessFactor that is not applied to its metallicRoughnessTexture")
    if occlusion_path is not None and occlusion_path != metallic_roughness_path:
        warn(f"{label} has an occlusionTexture that is not shared with its metallicRoughnessTexture. The occlusionTexture is ignored")

    # Normal.
    normal_path = resolver.path(material.normal_texture, label)
    if normal_path is not None:
        material_data.normal_texture_path = normal_path
        if material.normal_texture.scale is not None and not np.isclose(material.normal_texture.scale, 1.0):
            warn(f"{label} has a normalTexture scale that is not applied")

    # Emissive.
    emissive = np.maximum(_factor(material.emissive_factor, 3, 0.0), 0.0)
    material_data.emissive_color = Gf.Vec3f(*(float(c) for c in emissive))
    emissive_path = resolver.path(material.emissive_texture, label)
    if emissive_path is not None and not emissive.any():
        # glTF multiplies the texture by the default factor of 0, so there is no emission.
        emissive_path = None
    if emissive_path is not None:
        material_data.emissive_texture_path = emissive_path
        if not np.allclose(emissive, 1.0):
            warn(f"{label} has an emissiveFactor that is not applied to its emissiveTexture")

    return material_data
