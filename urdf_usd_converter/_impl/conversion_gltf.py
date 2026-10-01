# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import numpy as np
import trimesh
import usdex.core
from PIL import Image
from pxr import Gf, Tf, Usd, UsdGeom, Vt

from .data import ConversionData, Tokens
from .material import store_mesh_material_reference
from .material_data import MaterialData
from .numpy import convert_vec2f_array, convert_vec3f_array

__all__ = ["convert_glb"]


def convert_glb(prim: Usd.Prim, input_path: pathlib.Path, data: ConversionData) -> Usd.Prim:
    """Convert static meshes from a binary glTF scene."""
    with input_path.open("rb") as stream:
        scene = trimesh.load_scene(stream, file_type="glb", process=False)

    nodes = []
    for node_name in scene.graph.nodes_geometry:
        transform, geometry_name = scene.graph[node_name]
        nodes.append((str(geometry_name), np.asarray(transform, dtype=np.float64), scene.geometry[geometry_name]))

    nodes.sort(key=lambda item: (item[0], tuple(item[1].flat)))
    if not nodes:
        raise ValueError("GLB contains no mesh geometry")

    material_names: dict[int, str] = {}
    used_material_names: set[str] = set()
    texture_directory = pathlib.Path(data.content[Tokens.Contents].GetRootLayer().identifier).parent / Tokens.Textures
    texture_paths: dict[int, pathlib.Path] = {}
    converted = 0
    for geometry_name, transform, mesh in nodes:
        if not isinstance(mesh, trimesh.Trimesh):
            Tf.Warn(f'Unsupported GLB geometry "{geometry_name}" in "{input_path}"')
            continue

        usd_mesh = _convert_mesh(
            prim,
            geometry_name,
            mesh,
            transform,
            single_mesh=len(nodes) == 1,
            data=data,
        )
        if not usd_mesh:
            continue

        material_name = _store_material(
            input_path,
            geometry_name,
            mesh,
            material_names,
            used_material_names,
            texture_directory,
            texture_paths,
            data,
        )
        if material_name:
            store_mesh_material_reference(input_path, usd_mesh.GetPrim().GetName(), [material_name], data)
        converted += 1

    if converted == 0:
        raise ValueError("GLB contains no supported triangle meshes")
    return prim


def _convert_mesh(
    prim: Usd.Prim,
    geometry_name: str,
    mesh: trimesh.Trimesh,
    transform: np.ndarray,
    single_mesh: bool,
    data: ConversionData,
) -> UsdGeom.Mesh | None:
    vertices = trimesh.transformations.transform_points(mesh.vertices, transform)
    faces = np.asarray(mesh.faces, dtype=np.int32)
    if faces.ndim != 2 or faces.shape[1] != 3 or len(vertices) == 0 or len(faces) == 0:
        Tf.Warn(f'Unsupported non-triangle or empty GLB mesh "{geometry_name}"')
        return None

    if np.linalg.det(transform[:3, :3]) < 0.0:
        faces = np.fliplr(faces)

    normals = _transform_normals(mesh, transform, geometry_name)
    uvs = getattr(mesh.visual, "uv", None)
    if uvs is not None:
        uvs = np.asarray(uvs, dtype=np.float32)
        if uvs.shape != (len(vertices), 2):
            Tf.Warn(f'Ignoring invalid UVs on GLB mesh "{geometry_name}"')
            uvs = None

    if single_mesh:
        parent = prim.GetParent()
        safe_name = prim.GetName()
    else:
        parent = prim
        safe_name = data.name_cache.getPrimName(prim, geometry_name)

    normal_data = None
    if normals is not None:
        normal_data = usdex.core.Vec3fPrimvarData(UsdGeom.Tokens.vertex, convert_vec3f_array(normals))
        normal_data.index()

    uv_data = None
    if uvs is not None:
        uv_data = usdex.core.Vec2fPrimvarData(UsdGeom.Tokens.vertex, convert_vec2f_array(uvs))
        uv_data.index()

    usd_mesh = usdex.core.definePolyMesh(
        parent,
        safe_name,
        faceVertexCounts=Vt.IntArray([3] * len(faces)),
        faceVertexIndices=Vt.IntArray.FromNumpy(np.ascontiguousarray(faces).reshape(-1)),
        points=convert_vec3f_array(np.asarray(vertices, dtype=np.float32)),
        normals=normal_data,
        uvs=uv_data,
    )
    if not usd_mesh:
        Tf.Warn(f'Failed to convert GLB mesh "{geometry_name}"')
        return None

    if geometry_name != safe_name:
        usdex.core.setDisplayName(usd_mesh.GetPrim(), geometry_name)
    return usd_mesh


def _transform_normals(mesh: trimesh.Trimesh, transform: np.ndarray, geometry_name: str) -> np.ndarray | None:
    normals = np.asarray(mesh.vertex_normals, dtype=np.float64)
    if normals.shape != (len(mesh.vertices), 3):
        Tf.Warn(f'Ignoring invalid normals on GLB mesh "{geometry_name}"')
        return None

    try:
        normals = normals @ np.linalg.inv(transform[:3, :3])
    except np.linalg.LinAlgError:
        Tf.Warn(f'Ignoring normals on GLB mesh "{geometry_name}" because its transform is singular')
        return None

    lengths = np.linalg.norm(normals, axis=1)
    if np.any(lengths == 0.0):
        Tf.Warn(f'Ignoring zero-length normals on GLB mesh "{geometry_name}"')
        return None
    return np.asarray(normals / lengths[:, None], dtype=np.float32)


def _store_material(
    input_path: pathlib.Path,
    geometry_name: str,
    mesh: trimesh.Trimesh,
    material_names: dict[int, str],
    used_material_names: set[str],
    texture_directory: pathlib.Path,
    texture_paths: dict[int, pathlib.Path],
    data: ConversionData,
) -> str | None:
    material = getattr(mesh.visual, "material", None)
    if not isinstance(material, trimesh.visual.material.PBRMaterial):
        return None

    key = id(material)
    if key in material_names:
        return material_names[key]

    base_name = material.name or f"{geometry_name}_material"
    name = base_name
    suffix = 1
    while name in used_material_names:
        suffix += 1
        name = f"{base_name}_{suffix}"
    used_material_names.add(name)
    material_names[key] = name

    base_color = _color_factor(material.baseColorFactor, 4, [1.0, 1.0, 1.0, 1.0])
    emissive = _color_factor(material.emissiveFactor, 3, [0.0, 0.0, 0.0])
    metallic = float(material.metallicFactor if material.metallicFactor is not None else 1.0)
    roughness = float(material.roughnessFactor if material.roughnessFactor is not None else 1.0)
    alpha_mode = material.alphaMode or "OPAQUE"

    material_data = MaterialData()
    material_data.mesh_file_path = input_path
    material_data.name = name
    material_data.material_name = material.name
    material_data.diffuse_color = usdex.core.linearToSrgb(Gf.Vec3f(*base_color[:3]))
    material_data.emissive_color = usdex.core.linearToSrgb(Gf.Vec3f(*emissive))
    material_data.opacity = 1.0 if alpha_mode == "OPAQUE" else float(base_color[3])
    material_data.metallic = metallic
    material_data.roughness = roughness
    if alpha_mode == "MASK":
        material_data.opacity_threshold = float(material.alphaCutoff if material.alphaCutoff is not None else 0.5)

    base_color_texture = _extract_texture(material.baseColorTexture, texture_directory, texture_paths)
    if base_color_texture:
        material_data.diffuse_texture_path = base_color_texture
        material_data.diffuse_texture_scale = Gf.Vec4f(*base_color)
        if alpha_mode != "OPAQUE":
            material_data.opacity_texture_path = base_color_texture
            material_data.opacity_texture_channel = "a"
            material_data.opacity_texture_scale = float(base_color[3])

    metallic_roughness_texture = _extract_texture(material.metallicRoughnessTexture, texture_directory, texture_paths)
    if metallic_roughness_texture:
        material_data.roughness_texture_path = metallic_roughness_texture
        material_data.roughness_texture_channel = "g"
        material_data.roughness_texture_scale = roughness
        material_data.metallic_texture_path = metallic_roughness_texture
        material_data.metallic_texture_channel = "b"
        material_data.metallic_texture_scale = metallic

    material_data.normal_texture_path = _extract_texture(material.normalTexture, texture_directory, texture_paths)
    material_data.emissive_texture_path = _extract_texture(material.emissiveTexture, texture_directory, texture_paths)
    if material_data.emissive_texture_path:
        material_data.emissive_texture_scale = Gf.Vec4f(emissive[0], emissive[1], emissive[2], 1.0)

    data.material_data_list.append(material_data)
    return name


def _extract_texture(
    image: Image.Image | None,
    texture_directory: pathlib.Path,
    texture_paths: dict[int, pathlib.Path],
) -> pathlib.Path | None:
    if image is None:
        return None
    if not isinstance(image, Image.Image):
        Tf.Warn(f"Ignoring unsupported GLB texture type: {type(image).__name__}")
        return None

    key = id(image)
    if key in texture_paths:
        return texture_paths[key]

    image_format = (image.format or "PNG").upper()
    suffix = {"JPG": ".jpg", "JPEG": ".jpg", "PNG": ".png", "WEBP": ".webp"}.get(image_format, ".png")
    texture_directory.mkdir(parents=True, exist_ok=True)
    texture_path = texture_directory / f"glb_texture_{len(tuple(texture_directory.iterdir())):03d}{suffix}"

    source = getattr(image, "fp", None)
    payload = source.getvalue() if hasattr(source, "getvalue") else None
    if payload and image_format in {"JPEG", "PNG", "WEBP"}:
        texture_path.write_bytes(payload)
    else:
        image.save(texture_path, format=image_format if image_format in {"JPEG", "PNG", "WEBP"} else "PNG")

    texture_paths[key] = texture_path
    return texture_path


def _color_factor(value, size: int, default: list[float]) -> np.ndarray:
    if value is None:
        return np.asarray(default, dtype=np.float64)

    factor = np.asarray(value, dtype=np.float64).reshape(-1)
    if len(factor) != size:
        return np.asarray(default, dtype=np.float64)
    if np.issubdtype(np.asarray(value).dtype, np.integer) or np.max(factor) > 1.0:
        factor = factor / 255.0
    return factor
