# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import collada
import numpy as np
import usdex.core
from pxr import Gf, Tf, Usd, UsdGeom, Vt

from .data import ConversionData
from .numpy import convert_face_indices_array, convert_matrix4d, convert_vec2f_array, convert_vec3f_array

__all__ = ["convert_collada"]


def convert_collada(prim: Usd.Prim, input_path: pathlib.Path, data: ConversionData) -> Usd.Prim | None:
    try:
        _collada = collada.Collada(str(input_path))

        for scene in _collada.scenes:
            for node in scene.nodes:
                _traverse_scene(_collada, prim, None, node, np.identity(4), data)
        return prim

    except Exception as e:
        Tf.Warn(f'Invalid input_path: "{input_path}" could not be parsed. {e}')
        return None


def _multiply_root_matrix(_collada: collada.Collada, matrix: np.ndarray) -> np.ndarray:
    """
    Multiply the matrix by the scale matrix.
    """
    # The default unit for the scene is meters (= 1.0).
    unit_meter = _collada.assetInfo.unitmeter if _collada.assetInfo.unitmeter is not None else 1.0

    if not Gf.IsClose(unit_meter, 1.0, 1e-6):
        scale_matrix = np.diag([unit_meter, unit_meter, unit_meter, 1.0])
        matrix = np.matmul(scale_matrix, matrix)

    return matrix


def _convert_mesh(
    _collada: collada.Collada,
    prim: Usd.Prim,
    name: str,
    geometry: collada.geometry.Geometry,
    matrix: np.ndarray,
    data: ConversionData,
) -> Usd.Prim:
    """
    Gets and stores primitives from a dae Geometry.
    """
    stage = prim.GetStage()

    # Multiply the matrix by the up axis matrix and the scale matrix.
    matrix = _multiply_root_matrix(_collada, matrix)

    all_face_vertex_counts: list[int] = []
    all_face_vertex_indices: list[int] = []
    all_normals: Vt.Vec3fArray | None = None
    all_normal_indices: list[int] = []
    all_uvs: Vt.Vec2fArray | None = None
    all_uv_indices: list[int] = []
    face_offsets: list[int] = []
    current_normal_offset = 0
    current_uv_offset = 0

    # The list of vertex coordinates is shared among the primitives.
    all_vertices = convert_vec3f_array(geometry.primitives[0].vertex) if hasattr(geometry.primitives[0], "vertex") else None

    for primitive in geometry.primitives:
        primitive_type = type(primitive).__name__

        # The pycollada library always treats Triangles as TriangleSets.
        if primitive_type not in ["TriangleSet", "Triangles", "Polylist", "Polygons"]:
            Tf.Warn(f'Unsupported primitive type: {primitive_type} for geometry: {geometry.name} in file: "{_collada.filename}"')
            continue

        # Determine if this is a triangle-based or polygon-based primitive once
        is_triangle_type = primitive_type in ["TriangleSet", "Triangles"]

        # vertex indices.
        if is_triangle_type:
            face_vertex_counts, face_vertex_indices = convert_face_indices_array(primitive.vertex_index)
        else:  # Polylist or Polygons
            # Use numpy for faster conversion
            face_vertex_counts = primitive.vcounts.tolist()
            face_vertex_indices = primitive.vertex_index.tolist()
        all_face_vertex_counts.extend(face_vertex_counts)
        all_face_vertex_indices.extend(face_vertex_indices)

        face_offsets.append(len(face_vertex_counts))

        # normals.
        if hasattr(primitive, "normal") and len(primitive.normal) > 0:
            primitive_normals = convert_vec3f_array(primitive.normal)
            all_normals = primitive_normals if all_normals is None else Vt.Vec3fArray(list(all_normals) + list(primitive_normals))
            normal_indices = primitive.normal_index

            # Optimize flattening operation using numpy when possible
            if is_triangle_type:
                # Flatten 2D array more efficiently
                if isinstance(normal_indices, np.ndarray):
                    normal_indices = normal_indices.ravel().tolist()
            else:  # Polylist or Polygons
                normal_indices = normal_indices.tolist()

            normal_indices = (np.array(normal_indices, dtype=np.int32) + current_normal_offset).tolist()
            all_normal_indices.extend(normal_indices)
            current_normal_offset += len(primitive_normals)

        # uvs.
        if hasattr(primitive, "texcoordset") and len(primitive.texcoordset) > 0:
            uv_data = convert_vec2f_array(primitive.texcoordset[0])
            all_uvs = uv_data if all_uvs is None else Vt.Vec2fArray(list(all_uvs) + list(uv_data))
            uv_indices = primitive.texcoord_index if hasattr(primitive, "texcoord_index") else list(range(len(uv_data)))
            uv_indices = (np.array(uv_indices, dtype=np.int32) + current_uv_offset).tolist()
            all_uv_indices.extend(uv_indices)
            current_uv_offset += len(uv_data)

    if len(all_face_vertex_counts) > 0 and len(all_face_vertex_indices) > 0 and all_vertices is not None:
        # create a normal primvar data for the geometry.
        normals = None
        if all_normals and all_normal_indices and len(all_normal_indices) == len(all_face_vertex_indices):
            normals = usdex.core.Vec3fPrimvarData(UsdGeom.Tokens.faceVarying, all_normals, Vt.IntArray(all_normal_indices))
            normals.index()  # re-index the normals to remove duplicates

        # create a uv primvar data for the geometry.
        uvs = None
        if all_uvs and all_uv_indices and len(all_uv_indices) == len(all_face_vertex_indices):
            uvs = usdex.core.Vec2fPrimvarData(UsdGeom.Tokens.faceVarying, all_uvs, Vt.IntArray(all_uv_indices))
            uvs.index()  # re-index the uvs to remove duplicates

        # If only one geometry exists within the dae, only one mesh will be placed.
        if len(_collada.geometries) == 1:
            _prim = prim.GetParent()
            _safe_name = prim.GetName()
        else:
            _prim = prim
            _safe_name = data.name_cache.getPrimName(prim, name)

        usd_mesh = usdex.core.definePolyMesh(
            _prim,
            _safe_name,
            faceVertexCounts=Vt.IntArray(all_face_vertex_counts),
            faceVertexIndices=Vt.IntArray(all_face_vertex_indices),
            points=Vt.Vec3fArray(all_vertices),
            normals=normals,
            uvs=uvs,
        )
        if not usd_mesh:
            Tf.Warn(f'Failed to convert mesh "{prim.GetPath()}"')
            return None

        if name != _safe_name:
            usdex.core.setDisplayName(usd_mesh.GetPrim(), name)

        # Specifies the offset in the Mesh subset.
        if len(face_offsets) > 1:
            subset_offset = 0
            for i, face_offset in enumerate(face_offsets):
                subset_name = f"GeomSubset_{(i+1):03d}"

                # Create a list of face indices from face_offsets.
                face_indices = list(range(subset_offset, subset_offset + face_offset))

                geom_subset = UsdGeom.Subset.Define(stage, usd_mesh.GetPath().AppendChild(subset_name))
                if geom_subset:
                    geom_subset.GetIndicesAttr().Set(Vt.IntArray(face_indices))

                    # TODO: Apply material binding to the subset.

                subset_offset += face_offset

        # Convert the matrix to a Gf.Matrix4d.
        usd_matrix = convert_matrix4d(matrix)

        # Decompose the matrix to get the translate, orient, and scale.
        transform = Gf.Transform(usd_matrix)
        translate = transform.GetTranslation()
        orient = Gf.Quatf(transform.GetRotation().GetQuat())
        scale = Gf.Vec3f(transform.GetScale())

        usdex.core.setLocalTransform(usd_mesh, translate, orient, scale)

    return prim


def _traverse_scene(
    _collada: collada.Collada,
    prim: Usd.Prim,
    parent_node: collada.scene.Node | None,
    node: collada.scene.Node,
    matrix: np.ndarray,
    data: ConversionData,
):
    """
    Traverse the scene hierarchy, and upon reaching the geometry,
    provide the accumulated matrix to store it flat in the GeometryLibrary.
    """
    if isinstance(node, collada.scene.Node) and hasattr(node, "name"):
        # Set the transformation matrix if available
        node_matrix = node.matrix if hasattr(node, "matrix") else np.identity(4)
        matrix = np.matmul(node_matrix, matrix)

    # Geometry Node.
    if isinstance(node, collada.scene.GeometryNode):
        name = parent_node.name if parent_node else node.geometry.name

        # Converts geometry to usd meshes.
        _convert_mesh(_collada, prim, name, node.geometry, matrix, data)

    if hasattr(node, "children") and node.children:
        for child in node.children:
            _traverse_scene(_collada, prim, node, child, matrix, data)
