# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import collada
import numpy as np
import usdex.core
from pxr import Gf, Tf, Usd, UsdGeom, Vt

from .data import ConversionData
from .numpy import convert_face_indices_array, convert_matrix4d, convert_vec2f_array, convert_vec3f_array

__all__ = ["ConversionCollada"]


class ConversionCollada:
    def __init__(self, input_path: pathlib.Path):
        try:
            self.collada = collada.Collada(str(input_path))

            # The default unit for the scene is meters (= 1.0).
            self.unit_meter = self.collada.assetInfo.unitmeter if self.collada.assetInfo.unitmeter is not None else 1.0

        except Exception as e:
            self.collada = None
            Tf.Warn(f'Invalid input_path: "{input_path!s}" could not be parsed. {e}')

    def convert(self, prim: Usd.Prim, data: ConversionData) -> Usd.Prim | None:
        """
        Trace the hierarchical structure and place the geometry.
        """
        if not self.collada:
            return None

        for scene in self.collada.scenes:
            for node in scene.nodes:
                self._traverse_scene(prim, None, node, Gf.Matrix4d(), data)
        return prim

    def _multiply_root_matrix(self, matrix: Gf.Matrix4d) -> Gf.Matrix4d:
        """
        Multiply the matrix by the scale matrix.
        """
        if not Gf.IsClose(self.unit_meter, 1.0, 1e-6):
            scale_matrix = Gf.Matrix4d().SetScale(self.unit_meter)
            matrix = scale_matrix * matrix

        return matrix

    def _convert_mesh(self, prim: Usd.Prim, name: str, geometry: collada.geometry.Geometry, matrix: Gf.Matrix4d, data: ConversionData) -> Usd.Prim:
        """
        Gets and stores primitives from a dae Geometry.
        """
        # Multiply the matrix by the up axis matrix and the scale matrix.
        matrix = self._multiply_root_matrix(matrix)

        # Since dae does not have primitive names, sequential numbers are assigned.
        names = []
        for primitive in geometry.primitives:
            if len(names) == 0:
                names.append(name)
            else:
                names.append(f"{name}_{len(names)}")

        safe_names = data.name_cache.getPrimNames(prim, names)

        # When multiple geometries are present. And when each geometry contains multiple primitives.
        # In this case, an Xform is created, and the meshes are placed within it.
        # In the case of a single primitive, or a single mesh, the mesh is placed as a direct child of prim.
        if len(names) > 1 and len(self.collada.geometries) > 1:
            parent_prim = usdex.core.defineXform(prim, safe_names[0]).GetPrim()
            if safe_names[0] != name:
                usdex.core.setDisplayName(parent_prim, name)
            prim = parent_prim

        for primitive, _name, _safe_name in zip(geometry.primitives, names, safe_names):
            primitive_type = type(primitive).__name__

            # The pycollada library always treats Triangles as TriangleSets.
            if primitive_type not in ["TriangleSet", "Triangles", "Polylist", "Polygons"]:
                continue

            # Determine if this is a triangle-based or polygon-based primitive once
            is_triangle_type = primitive_type in ["TriangleSet", "Triangles"]

            # If there is only one geometry, the prim is the parent of the geometry.
            _prim = prim
            if len(names) == 1 and len(self.collada.geometries) == 1:
                _prim = prim.GetParent()
                _safe_name = prim.GetName()

            vertices = None
            normals = None
            uvs = None
            face_vertex_indices = None
            face_vertex_counts = None

            # vertex indices.
            if is_triangle_type:
                face_vertex_counts, face_vertex_indices = convert_face_indices_array(primitive.vertex_index)
            else:  # Polylist or Polygons
                # Use numpy for faster conversion
                face_vertex_counts = primitive.vcounts.tolist() if hasattr(primitive.vcounts, "tolist") else [int(i) for i in primitive.vcounts]
                face_vertex_indices = (
                    primitive.vertex_index.tolist() if hasattr(primitive.vertex_index, "tolist") else [int(i) for i in primitive.vertex_index]
                )

            if hasattr(primitive, "vertex"):
                vertices = convert_vec3f_array(primitive.vertex)

            # Remove unused vertices and rebuild vertex indices
            if len(names) > 1 and vertices is not None and face_vertex_indices is not None:
                # Find all unique vertex indices that are actually used
                used_indices = np.unique(face_vertex_indices)

                # Create a mapping from old indices to new indices
                index_mapping = np.full(len(vertices), -1, dtype=np.int32)
                index_mapping[used_indices] = np.arange(len(used_indices))

                # Create new vertices array with only used vertices
                used_indices_list = used_indices.tolist()
                vertices = Vt.Vec3fArray([vertices[i] for i in used_indices_list])

                # Update face_vertex_indices with new indices using numpy vectorization
                face_vertex_indices = index_mapping[np.array(face_vertex_indices, dtype=np.int32)].tolist()

            if hasattr(primitive, "normal"):
                primitive_normals = convert_vec3f_array(primitive.normal)
                normal_indices = primitive.normal_index

                # Optimize flattening operation using numpy when possible
                if is_triangle_type:
                    # Flatten 2D array more efficiently
                    if isinstance(normal_indices, np.ndarray):
                        normal_indices = normal_indices.ravel().tolist()
                    else:
                        normal_indices = [j for sublist in normal_indices for j in sublist]
                else:  # Polylist or Polygons
                    normal_indices = normal_indices.tolist() if hasattr(normal_indices, "tolist") else [int(j) for j in normal_indices]

                normals = usdex.core.Vec3fPrimvarData(UsdGeom.Tokens.faceVarying, primitive_normals, Vt.IntArray(normal_indices))
                normals.index()  # re-index the normals to remove duplicates

            # UVs.
            # If there are multiple UV layers, the first UV layer is used.
            if hasattr(primitive, "texcoordset") and len(primitive.texcoordset) > 0:
                uv_data = convert_vec2f_array(primitive.texcoordset[0])
                if len(uv_data) == len(face_vertex_indices):
                    # Use Vt.IntArray constructor with range directly for better performance
                    uvs = usdex.core.Vec2fPrimvarData(UsdGeom.Tokens.faceVarying, uv_data, Vt.IntArray(len(uv_data)))
                    uvs.index()  # re-index the uvs to remove duplicates

            if face_vertex_counts is not None and face_vertex_indices is not None and vertices is not None:
                usd_mesh = usdex.core.definePolyMesh(
                    _prim,
                    _safe_name,
                    faceVertexCounts=Vt.IntArray(face_vertex_counts),
                    faceVertexIndices=Vt.IntArray(face_vertex_indices),
                    points=Vt.Vec3fArray(vertices),
                    normals=normals,
                    uvs=uvs,
                )
                if not usd_mesh:
                    Tf.Warn(f'Failed to convert mesh "{prim.GetPath()}"')
                    return None

                if _name != _safe_name:
                    usdex.core.setDisplayName(usd_mesh.GetPrim(), _name)

                usdex.core.setLocalTransform(usd_mesh, matrix)

        return prim

    def _traverse_scene(
        self, prim: Usd.Prim, parent_node: collada.scene.Node | None, node: collada.scene.Node, matrix: Gf.Matrix4d, data: ConversionData
    ):
        """
        Traverse the scene hierarchy, and upon reaching the geometry,
        provide the accumulated matrix to store it flat in the GeometryLibrary.
        """
        if isinstance(node, collada.scene.Node) and hasattr(node, "name"):
            # Set the transformation matrix if available
            node_matrix = convert_matrix4d(node.matrix) if hasattr(node, "matrix") else Gf.Matrix4d()
            matrix = node_matrix * matrix

        # Geometry Node.
        if isinstance(node, collada.scene.GeometryNode):
            name = parent_node.name if parent_node else node.geometry.name

            # Converts geometry to usd meshes.
            self._convert_mesh(prim, name, node.geometry, matrix, data)

        if hasattr(node, "children") and node.children:
            for child in node.children:
                self._traverse_scene(prim, node, child, matrix, data)
