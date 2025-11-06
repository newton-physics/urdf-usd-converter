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
        # Collada mesh IDs and USD.prim dictionary.
        self.meshes: dict[str, Usd.Prim] = {}

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

        # Store the meshes.
        self._store_meshes(prim, data)

        # Convert the scene hierarchy.
        for scene in self.collada.scenes:
            for node in scene.nodes:
                target_prim = self._convert_scene(node, prim, data)

                # Metric scaling.
                if target_prim and not Gf.IsClose(self.unit_meter, 1.0, 1e-6):
                    scale_op = UsdGeom.Xformable(target_prim).AddScaleOp()
                    scale_op.Set(Gf.Vec3f(self.unit_meter))

        return prim

    def _store_meshes(self, prim: Usd.Prim, data: ConversionData):
        """
        Meshes contained in the dae file are stored in the "Meshes" scope.
        The scene hierarchy of dae internally references these meshes.
        """
        meshes_scope = usdex.core.defineScope(prim, "Meshes")

        names = [geometry.name for geometry in self.collada.geometries]
        safe_names = data.name_cache.getPrimNames(meshes_scope.GetPrim(), names)

        for geometry, name, safe_name in zip(self.collada.geometries, names, safe_names):
            prim = self._convert_mesh(meshes_scope.GetPrim(), geometry, safe_name, data)
            if prim:
                # Since it is an internal reference, it is hidden here.
                UsdGeom.Imageable(prim).GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)
                self.meshes[geometry.id] = prim

    def _convert_mesh(self, prim: Usd.Prim, geometry: collada.geometry.Geometry, safe_name: str, data: ConversionData) -> Usd.Prim:
        """
        Gets and stores primitives from a dae Geometry.
        """
        # An Xform is created here as it will be used for internal references.
        parent_prim = usdex.core.defineXform(prim, safe_name).GetPrim()

        # Since dae does not have primitive names, sequential numbers are assigned.
        names = []
        for primitive in geometry.primitives:
            if len(names) == 0:
                names.append(geometry.name)
            else:
                names.append(f"{geometry.name}_{len(names)}")

        safe_names = data.name_cache.getPrimNames(parent_prim, names)

        for primitive, _name, _safe_name in zip(geometry.primitives, names, safe_names):
            primitive_type = type(primitive).__name__

            # The pycollada library always treats Triangles as TriangleSets.
            if primitive_type not in ["TriangleSet", "Triangles", "Polylist", "Polygons"]:
                continue

            # Determine if this is a triangle-based or polygon-based primitive once
            is_triangle_type = primitive_type in ["TriangleSet", "Triangles"]

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
                    parent_prim,
                    _safe_name,
                    faceVertexCounts=Vt.IntArray(face_vertex_counts),
                    faceVertexIndices=Vt.IntArray(face_vertex_indices),
                    points=Vt.Vec3fArray(vertices),
                    normals=normals,
                    uvs=uvs,
                )
                if not usd_mesh:
                    Tf.Warn(f'Failed to convert mesh "{parent_prim.GetPath()}"')
                    return None

                if _name != _safe_name:
                    usdex.core.setDisplayName(usd_mesh.GetPrim(), _name)

        return parent_prim

    def _convert_scene(self, node: collada.scene.Node, prim: Usd.Prim, data: ConversionData) -> Usd.Prim:
        """
        Converts the scene hierarchy.
        """
        node_name = node.name if hasattr(node, "name") else None

        target_prim = prim
        if isinstance(node, collada.scene.Node) and node_name is not None:
            safe_name = data.name_cache.getPrimName(prim, node_name)
            xform = usdex.core.defineXform(prim, safe_name)
            if node_name != safe_name:
                usdex.core.setDisplayName(xform.GetPrim(), node_name)

            # Set the transformation matrix if available
            usd_matrix = convert_matrix4d(node.matrix) if hasattr(node, "matrix") else Gf.Matrix4d.GetIdentity()
            usdex.core.setLocalTransform(xform, usd_matrix)

            target_prim = xform.GetPrim()

        # Geometry Node.
        if isinstance(node, collada.scene.GeometryNode):
            # The mesh is internally referenced from the dictionary stored in self.meshes.
            prim = self.meshes.get(node.geometry.id)
            if prim:
                target_prim.GetReferences().AddInternalReference(prim.GetPath())
                UsdGeom.Imageable(target_prim).GetVisibilityAttr().Set(UsdGeom.Tokens.inherited)

        if hasattr(node, "children") and node.children:
            for child in node.children:
                self._convert_scene(child, target_prim, data)

        return target_prim
