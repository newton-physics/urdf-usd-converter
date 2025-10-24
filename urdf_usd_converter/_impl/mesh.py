# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
import pathlib

import stl
import usdex.core
from pxr import Gf, Tf, Usd, UsdGeom, Vt

from .data import ConversionData, Tokens
from .numpy import convert_vec3f_array
from .utils import float3_to_vec3d

__all__ = ["convert_meshes"]


def convert_meshes(data: ConversionData):
    # A list of file paths and scale values ​​that can be obtained from URDF files.
    meshes = data.urdf_parser.get_meshes()
    if not len(meshes):
        return

    data.libraries[Tokens.Geometry] = usdex.core.addAssetLibrary(data.content[Tokens.Contents], Tokens.Geometry, format="usdc")
    data.references[Tokens.Geometry] = {}

    geo_scope = data.libraries[Tokens.Geometry].GetDefaultPrim()

    # Get and store the mesh name.
    data.mesh_data.store_mesh_data(geo_scope, data.name_cache, data.urdf_parser)

    # URDF file directory
    urdf_dir = data.urdf_parser.input_file.parent

    for mesh, mesh_name, safe_name in zip(meshes, data.mesh_data.mesh_names, data.mesh_data.safe_names):
        filename = urdf_dir / pathlib.Path(mesh[0])
        scale = float3_to_vec3d(mesh[1])

        mesh_prim: Usd.Prim = usdex.core.defineXform(geo_scope, safe_name).GetPrim()

        # If there are multiple mesh names (using file names), the meshes may have the same name but different scale values.
        # Therefore, this reference is keyed by a unique safe-name.
        data.references[Tokens.Geometry][safe_name] = mesh_prim

        if mesh_name != safe_name:
            usdex.core.setDisplayName(mesh_prim, mesh_name)
        convert_mesh(mesh_prim, filename, scale)

    usdex.core.saveStage(data.libraries[Tokens.Geometry], comment=f"Mesh Library for {data.urdf_parser.get_robot_name()}. {data.comment}")


def convert_mesh(prim: Usd.Prim, input_path: pathlib.Path, scale: Gf.Vec3d):
    mesh_prim = None
    if input_path.suffix.lower() == ".stl":
        mesh_prim = convert_stl(prim, input_path)
    elif input_path.suffix.lower() == ".obj":
        # TODO: Implement OBJ conversion.
        Tf.Warn(f"The obj format is not yet supported: {input_path.suffix}")
    elif input_path.suffix.lower() == ".dae":
        # TODO: Implement DAE conversion.
        Tf.Warn(f"The dae format is not yet supported: {input_path.suffix}")
    else:
        Tf.Warn(f"Unsupported mesh format: {input_path.suffix}")

    if mesh_prim and scale != Gf.Vec3d(1):
        usdex.core.setLocalTransform(mesh_prim.GetPrim(), Gf.Vec3d(0), Gf.Quatf.GetIdentity(), Gf.Vec3f(scale))

    return mesh_prim


def convert_stl(prim: Usd.Prim, input_path: pathlib.Path) -> UsdGeom.Mesh:
    stl_mesh = stl.Mesh.from_file(input_path, calculate_normals=False)

    points = usdex.core.Vec3fPrimvarData(UsdGeom.Tokens.vertex, convert_vec3f_array(stl_mesh.points))
    points.index()
    face_vertex_indices = points.indices()
    face_vertex_counts = [3] * stl_mesh.points.shape[0]

    normals = None
    if stl_mesh.normals.any():
        normals = usdex.core.Vec3fPrimvarData(UsdGeom.Tokens.uniform, convert_vec3f_array(stl_mesh.normals))
        normals.index()

    usd_mesh = usdex.core.definePolyMesh(
        prim.GetParent(),
        prim.GetName(),
        faceVertexCounts=Vt.IntArray(face_vertex_counts),
        faceVertexIndices=Vt.IntArray(face_vertex_indices),
        points=points.values(),
        normals=normals,
    )
    if not usd_mesh:
        Tf.RaiseRuntimeError(f'Failed to convert mesh "{prim.GetPath()}" from {input_path}')
    return usd_mesh
