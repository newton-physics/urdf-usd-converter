# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import usdex.core
from pxr import Gf, Tf, Usd

from .data import ConversionData, Tokens
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
        filename = pathlib.Path(mesh[0]) if pathlib.Path(mesh[0]).is_absolute() else urdf_dir / pathlib.Path(mesh[0])
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
        # TODO: Implement STL conversion.
        Tf.Warn(f"The stl format is not yet supported: {input_path}")
    elif input_path.suffix.lower() == ".obj":
        # TODO: Implement OBJ conversion.
        Tf.Warn(f"The obj format is not yet supported: {input_path}")
    elif input_path.suffix.lower() == ".dae":
        # TODO: Implement DAE conversion.
        Tf.Warn(f"The dae format is not yet supported: {input_path}")
    else:
        Tf.Warn(f"Unsupported mesh format: {input_path}")

    if mesh_prim and scale != Gf.Vec3d(1):
        usdex.core.setLocalTransform(mesh_prim.GetPrim(), Gf.Vec3d(0), Gf.Quatf.GetIdentity(), Gf.Vec3f(scale))

    return mesh_prim
