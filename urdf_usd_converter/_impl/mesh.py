# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import stl
import tinyobjloader
import usdex.core
from pxr import Gf, Tf, Usd, UsdGeom, Vt

from .data import ConversionData, Tokens
from .numpy import convert_vec3f_array

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
    data.mesh_cache.store_mesh_cache(geo_scope, data.name_cache, data.urdf_parser)

    # URDF file directory
    urdf_dir = data.urdf_parser.input_file.parent

    # Get a list of names and safe names keyed by mesh paths.
    mesh_names = data.mesh_cache.get_mesh_names()

    for filename in mesh_names:
        name = mesh_names[filename]["name"]
        safe_name = mesh_names[filename]["safe_name"]

        filename = pathlib.Path(filename) if pathlib.Path(filename).is_absolute() else urdf_dir / pathlib.Path(filename)
        mesh_prim: Usd.Prim = usdex.core.defineXform(geo_scope, safe_name).GetPrim()

        # If there are multiple mesh names (using file names), the meshes may have the same name but different scale values.
        # Therefore, this reference is keyed by a unique safe-name.
        data.references[Tokens.Geometry][safe_name] = mesh_prim

        if name != safe_name:
            usdex.core.setDisplayName(mesh_prim, name)
        convert_mesh(mesh_prim, filename, data)

    usdex.core.saveStage(data.libraries[Tokens.Geometry], comment=f"Mesh Library for {data.urdf_parser.get_robot_name()}. {data.comment}")


def convert_mesh(prim: Usd.Prim, input_path: pathlib.Path, data: ConversionData):
    mesh_prim = None
    if input_path.suffix.lower() == ".stl":
        mesh_prim = convert_stl(prim, input_path, data)
    elif input_path.suffix.lower() == ".obj":
        mesh_prim = convert_obj(prim, input_path, data)
    elif input_path.suffix.lower() == ".dae":
        # TODO: Implement DAE conversion.
        Tf.Warn(f"The dae format is not yet supported: {input_path}")
    else:
        Tf.Warn(f"Unsupported mesh format: {input_path}")

    return mesh_prim


def convert_stl(prim: Usd.Prim, input_path: pathlib.Path, data: ConversionData) -> UsdGeom.Mesh:
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


def convert_obj(parent_prim: Usd.Prim, input_path: pathlib.Path, data: ConversionData) -> UsdGeom.Mesh:
    reader = tinyobjloader.ObjReader()
    if not reader.ParseFromFile(str(input_path)):
        Tf.RaiseRuntimeError(f'Invalid input_path: "{input_path}" could not be parsed. {reader.Error()}')

    shapes = reader.GetShapes()
    if len(shapes) == 0:
        Tf.RaiseRuntimeError(f'Invalid input_path: "{input_path}" contains no meshes')

    attrib = reader.GetAttrib()

    names = []
    for shape in shapes:
        name = shape.name if shape.name else parent_prim.GetName()
        names.append(name)
    safe_names = data.name_cache.getPrimNames(parent_prim, names)

    for shape, name, safe_name in zip(shapes, names, safe_names):
        obj_mesh = shape.mesh
        face_vertex_counts = obj_mesh.num_face_vertices

        # Collect vertex indices used in this shape
        vertex_indices_in_shape = [idx.vertex_index for idx in obj_mesh.indices]
        unique_vertex_indices = sorted(set(vertex_indices_in_shape))

        # Create mapping from old indices to new indices
        vertex_index_map = {old_idx: new_idx for new_idx, old_idx in enumerate(unique_vertex_indices)}

        # Extract only vertices used in this shape
        points = []
        for old_idx in unique_vertex_indices:
            i = old_idx * 3
            points.append(Gf.Vec3f(attrib.vertices[i], attrib.vertices[i + 1], attrib.vertices[i + 2]))

        # Remap to new indices
        face_vertex_indices = [vertex_index_map[old_idx] for old_idx in vertex_indices_in_shape]

        # Process normals
        normals = None
        if len(attrib.normals) > 0:
            normal_indices_in_shape = [idx.normal_index for idx in obj_mesh.indices]
            unique_normal_indices = sorted(set(normal_indices_in_shape))
            normal_index_map = {old_idx: new_idx for new_idx, old_idx in enumerate(unique_normal_indices)}

            normals_data = []
            for old_idx in unique_normal_indices:
                i = old_idx * 3
                normals_data.append(Gf.Vec3f(attrib.normals[i], attrib.normals[i + 1], attrib.normals[i + 2]))

            remapped_normal_indices = [normal_index_map[old_idx] for old_idx in normal_indices_in_shape]
            normals = usdex.core.Vec3fPrimvarData(UsdGeom.Tokens.faceVarying, Vt.Vec3fArray(normals_data), Vt.IntArray(remapped_normal_indices))
            normals.index()  # re-index the normals to remove duplicates

        # Process UV coordinates
        uvs = None
        if len(attrib.texcoords) > 0:
            texcoord_indices_in_shape = [idx.texcoord_index for idx in obj_mesh.indices]
            unique_texcoord_indices = sorted(set(texcoord_indices_in_shape))
            texcoord_index_map = {old_idx: new_idx for new_idx, old_idx in enumerate(unique_texcoord_indices)}

            uv_data = []
            for old_idx in unique_texcoord_indices:
                i = old_idx * 2
                uv_data.append(Gf.Vec2f(attrib.texcoords[i], attrib.texcoords[i + 1]))

            remapped_texcoord_indices = [texcoord_index_map[old_idx] for old_idx in texcoord_indices_in_shape]
            uvs = usdex.core.Vec2fPrimvarData(UsdGeom.Tokens.faceVarying, Vt.Vec2fArray(uv_data), Vt.IntArray(remapped_texcoord_indices))
            uvs.index()  # re-index the uvs to remove duplicates

        usd_mesh = usdex.core.definePolyMesh(
            parent_prim,
            safe_name,
            faceVertexCounts=Vt.IntArray(face_vertex_counts),
            faceVertexIndices=Vt.IntArray(face_vertex_indices),
            points=Vt.Vec3fArray(points),
            normals=normals,
            uvs=uvs,
        )
        if not usd_mesh:
            Tf.RaiseRuntimeError(f'Failed to convert mesh "{parent_prim.GetPath()}" from {input_path}')

        if name != safe_name:
            usdex.core.setDisplayName(usd_mesh.GetPrim(), name)

    return parent_prim
