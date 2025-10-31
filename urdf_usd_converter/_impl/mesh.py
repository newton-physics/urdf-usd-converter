# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import numpy as np
import stl
import tinyobjloader
import usdex.core
from pxr import Gf, Tf, Usd, UsdGeom, Vt

from .data import ConversionData, Tokens
from .numpy import convert_vec3f_array
from .ros_package import resolve_ros_package_paths

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
        safe_name = mesh_names[filename]["safe_name"]

        # Resolve the ROS package paths.
        resolved_path = resolve_ros_package_paths(filename, data)
        if resolved_path != pathlib.Path(filename):
            Tf.Status(f"Resolved ROS package path: {filename} -> {resolved_path!s}")

        resolved_path = resolved_path if resolved_path.is_absolute() else urdf_dir / resolved_path
        mesh_prim: Usd.Prim = usdex.core.defineXform(geo_scope, safe_name).GetPrim()

        # If there are multiple mesh names (using file names), the meshes may have the same name but different scale values.
        # Therefore, this reference is keyed by a unique safe-name.
        data.references[Tokens.Geometry][safe_name] = mesh_prim

        try:
            convert_mesh(mesh_prim, resolved_path, data)
        except Exception as e:
            Tf.Warn(f"Failed to convert mesh: {resolved_path!s}: {e}")

    usdex.core.saveStage(data.libraries[Tokens.Geometry], comment=f"Mesh Library for {data.urdf_parser.get_robot_name()}. {data.comment}")


def convert_mesh(prim: Usd.Prim, input_path: pathlib.Path, data: ConversionData):
    if input_path.suffix.lower() == ".stl":
        convert_stl(prim, input_path, data)
    elif input_path.suffix.lower() == ".obj":
        convert_obj(prim, input_path, data)
    elif input_path.suffix.lower() == ".dae":
        # TODO: Implement DAE conversion.
        Tf.Warn(f"The dae format is not yet supported: {input_path}")
    else:
        if not input_path.is_dir():
            Tf.Warn(f"Unsupported mesh format: {input_path}")


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
        Tf.Warn(f'Failed to convert mesh "{prim.GetPath()}" from {input_path}')
    return usd_mesh


def _convert_single_obj(prim: Usd.Prim, input_path: pathlib.Path, reader: tinyobjloader.ObjReader) -> UsdGeom.Mesh:
    """
    Convert a single OBJ mesh to a USD mesh.
    """
    shapes = reader.GetShapes()
    attrib = reader.GetAttrib()
    obj_mesh = shapes[0].mesh

    vertices = attrib.vertices
    face_vertex_counts = obj_mesh.num_face_vertices
    face_vertex_indices = obj_mesh.vertex_indices()

    points = [Gf.Vec3f(vertices[i], vertices[i + 1], vertices[i + 2]) for i in range(0, len(vertices), 3)]

    normals = None
    source_normals = attrib.normals
    if len(source_normals) > 0:
        normals_data = [Gf.Vec3f(source_normals[i], source_normals[i + 1], source_normals[i + 2]) for i in range(0, len(source_normals), 3)]
        normals = usdex.core.Vec3fPrimvarData(UsdGeom.Tokens.faceVarying, Vt.Vec3fArray(normals_data), Vt.IntArray(obj_mesh.normal_indices()))
        normals.index()  # re-index the normals to remove duplicates

    uvs = None
    source_uvs = attrib.texcoords
    if len(source_uvs) > 0:
        uv_data = [Gf.Vec2f(source_uvs[i], source_uvs[i + 1]) for i in range(0, len(source_uvs), 2)]
        uvs = usdex.core.Vec2fPrimvarData(UsdGeom.Tokens.faceVarying, Vt.Vec2fArray(uv_data), Vt.IntArray(obj_mesh.texcoord_indices()))
        uvs.index()  # re-index the uvs to remove duplicates

    usd_mesh = usdex.core.definePolyMesh(
        prim.GetParent(),
        prim.GetName(),
        faceVertexCounts=Vt.IntArray(face_vertex_counts),
        faceVertexIndices=Vt.IntArray(face_vertex_indices),
        points=Vt.Vec3fArray(points),
        normals=normals,
        uvs=uvs,
    )
    if not usd_mesh:
        Tf.Warn(f'Failed to convert mesh "{prim.GetPath()}" from {input_path}')
    return usd_mesh


def convert_obj(prim: Usd.Prim, input_path: pathlib.Path, data: ConversionData) -> UsdGeom.Mesh | UsdGeom.Xform:
    reader = tinyobjloader.ObjReader()
    if not reader.ParseFromFile(str(input_path)):
        Tf.Warn(f'Invalid input_path: "{input_path}" could not be parsed. {reader.Error()}')
        return None

    shapes = reader.GetShapes()
    if len(shapes) == 0:
        Tf.Warn(f'Invalid input_path: "{input_path}" contains no meshes')
        return None
    elif len(shapes) == 1:
        return _convert_single_obj(prim, input_path, reader)

    attrib = reader.GetAttrib()

    names = []
    for shape in shapes:
        name = shape.name if shape.name else prim.GetName()
        names.append(name)
    safe_names = data.name_cache.getPrimNames(prim, names)

    for shape, name, safe_name in zip(shapes, names, safe_names):
        obj_mesh = shape.mesh
        face_vertex_counts = obj_mesh.num_face_vertices

        # Get indices directly as arrays
        vertex_indices_in_shape = np.array(obj_mesh.vertex_indices(), dtype=np.int32)

        # Process vertices using NumPy for speed
        unique_vertex_indices = np.unique(vertex_indices_in_shape)

        # Extract vertices: reshape attrib.vertices and use NumPy indexing
        vertices_array = np.array(attrib.vertices, dtype=np.float32).reshape(-1, 3)
        points_array = vertices_array[unique_vertex_indices]
        points = [Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in points_array]

        # Remap indices using NumPy searchsorted
        face_vertex_indices = np.searchsorted(unique_vertex_indices, vertex_indices_in_shape).tolist()

        # Process normals
        normals = None
        if len(attrib.normals) > 0:
            normal_indices_in_shape = np.array(obj_mesh.normal_indices(), dtype=np.int32)
            unique_normal_indices = np.unique(normal_indices_in_shape)

            normals_array = np.array(attrib.normals, dtype=np.float32).reshape(-1, 3)
            normals_data_array = normals_array[unique_normal_indices]
            normals_data = [Gf.Vec3f(float(n[0]), float(n[1]), float(n[2])) for n in normals_data_array]

            remapped_normal_indices = np.searchsorted(unique_normal_indices, normal_indices_in_shape).tolist()
            normals = usdex.core.Vec3fPrimvarData(UsdGeom.Tokens.faceVarying, Vt.Vec3fArray(normals_data), Vt.IntArray(remapped_normal_indices))
            normals.index()  # re-index the normals to remove duplicates

        # Process UV coordinates
        uvs = None
        if len(attrib.texcoords) > 0:
            texcoord_indices_in_shape = np.array(obj_mesh.texcoord_indices(), dtype=np.int32)
            unique_texcoord_indices = np.unique(texcoord_indices_in_shape)

            texcoords_array = np.array(attrib.texcoords, dtype=np.float32).reshape(-1, 2)
            uv_data_array = texcoords_array[unique_texcoord_indices]
            uv_data = [Gf.Vec2f(float(uv[0]), float(uv[1])) for uv in uv_data_array]

            remapped_texcoord_indices = np.searchsorted(unique_texcoord_indices, texcoord_indices_in_shape).tolist()
            uvs = usdex.core.Vec2fPrimvarData(UsdGeom.Tokens.faceVarying, Vt.Vec2fArray(uv_data), Vt.IntArray(remapped_texcoord_indices))
            uvs.index()  # re-index the uvs to remove duplicates

        usd_mesh = usdex.core.definePolyMesh(
            prim,
            safe_name,
            faceVertexCounts=Vt.IntArray(face_vertex_counts),
            faceVertexIndices=Vt.IntArray(face_vertex_indices),
            points=Vt.Vec3fArray(points),
            normals=normals,
            uvs=uvs,
        )
        if not usd_mesh:
            Tf.Warn(f'Failed to convert mesh "{prim.GetPath()}" from {input_path}')
            return None

        if name != safe_name:
            usdex.core.setDisplayName(usd_mesh.GetPrim(), name)

    return prim
