# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib
import shutil

import tinyobjloader
import usdex.core
from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdShade

from .data import ConversionData, Tokens
from .ros_package import resolve_ros_package_paths

__all__ = [
    "add_diffuse_texture_to_preview_material",
    "bind_material",
    "convert_materials",
    "get_material_texture_paths",
]


def convert_materials(data: ConversionData):
    # Copy the textures to the payload directory.
    _copy_textures(data)

    materials = data.urdf_parser.get_materials()
    if not len(materials):
        return

    data.libraries[Tokens.Materials] = usdex.core.addAssetLibrary(data.content[Tokens.Contents], Tokens.Materials, format="usdc")
    data.references[Tokens.Materials] = {}

    materials_scope = data.libraries[Tokens.Materials].GetDefaultPrim()

    source_names = [material[0] for material in materials]
    safe_names = data.name_cache.getPrimNames(materials_scope, source_names)

    # In this case, source_name is a unique name.
    for material, source_name, safe_name in zip(materials, source_names, safe_names):
        material_prim = convert_material(materials_scope, safe_name, material, data).GetPrim()
        data.references[Tokens.Materials][source_name] = material_prim
        if source_name != safe_name:
            usdex.core.setDisplayName(material_prim, source_name)

    robot_name = data.urdf_parser.get_robot_name()
    usdex.core.saveStage(data.libraries[Tokens.Materials], comment=f"Material Library for {robot_name}. {data.comment}")

    # setup a content layer for referenced materials
    data.content[Tokens.Materials] = usdex.core.addAssetContent(data.content[Tokens.Contents], Tokens.Materials, format="usda")


def get_material_texture_paths(data: ConversionData) -> dict[pathlib.Path, str]:
    """
    Create a dictionary of resolved texture paths and unique names.
    These include all global materials and the textures of materials referenced by meshes.

    Args:
        data: The conversion data.

    Returns:
        A dictionary of texture paths and unique names.
    """
    materials = data.urdf_parser.get_materials()

    # URDF file directory
    urdf_dir = data.urdf_parser.input_file.parent

    existing_texture_paths: list[pathlib.Path] = []

    # Get the texture paths from the materials.
    for material in materials:
        texture_path = material[2]
        if texture_path:
            # Resolve the ROS package paths.
            # If the path is not a ROS package, it will return the original path.
            resolved_texture_path = resolve_ros_package_paths(texture_path, data)
            if resolved_texture_path and resolved_texture_path.exists():
                if resolved_texture_path not in existing_texture_paths:
                    existing_texture_paths.append(resolved_texture_path)
            else:
                Tf.Warn(f"Texture file not found: {texture_path}")

    # Get the texture paths from the meshes.
    meshes = data.urdf_parser.get_meshes()
    for mesh in meshes:
        mesh_path = urdf_dir / mesh[0]
        # _get_mesh_texture_paths already checks for the existence of texture files,
        # so there is no need to perform an existence check here.
        mesh_texture_paths = _get_mesh_texture_paths(mesh_path)
        for mesh_texture_path in mesh_texture_paths:
            if mesh_texture_path and mesh_texture_path not in existing_texture_paths:
                existing_texture_paths.append(mesh_texture_path)

    # Create a list of texture filenames.
    names = [texture_path.name for texture_path in existing_texture_paths]

    # Rename the list of image filenames to unique names.
    unique_file_names = []
    name_counts = {}
    for name in names:
        if name not in name_counts:
            name_counts[name] = 0
            unique_file_names.append(name)
        else:
            name_counts[name] += 1
            stem = pathlib.Path(name).stem
            suffix = pathlib.Path(name).suffix
            unique_name = f"{stem}_{name_counts[name]}{suffix}"
            unique_file_names.append(unique_name)

    texture_paths = dict(zip(existing_texture_paths, unique_file_names))

    return texture_paths


def _copy_textures(data: ConversionData):
    """
    Copy the textures to the payload directory.

    Args:
        texture_paths: A dictionary of texture paths and unique names.
        data: The conversion data.
    """
    if not data.texture_paths:
        return

    # copy the texture to the payload directory
    local_texture_dir = pathlib.Path(data.content[Tokens.Contents].GetRootLayer().identifier).parent / Tokens.Textures
    if not local_texture_dir.exists():
        local_texture_dir.mkdir(parents=True)

    for texture_path in data.texture_paths:
        unique_file_name = data.texture_paths[texture_path]

        # At this stage, the existence has already been checked.
        local_texture_path = local_texture_dir / unique_file_name
        shutil.copyfile(texture_path, local_texture_path)
        Tf.Status(f"Copied texture {texture_path} to {local_texture_path}")


def convert_material(
    parent: Usd.Prim,
    name: str,
    material: tuple[str, tuple[float, float, float, float], str | None],
    data: ConversionData,
) -> Usd.Prim:
    """
    Convert a material to USD.
    """
    color = usdex.core.sRgbToLinear(Gf.Vec3f(*material[1][:3]))
    opacity = material[1][3]
    texture_path = material[2]

    # Build kwargs for material properties
    material_kwargs = {
        "color": color,
        "opacity": opacity,
    }

    # Define the material.
    material_prim = usdex.core.definePreviewMaterial(parent, name, **material_kwargs)
    if not material_prim:
        Tf.RaiseRuntimeError(f'Failed to convert material "{name}"')

    # Resolve the ROS package paths.
    # If the path is not a ROS package, it will return the original path.
    resolved_texture_path = resolve_ros_package_paths(texture_path, data) if texture_path else None

    # Add the diffuse texture to the preview material.
    if resolved_texture_path:
        add_diffuse_texture_to_preview_material(material_prim, resolved_texture_path, data)

        # If both color and texture are present, they are multiplied together.
        # For UsdPreviewSurface, multiplication uses bias.
        if texture_path and color != Gf.Vec3f(1, 1, 1):
            shader = usdex.core.computeEffectivePreviewSurfaceShader(material_prim)
            diffuse_color_input = shader.GetInput("diffuseColor")
            if diffuse_color_input.HasConnectedSource():
                source = diffuse_color_input.GetConnectedSource()
                if len(source) > 0 and isinstance(source[0], UsdShade.ConnectableAPI) and source[0].GetPrim().IsA(UsdShade.Shader):
                    diffuse_texture_shader = UsdShade.Shader(source[0].GetPrim())
                    diffuse_texture_shader.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(color[0], color[1], color[2], 1.0))

    return material_prim


def add_diffuse_texture_to_preview_material(material_prim: UsdShade.Material, texture_path: pathlib.Path, data: ConversionData):
    """
    Add the diffuse texture to the preview material.

    Args:
        material_prim: The preview material prim.
        texture_path: The path to the texture.
        data: The conversion data.
    """
    # The path to the texture to reference. If None, the texture does not exist.
    unique_file_name = data.texture_paths.get(texture_path, None)

    # If the texture exists, add the texture to the material.
    if unique_file_name:
        payload_dir = pathlib.Path(data.content[Tokens.Contents].GetRootLayer().identifier).parent
        local_texture_dir = payload_dir / Tokens.Textures
        local_texture_path = local_texture_dir / unique_file_name
        relative_texture_path = local_texture_path.relative_to(payload_dir)
        asset_path = Sdf.AssetPath(f"./{relative_texture_path.as_posix()}")
        usdex.core.addDiffuseTextureToPreviewMaterial(material_prim, asset_path)


def _get_mesh_texture_paths(input_path: pathlib.Path) -> list[pathlib.Path]:
    """
    Get the texture paths from the mesh file.

    Args:
        input_path: The path to the OBJ file.

    Returns:
        A list of texture paths.
    """
    if input_path.suffix.lower() == ".obj":
        return _get_obj_texture_paths(input_path)
    return []


def _get_obj_texture_paths(input_path: pathlib.Path) -> list[pathlib.Path]:
    """
    Get the textures from the OBJ file.

    Args:
        input_path: The path to the OBJ file.

    Returns:
        A list of texture paths.
    """
    texture_paths = []
    reader = tinyobjloader.ObjReader()
    if not reader.ParseFromFile(str(input_path)):
        return texture_paths

    # Get texture names from materials.
    tex_names = []
    materials = reader.GetMaterials()
    for material in materials:
        # Diffuse/Base Color texture (map_Kd)
        if material.diffuse_texname and material.diffuse_texname not in tex_names:
            tex_names.append(material.diffuse_texname)

    # obj file directory
    obj_dir = input_path.parent

    for tex_name in tex_names:
        texture_path = obj_dir / tex_name
        if texture_path.exists():
            texture_paths.append(texture_path)

    return texture_paths


def bind_material(geom_prim: Usd.Prim, name: str, data: ConversionData):
    """
    Bind the material to the geometries.
    This is used for binding global materials in urdf.
    If there are meshes in the Xform, it will traverse the meshes and assign materials to them.

    Args:
        geom_prim: The geometry prim.
        name: The name of the material.
        data: The conversion data.
    """
    local_materials = data.content[Tokens.Materials].GetDefaultPrim().GetChild(Tokens.Materials)
    ref_material: Usd.Prim = data.references[Tokens.Materials].get(name)
    if not ref_material:
        Tf.Warn(f"Material '{name}' not found in Material Library {data.libraries[Tokens.Materials].GetRootLayer().identifier}")
        return
    material_prim = UsdShade.Material(local_materials.GetChild(ref_material.GetName()))
    if not material_prim:
        material_prim = UsdShade.Material(usdex.core.defineReference(local_materials, ref_material, ref_material.GetName()))

    # If the geometry is a cube, sphere, or cylinder, check if the material has a texture.
    if geom_prim.IsA(UsdGeom.Cube) or geom_prim.IsA(UsdGeom.Sphere) or geom_prim.IsA(UsdGeom.Cylinder):
        # Get the texture from the material.
        materials = data.urdf_parser.get_materials()
        for material in materials:
            if material[0] == name:
                if material[2]:
                    Tf.Warn(f"Textures are not projection mapped for Cube, Sphere, and Cylinder: {geom_prim.GetPath()}")
                break

    # Bind the material to the geometry.
    for prim in Usd.PrimRange(geom_prim):
        if prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Cube) or prim.IsA(UsdGeom.Sphere) or prim.IsA(UsdGeom.Cylinder):
            geom_over = data.content[Tokens.Materials].OverridePrim(prim.GetPath())
            usdex.core.bindMaterial(geom_over, material_prim)
