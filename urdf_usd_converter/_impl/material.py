# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib
import shutil

import tinyobjloader
import usdex.core
from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdShade, UsdUtils

from .data import ConversionData, Tokens
from .material_cache import MaterialCache
from .material_data import MaterialData
from .ros_package import resolve_ros_package_paths

__all__ = [
    "bind_material",
    "bind_mesh_material",
    "convert_materials",
    "store_mesh_material_reference",
    "store_obj_material_data",
]


def convert_materials(data: ConversionData):
    # Acquire the global material data for URDF and the material data for obj/dae files.
    material_cache = MaterialCache(data)
    if not len(data.material_data_list):
        return

    # Copy the textures to the payload directory.
    _copy_textures(material_cache, data)

    data.libraries[Tokens.Materials] = usdex.core.addAssetLibrary(data.content[Tokens.Contents], Tokens.Materials, format="usdc")
    data.references[Tokens.Materials] = {}

    materials_scope = data.libraries[Tokens.Materials].GetDefaultPrim()

    # Set the safe names of the material data list.
    material_cache.store_safe_names(data)

    # Convert the material data to USD.
    for material_data in data.material_data_list:
        material_prim = _convert_material(
            materials_scope,
            material_data.safe_name,
            material_data.diffuse_color,
            material_data.specular_color,
            material_data.opacity,
            material_data.roughness,
            material_data.metallic,
            material_data.ior,
            material_data.diffuse_texture_path,
            material_data.specular_texture_path,
            material_data.normal_texture_path,
            material_data.roughness_texture_path,
            material_data.metallic_texture_path,
            material_data.opacity_texture_path,
            material_cache.texture_paths,
            data,
        )
        data.references[Tokens.Materials][material_data.safe_name] = material_prim
        if material_data.name != material_data.safe_name:
            usdex.core.setDisplayName(material_prim.GetPrim(), material_data.name)

    robot_name = data.urdf_parser.get_robot_name()
    usdex.core.saveStage(data.libraries[Tokens.Materials], comment=f"Material Library for {robot_name}. {data.comment}")

    # setup a content layer for referenced materials
    data.content[Tokens.Materials] = usdex.core.addAssetContent(data.content[Tokens.Contents], Tokens.Materials, format="usda")


def _copy_textures(material_cache: MaterialCache, data: ConversionData):
    """
    Copy the textures to the payload directory.

    Args:
        material_cache: The material cache.
        data: The conversion data.
    """
    if not len(material_cache.texture_paths):
        return

    # copy the texture to the payload directory
    local_texture_dir = pathlib.Path(data.content[Tokens.Contents].GetRootLayer().identifier).parent / Tokens.Textures
    if not local_texture_dir.exists():
        local_texture_dir.mkdir(parents=True)

    for texture_path in material_cache.texture_paths:
        # At this stage, the existence has already been checked.
        if texture_path.exists():
            unique_file_name = material_cache.texture_paths[texture_path]

            local_texture_path = local_texture_dir / unique_file_name
            shutil.copyfile(texture_path, local_texture_path)
            Tf.Status(f"Copied texture {texture_path} to {local_texture_path}")


def _convert_material(
    parent: Usd.Prim,
    safe_name: str,
    diffuse_color: Gf.Vec3f,
    specular_color: Gf.Vec3f,
    opacity: float,
    roughness: float,
    metallic: float,
    ior: float,
    diffuse_texture_path: pathlib.Path | None,
    specular_texture_path: pathlib.Path | None,
    normal_texture_path: pathlib.Path | None,
    roughness_texture_path: pathlib.Path | None,
    metallic_texture_path: pathlib.Path | None,
    opacity_texture_path: pathlib.Path | None,
    texture_paths: dict[pathlib.Path, str],
    data: ConversionData,
) -> UsdShade.Material:
    """
    Convert a material to USD.
    This is used for both URDF global materials and materials in obj/dae files.

    Args:
        parent: The parent prim.
        safe_name: The safe name of the material. This is a unique name that does not overlap with other material names.
        diffuse_color: The diffuse color of the material.
        specular_color: The specular color of the material.
        opacity: The opacity of the material.
        roughness: The roughness of the material.
        metallic: The metallic of the material.
        ior: The ior of the material.
        diffuse_texture_path: The path to the diffuse texture.
        specular_texture_path: The path to the specular texture.
        normal_texture_path: The path to the normal texture.
        roughness_texture_path: The path to the roughness texture.
        metallic_texture_path: The path to the metallic texture.
        opacity_texture_path: The path to the opacity texture.
        texture_paths: A dictionary of texture paths and unique names.
        data: The conversion data.

    Returns:
        The material prim.
    """
    diffuse_color = usdex.core.sRgbToLinear(diffuse_color)
    specular_color = usdex.core.sRgbToLinear(specular_color)

    # Build kwargs for material properties
    material_kwargs = {
        "color": diffuse_color,
        "opacity": opacity,
        "roughness": roughness,
        "metallic": metallic,
    }

    # Define the material.
    material_prim = usdex.core.definePreviewMaterial(parent, safe_name, **material_kwargs)
    if not material_prim:
        Tf.RaiseRuntimeError(f'Failed to convert material "{safe_name}"')

    surface_shader: UsdShade.Shader = usdex.core.computeEffectivePreviewSurfaceShader(material_prim)
    if ior != 0.0:
        surface_shader.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(ior)

    if diffuse_texture_path:
        usdex.core.addDiffuseTextureToPreviewMaterial(material_prim, _get_texture_asset_path(diffuse_texture_path, texture_paths, data))

    if normal_texture_path:
        usdex.core.addNormalTextureToPreviewMaterial(material_prim, _get_texture_asset_path(normal_texture_path, texture_paths, data))

    if roughness_texture_path:
        usdex.core.addRoughnessTextureToPreviewMaterial(material_prim, _get_texture_asset_path(roughness_texture_path, texture_paths, data))

    if metallic_texture_path:
        usdex.core.addMetallicTextureToPreviewMaterial(material_prim, _get_texture_asset_path(metallic_texture_path, texture_paths, data))

    if opacity_texture_path:
        usdex.core.addOpacityTextureToPreviewMaterial(material_prim, _get_texture_asset_path(opacity_texture_path, texture_paths, data))

    # If the specular color is not black or the specular texture exists, use the specular workflow.
    if specular_color != [0, 0, 0] or specular_texture_path:
        surface_shader.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(1)
        surface_shader.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(specular_color)
        if specular_texture_path:
            _add_specular_texture_to_preview_material(material_prim, _get_texture_asset_path(specular_texture_path, texture_paths, data))

    result = usdex.core.addPreviewMaterialInterface(material_prim)
    if not result:
        Tf.RaiseRuntimeError(f'Failed to add material instance to material prim "{material_prim.GetPath()}"')

    material_prim.GetPrim().SetInstanceable(True)

    return material_prim


def _add_specular_texture_to_preview_material(material_prim: UsdShade.Material, specular_texture_path: Sdf.AssetPath):
    """
    Add the specular texture to the preview material.

    Args:
        material_prim: The material prim.
        specular_texture_path: The path to the specular texture.
    """
    surface: UsdShade.Shader = usdex.core.computeEffectivePreviewSurfaceShader(material_prim)

    specular_color = Gf.Vec3f(0.0, 0.0, 0.0)
    specular_color_input = surface.GetInput("specularColor")
    if specular_color_input:
        value_attrs = specular_color_input.GetValueProducingAttributes()
        if value_attrs and len(value_attrs) > 0:
            specular_color = value_attrs[0].Get()
            specular_color_input.GetAttr().Clear()
    else:
        specular_color_input = surface.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f)
    fallback = Gf.Vec4f(specular_color[0], specular_color[1], specular_color[2], 1.0)

    # Acquire the texture reader.
    texture_reader: UsdShade.Shader = _acquire_texture_reader(
        material_prim, "SpecularTexture", specular_texture_path, usdex.core.ColorSpace.eAuto, fallback
    )

    # Connect the PreviewSurface shader "specularColor" to the specular texture shader output
    specular_color_input.ConnectToSource(texture_reader.CreateOutput("rgb", Sdf.ValueTypeNames.Float3))


def _acquire_texture_reader(
    material_prim: UsdShade.Material,
    shader_name: str,
    texture_path: pathlib.Path,
    color_space: usdex.core.ColorSpace,
    fallback: Gf.Vec4f,
) -> UsdShade.Shader:
    """
    Acquire the texture reader.

    Args:
        material_prim: The material prim.
        shader_name: The name of the shader.
        texture_path: The path to the texture.
        color_space: The color space of the texture.
        fallback: The fallback value for the texture.

    Returns:
        The texture reader.
    """
    shader_path = material_prim.GetPath().AppendChild(shader_name)
    tex_shader = UsdShade.Shader.Define(material_prim.GetPrim().GetStage(), shader_path)
    tex_shader.SetShaderId("UsdUVTexture")
    tex_shader.CreateInput("fallback", Sdf.ValueTypeNames.Float4).Set(fallback)
    tex_shader.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture_path)
    tex_shader.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set(usdex.core.getColorSpaceToken(color_space))
    st_input = tex_shader.CreateInput("st", Sdf.ValueTypeNames.Float2)
    connected = usdex.core.connectPrimvarShader(st_input, UsdUtils.GetPrimaryUVSetName())
    if not connected:
        return UsdShade.Shader()

    return tex_shader


def _get_texture_asset_path(texture_path: pathlib.Path, texture_paths: dict[pathlib.Path, str], data: ConversionData) -> Sdf.AssetPath:
    """
    Get the asset path for the texture.

    Args:
        texture_path: The path to the texture.
        texture_paths: A dictionary of texture paths and unique names.

    Returns:
        The asset path for the texture.
    """
    # The path to the texture to reference. If None, the texture does not exist.
    unique_file_name = texture_paths.get(texture_path)

    # If the texture exists, add the texture to the material.
    payload_dir = pathlib.Path(data.content[Tokens.Contents].GetRootLayer().identifier).parent
    local_texture_dir = payload_dir / Tokens.Textures
    local_texture_path = local_texture_dir / unique_file_name
    if local_texture_path.exists():
        relative_texture_path = local_texture_path.relative_to(payload_dir)
        return Sdf.AssetPath(f"./{relative_texture_path.as_posix()}")
    else:
        return Sdf.AssetPath("")


def store_obj_material_data(mesh_file_path: pathlib.Path, reader: tinyobjloader.ObjReader, data: ConversionData):
    """
    Store the material data from the OBJ file.
    This is used to temporarily cache material parameters when loading an OBJ mesh.

    Args:
        mesh_file_path: The path to the mesh file.
        reader: The tinyobjloader reader.
        data: The conversion data.
    """
    materials = reader.GetMaterials()
    for material in materials:
        material_data = MaterialData()
        material_data.mesh_file_path = mesh_file_path
        material_data.name = material.name
        material_data.diffuse_color = Gf.Vec3f(material.diffuse[0], material.diffuse[1], material.diffuse[2])
        material_data.specular_color = Gf.Vec3f(material.specular[0], material.specular[1], material.specular[2])
        material_data.opacity = material.dissolve
        material_data.ior = material.ior if material.ior else 0.0

        # The following is the extended specification of obj.
        material_data.roughness = material.roughness if material.roughness else 0.5
        material_data.metallic = material.metallic if material.metallic else 0.0

        material_data.diffuse_texture_path = (mesh_file_path.parent / material.diffuse_texname) if material.diffuse_texname else None
        material_data.specular_texture_path = (mesh_file_path.parent / material.specular_texname) if material.specular_texname else None
        material_data.normal_texture_path = (mesh_file_path.parent / material.normal_texname) if material.normal_texname else None
        material_data.roughness_texture_path = (mesh_file_path.parent / material.roughness_texname) if material.roughness_texname else None
        material_data.metallic_texture_path = (mesh_file_path.parent / material.metallic_texname) if material.metallic_texname else None
        material_data.opacity_texture_path = (mesh_file_path.parent / material.alpha_texname) if material.alpha_texname else None

        # If the normal texture is not specified, use the bump texture.
        if material_data.normal_texture_path is None:
            material_data.normal_texture_path = (mesh_file_path.parent / material.bump_texname) if material.bump_texname else None

        data.material_data_list.append(material_data)


def store_mesh_material_reference(mesh_file_path: pathlib.Path, mesh_safe_name: str, material_name: str, data: ConversionData):
    """
    Store the per-mesh material reference.

    Args:
        mesh_file_path: The path to the source file.
        mesh_safe_name: The safe name of the mesh.
        material_name: The name of the material.
        data: The conversion data.
    """
    if mesh_file_path not in data.mesh_material_references:
        data.mesh_material_references[mesh_file_path] = {}
    data.mesh_material_references[mesh_file_path][mesh_safe_name] = material_name


def _get_material_by_name(mesh_file_path: pathlib.Path | None, material_name: str, data: ConversionData) -> UsdShade.Material:
    """
    Get the material by the mesh path and material name.

    Args:
        mesh_file_path: The path to the mesh file. If None, the material is a global material.
        material_name: The name of the material.
        data: The conversion data.

    Returns:
        The material if found, otherwise None.
    """
    for material_data in data.material_data_list:
        if material_data.name == material_name and material_data.mesh_file_path == mesh_file_path:
            return data.references[Tokens.Materials][material_data.safe_name]
    return None


def bind_material(geom_prim: Usd.Prim, mesh_file_path: pathlib.Path | None, material_name: str, data: ConversionData):
    """
    Bind the material to the geometries.
    If there are meshes in the Xform, it will traverse the meshes and assign materials to them.

    Args:
        geom_prim: The geometry prim.
        mesh_file_path: The path to the mesh file. If None, the material is a global material.
        material_name: The name of the material.
        data: The conversion data.
    """
    local_materials = data.content[Tokens.Materials].GetDefaultPrim().GetChild(Tokens.Materials)

    # Get the material by the mesh path and material name.
    ref_material = _get_material_by_name(mesh_file_path, material_name, data)
    if not ref_material:
        Tf.Warn(f"Material '{material_name}' not found in Material Library {data.libraries[Tokens.Materials].GetRootLayer().identifier}")
        return

    # If the material does not exist in the Material layer, define the reference.
    material_prim = UsdShade.Material(local_materials.GetChild(ref_material.GetPrim().GetName()))
    if not material_prim:
        material_prim = UsdShade.Material(usdex.core.defineReference(local_materials, ref_material.GetPrim(), ref_material.GetPrim().GetName()))

    # If the geometry is a cube, sphere, or cylinder, check if the material has a texture.
    if mesh_file_path is None and (geom_prim.IsA(UsdGeom.Cube) or geom_prim.IsA(UsdGeom.Sphere) or geom_prim.IsA(UsdGeom.Cylinder)):
        # Get the texture from the material.
        materials = data.urdf_parser.get_materials()
        for material in materials:
            if material[0] == material_name:
                if material[2]:
                    Tf.Warn(f"Textures are not projection mapped for Cube, Sphere, and Cylinder: {geom_prim.GetPath()}")
                break

    # Bind the material to the geometry.
    geom_over = data.content[Tokens.Materials].OverridePrim(geom_prim.GetPath())
    usdex.core.bindMaterial(geom_over, material_prim)


def bind_mesh_material(geom_prim: Usd.Prim, mesh_file_name: str, data: ConversionData):
    """
    Bind the material to the meshes in the geometry.
    Each mesh references a mesh within the GeometryLibrary,
    and if a material exists for the prim name at that time, it searches for and binds it.

    Args:
        geom_prim: The geometry prim.
        mesh_file_name: The name of the mesh file.
        data: The conversion data.
    """
    resolved_file_path = resolve_ros_package_paths(mesh_file_name, data)
    for prim in Usd.PrimRange(geom_prim):
        if prim.IsA(UsdGeom.Mesh):
            mesh_name = prim.GetName()
            if resolved_file_path in data.mesh_material_references and mesh_name in data.mesh_material_references[resolved_file_path]:
                material_name = data.mesh_material_references[resolved_file_path][mesh_name]
                bind_material(prim, resolved_file_path, material_name, data)
