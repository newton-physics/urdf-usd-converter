# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib
import shutil
from collections import Counter

import collada
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
    "store_dae_material_data",
    "store_mesh_material_reference",
    "store_obj_material_data",
    "use_material_id",
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
            material_data,
            material_cache.texture_paths,
            data,
        )
        data.references[Tokens.Materials][material_data.safe_name] = material_prim
        display_name = material_data.get_display_name()
        if display_name != material_data.safe_name:
            usdex.core.setDisplayName(material_prim.GetPrim(), display_name)

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
    material_data: MaterialData,
    texture_paths: dict[pathlib.Path, str],
    data: ConversionData,
) -> UsdShade.Material:
    """
    Convert a material to USD.
    This is used for both URDF global materials and materials in obj/dae files.

    Args:
        parent: The parent prim.
        safe_name: The safe name of the material. This is a unique name that does not overlap with other material names.
        material_data: The material data. Various material parameters, texture paths, and other settings are stored here.
        texture_paths: A dictionary of texture paths and unique names.
        data: The conversion data.

    Returns:
        The material prim.
    """
    diffuse_color = usdex.core.sRgbToLinear(material_data.diffuse_color)
    specular_color = usdex.core.sRgbToLinear(material_data.specular_color)
    emissive_color = usdex.core.sRgbToLinear(material_data.emissive_color)

    # Build kwargs for material properties
    material_kwargs = {
        "color": diffuse_color,
        "opacity": material_data.opacity,
        "roughness": material_data.roughness,
        "metallic": material_data.metallic,
    }

    # Define the material.
    material_prim = usdex.core.definePreviewMaterial(parent, safe_name, **material_kwargs)
    if not material_prim:
        Tf.RaiseRuntimeError(f'Failed to convert material "{safe_name}"')

    surface_shader: UsdShade.Shader = usdex.core.computeEffectivePreviewSurfaceShader(material_prim)
    if material_data.ior != 0.0:
        surface_shader.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(material_data.ior)

    if material_data.diffuse_texture_path:
        usdex.core.addDiffuseTextureToPreviewMaterial(material_prim, _get_texture_asset_path(material_data.diffuse_texture_path, texture_paths, data))

    if material_data.normal_texture_path:
        usdex.core.addNormalTextureToPreviewMaterial(material_prim, _get_texture_asset_path(material_data.normal_texture_path, texture_paths, data))

    if material_data.roughness_texture_path:
        usdex.core.addRoughnessTextureToPreviewMaterial(
            material_prim, _get_texture_asset_path(material_data.roughness_texture_path, texture_paths, data)
        )

    if material_data.metallic_texture_path:
        usdex.core.addMetallicTextureToPreviewMaterial(
            material_prim, _get_texture_asset_path(material_data.metallic_texture_path, texture_paths, data)
        )

    if material_data.opacity_texture_path:
        usdex.core.addOpacityTextureToPreviewMaterial(material_prim, _get_texture_asset_path(material_data.opacity_texture_path, texture_paths, data))

    # If the specular color is not black or the specular texture exists, use the specular workflow.
    if specular_color != [0, 0, 0] or material_data.specular_texture_path:
        surface_shader.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(1)
        surface_shader.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(specular_color)
        if material_data.specular_texture_path:
            _add_color_texture_to_preview_material(
                material_prim, "specularColor", "SpecularTexture", _get_texture_asset_path(material_data.specular_texture_path, texture_paths, data)
            )

    # Add the emissive color to the preview material.
    if emissive_color != [0, 0, 0] or material_data.emissive_texture_path:
        surface_shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(emissive_color)
        if material_data.emissive_texture_path:
            _add_color_texture_to_preview_material(
                material_prim, "emissiveColor", "EmissiveTexture", _get_texture_asset_path(material_data.emissive_texture_path, texture_paths, data)
            )

    # Add the material interface.
    result = usdex.core.addPreviewMaterialInterface(material_prim)
    if not result:
        Tf.RaiseRuntimeError(f'Failed to add material instance to material prim "{material_prim.GetPath()}"')

    # Set the wrap mode to repeat.
    _set_wrap_mode(material_prim, "repeat")

    material_prim.GetPrim().SetInstanceable(True)

    return material_prim


def _add_color_texture_to_preview_material(material_prim: UsdShade.Material, input_name: str, shader_name: str, texture_path: Sdf.AssetPath):
    """
    Add the color texture(e.g., specular, emissive) to the preview material.

    Args:
        material_prim: The material prim.
        input_name: The name of the input (e.g., "specularColor", "emissiveColor").
        shader_name: The name of the shader (e.g., "SpecularTexture", "EmissiveTexture").
        texture_path: The path to the texture.
    """
    surface: UsdShade.Shader = usdex.core.computeEffectivePreviewSurfaceShader(material_prim)

    color = Gf.Vec3f(0.0, 0.0, 0.0)
    color_input = surface.GetInput(input_name)
    if color_input:
        value_attrs = color_input.GetValueProducingAttributes()
        if value_attrs and len(value_attrs) > 0:
            color = value_attrs[0].Get()
            color_input.GetAttr().Clear()
    fallback = Gf.Vec4f(color[0], color[1], color[2], 1.0)

    # Acquire the texture reader.
    texture_reader: UsdShade.Shader = _acquire_texture_reader(material_prim, shader_name, texture_path, usdex.core.ColorSpace.eAuto, fallback)

    # Connect the PreviewSurface shader "input_name" to the color texture shader output
    color_input.ConnectToSource(texture_reader.CreateOutput("rgb", Sdf.ValueTypeNames.Float3))


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


def _set_wrap_mode(material_prim: UsdShade.Material, wrap_mode: str):
    wrap_mode_input = material_prim.CreateInput("wrapMode", Sdf.ValueTypeNames.Token)
    wrap_mode_input.Set(wrap_mode)
    for child in material_prim.GetPrim().GetAllChildren():
        shader = UsdShade.Shader(child)
        if shader.GetShaderId() == "UsdUVTexture":
            shader.CreateInput("wrapS", Sdf.ValueTypeNames.Token).ConnectToSource(wrap_mode_input)
            shader.CreateInput("wrapT", Sdf.ValueTypeNames.Token).ConnectToSource(wrap_mode_input)


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


def _process_dae_effect_color_property(
    material_effect: collada.material.Effect,
    property_name: str,
    mesh_file_path: pathlib.Path,
    material_data: MaterialData,
    texture_attr_name: str,
    color_attr_name: str,
):
    """
    Process a single DAE effect color property (diffuse, specular, emission).
    'texture_attr_name' and 'color_attr_name' are the property names in the MaterialData class.

    Args:
        material_effect: The material effect.
        property_name: The name of the material efect property (e.g., 'diffuse', 'specular', 'emission').
        mesh_file_path: The path to the mesh file.
        material_data: The material data to update.
        texture_attr_name: The attribute name for the texture path (e.g., 'diffuse_texture_path').
        color_attr_name: The attribute name for the color (e.g., 'diffuse_color').
    """
    if hasattr(material_effect, property_name):
        effect_element = getattr(material_effect, property_name)
        if isinstance(effect_element, collada.material.Map):
            image = effect_element.sampler.surface.image
            texture_path = (mesh_file_path.parent / image.path) if image.path else None
            setattr(material_data, texture_attr_name, texture_path)
        elif effect_element:
            color = Gf.Vec3f(effect_element[0], effect_element[1], effect_element[2])
            setattr(material_data, color_attr_name, color)


def store_dae_material_data(mesh_file_path: pathlib.Path, _collada: collada.Collada, data: ConversionData):
    """
    Store the material data from the DAE file.
    This is used to temporarily cache material parameters when loading a DAE file.

    Args:
        mesh_file_path: The path to the mesh file.
        _collada: The DAE file.
        data: The conversion data.
    """

    # Check for duplicate material names.
    # If duplicate material names are found, the material ID will be used as the distinguishing identifier.
    material_name_counts = Counter(material.name for material in _collada.materials)
    use_material_id = any(count > 1 for count in material_name_counts.values())

    for material in _collada.materials:
        material_data = MaterialData()
        material_data.mesh_file_path = mesh_file_path

        # If use_material_id is True, the "material name" is used as the material identification name.
        # If use_material_id is False, the "material ID" is used as the material identification name.
        # For the displayName of USD, use material.name.
        material_data.name = material.id if use_material_id else material.name
        material_data.material_name = material.name
        material_data.use_material_id = use_material_id

        # Process the color properties.
        _process_dae_effect_color_property(material.effect, "diffuse", mesh_file_path, material_data, "diffuse_texture_path", "diffuse_color")
        _process_dae_effect_color_property(material.effect, "specular", mesh_file_path, material_data, "specular_texture_path", "specular_color")
        _process_dae_effect_color_property(material.effect, "emission", mesh_file_path, material_data, "emissive_texture_path", "emissive_color")
        _process_dae_effect_color_property(material.effect, "transparent", mesh_file_path, material_data, "opacity_texture_path", "opacity_color")

        # OPAQUE mode ("A_ONE", "RGB_ZERO", None).
        opaque_mode = material.effect.opaque_mode if hasattr(material.effect, "opaque_mode") else None

        # Translucency is achieved by multiplying "transparency" and "transparent".
        _opacity = 1.0
        if (
            opaque_mode is not None
            and hasattr(material.effect, "transparency")
            and material.effect.transparency is not None
            and not isinstance(material.effect.transparency, collada.material.Map)
        ):
            _opacity = material.effect.transparency

        # A_ONE: "transparent" has RGBA, and the Alpha value goes into transparent[3].
        # RGB_ZERO: "Transparent" has RGB values, and the average of these RGB values is used.
        if (
            opaque_mode is not None
            and hasattr(material.effect, "transparent")
            and material.effect.transparent is not None
            and not isinstance(material.effect.transparent, collada.material.Map)
        ):
            transparent = material.effect.transparent
            transparent = transparent[3] if opaque_mode == "A_ONE" else (transparent[0] + transparent[1] + transparent[2]) / 3.0
            _opacity *= transparent

        material_data.opacity = _opacity if opaque_mode == "A_ONE" else 1.0 - _opacity

        data.material_data_list.append(material_data)


def use_material_id(mesh_file_path: pathlib.Path, data: ConversionData) -> bool:
    """
    Check if the material ID should be used for identification of the material prim in USD.

    Args:
        mesh_file_path: The path to the mesh file.
        data: The conversion data.

    Returns:
        True if the material ID is used for identification, False if the material name is used.
    """
    material_data = next((material_data for material_data in data.material_data_list if material_data.mesh_file_path == mesh_file_path), None)
    return material_data.use_material_id if material_data else False


def store_mesh_material_reference(mesh_file_path: pathlib.Path, mesh_safe_name: str, material_name_list: list[str], data: ConversionData):
    """
    Store the per-mesh material reference.
    When a single mesh has a GeomSubset, such as dae, it stores multiple materials.

    Args:
        mesh_file_path: The path to the source file.
        mesh_safe_name: The safe name of the mesh.
        material_name_list: The list of material names.
        data: The conversion data.
    """
    if mesh_file_path not in data.mesh_material_references:
        data.mesh_material_references[mesh_file_path] = {}
    data.mesh_material_references[mesh_file_path][mesh_safe_name] = material_name_list


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

    geom_over = data.content[Tokens.Materials].OverridePrim(geom_prim.GetPath())

    # If the geometry already has a material binding, skip the binding.
    material_binding = UsdShade.MaterialBindingAPI(geom_over)
    if material_binding:
        binding_rel = material_binding.GetDirectBindingRel()
        if len(binding_rel.GetTargets()) > 0:
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
    usdex.core.bindMaterial(geom_over, material_prim)


def _get_material_name_from_prim(prim: Usd.Prim, resolved_file_path: pathlib.Path, data: ConversionData) -> str | None:
    """
    Get the material name from the prim.

    From 'data.mesh_material_references',
    retrieve the corresponding material name using resolved_file_path and prim name as keys.

    If prim is defined by reference and is of type Mesh,
    the prim name may differ from the prim name specified in the Geometry library.
    In that case, it becomes a single mesh, and no child prims exist except for GeomSubset.
    In this case, the prim name is not used in the material search.
    instead, the single material name associated with the prim is retrieved.

    Args:
        prim: The prim.
        resolved_file_path: The resolved file path.
        data: The conversion data.

    Returns:
        The material name if found, otherwise None.
    """
    # Get the material dictionary from the mesh file path.
    material_dict = data.mesh_material_references.get(resolved_file_path, None)
    if not material_dict:
        return None

    prim_name = prim.GetName()
    if prim.IsA(UsdGeom.Mesh) and len(prim.GetAllChildrenNames()) == 0:
        # If there are no child prims, it is a single mesh with no GeomSubset.
        if prim.HasAuthoredReferences():
            # Get the first material name list.
            # In this case, a prim reference exists, and it refers to the single mesh itself.
            # When referencing, the prim_name may not match the name stored in 'data.mesh_material_references',
            # so a single unique material is used.
            material_name_list = next(iter(material_dict.values()))
            return material_name_list[0]
        else:
            if prim_name in material_dict:
                return material_dict[prim_name][0]

    elif prim.IsA(UsdGeom.Subset):
        parent_prim = prim.GetParent()
        if parent_prim.IsA(UsdGeom.Mesh):
            parent_prim_name = parent_prim.GetName()
            all_children_names = parent_prim.GetAllChildrenNames()

            # When the parent prim has a reference, the material_dict contains only one element.
            # When referencing, the prim_name may not match the name stored in 'data.mesh_material_references'.
            # Therefore, the first material name list is used.
            material_name_list = (
                next(iter(material_dict.values())) if parent_prim.HasAuthoredReferences() else material_dict.get(parent_prim_name, None)
            )

            if material_name_list and len(all_children_names) == len(material_name_list):
                index = all_children_names.index(prim_name)
                if index < len(material_name_list):
                    return material_name_list[index]

    return None


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
        if prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Subset):
            material_name = _get_material_name_from_prim(prim, resolved_file_path, data)
            if material_name:
                bind_material(prim, resolved_file_path, material_name, data)
