# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

from pxr import Gf, Tf

from .data import ConversionData, Tokens
from .material_data import MaterialData
from .ros_package import resolve_ros_package_paths

__all__ = ["MaterialCache"]


class MaterialCache:
    def __init__(self, data: ConversionData):
        # A dictionary of texture paths and unique names.
        self.texture_paths: dict[pathlib.Path, str] = {}

        # Store the material data.
        self._store_materials(data)

    def store_safe_names(self, data: ConversionData):
        """
        Store the safe names of the material data list.

        Args:
            material_data_list: The list of material data.
            data: The conversion data.
        """
        materials_scope = data.libraries[Tokens.Materials].GetDefaultPrim()
        material_names = [material_data.name for material_data in data.material_data_list]
        safe_names = data.name_cache.getPrimNames(materials_scope, material_names)

        for material_data, safe_name in zip(data.material_data_list, safe_names):
            material_data.safe_name = safe_name

    def _store_materials(self, data: ConversionData):
        """
        Get the material data from the URDF file and the OBJ/DAE files.
        Material data is stored in `data.material_data_list`.
        If the material is stored in an obj or dae file,
        the material data will already be stored when this method is called.
        Here, global materials in URDF are added, and a list of texture paths is created.

        Args:
            data: The conversion data.
        """
        # Get the material data from the URDF file.
        data.material_data_list.extend(self._get_urdf_material_data_list(data))

        # Get a dictionary of resolved texture paths and unique names.
        # It stores all the texture file paths referenced by urdf materials and each mesh.
        self.texture_paths = self._get_material_texture_paths(data)

    def _get_material_texture_paths(self, data: ConversionData) -> dict[pathlib.Path, str]:
        """
        Create a dictionary of resolved texture paths and unique names.
        These include all global materials and the textures of materials referenced by meshes.

        Args:
            data: The conversion data.

        Returns:
            A dictionary of texture paths and unique names.
        """
        # Get the texture paths from the materials.
        texture_paths_list: list[pathlib.Path] = []
        for material_data in data.material_data_list:
            texture_paths = [
                material_data.diffuse_texture_path,
                material_data.specular_texture_path,
                material_data.normal_texture_path,
                material_data.roughness_texture_path,
                material_data.metallic_texture_path,
                material_data.opacity_texture_path,
                material_data.emissive_texture_path,
            ]
            for texture_path in texture_paths:
                if texture_path and texture_path not in texture_paths_list:
                    texture_paths_list.append(texture_path)
                    if not texture_path.exists():
                        Tf.Warn(f"Texture file not found: {texture_path}")

        # Create a list of texture filenames.
        names = [texture_path.name for texture_path in texture_paths_list]

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

        texture_paths = dict(zip(texture_paths_list, unique_file_names))

        return texture_paths

    def _get_urdf_material_data_list(self, data: ConversionData) -> list[MaterialData]:
        """
        Get the material data from the URDF file (Global Materials).

        Args:
            data: The conversion data.

        Returns:
            A list of material data.
        """
        material_data_list = []

        materials = data.urdf_parser.get_materials()
        for material in materials:
            material_data = MaterialData()
            material_data.name = material[0]
            material_data.diffuse_color = Gf.Vec3f(*material[1][:3])
            material_data.opacity = material[1][3]

            # material[2] is the path to the texture file.
            if material[2]:
                # Resolve the ROS package paths.
                # If the path is not a ROS package, it will return the original path.
                # It also converts the path to a relative path based on the urdf file.
                material_data.diffuse_texture_path = resolve_ros_package_paths(material[2], data)

            material_data_list.append(material_data)

        return material_data_list
