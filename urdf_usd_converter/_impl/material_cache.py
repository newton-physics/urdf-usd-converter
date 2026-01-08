# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import tinyobjloader
from pxr import Gf, Tf, UsdShade

from .data import ConversionData, Tokens
from .ros_package import resolve_ros_package_paths

__all__ = ["MaterialCache", "MaterialData"]


class MaterialData:
    """
    Temporary data when storing material.
    """

    def __init__(self):
        # The path to the mesh file. For global materials used in URDF, None is entered.
        self.mesh_file_path: pathlib.Path | None = None

        # The name of the material.
        self.name: str | None = None

        # The safe name of the material.
        # This is a unique name that does not overlap with other material names.
        self.safe_name: str | None = None

        # The material in USD.
        self.material: UsdShade.Material | None = None

        # The material properties.
        self.diffuse_color: Gf.Vec3f = Gf.Vec3f(1.0, 1.0, 1.0)
        self.opacity: float = 1.0
        self.roughness: float = 0.5
        self.metallic: float = 0.0
        self.diffuse_texture_path: pathlib.Path | None = None
        self.normal_texture_path: pathlib.Path | None = None
        self.roughness_texture_path: pathlib.Path | None = None
        self.metallic_texture_path: pathlib.Path | None = None


class MaterialCache:
    def __init__(self, data: ConversionData):
        # A list of material data.
        self.material_data_list: list[MaterialData] = []

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
        material_names = [material_data.name for material_data in self.material_data_list]
        safe_names = data.name_cache.getPrimNames(materials_scope, material_names)

        for material_data, safe_name in zip(self.material_data_list, safe_names):
            material_data.safe_name = safe_name

    def _store_materials(self, data: ConversionData):
        """
        Get the material data from the URDF file and the OBJ/DAE files.

        Args:
            data: The conversion data.

        Returns:
            A list of material data.
        """
        self.material_data_list = []
        self.material_data_list.extend(self._get_urdf_material_data_list(data))
        self.material_data_list.extend(self._get_mesh_material_data_list(data))

        # Get a dictionary of resolved texture paths and unique names.
        # It stores all the texture file paths referenced by urdf materials and each mesh.
        self.texture_paths = self._get_material_texture_paths()

    def _get_material_texture_paths(self) -> dict[pathlib.Path, str]:
        """
        Create a dictionary of resolved texture paths and unique names.
        These include all global materials and the textures of materials referenced by meshes.

        Returns:
            A dictionary of texture paths and unique names.
        """
        # Get the texture paths from the materials.
        existing_texture_paths: list[pathlib.Path] = []
        for material_data in self.material_data_list:
            texture_paths = [
                material_data.diffuse_texture_path,
                material_data.normal_texture_path,
                material_data.roughness_texture_path,
                material_data.metallic_texture_path,
            ]
            for texture_path in texture_paths:
                if texture_path and texture_path not in existing_texture_paths:
                    if texture_path.exists():
                        existing_texture_paths.append(texture_path)
                    else:
                        Tf.Warn(f"Texture file not found: {texture_path}")

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

    def _get_mesh_material_data_list(self, data: ConversionData) -> list[MaterialData]:
        """
        Get the material data from the OBJ/DAE files.

        Args:
            data: The conversion data.

        Returns:
            A list of material data.
        """
        material_data_list = []

        # Get the material names from the meshes.
        meshes = data.urdf_parser.get_meshes()
        for mesh in meshes:
            # mesh[0] is the path to the mesh file.

            # Resolve the ROS package paths.
            # If the path is not a ROS package, it will return the original path.
            # It also converts the path to a relative path based on the urdf file.
            resolved_path = resolve_ros_package_paths(mesh[0], data)

            if resolved_path.exists() and resolved_path.suffix.lower() == ".obj":
                material_data_list.extend(self._get_obj_material_data_list(resolved_path))

            # TODO: Add DAE file support.

        return material_data_list

    def _get_obj_material_data_list(self, mesh_file_path: pathlib.Path) -> list[MaterialData]:
        """
        Get the material data from the OBJ file.

        Args:
            mesh_file_path: The path to the file.

        Returns:
            A list of material data.
        """
        material_data_list: list[MaterialData] = []
        reader = tinyobjloader.ObjReader()
        if not reader.ParseFromFile(str(mesh_file_path)):
            return material_data_list

        materials = reader.GetMaterials()
        for material in materials:
            material_data = MaterialData()
            material_data.mesh_file_path = mesh_file_path
            material_data.name = material.name
            material_data.diffuse_color = Gf.Vec3f(material.diffuse[0], material.diffuse[1], material.diffuse[2])
            material_data.opacity = material.dissolve

            # The following is the extended specification of obj.
            material_data.roughness = material.roughness if material.roughness else 0.5
            material_data.metallic = material.metallic if material.metallic else 0.0

            material_data.diffuse_texture_path = (mesh_file_path.parent / material.diffuse_texname) if material.diffuse_texname else None
            material_data.normal_texture_path = (mesh_file_path.parent / material.normal_texname) if material.normal_texname else None
            material_data.roughness_texture_path = (mesh_file_path.parent / material.roughness_texname) if material.roughness_texname else None
            material_data.metallic_texture_path = (mesh_file_path.parent / material.metallic_texname) if material.metallic_texname else None

            material_data_list.append(material_data)

        return material_data_list
