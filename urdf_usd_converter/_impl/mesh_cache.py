# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import usdex.core
from pxr import Usd

from .urdf_parser.parser import URDFParser

__all__ = ["MeshCache"]


class MeshCache:
    def __init__(self):
        # Get the mesh names.
        self.mesh_names: dict = {}

    def store_mesh_cache(self, geo_scope: Usd.Prim, name_cache: usdex.core.NameCache, urdf_parser: URDFParser):
        """
        Store the mesh data.

        Args:
            geo_scope: The scope of the geometry.
            name_cache: The name cache.
            urdf_parser: The URDF parser.
        """
        # A list of mesh file paths and scale values.
        meshes = urdf_parser.get_meshes()

        # Store the name and safe name using the mesh path as the key.
        for mesh in meshes:
            if mesh[0] and mesh[0] not in self.mesh_names:
                name = pathlib.Path(mesh[0]).stem
                safe_name = name_cache.getPrimName(geo_scope, name)
                self.mesh_names[mesh[0]] = {"name": name, "safe_name": safe_name}

    def get_mesh_names(self) -> dict:
        return self.mesh_names

    def get_name_from_safe_name(self, safe_name: str) -> str:
        """
        Get the name using the safe_name.

        Args:
            safe_name: The safe_name of the mesh.

        Returns:
            The name if found, otherwise None.
        """
        for data in self.mesh_names.values():
            if data["safe_name"] == safe_name:
                return data["name"]
        return None

    def get_safe_name(self, filename: str) -> str:
        """
        Get the safe_name using the filename obtained from the URDF.

        Args:
            filename: The filename of the mesh.

        Returns:
            The safe_name if found, otherwise None.
        """
        if filename in self.mesh_names:
            return self.mesh_names[filename]["safe_name"]

        return None  # pragma: no cover
