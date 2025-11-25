# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import usdex.core
from pxr import Usd

from .urdf_parser.parser import URDFParser

__all__ = ["ConversionMeshData"]


class ConversionMeshData:
    def __init__(self):
        # A list of mesh file paths and scale values.
        self.meshes: list[tuple[str, tuple[float, float, float]]] = []

        # Get the mesh names.
        self.mesh_names: list[str] = []

        # Get the mesh safe names.
        self.safe_names: list[str] = []

    def store_mesh_data(self, geo_scope: Usd.Prim, name_cache: usdex.core.NameCache, urdf_parser: URDFParser):
        """
        Store the mesh data.

        Args:
            geo_scope: The scope of the geometry.
            name_cache: The name cache.
            urdf_parser: The URDF parser.
        """
        # A list of mesh file paths and scale values.
        self.meshes = urdf_parser.get_meshes()

        # Get the mesh names.
        self.mesh_names = [pathlib.Path(mesh[0]).stem for mesh in self.meshes]

        # Get the mesh safe names.
        self.safe_names = name_cache.getPrimNames(geo_scope, self.mesh_names)

    def get_safe_name(self, filename: str, scale: tuple[float, float, float]) -> str:
        """
        Get the safe_name using the filename and scale obtained from the URDF.

        Args:
            filename: The filename of the mesh.
            scale: The scale of the mesh.

        Returns:
            The safe_name if found, otherwise None.
        """
        for mesh, safe_name in zip(self.meshes, self.safe_names):
            if mesh[0] == filename and mesh[1] == scale:
                return safe_name
        return None  # pragma: no cover
