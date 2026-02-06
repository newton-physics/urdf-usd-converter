# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib
from dataclasses import dataclass

import usdex.core
from pxr import Usd

from .link_hierarchy import LinkHierarchy
from .material_data import MaterialData
from .mesh_cache import MeshCache
from .urdf_parser.parser import URDFParser
from .urdf_parser.undefined_data import UndefinedData

__all__ = ["ConversionData", "Tokens"]


class Tokens:
    Asset = usdex.core.getAssetToken()
    Library = usdex.core.getLibraryToken()
    Contents = usdex.core.getContentsToken()
    Geometry = usdex.core.getGeometryToken()
    Materials = usdex.core.getMaterialsToken()
    Textures = usdex.core.getTexturesToken()
    Payload = usdex.core.getPayloadToken()
    Physics = usdex.core.getPhysicsToken()


@dataclass
class ConversionData:
    urdf_parser: URDFParser
    content: dict[Tokens, Usd.Stage]
    libraries: dict[Tokens, Usd.Stage]
    references: dict[Tokens, dict[str, Usd.Prim]]
    name_cache: usdex.core.NameCache
    scene: bool
    comment: str
    link_hierarchy: LinkHierarchy
    mesh_cache: MeshCache
    ros_packages: list[dict[str, str]]
    resolved_file_paths: dict[str, pathlib.Path]  # [mesh_file_name, resolved_file_path]
    material_data_list: list[MaterialData]  # Store all material parameters.
    mesh_material_references: dict[pathlib.Path, dict[str, list[str]]]  # [mesh_file_path, [mesh_safe_name, material_name_list]]
    undefined_elements: list[UndefinedData]  # Store all undefined elements.
