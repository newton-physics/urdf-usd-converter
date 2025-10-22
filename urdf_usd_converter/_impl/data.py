# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
from dataclasses import dataclass

import usdex.core
from pxr import Usd

from .urdf_parser.parser import URDFParser

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
