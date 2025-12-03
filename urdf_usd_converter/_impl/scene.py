# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import usdex.core
from pxr import Usd, UsdPhysics

from .data import ConversionData, Tokens

__all__ = ["convert_scene"]


def convert_scene(data: ConversionData):
    asset_stage: Usd.Stage = data.content[Tokens.Asset]
    content_stage: Usd.Stage = data.content[Tokens.Contents]
    physics_stage: Usd.Stage = data.content[Tokens.Physics]

    # ensure the name is valid across all layers
    safe_name = data.name_cache.getPrimName(asset_stage.GetPseudoRoot(), "PhysicsScene")

    # author the scene in the physics layer
    UsdPhysics.Scene.Define(physics_stage, asset_stage.GetPseudoRoot().GetPath().AppendChild(safe_name))

    # reference the scene in the asset layer, but from the content layer
    content_scene: Usd.Prim = content_stage.GetPseudoRoot().GetChild(safe_name)
    usdex.core.definePayload(asset_stage.GetPseudoRoot(), content_scene, safe_name)
