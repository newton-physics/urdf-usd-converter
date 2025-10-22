# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
import usdex.core

from .data import ConversionData, Tokens

__all__ = ["convert_materials"]


def convert_materials(data: ConversionData):
    materials = data.urdf_parser.get_materials()
    if not len(materials):
        return

    data.libraries[Tokens.Materials] = usdex.core.addAssetLibrary(data.content[Tokens.Contents], Tokens.Materials, format="usdc")
    data.references[Tokens.Materials] = {}

    # TODO: Implement
