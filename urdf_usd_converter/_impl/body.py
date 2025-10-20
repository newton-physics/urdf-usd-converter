# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
from pxr import Usd, UsdGeom

from .data import ConversionData, Tokens

__all__ = ["convert_bodies"]


def convert_bodies(data: ConversionData):
    geo_scope = data.content[Tokens.Geometry].GetDefaultPrim().GetChild(Tokens.Geometry).GetPrim()
    convert_body(parent=geo_scope, name=data.urdf_parser.get_robot_name(), data=data)


def convert_body(parent: Usd.Prim, name: str, data: ConversionData) -> UsdGeom.Xform:
    # TODO: Implement
    pass
