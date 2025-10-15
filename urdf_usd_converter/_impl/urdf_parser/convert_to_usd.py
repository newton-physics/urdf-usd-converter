# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from pxr import Gf

__all__ = [
    "convert_color",
    "convert_vec3d",
    "convert_vec4d",
]


def convert_vec3d(value: str) -> Gf.Vec3d:
    values = value.split(" ")
    if len(values) != 3:
        raise ValueError(f"Invalid value: {value}")
    return Gf.Vec3d(float(values[0]), float(values[1]), float(values[2]))


def convert_vec4d(value: str) -> Gf.Vec4d:
    values = value.split(" ")
    if len(values) != 4:
        raise ValueError(f"Invalid value: {value}")
    return Gf.Vec4d(float(values[0]), float(values[1]), float(values[2]), float(values[3]))


def convert_color(color: Gf.Vec4d) -> tuple[Gf.Vec3f, float]:
    return Gf.Vec3f(color[0], color[1], color[2]), color[3]
