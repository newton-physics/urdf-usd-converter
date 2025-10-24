# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import math
from typing import Any

import usdex.core
from pxr import Gf, Sdf, Usd, UsdGeom

from urdf_usd_converter._impl.urdf_parser.elements import ElementJoint, ElementLink

from .._version import __version__

__all__ = [
    "float3_to_quatf",
    "float3_to_vec3d",
    "get_authoring_metadata",
    "radians_to_degrees",
    "set_custom_attribute",
    "set_transform",
]


def get_authoring_metadata() -> str:
    return f"URDF USD Converter v{__version__}"


def radians_to_degrees(radians: float) -> float:
    return radians * 180.0 / math.pi


def float3_to_vec3d(xyz: tuple[float, float, float]) -> Gf.Vec3d:
    return Gf.Vec3d(xyz[0], xyz[1], xyz[2])


def float3_to_quatf(rpy: tuple[float, float, float]) -> Gf.Quatf:
    """
    Convert a tuple of roll, pitch, yaw angles to a Gf.Quatf.
    The roll, pitch, yaw angles are in radians.
    USD converts this to degrees.
    """
    rotation = Gf.Rotation(Gf.Vec3d(1, 0, 0), radians_to_degrees(rpy[0]))  # X-axis
    rotation = rotation * Gf.Rotation(Gf.Vec3d(0, 1, 0), radians_to_degrees(rpy[1]))  # Y-axis
    rotation = rotation * Gf.Rotation(Gf.Vec3d(0, 0, 1), radians_to_degrees(rpy[2]))  # Z-axis
    return Gf.Quatf(rotation.GetQuat())


def set_transform(prim: UsdGeom.Xformable, element: ElementJoint | ElementLink, collision: bool = False) -> None:
    # get the current transform (including any inherited via references)
    pos, pivot, orient, scale = usdex.core.getLocalTransformComponentsQuat(prim)
    current_transform = Gf.Transform(translation=pos, rotation=Gf.Rotation(orient), scale=Gf.Vec3d(scale), pivotPosition=pivot)

    position = Gf.Vec3d(0, 0, 0)
    orientation = Gf.Quatf.GetIdentity()

    if isinstance(element, ElementLink):
        visual_collision = element.visual if not collision else element.collision
        if visual_collision and visual_collision.geometry:
            geometry = visual_collision.geometry.geometry
            if geometry and visual_collision.origin:
                position = float3_to_vec3d(visual_collision.origin.get_with_default("xyz"))
                orientation = float3_to_quatf(visual_collision.origin.get_with_default("rpy"))
    elif isinstance(element, ElementJoint) and element.origin:
        position = float3_to_vec3d(element.origin.get_with_default("xyz"))
        orientation = float3_to_quatf(element.origin.get_with_default("rpy"))

    local_transform: Gf.Transform = Gf.Transform(translation=position, rotation=Gf.Rotation(orientation))
    final_transform: Gf.Transform = multiply_transforms_preserve_scale(local_transform, current_transform)

    # extract the translation, orientation, and scale so we can set them as components
    pos = final_transform.GetTranslation()
    orient = Gf.Quatf(final_transform.GetRotation().GetQuat())
    scale = Gf.Vec3f(final_transform.GetScale())

    usdex.core.setLocalTransform(prim, pos, orient, scale)


def multiply_transforms_preserve_scale(transform1: Gf.Transform, transform2: Gf.Transform) -> Gf.Transform:
    """
    Multiply two Gf.Transform objects while preserving non-uniform scales.

    This function uses matrix multiplication but then carefully decomposes the result
    to extract and preserve the non-uniform scale components that would otherwise
    be lost or corrupted in standard matrix decomposition.

    Args:
        transform1: The first transform (applied second in the composition)
        transform2: The second transform (applied first in the composition)

    Returns:
        A new Gf.Transform representing transform1 * transform2 with preserved scales
    """
    # Extract scale components before matrix multiplication
    s1 = transform1.GetScale()
    s2 = transform2.GetScale()

    # Create transforms without scale for matrix multiplication
    transform1_no_scale = Gf.Transform()
    transform1_no_scale.SetTranslation(transform1.GetTranslation())
    transform1_no_scale.SetRotation(transform1.GetRotation())

    transform2_no_scale = Gf.Transform()
    transform2_no_scale.SetTranslation(transform2.GetTranslation())
    transform2_no_scale.SetRotation(transform2.GetRotation())

    # Multiply the transforms without scale using standard matrix multiplication
    result_no_scale = transform1_no_scale * transform2_no_scale

    # Compute the combined scale (component-wise multiplication)
    combined_scale = Gf.CompMult(s1, s2)

    # Create the final result with the preserved scale
    result = Gf.Transform()
    result.SetTranslation(result_no_scale.GetTranslation())
    result.SetRotation(result_no_scale.GetRotation())
    result.SetScale(combined_scale)

    return result


def set_custom_attribute(prim: Usd.Prim, name: str, type_name: Sdf.ValueTypeNames, value: Any):
    """
    Set a custom attribute on a prim.

    Args:
        prim: The prim to set the custom attribute on.
        name: The name of the custom attribute.
        type_name: The type of the custom attribute.
        value: The value of the custom attribute.
    """
    if value is None:
        return

    attr: Usd.Attribute = prim.CreateAttribute(name, type_name, custom=True)
    attr.Set(value)
