# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import math
import pathlib
from typing import Any

import usdex.core
from pxr import Gf, Tf, Usd, UsdGeom

from .._version import __version__
from .urdf_parser.elements import (
    ElementCollision,
    ElementJoint,
    ElementMesh,
    ElementVisual,
)

__all__ = [
    "float3_to_quatf",
    "get_authoring_metadata",
    "get_geometry_name",
    "set_schema_attribute",
    "set_transform",
]


def get_authoring_metadata() -> str:
    return f"URDF USD Converter v{__version__}"


def float3_to_quatf(rpy: tuple[float, float, float]) -> Gf.Quatf:
    """
    Convert a tuple of roll, pitch, yaw angles to a Gf.Quatf.
    The roll, pitch, yaw angles are in radians.
    USD converts this to degrees.
    """
    rotation = (
        Gf.Rotation(Gf.Vec3d(1, 0, 0), math.degrees(rpy[0]))
        * Gf.Rotation(Gf.Vec3d(0, 1, 0), math.degrees(rpy[1]))
        * Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(rpy[2]))
    )
    return Gf.Quatf(rotation.GetQuat())


def get_geometry_name(element: ElementVisual | ElementCollision) -> str:
    """
    Get the name of the geometry.
    If element.name exists, it is the element name.
    For meshes, the name is taken from the file name.
    For other geometries, the name is taken from the element type.
    """
    if element.name:
        return element.name

    if element.geometry:
        geometry = element.geometry.shape
        if geometry and isinstance(geometry, ElementMesh):
            return pathlib.Path(geometry.filename).stem

    return element.geometry.shape.tag


def set_transform(prim: UsdGeom.Xformable, element: ElementJoint | ElementVisual | ElementCollision) -> None:
    # get the current transform (including any inherited via references)
    pos, pivot, orient, scale = usdex.core.getLocalTransformComponentsQuat(prim)
    current_transform = Gf.Transform(translation=pos, rotation=Gf.Rotation(orient), scale=Gf.Vec3d(scale), pivotPosition=pivot)

    position = Gf.Vec3d(0, 0, 0)
    orientation = Gf.Quatf.GetIdentity()

    if element.origin:
        position = Gf.Vec3d(element.origin.get_with_default("xyz"))
        orientation = float3_to_quatf(element.origin.get_with_default("rpy"))

    local_transform: Gf.Transform = Gf.Transform(translation=position, rotation=Gf.Rotation(orientation))
    final_transform: Gf.Transform = multiply_transforms_preserve_scale(current_transform, local_transform)

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


def set_schema_attribute(prim: Usd.Prim, name: str, value: Any):
    attr: Usd.Attribute = prim.GetAttribute(name)
    if not attr.IsValid():
        Tf.RaiseCodingError(f'Attribute "{name}" is not valid for prim <{prim.GetPath()}> with schemas {prim.GetAppliedSchemas()}')
    # Only set the value if it is different from the schema default value
    default = attr.Get()
    if default is None or value != default:
        attr.Set(value)
