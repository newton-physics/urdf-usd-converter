# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
import numpy as np
import usdex.core
from pxr import Gf, Sdf, Tf, Usd, UsdPhysics

__all__ = ["define_physics_planar_joint"]


def define_physics_planar_joint(
    parent: Usd.Prim, name: str, body0: Usd.Prim, body1: Usd.Prim, joint_frame: usdex.core.JointFrame, axis: Gf.Vec3f
) -> UsdPhysics.Joint:
    """
    Defines functionality equivalent to URDF Planar Joint.
    """
    stage = parent.GetStage()
    path = parent.GetPath().AppendChild(name)

    joint = UsdPhysics.Joint.Define(stage, path)
    if not joint:
        Tf.Error(f'Unable to define UsdPhysics.Joint at "{path}"')
        return None

    prim = joint.GetPrim()
    prim.SetSpecifier(Sdf.SpecifierDef)
    prim.SetTypeName(prim.GetTypeName())

    usdex.core.connectPhysicsJoint(joint, body0, body1, joint_frame, axis)

    # Get the axis token for the given axis.
    axis_token = _get_axis_token(axis)

    if axis_token == UsdPhysics.Tokens.x:
        # Constrain in the X-axis direction.
        limit_api_x = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), UsdPhysics.Tokens.transX)
        limit_api_x.GetLowAttr().Set(0.0)
        limit_api_x.GetHighAttr().Set(0.0)

        # Rotation is only permitted around the X axis (Constrain rotation on the Y and Z axes).
        limit_api_rotation_y = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), UsdPhysics.Tokens.rotY)
        limit_api_rotation_y.GetLowAttr().Set(0.0)
        limit_api_rotation_y.GetHighAttr().Set(0.0)
        limit_api_rotation_z = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), UsdPhysics.Tokens.rotZ)
        limit_api_rotation_z.GetLowAttr().Set(0.0)
        limit_api_rotation_z.GetHighAttr().Set(0.0)
    elif axis_token == UsdPhysics.Tokens.y:
        # Constrain in the Y-axis direction.
        limit_api_y = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), UsdPhysics.Tokens.transY)
        limit_api_y.GetLowAttr().Set(0.0)
        limit_api_y.GetHighAttr().Set(0.0)

        # Rotation is only permitted around the Y axis (Constrain rotation on the X and Z axes).
        limit_api_rotation_x = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), UsdPhysics.Tokens.rotX)
        limit_api_rotation_x.GetLowAttr().Set(0.0)
        limit_api_rotation_x.GetHighAttr().Set(0.0)
        limit_api_rotation_z = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), UsdPhysics.Tokens.rotZ)
        limit_api_rotation_z.GetLowAttr().Set(0.0)
        limit_api_rotation_z.GetHighAttr().Set(0.0)
    elif axis_token == UsdPhysics.Tokens.z:
        # Constrain in the Z-axis direction.
        limit_api_z = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), UsdPhysics.Tokens.transZ)
        limit_api_z.GetLowAttr().Set(0.0)
        limit_api_z.GetHighAttr().Set(0.0)

        # Rotation is only permitted around the Z axis (Constrain rotation on the X and Y axes).
        limit_api_rotation_x = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), UsdPhysics.Tokens.rotX)
        limit_api_rotation_x.GetLowAttr().Set(0.0)
        limit_api_rotation_x.GetHighAttr().Set(0.0)
        limit_api_rotation_y = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), UsdPhysics.Tokens.rotY)
        limit_api_rotation_y.GetLowAttr().Set(0.0)
        limit_api_rotation_y.GetHighAttr().Set(0.0)

    return joint


def _get_axis_token(axis: Gf.Vec3f) -> str:
    """
    Get the axis token for the given axis.
    """
    epsilon = np.finfo(np.float32).eps
    _axis = axis.GetNormalized()
    axis_token = UsdPhysics.Tokens.x

    if abs(abs(_axis[0]) - 1.0) < epsilon:
        # When _axis is (1, 0, 0) or (-1, 0, 0).
        axis_token = UsdPhysics.Tokens.x
    elif abs(abs(_axis[1]) - 1.0) < epsilon:
        # When _axis is (0, 1, 0) or (0, -1, 0).
        axis_token = UsdPhysics.Tokens.y
    elif abs(abs(_axis[2]) - 1.0) < epsilon:
        # When _axis is (0, 0, 1) or (0, 0, -1).
        axis_token = UsdPhysics.Tokens.z

    return axis_token
