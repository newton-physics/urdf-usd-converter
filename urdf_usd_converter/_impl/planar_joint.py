# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
import math

import numpy as np
import usdex.core
from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdPhysics

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

    if body0 and not joint.GetBody0Rel().SetTargets([body0.GetPath()]):
        Tf.Error(f'Unable to set body0( "{body0.GetPath()}" ) for PhysicsPlanarJoint at "{path}"')
        return None

    if body1 and not joint.GetBody1Rel().SetTargets([body1.GetPath()]):
        Tf.Error(f'Unable to set body1( "{body1.GetPath()}" ) for PhysicsPlanarJoint at "{path}"')
        return None

    _orientation = joint_frame.orientation

    # Get the axis alignment and orientation for the given axis.
    axis_token, _orientation = _get_axis_alignment(axis)

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

    # Get the local to world coordinate transformation matrix for body0 and body1.
    xform_cache = UsdGeom.XformCache()
    body0_transform = xform_cache.GetLocalToWorldTransform(body0) if body0 else Gf.Matrix4d(1.0)
    body1_transform = xform_cache.GetLocalToWorldTransform(body1) if body1 else Gf.Matrix4d(1.0)

    if body0:
        # Compute the local position and rotation of body0.
        local_pos, local_rot = _compute_local_transform(
            body0_transform, body1_transform, usdex.core.JointFrame.Space.Body0, joint_frame.space, joint_frame.position, _orientation
        )
        joint.GetLocalPos0Attr().Set(Gf.Vec3f(local_pos))
        joint.GetLocalRot0Attr().Set(Gf.Quatf(local_rot))

    if body1:
        # Compute the local position and rotation of body1.
        local_pos, local_rot = _compute_local_transform(
            body1_transform, body0_transform, usdex.core.JointFrame.Space.Body1, joint_frame.space, joint_frame.position, _orientation
        )
        joint.GetLocalPos1Attr().Set(Gf.Vec3f(local_pos))
        joint.GetLocalRot1Attr().Set(Gf.Quatf(local_rot))

    return joint


def _align_vector_to_x_axis(axis: Gf.Vec3f) -> Gf.Quatd:
    """
    Calculates the rotation of a vector along the x-axis.
    """
    epsilon = np.finfo(np.float32).eps

    # Calculate the rotation axis (cross product of XAxis and axis)
    rotation_axis = Gf.Cross(Gf.Vec3f(1.0, 0.0, 0.0), axis)
    rotation_axis_norm = rotation_axis.GetNormalized()
    if rotation_axis_norm.GetLength() < epsilon:
        return Gf.Quatd.GetIdentity()

    # Calculate the angle (dot product of axis and XAxis)
    dot_product = Gf.Dot(axis, Gf.Vec3f.XAxis())

    # Clip to avoid floating point errors
    angle = math.acos(min(max(dot_product, -1.0), 1.0))

    # Construct the quaternion (wxyz order)
    w = math.cos(angle / 2.0)
    x = rotation_axis_norm[0] * math.sin(angle / 2.0)
    y = rotation_axis_norm[1] * math.sin(angle / 2.0)
    z = rotation_axis_norm[2] * math.sin(angle / 2.0)

    return Gf.Quatd(w, x, y, z)


# void getAxisAlignment(const GfVec3f& axis, TfToken& axisToken, GfQuatd& orientation)
def _get_axis_alignment(axis: Gf.Vec3f) -> tuple[str, Gf.Quatd]:
    """
    Get the axis alignment and orientation for the given axis.
    """
    epsilon = np.finfo(np.float32).eps
    _axis = axis.GetNormalized()
    axis_token = UsdPhysics.Tokens.x
    orientation = Gf.Quatd.GetIdentity()

    if _axis.GetLength() < epsilon:
        return axis_token, orientation

    if abs(_axis[0] - 1.0) < epsilon:
        # When _axis is (1, 0, 0).
        axis_token = UsdPhysics.Tokens.x
    elif abs(_axis[1] - 1.0) < epsilon:
        # When _axis is (0, 1, 0).
        axis_token = UsdPhysics.Tokens.y
    elif abs(_axis[2] - 1.0) < epsilon:
        # When _axis is (0, 0, 1).
        axis_token = UsdPhysics.Tokens.z
    elif abs(_axis[0] + 1.0) < epsilon:
        # When _axis is (-1, 0, 0).
        axis_token = UsdPhysics.Tokens.x
        axis_to_x = Gf.Quatd(_axis[1], _axis[2], _axis[0], 0.0)
        orientation = orientation * axis_to_x
    elif abs(_axis[1] + 1.0) < epsilon:
        # When _axis is (0, -1, 0).
        axis_token = UsdPhysics.Tokens.y
        axis_to_y = Gf.Quatd(_axis[0], _axis[1], _axis[2], 0.0)
        orientation = orientation * axis_to_y
    elif abs(_axis[2] + 1.0) < epsilon:
        # When _axis is (0, 0, -1).
        axis_token = UsdPhysics.Tokens.z
        axis_to_z = Gf.Quatd(_axis[1], _axis[2], _axis[0], 0.0)
        orientation = orientation * axis_to_z
    else:
        # If neither XYZ applies, rotation is performed around _axis.
        axis_token = UsdPhysics.Tokens.x
        rotation = _align_vector_to_x_axis(_axis)
        orientation = orientation * rotation

    return axis_token, orientation


def _compute_local_transform(
    target_body_transform: Gf.Matrix4d,
    other_body_transform: Gf.Matrix4d,
    target_space: usdex.core.JointFrame.Space,
    frame_space: usdex.core.JointFrame.Space,
    position: Gf.Vec3d,
    orientation: Gf.Quatd,
) -> tuple[Gf.Vec3d, Gf.Quatd]:
    """
    Compute the local transform of the joint.
    This function calculates the local position and rotation (orientation) of body0 and body1, which are the parameters of the physics joint.
    Transforms the 'position' and 'orientation' given in the coordinate system of 'frameSpace' into local coordinates of 'targetSpace'
    (body0 or body1).
    """
    world_pos = position
    world_rot = orientation

    # If the transformation on body0 is for frameSpace = body0, it will be returned as local coordinates.
    # If the transformation on body1 is for frameSpace = body1, it will be returned as local coordinates.
    if (frame_space == usdex.core.JointFrame.Space.Body0 and target_space == usdex.core.JointFrame.Space.Body0) or (
        frame_space == usdex.core.JointFrame.Space.Body1 and target_space == usdex.core.JointFrame.Space.Body1
    ):
        return position, orientation

    # When transforming on body1, if frameSpace is body0, convert position and rotation to world coordinates.
    elif (frame_space == usdex.core.JointFrame.Space.Body0 and target_space == usdex.core.JointFrame.Space.Body1) or (
        frame_space == usdex.core.JointFrame.Space.Body1 and target_space == usdex.core.JointFrame.Space.Body0
    ):
        world_pos = other_body_transform.Transform(position)
        world_rot = other_body_transform.RemoveScaleShear().ExtractRotation().GetQuat() * orientation
    # Otherwise, worldPos and worldRot contain the position and rotation in world coordinates, respectively.

    # The world transformation matrix for body0 or body1 is in 'targetBodyTransform'.
    # This matrix is used to convert to local coordinate position and rotation by multiplying with the inverse matrix.
    # USD physics does not allow unequal scales and shear components to be introduced in joint localRot.
    # Therefore, we first remove the scale and shear from the matrix.
    local_pos = target_body_transform.GetInverse().Transform(Gf.Vec3d(world_pos))
    local_rot = target_body_transform.RemoveScaleShear().ExtractRotation().GetInverse().GetQuat() * world_rot
    return local_pos, local_rot
