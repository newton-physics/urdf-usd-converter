# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0

import math

import numpy as np
import usdex.core
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from .data import ConversionData, Tokens
from .geometry import convert_geometry
from .planar_joint import define_physics_planar_joint
from .urdf_parser.elements import (
    ElementCollision,
    ElementInertia,
    ElementLink,
    ElementVisual,
)
from .utils import (
    float3_to_quatf,
    get_geometry_name,
    set_transform,
)

__all__ = ["convert_links"]


def convert_links(data: ConversionData):
    geo_scope = data.content[Tokens.Geometry].GetDefaultPrim().GetChild(Tokens.Geometry).GetPrim()
    root_link = data.link_hierarchy.get_root_link()

    # Creating a Link Hierarchy.
    convert_link(parent=geo_scope, link=root_link, data=data)

    # Create Physics joints
    physics_scope = data.content[Tokens.Physics].GetDefaultPrim().GetChild(Tokens.Physics).GetPrim()
    physics_joints(parent=physics_scope, link=root_link, data=data)


def convert_link(parent: Usd.Prim, link: ElementLink, data: ConversionData) -> UsdGeom.Xform:
    link_safe_name = data.name_cache.getPrimName(parent, link.name)
    link_xform = usdex.core.defineXform(parent, link_safe_name)
    link_prim = link_xform.GetPrim()
    if link.name != link_safe_name:
        usdex.core.setDisplayName(link_prim, link.name)

    data.references[Tokens.Physics][link.name] = link_prim

    # Apply RigidBodyAPI to a link.
    prim_over = data.content[Tokens.Physics].OverridePrim(link_prim.GetPath())
    UsdPhysics.RigidBodyAPI.Apply(prim_over)

    if link == data.link_hierarchy.get_root_link():
        # Set the root of the Link to ArticulationRoot.
        UsdPhysics.ArticulationRootAPI.Apply(prim_over)

    # Assigning MassAPI to a Rigid Body.
    apply_inertial(link_prim, link, data)

    # Create visual or collision geometry.
    geometries: list[ElementVisual | ElementCollision] = [visual for visual in link.visuals if visual.geometry and visual.geometry.shape] + [
        collision for collision in link.collisions if collision.geometry and collision.geometry.shape
    ]

    names = [get_geometry_name(geometry_base) for geometry_base in geometries]
    safe_names = data.name_cache.getPrimNames(link_prim, names)

    for geometry, name, safe_name in zip(geometries, names, safe_names):
        convert_geometry(link_prim, name, safe_name, geometry, data)

    children = data.link_hierarchy.get_link_children(link.name)
    joints = data.link_hierarchy.get_link_joints(link.name)

    if len(children) > 0:
        for child, joint in zip(children, joints):
            child_xform = convert_link(link_prim, child, data)
            set_transform(child_xform, joint)

    return link_xform


def apply_inertial(prim: Usd.Prim, link: ElementLink, data: ConversionData):
    """
    Set the inertial parameters of a link.
    """
    if not link.inertial or (not link.inertial.origin and not link.inertial.mass and not link.inertial.inertia):
        return

    prim_over = data.content[Tokens.Physics].OverridePrim(prim.GetPath())
    mass_api: UsdPhysics.MassAPI = UsdPhysics.MassAPI.Apply(prim_over)

    if link.inertial and link.inertial.inertia:
        orientation, diag_inertia = extract_inertia(link.inertial.inertia)
        mass_api.GetPrincipalAxesAttr().Set(orientation)
        mass_api.GetDiagonalInertiaAttr().Set(diag_inertia)

    if link.inertial.origin:
        position = Gf.Vec3f(link.inertial.origin.get_with_default("xyz"))
        orientation = float3_to_quatf(link.inertial.origin.get_with_default("rpy"))
        mass_api.GetCenterOfMassAttr().Set(position)
        axes = mass_api.GetPrincipalAxesAttr().Get()
        mass_api.GetPrincipalAxesAttr().Set(orientation * axes)

    if link.inertial.mass:
        mass = link.inertial.mass.get_with_default("value")
        mass_api.GetMassAttr().Set(mass)


def extract_inertia(inertia: ElementInertia) -> tuple[Gf.Quatf, Gf.Vec3f]:
    """
    Extract the principal moments of inertia (diagonal inertia) and orientation
    from URDF link inertia tensor using eigenvalue decomposition.

    Args:
        inertia: URDF inertia tensor element

    Returns:
        tuple[Gf.Quatf, Gf.Vec3f]: (orientation, diagonal_inertia)
    """
    ixx = inertia.get_with_default("ixx")
    ixy = inertia.get_with_default("ixy")
    ixz = inertia.get_with_default("ixz")
    iyy = inertia.get_with_default("iyy")
    iyz = inertia.get_with_default("iyz")
    izz = inertia.get_with_default("izz")

    # Build inertia tensor matrix (symmetric matrix)
    mat = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])

    # Eigenvalue decomposition (using eigh for symmetric matrix)
    # eigenvalues: principal moments of inertia
    # eigenvectors: rotation matrix representing principal axes
    eigenvalues, eigenvectors = np.linalg.eigh(mat)

    # Use eigenvalues as diagonal inertia
    diag_inertia = Gf.Vec3f(float(eigenvalues[0]), float(eigenvalues[1]), float(eigenvalues[2]))

    # Convert eigenvector matrix (rotation matrix) to quaternion
    # Use Gf.Matrix3d to extract quaternion from rotation matrix
    rotation_matrix = Gf.Matrix3d(
        eigenvectors[0, 0],
        eigenvectors[0, 1],
        eigenvectors[0, 2],
        eigenvectors[1, 0],
        eigenvectors[1, 1],
        eigenvectors[1, 2],
        eigenvectors[2, 0],
        eigenvectors[2, 1],
        eigenvectors[2, 2],
    )

    # Extract quaternion from rotation matrix
    orientation = Gf.Quatf(rotation_matrix.ExtractRotation().GetQuat())
    return orientation, diag_inertia


def physics_joints(parent: Usd.Prim, link: ElementLink, data: ConversionData):
    """
    Create physics joints.
    """
    joints = data.urdf_parser.get_root_element().joints
    joint_names = [joint.name for joint in joints] if joints else []
    joint_safe_names = data.name_cache.getPrimNames(parent, joint_names)

    # Create physics joints.
    for joint, joint_safe_name in zip(joints, joint_safe_names):
        body0_link_name = joint.parent.get_with_default("link")
        body1_link_name = joint.child.get_with_default("link")
        body0 = data.references[Tokens.Physics][body0_link_name]
        body1 = data.references[Tokens.Physics][body1_link_name]

        # Specifies that the origin position of Body1 (the "child" of the joint in the URDF) is the center.
        joint_frame = usdex.core.JointFrame(usdex.core.JointFrame.Space.Body1, Gf.Vec3d(0), Gf.Quatd.GetIdentity())

        axis = Gf.Vec3f(joint.axis.get_with_default("xyz")) if joint.axis else Gf.Vec3f(1, 0, 0)

        # If limit is omitted, set to 0.0
        limit_lower = joint.limit.get_with_default("lower") if joint.limit is not None else 0.0
        limit_upper = joint.limit.get_with_default("upper") if joint.limit is not None else 0.0

        physics_joint = None
        if joint.type == "fixed":
            physics_joint = usdex.core.definePhysicsFixedJoint(parent, joint_safe_name, body0, body1, joint_frame)
        elif joint.type == "revolute" or joint.type == "continuous":
            limit_lower = None if joint.type == "continuous" else math.degrees(limit_lower)
            limit_upper = None if joint.type == "continuous" else math.degrees(limit_upper)
            physics_joint = usdex.core.definePhysicsRevoluteJoint(parent, joint_safe_name, body0, body1, joint_frame, axis, limit_lower, limit_upper)
        elif joint.type == "prismatic":
            physics_joint = usdex.core.definePhysicsPrismaticJoint(parent, joint_safe_name, body0, body1, joint_frame, axis, limit_lower, limit_upper)
        elif joint.type == "floating":
            # Floating uses a rigid body for free movement.
            # This rigid body does not belong to the joint structure.
            # So there's nothing to do here.
            pass
        elif joint.type == "planar":
            physics_joint = define_physics_planar_joint(parent, joint_safe_name, body0, body1, joint_frame, axis)

        if physics_joint and joint.name != joint_safe_name:
            usdex.core.setDisplayName(physics_joint.GetPrim(), joint.name)

        # Custom attributes.
        if physics_joint and joint.limit:
            limit_effort = joint.limit.get_with_default("effort")
            limit_velocity = joint.limit.get_with_default("velocity")
            if limit_effort is not None:
                physics_joint.GetPrim().CreateAttribute("urdf:limit:effort", Sdf.ValueTypeNames.Float, custom=True).Set(limit_effort)
            if limit_velocity is not None:
                physics_joint.GetPrim().CreateAttribute("urdf:limit:velocity", Sdf.ValueTypeNames.Float, custom=True).Set(limit_velocity)
