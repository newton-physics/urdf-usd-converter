# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
from typing import Any

import numpy as np
import usdex.core
from pxr import Gf, Tf, Usd, UsdGeom, UsdPhysics

from .data import ConversionData, Tokens
from .geometry import convert_geometry
from .planar_joint import define_physics_planar_joint
from .urdf_parser.elements import (
    ElementCollision,
    ElementInertia,
    ElementJoint,
    ElementLink,
    ElementRobot,
    ElementVisual,
)
from .utils import (
    float3_to_quatf,
    float3_to_vec3d,
    get_geometry_name,
    radians_to_degrees,
    set_transform,
)

__all__ = ["convert_links"]


def convert_links(data: ConversionData):
    geo_scope = data.content[Tokens.Geometry].GetDefaultPrim().GetChild(Tokens.Geometry).GetPrim()
    root_element: ElementRobot = data.urdf_parser.get_root_element()

    # Get link hierarchy.
    link_hierarchy = LinkHierarchy(root_element)
    link_hierarchy.create_link_hierarchy()
    root_link = link_hierarchy.get_root_link()

    # Creating a Link Hierarchy.
    convert_link(parent=geo_scope, link_hierarchy=link_hierarchy, link=root_link, data=data)

    # Create Physics joints
    physics_scope = data.content[Tokens.Physics].GetDefaultPrim().GetChild(Tokens.Physics).GetPrim()
    physics_joints(parent=physics_scope, link_hierarchy=link_hierarchy, link=root_link, data=data)


class LinkHierarchy:
    """
    Maintains the link hierarchy from joints.
    """

    def __init__(self, root_element: ElementRobot):
        self.root_element = root_element

        # A dictionary of link names and their child link names.
        self.link_tree: dict[str, dict[str, Any]] = {}

    def create_link_hierarchy(self):
        """
        Create a hierarchy of links and their children from the joints.
        """
        for joint in self.root_element.joints:
            parent_link_name = joint.parent.get_with_default("link")

            if parent_link_name not in self.link_tree:
                self.link_tree[parent_link_name] = {
                    "link": self.get_link_by_name(parent_link_name),  # link
                    "children": [],  # children links
                    "joints": [],  # The joints corresponding to the "children" links
                }
            if joint.child not in self.link_tree[parent_link_name]["children"]:
                link = self.get_link_by_name(joint.child.get_with_default("link"))
                self.link_tree[parent_link_name]["children"].append(link)
                self.link_tree[parent_link_name]["joints"].append(joint)

        # If the link tree is empty, make the first link the root.
        if len(self.link_tree) == 0:
            link = self.root_element.links[0]
            self.link_tree[link.name] = {
                "link": link,
                "joints": [],
                "children": [],
            }

    def get_root_link(self) -> ElementLink:
        """
        Get the root link name from the link hierarchy.
        """
        links = [data["link"] for data in self.link_tree.values()]
        for link in links:
            is_child = False
            for d in self.link_tree.values():
                if link in d["children"]:
                    is_child = True
                    break
            if not is_child:
                return link

        # If it is a looping joint structure, the process reaches this point.
        raise ValueError("No root link found. The joint structure is a loop.")

    def get_link_joints(self, link_name: str) -> list[ElementJoint]:
        """
        Get the joints that connect to a link.
        """
        if link_name not in self.link_tree:
            return None
        return self.link_tree[link_name]["joints"]

    def get_link_children(self, link_name: str) -> list[ElementLink]:
        """
        Get the children of a link.
        """
        if link_name not in self.link_tree:
            return []
        return self.link_tree[link_name]["children"]

    def get_link_by_name(self, link_name: str) -> ElementLink:
        """
        Get a link by name.
        """
        return next((link for link in self.root_element.links if link.name == link_name), None)


def convert_link(parent: Usd.Prim, link_hierarchy: LinkHierarchy, link: ElementLink, data: ConversionData) -> UsdGeom.Xform:
    link_safe_name = data.name_cache.getPrimName(parent, link.name)
    link_xform = usdex.core.defineXform(parent, link_safe_name)
    link_prim = link_xform.GetPrim()
    if link.name != link_safe_name:
        usdex.core.setDisplayName(link_prim, link.name)

    data.references[Tokens.Physics][link.name] = link_prim

    apply_physics_rigidbody(link_prim, data)

    # Create visual or collision geometry.
    geometry_basses: list[ElementVisual | ElementCollision] = [visual for visual in link.visuals if visual.geometry and visual.geometry.geometry] + [
        collision for collision in link.collisions if collision.geometry and collision.geometry.geometry
    ]

    for geometry_base in geometry_basses:
        name = get_geometry_name(geometry_base)
        geom_safe_name = data.name_cache.getPrimName(link_prim, name)
        geom_prim = convert_geometry(link_prim, geom_safe_name, geometry_base.geometry, data)
        if geom_prim:
            is_collision = isinstance(geometry_base, ElementCollision)
            if name != geom_safe_name:
                usdex.core.setDisplayName(geom_prim.GetPrim(), name)
            set_transform(geom_prim, geometry_base)
            if is_collision:
                geom_prim.GetPurposeAttr().Set(UsdGeom.Tokens.guide)

            # Set the physics rigidbody and collision for the geometry.
            if is_collision:
                # Apply CollisionAPI to collision geometry
                apply_physics_collision_mesh(geom_prim.GetPrim(), data)
            else:
                # Apply inertial to visual geometry
                apply_inertial(geom_prim.GetPrim(), link, data)

    children = link_hierarchy.get_link_children(link.name)
    joints = link_hierarchy.get_link_joints(link.name)

    if len(children) > 0:
        for child, joint in zip(children, joints):
            child_xform = convert_link(link_prim, link_hierarchy, child, data)
            set_transform(child_xform, joint)

    return link_xform


def apply_physics_rigidbody(prim: Usd.Prim, data: ConversionData):
    """
    Apply the physics rigidbody to a prim.
    """
    prim_over = data.content[Tokens.Physics].OverridePrim(prim.GetPath())
    UsdPhysics.RigidBodyAPI.Apply(prim_over)


def apply_physics_collision_mesh(geom_prim: Usd.Prim, data: ConversionData):
    """
    Apply the physics collision to a collision mesh geometry.
    This applies CollisionAPI and MeshCollisionAPI (for mesh types).
    """
    geom_over = data.content[Tokens.Physics].OverridePrim(geom_prim.GetPath())

    # Apply CollisionAPI
    collider: UsdPhysics.CollisionAPI = UsdPhysics.CollisionAPI.Apply(geom_over)
    collider.CreateCollisionEnabledAttr().Set(True)

    # If it's a mesh, apply MeshCollisionAPI with appropriate approximation
    if geom_prim.IsA(UsdGeom.Mesh):
        mesh_collider: UsdPhysics.MeshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(geom_over)
        # Use 'none' to use the mesh as-is
        mesh_collider.CreateApproximationAttr().Set(UsdPhysics.Tokens.none)


def apply_inertial(geom_prim: Usd.Prim, link: ElementLink, data: ConversionData):
    """
    Set the inertial parameters of a link.
    """
    if not link.inertial or (not link.inertial.origin and not link.inertial.mass and not link.inertial.inertia):
        return

    geom_over = data.content[Tokens.Physics].OverridePrim(geom_prim.GetPath())
    mass_api: UsdPhysics.MassAPI = UsdPhysics.MassAPI.Apply(geom_over)

    if link.inertial and link.inertial.inertia:
        orientation, diag_inertia = extract_inertia(link.inertial.inertia)
        mass_api.CreatePrincipalAxesAttr().Set(orientation)
        mass_api.CreateDiagonalInertiaAttr().Set(diag_inertia)

    if link.inertial.origin:
        position = float3_to_vec3d(link.inertial.origin.get_with_default("xyz"))
        orientation = float3_to_quatf(link.inertial.origin.get_with_default("rpy"))
        mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(position))
        axes = mass_api.GetPrincipalAxesAttr().Get()
        mass_api.CreatePrincipalAxesAttr().Set(orientation * axes)

    if link.inertial.mass:
        mass = link.inertial.mass.get_with_default("value")
        mass_api.CreateMassAttr().Set(mass)


def extract_inertia(inertia: ElementInertia) -> tuple[Gf.Quatf, Gf.Vec3f]:
    ixx = inertia.get_with_default("ixx")
    ixy = inertia.get_with_default("ixy")
    ixz = inertia.get_with_default("ixz")
    iyy = inertia.get_with_default("iyy")
    iyz = inertia.get_with_default("iyz")
    izz = inertia.get_with_default("izz")

    mat = np.zeros((3, 3))
    mat[0, 0] = ixx
    mat[1, 1] = iyy
    mat[2, 2] = izz
    mat[0, 1] = ixy
    mat[1, 0] = ixy
    mat[0, 2] = ixz
    mat[2, 0] = ixz
    mat[1, 2] = iyz
    mat[2, 1] = iyz

    # TODO: Calculated by eigenvalue decomposition
    return Gf.Quatf(0, 0, 0, 0), Gf.Vec3f(0, 0, 0)


def physics_joints(parent: Usd.Prim, link_hierarchy: LinkHierarchy, link: ElementLink, data: ConversionData):
    """
    Create physics joints.
    """
    joints = data.urdf_parser.get_root_element().joints
    joint_names = [joint.name for joint in joints] if joints else []
    joint_safe_names = data.name_cache.getPrimNames(parent, joint_names)

    # Set the root of the Link to ArticulationRoot.
    root_link_prim = data.references[Tokens.Physics][link.name]
    UsdPhysics.ArticulationRootAPI.Apply(root_link_prim)

    # Create physics joints.
    for joint, joint_safe_name in zip(joints, joint_safe_names):
        body0_link_name = joint.parent.get_with_default("link")
        body1_link_name = joint.child.get_with_default("link")
        body0 = data.references[Tokens.Physics][body0_link_name]
        body1 = data.references[Tokens.Physics][body1_link_name]

        # Specifies that the origin position of Body1 (the "child" of the joint in the URDF) is the center.
        joint_frame = usdex.core.JointFrame(usdex.core.JointFrame.Space.Body1, Gf.Vec3d(0), Gf.Quatd.GetIdentity())

        axis = Gf.Vec3f(float3_to_vec3d(joint.axis.get_with_default("xyz"))) if joint.axis else Gf.Vec3f(1, 0, 0)

        # If limit is omitted, set to 0.0
        limit_lower = joint.limit.get_with_default("lower") if joint.limit is not None else 0.0
        limit_upper = joint.limit.get_with_default("upper") if joint.limit is not None else 0.0

        physics_joint = None
        if joint.type == "fixed":
            physics_joint = usdex.core.definePhysicsFixedJoint(parent, joint_safe_name, body0, body1, joint_frame)
        elif joint.type == "revolute" or joint.type == "continuous":
            limit_lower = None if joint.type == "continuous" else radians_to_degrees(limit_lower)
            limit_upper = None if joint.type == "continuous" else radians_to_degrees(limit_upper)
            physics_joint = usdex.core.definePhysicsRevoluteJoint(parent, joint_safe_name, body0, body1, joint_frame, axis, limit_lower, limit_upper)
        elif joint.type == "prismatic":
            physics_joint = usdex.core.definePhysicsPrismaticJoint(parent, joint_safe_name, body0, body1, joint_frame, axis, limit_lower, limit_upper)
        elif joint.type == "floating":
            Tf.Warn("Floating joints are not supported.")
        elif joint.type == "planar":
            physics_joint = define_physics_planar_joint(parent, joint_safe_name, body0, body1, joint_frame, axis)

        if physics_joint and joint.name != joint_safe_name:
            usdex.core.setDisplayName(physics_joint.GetPrim(), joint.name)

        # TODO: Custom attributes.
        # This requires creating and storing a schema such as URDFPhysicsJointAPI.
