# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import enum
from typing import Any

from .urdf_parser.elements import (
    ElementInertial,
    ElementJoint,
    ElementLink,
    ElementRobot,
)

__all__ = ["LinkHierarchy", "LinkRole"]


class LinkRole(enum.Enum):
    """
    How a link is authored as a physics body.

    `RIGID` gets a rigid body and has a mass source, or is the kinematic root.
    `GHOST` is not a rigid body. `NO_MASS_SOURCE` is still a rigid body, but
    neither it nor any descendant has a collision or a positive-mass inertial.
    """

    RIGID = "rigid"
    GHOST = "ghost"
    NO_MASS_SOURCE = "no-mass-source"


class LinkHierarchy:
    """
    Maintains the link hierarchy from joints.
    """

    def __init__(self, root_element: ElementRobot):
        self.root_element = root_element

        # A dictionary of link names and their child link names.
        self.link_tree: dict[str, dict[str, Any]] = {}

        # Store data for Ghost Link.
        self.links: dict[str, dict[str, Any]] = {}

        # A dictionary of link names and their parent links.
        self.link_parents: dict[str, ElementLink] = {}

        # A list of link names that are referenced by mimic joints.
        self.referenced_link_names_by_mimic_joint: list[str] = []

        # Create a hierarchy of links and their children from the joints.
        self._create_link_hierarchy()

        # Store the link names that are referenced by mimic joints.
        self._store_referenced_link_names_by_mimic_joint()

        # The root link of the kinematic hierarchy.
        self.root_link = self._find_root_link()

        # Preprocessing is performed to identify Ghost Links.
        self._ghost_links_chain(None, self.root_link)

    def _create_link_hierarchy(self):
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
                self.link_parents[link.name] = self.link_tree[parent_link_name]["link"]

        # If the link tree is empty, make the first link the root.
        if len(self.link_tree) == 0 and len(self.root_element.links) > 0:
            link = self.root_element.links[0]
            self.link_tree[link.name] = {
                "link": link,
                "joints": [],
                "children": [],
            }

    def _store_referenced_link_names_by_mimic_joint(self):
        """
        Store the link names that are referenced by mimic joints.
        """
        for joint in self.root_element.joints:
            if joint.mimic and joint.mimic.joint:
                ref_joint = next((j for j in self.root_element.joints if j.name == joint.mimic.joint), None)
                if ref_joint and ref_joint.type == "fixed":
                    self.referenced_link_names_by_mimic_joint.append(ref_joint.child.get_with_default("link"))

    def _is_zero_inertial(self, inertial: ElementInertial) -> bool:
        """
        True when inertial data is absent or equivalent to zero mass.

        Positive mass counts. Mass of zero with a non-zero diagonal inertia does not.
        Ghost classification and the mass-source rule both use this check.
        """
        if inertial is None:
            return True
        mass_attr = getattr(inertial, "mass", None)
        mass = mass_attr.get_with_default("value") if mass_attr is not None else None
        if mass is None or mass > 0.0:
            return mass is None
        inertia = getattr(inertial, "inertia", None)
        if inertia is None:
            return True
        diag = (inertia.get_with_default("ixx"), inertia.get_with_default("iyy"), inertia.get_with_default("izz"))
        return all(v == 0.0 or v is None for v in diag)

    def _ghost_links_chain(self, parent_link: ElementLink | None, link: ElementLink):
        """
        Store data for Ghost Link.
        """
        # Determines if the link is a ghost link.
        has_ghost_link = self._is_zero_inertial(link.inertial) and len(link.visuals) == 0 and len(link.collisions) == 0

        # Determines if the link belongs to a fixed joint.
        belongs_to_fixed_joint = False
        belongs_to_mimic_joint = False
        if parent_link:
            parent_joints = self.get_link_joints(parent_link.name)
            if parent_joints:
                for joint in parent_joints:
                    if joint.child.get_with_default("link") == link.name and joint.type == "fixed":
                        belongs_to_fixed_joint = True
                        break

                # Determines if the link belongs to a mimic joint.
                # A link referenced by a mimic joint must not be a candidate for ghost link removal.
                if belongs_to_fixed_joint and link.name in self.referenced_link_names_by_mimic_joint:
                    belongs_to_mimic_joint = True

        self.links[link.name] = {
            "ghost_link": has_ghost_link,
            "belongs_to_fixed_joint": belongs_to_fixed_joint,
            "belongs_to_mimic_joint": belongs_to_mimic_joint,
        }

        children = self.get_link_children(link.name)
        for child in children:
            self._ghost_links_chain(link, child)

    def _find_root_link(self) -> ElementLink:
        """
        Find the root link from the link hierarchy.
        """
        links = [data["link"] for data in self.link_tree.values()]
        if len(links) == 0:
            raise ValueError("The link does not exist.")

        for link in links:
            is_child = False
            for d in self.link_tree.values():
                if link in d["children"]:
                    is_child = True
                    break
            if not is_child:
                return link

        # If it is a looping joint structure, the process reaches this point.
        raise ValueError("Closed loop articulations are not supported.")

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

    def get_link_parent(self, link_name: str) -> ElementLink | None:
        """
        Get the parent link of a link, or None when the link is the kinematic root.
        """
        # The kinematic root has no parent and is not stored in `link_parents`.
        if link_name == self.root_link.name:
            return None
        return self.link_parents.get(link_name)

    def get_parent_joint(self, link_name: str) -> ElementJoint | None:
        """
        Get the joint whose child is this link, or None when the link is the kinematic root.
        """
        parent = self.get_link_parent(link_name)
        if parent is not None:
            for joint in self.get_link_joints(parent.name) or []:
                if joint.child.get_with_default("link") == link_name:
                    return joint
        return None

    def get_link_by_name(self, link_name: str) -> ElementLink:
        """
        Get a link by name.
        """
        return next((link for link in self.root_element.links if link.name == link_name), None)

    def has_ghost_link(self, link_name: str) -> bool:
        """
        Check if a link is a ghost link.
        """
        return self.links[link_name]["ghost_link"] if link_name in self.links else False

    def check_ghost_link_with_fixed_joint(self, link: ElementLink) -> bool:
        """
        Check if a link is a ghost link.
        """
        ghost_link = self.links[link.name]["ghost_link"]
        belongs_to_fixed_joint = self.links[link.name]["belongs_to_fixed_joint"]
        belongs_to_mimic_joint = self.links[link.name]["belongs_to_mimic_joint"]

        # For root links, only 'ghost_link' is used for identification.
        if link.name == self.root_link.name:
            return ghost_link

        return ghost_link and belongs_to_fixed_joint and not belongs_to_mimic_joint

    def classify_link(self, link: ElementLink) -> LinkRole:
        """
        Classify a link as a rigid body, a ghost, or a rigid body with no mass source.

        Root, mimic, and dummy-inertia exceptions are decided here, next to ghost
        classification, so authoring and the mass warning use the same decision.
        The kinematic root stays a rigid body when it is not an empty ghost,
        because `root_joint` welds it to the world and its mass never matters.
        """
        if not self._is_authored_rigid_body(link):
            return LinkRole.GHOST
        if link.name == self.root_link.name or self._subtree_supplies_mass(link):
            return LinkRole.RIGID
        return LinkRole.NO_MASS_SOURCE

    def fixed_dummy_inertia_children(self, link: ElementLink) -> list[tuple[ElementJoint, ElementLink]]:
        """
        The first fixed joint from `link` to an inertial-only child that should be merged.

        A child referenced by a mimic joint is kept as its own rigid body, matching
        Ghost Link handling, so the mimic target joint is still authored.
        Only one child is returned. Further inertial-only siblings stay rigid bodies,
        because summing them about a shared center of mass is not done here.
        """
        children = self.get_link_children(link.name)
        joints = self.get_link_joints(link.name) or []
        for child, joint in zip(children, joints):
            if child.name in self.referenced_link_names_by_mimic_joint:
                continue
            if self._is_fixed_dummy_inertia_child(link, joint, child):
                return [(joint, child)]
        return []

    def is_merged_dummy_inertia_link(self, link: ElementLink) -> bool:
        """
        True when `link` is an inertial-only child whose inertia is merged into its parent.
        """
        parent = self.get_link_parent(link.name)
        if parent is None:
            return False
        return any(child.name == link.name for _, child in self.fixed_dummy_inertia_children(parent))

    def _is_authored_rigid_body(self, link: ElementLink) -> bool:
        """
        True when conversion applies a rigid body to this link.
        """
        if link.name not in self.links:
            return False
        if self.check_ghost_link_with_fixed_joint(link):
            return False
        if self.is_merged_dummy_inertia_link(link):
            return False
        return not self._treat_visual_only_as_ghost(link)

    def _is_visual_only_link(self, link: ElementLink) -> bool:
        """
        True when the link has a visual and no collision or usable inertial data.

        A collision can still define mass at simulation time, so a link with a
        collision is not visual-only.
        """
        return bool(link.visuals) and not link.collisions and self._is_zero_inertial(link.inertial)

    def _treat_visual_only_as_ghost(self, link: ElementLink) -> bool:
        """
        True when a visual-only link on a fixed joint is authored without a rigid body.

        The kinematic root stays a rigid body because `root_joint` welds it to the
        world. A link referenced by a mimic joint also stays a rigid body. A fixed
        inertial-only child supplies the mass instead, so that parent is a rigid body.
        """
        if not self._is_visual_only_link(link):
            return False
        if link.name == self.root_link.name:
            return False
        if link.name in self.referenced_link_names_by_mimic_joint:
            return False
        if self.fixed_dummy_inertia_children(link):
            return False
        joint = self.get_parent_joint(link.name)
        return joint is not None and joint.type == "fixed"

    def _link_supplies_mass(self, link: ElementLink) -> bool:
        """
        True when the link has a collision or an inertial element with positive mass.

        A visual does not supply mass.
        """
        return bool(link.collisions) or not self._is_zero_inertial(link.inertial)

    def _subtree_supplies_mass(self, link: ElementLink) -> bool:
        """
        True when the link or any descendant supplies mass.
        """
        if self._link_supplies_mass(link):
            return True
        return any(self._subtree_supplies_mass(child) for child in self.get_link_children(link.name))

    def _link_has_geometry(self, link: ElementLink) -> bool:
        """
        True when the link has at least one visual or collision element.
        """
        return bool(link.visuals or link.collisions)

    def _is_inertial_only_link(self, link: ElementLink) -> bool:
        """
        True when the link carries non-zero inertial data and no visual or collision.
        """
        return not self._is_zero_inertial(link.inertial) and not self._link_has_geometry(link)

    def _is_fixed_dummy_inertia_child(self, parent: ElementLink, joint: ElementJoint, child: ElementLink) -> bool:
        """
        Detect a KDL-style dummy inertia child that should be merged into its parent.

        The pattern is a fixed joint whose parent has visual or collision geometry
        and no inertial, and whose child is inertial-only. Gazebo lumps that child's
        inertia into the parent. The child is not a Ghost Link, because it has
        non-zero inertial data.

        A parent that already has inertial is not this pattern. The inverse layout,
        an empty parent with a child that has geometry and inertial, is also excluded.
        """
        if joint.type != "fixed":
            return False
        if not self._is_zero_inertial(parent.inertial) or not self._link_has_geometry(parent):
            return False
        return self._is_inertial_only_link(child)
