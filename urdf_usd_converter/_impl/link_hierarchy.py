# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
from typing import Any

from .urdf_parser.elements import (
    ElementJoint,
    ElementLink,
    ElementRobot,
)

__all__ = ["LinkHierarchy"]


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

        self._create_link_hierarchy()

        # Preprocessing is performed to identify Ghost Links.
        # If the end of a link is a Ghost Link, the rigid body associated with that link will be removed.
        self._ghost_links_chain(None, self.get_root_link())
        self._check_remove_rigid_body_flag(self.get_root_link())

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

        # If the link tree is empty, make the first link the root.
        if len(self.link_tree) == 0 and len(self.root_element.links) > 0:
            link = self.root_element.links[0]
            self.link_tree[link.name] = {
                "link": link,
                "joints": [],
                "children": [],
            }

    def _ghost_links_chain(self, parent_link: ElementLink | None, link: ElementLink):
        """
        Store data for Ghost Link.
        """
        # Determines if the link is a ghost link.
        has_ghost_link = link.inertial is None and len(link.visuals) == 0 and len(link.collisions) == 0

        # Determines if the link belongs to a fixed joint.
        belongs_to_fixed_joint = False
        if parent_link:
            parent_joints = self.get_link_joints(parent_link.name)
            if parent_joints:
                for joint in parent_joints:
                    if joint.child.get_with_default("link") == link.name and joint.type == "fixed":
                        belongs_to_fixed_joint = True
                        break

        self.links[link.name] = {
            "ghost_link": has_ghost_link,
            "belongs_to_fixed_joint": belongs_to_fixed_joint,
            "remove_rigid_body": False,
        }

        children = self.get_link_children(link.name)
        for child in children:
            self._ghost_links_chain(link, child)

    def _check_remove_rigid_body_flag(self, link: ElementLink):
        """
        Check if the link is a ghost link and all child elements are Ghost Links.
        If so, fill the remove_rigid_body flag for all ghost links.
        """
        rb = self._check_all_ghost_links(link)
        if rb:
            # If all child elements are Ghost Links, fill the remove_rigid_body flag for all ghost links.
            self._fill_all_remove_rigid_body_flag(link)
            return

        children = self.get_link_children(link.name)
        for child in children:
            self._check_remove_rigid_body_flag(child)

    def _fill_all_remove_rigid_body_flag(self, link: ElementLink):
        """
        Fill the remove_rigid_body flag for all ghost links.
        """
        self.links[link.name]["remove_rigid_body"] = True
        children = self.get_link_children(link.name)
        for child in children:
            self._fill_all_remove_rigid_body_flag(child)

    def _check_all_ghost_links(self, link: ElementLink) -> bool:
        """
        Check if all child elements are Ghost Links.
        """
        ghost_link = self.links[link.name]["ghost_link"]
        belongs_to_fixed_joint = self.links[link.name]["belongs_to_fixed_joint"]

        if not ghost_link or not belongs_to_fixed_joint:
            return False

        checked = True
        children = self.get_link_children(link.name)
        for child in children:
            if not self._check_all_ghost_links(child):
                checked = False
                break
        return checked

    def get_root_link(self) -> ElementLink:
        """
        Get the root link name from the link hierarchy.
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

    def get_link_remove_rigid_body(self, link_name: str) -> bool:
        """
        Get the remove_rigid_body flag for a link.

        If the Ghost Link extends all the way to the end of the link and
        belongs to a Fixed Joint, the rigid body can be removed.
        """
        return self.links[link_name]["remove_rigid_body"] if link_name in self.links else False
