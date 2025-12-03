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

        self._create_link_hierarchy()

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
