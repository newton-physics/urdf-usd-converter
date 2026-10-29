# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib
import unittest

from urdf_usd_converter._impl.link_hierarchy import LinkHierarchy, LinkRole
from urdf_usd_converter._impl.urdf_parser.parser import URDFParser


def _roles(relative_path: str) -> dict[str, LinkRole]:
    parser = URDFParser(pathlib.Path(relative_path))
    parser.parse()
    hierarchy = LinkHierarchy(parser.get_root_element())
    return {link.name: hierarchy.classify_link(link) for link in parser.get_root_element().links}


class TestLinkRole(unittest.TestCase):
    def test_classify_link_returns_one_role(self):
        """
        One classifier decides rigid, ghost, and no-mass-source.

        Root, mimic, and a fixed inertial-only child are decided with the ghost
        classification, so authoring and the mass warning cannot drift apart.
        """
        fixed = _roles("tests/data/visual_only_fixed.urdf")
        self.assertEqual(fixed["base"], LinkRole.RIGID)
        self.assertEqual(fixed["drawn"], LinkRole.GHOST)
        self.assertEqual(fixed["zero"], LinkRole.GHOST)
        self.assertEqual(fixed["solid"], LinkRole.RIGID)
        self.assertEqual(fixed["bare"], LinkRole.GHOST)

        revolute = _roles("tests/data/visual_only_revolute.urdf")
        self.assertEqual(revolute["base"], LinkRole.RIGID)
        self.assertEqual(revolute["arm"], LinkRole.NO_MASS_SOURCE)

        root = _roles("tests/data/visual_only_root.urdf")
        self.assertEqual(root["base"], LinkRole.RIGID)

        mimic = _roles("tests/data/visual_only_mimic.urdf")
        self.assertEqual(mimic["pad"], LinkRole.NO_MASS_SOURCE)

        dummy = _roles("tests/data/visual_only_dummy.urdf")
        self.assertEqual(dummy["pad"], LinkRole.RIGID)
        self.assertEqual(dummy["pad_inertia"], LinkRole.GHOST)

        chain = _roles("tests/data/mass_source_empty_chain.urdf")
        self.assertEqual(chain["empty"], LinkRole.RIGID)
        self.assertEqual(chain["arm"], LinkRole.RIGID)

        empty = _roles("tests/data/mass_source_empty_revolute.urdf")
        self.assertEqual(empty["empty"], LinkRole.NO_MASS_SOURCE)

        world = _roles("tests/data/articulation_root_visual_world.urdf")
        self.assertEqual(world["world"], LinkRole.GHOST)
        self.assertEqual(world["drawn"], LinkRole.GHOST)
        self.assertEqual(world["arm"], LinkRole.RIGID)

        descendant = _roles("tests/data/mass_source_descendant.urdf")
        self.assertEqual(descendant["drawn"], LinkRole.RIGID)
        self.assertEqual(descendant["tool"], LinkRole.RIGID)
