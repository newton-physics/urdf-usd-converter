# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import usdex.core
from pxr import Usd, UsdGeom, UsdPhysics

import urdf_usd_converter
from tests.util.ConverterTestCase import ConverterTestCase


class TestDisplayName(ConverterTestCase):
    def test_display_name(self):
        input_path = "tests/data/test_displayname.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        default_prim = stage.GetDefaultPrim()
        self.assertTrue(default_prim.IsValid())

        geometry_scope_prim = default_prim.GetChild("Geometry")
        self.assertIsNotNone(geometry_scope_prim)

        link_box_prim = geometry_scope_prim.GetChild("tn__linkbox_sA")
        self.assertTrue(link_box_prim.IsValid())
        self.assertTrue(link_box_prim.IsA(UsdGeom.Xform))
        self.assertEqual(usdex.core.getDisplayName(link_box_prim), "link-box")

        box_prim = link_box_prim.GetChild("box")
        self.assertTrue(box_prim.IsValid())
        self.assertTrue(box_prim.IsA(UsdGeom.Cube))

        link_box_2_prim = link_box_prim.GetChild("tn__linkbox2_bC")
        self.assertTrue(link_box_2_prim.IsValid())
        self.assertTrue(link_box_2_prim.IsA(UsdGeom.Xform))
        self.assertEqual(usdex.core.getDisplayName(link_box_2_prim), "link-box2")

        box_2_prim = link_box_2_prim.GetChild("box")
        self.assertTrue(box_2_prim.IsValid())
        self.assertTrue(box_2_prim.IsA(UsdGeom.Cube))

        physics_scope_prim = default_prim.GetChild("Physics")
        self.assertTrue(physics_scope_prim.IsValid())

        joint_root_prim = physics_scope_prim.GetChild("tn__jointroot_wH")
        self.assertTrue(joint_root_prim.IsValid())
        self.assertTrue(joint_root_prim.IsA(UsdPhysics.FixedJoint))
        self.assertEqual(usdex.core.getDisplayName(joint_root_prim), "joint:root")
