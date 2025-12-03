# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

from pxr import Usd, UsdGeom, UsdPhysics

import urdf_usd_converter
from tests.util.ConverterTestCase import ConverterTestCase


class TestChangeJointOrder(ConverterTestCase):
    def test_change_joint_order(self):
        input_path = "tests/data/change_joint_order.urdf"
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

        link_box_prim = geometry_scope_prim.GetChild("link_box")
        self.assertTrue(link_box_prim.IsValid())
        self.assertTrue(link_box_prim.IsA(UsdGeom.Xform))

        box_prim = link_box_prim.GetChild("box")
        self.assertTrue(box_prim.IsValid())
        self.assertTrue(box_prim.IsA(UsdGeom.Cube))

        link_box_2_prim = link_box_prim.GetChild("link_box2")
        self.assertTrue(link_box_2_prim.IsValid())
        self.assertTrue(link_box_2_prim.IsA(UsdGeom.Xform))

        box_2_prim = link_box_2_prim.GetChild("box")
        self.assertTrue(box_2_prim.IsValid())
        self.assertTrue(box_2_prim.IsA(UsdGeom.Cube))

        link_box_3_prim = link_box_2_prim.GetChild("link_box3")
        self.assertTrue(link_box_3_prim.IsValid())
        self.assertTrue(link_box_3_prim.IsA(UsdGeom.Xform))

        box_3_prim = link_box_3_prim.GetChild("box")
        self.assertTrue(box_3_prim.IsValid())
        self.assertTrue(box_3_prim.IsA(UsdGeom.Cube))

        physics_scope_prim = default_prim.GetChild("Physics")
        self.assertTrue(physics_scope_prim.IsValid())

        joint_root_prim = physics_scope_prim.GetChild("joint_root")
        self.assertTrue(joint_root_prim.IsValid())
        self.assertTrue(joint_root_prim.IsA(UsdPhysics.FixedJoint))
        joint_root = UsdPhysics.FixedJoint(joint_root_prim)
        self.assertEqual(joint_root.GetBody0Rel().GetTargets(), ["/change_joint_order/Geometry/link_box"])
        self.assertEqual(joint_root.GetBody1Rel().GetTargets(), ["/change_joint_order/Geometry/link_box/link_box2"])

        joint_box_prim = physics_scope_prim.GetChild("joint_box")
        self.assertTrue(joint_box_prim.IsValid())
        self.assertTrue(joint_box_prim.IsA(UsdPhysics.FixedJoint))
        joint_box = UsdPhysics.FixedJoint(joint_box_prim)
        self.assertEqual(joint_box.GetBody0Rel().GetTargets(), ["/change_joint_order/Geometry/link_box/link_box2"])
        self.assertEqual(joint_box.GetBody1Rel().GetTargets(), ["/change_joint_order/Geometry/link_box/link_box2/link_box3"])
