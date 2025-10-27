# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
import math
import pathlib
from unittest.mock import patch

from pxr import Gf, Usd, UsdPhysics

from tests.util.ConverterTestCase import ConverterTestCase
from urdf_usd_converter._impl.convert import Converter


class TestConverterJoints(ConverterTestCase):
    def setUp(self):
        super().setUp()
        self.tolerance = 1e-6

    def test_fixed_revolute_joints(self):
        input_path = "tests/data/simple_fixed_revolute_joints.urdf"
        output_dir = self.tmpDir()

        converter = Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        physics_scene_prim = stage.GetPrimAtPath("/PhysicsScene")
        self.assertIsNotNone(physics_scene_prim)

        default_prim = stage.GetDefaultPrim()
        self.assertIsNotNone(default_prim)
        default_prim_path = default_prim.GetPath()

        physics_scope_prim = stage.GetPrimAtPath(default_prim_path.AppendChild("Physics"))
        self.assertIsNotNone(physics_scope_prim)

        physics_fixed_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_root"))
        self.assertIsNotNone(physics_fixed_joint_prim)
        self.assertTrue(physics_fixed_joint_prim.IsA(UsdPhysics.FixedJoint))
        fixed_joint = UsdPhysics.FixedJoint(physics_fixed_joint_prim)
        self.assertEqual(fixed_joint.GetBody0Rel().GetTargets(), ["/simple_fixed_revolute_joints/Geometry/BaseLink"])
        self.assertEqual(fixed_joint.GetBody1Rel().GetTargets(), ["/simple_fixed_revolute_joints/Geometry/BaseLink/tn__Arm1_j8"])
        self.assertTrue(Gf.IsClose(fixed_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0.15, 0, 0), self.tolerance))
        self.assertTrue(Gf.IsClose(fixed_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), self.tolerance))
        self.assertEqual(fixed_joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
        self.assertEqual(fixed_joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))

        physics_revolute_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("tn__joint_arm1_wJ"))
        self.assertIsNotNone(physics_revolute_joint_prim)
        self.assertTrue(physics_revolute_joint_prim.IsA(UsdPhysics.RevoluteJoint))
        revolute_joint = UsdPhysics.RevoluteJoint(physics_revolute_joint_prim)
        self.assertEqual(revolute_joint.GetBody0Rel().GetTargets(), ["/simple_fixed_revolute_joints/Geometry/BaseLink/tn__Arm1_j8"])
        self.assertEqual(revolute_joint.GetBody1Rel().GetTargets(), ["/simple_fixed_revolute_joints/Geometry/BaseLink/tn__Arm1_j8/tn__Arm2_j8"])
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1.1, 0, 0), self.tolerance))
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), self.tolerance))
        self.assertEqual(revolute_joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
        self.assertEqual(revolute_joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
        self.assertEqual(revolute_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.y)
        self.assertTrue(Gf.IsClose(revolute_joint.GetLowerLimitAttr().Get(), -30.00007, self.tolerance))
        self.assertTrue(Gf.IsClose(revolute_joint.GetUpperLimitAttr().Get(), 30.00007, self.tolerance))

    def test_fixed_continuous_joints(self):
        input_path = "tests/data/simple_fixed_continuous_joints.urdf"
        output_dir = self.tmpDir()

        converter = Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        physics_scene_prim = stage.GetPrimAtPath("/PhysicsScene")
        self.assertIsNotNone(physics_scene_prim)

        default_prim = stage.GetDefaultPrim()
        self.assertIsNotNone(default_prim)
        default_prim_path = default_prim.GetPath()

        physics_scope_prim = stage.GetPrimAtPath(default_prim_path.AppendChild("Physics"))
        self.assertIsNotNone(physics_scope_prim)

        physics_fixed_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_root"))
        self.assertIsNotNone(physics_fixed_joint_prim)
        self.assertTrue(physics_fixed_joint_prim.IsA(UsdPhysics.FixedJoint))
        fixed_joint = UsdPhysics.FixedJoint(physics_fixed_joint_prim)
        self.assertEqual(fixed_joint.GetBody0Rel().GetTargets(), ["/simple_fixed_continuous_joints/Geometry/BaseLink"])
        self.assertEqual(fixed_joint.GetBody1Rel().GetTargets(), ["/simple_fixed_continuous_joints/Geometry/BaseLink/tn__Arm1_j8"])
        self.assertTrue(Gf.IsClose(fixed_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0.15, 0, 0), self.tolerance))
        self.assertTrue(Gf.IsClose(fixed_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), self.tolerance))
        self.assertEqual(fixed_joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
        self.assertEqual(fixed_joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))

        physics_revolute_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("tn__joint_arm1_wJ"))
        self.assertIsNotNone(physics_revolute_joint_prim)
        self.assertTrue(physics_revolute_joint_prim.IsA(UsdPhysics.RevoluteJoint))
        revolute_joint = UsdPhysics.RevoluteJoint(physics_revolute_joint_prim)
        self.assertEqual(revolute_joint.GetBody0Rel().GetTargets(), ["/simple_fixed_continuous_joints/Geometry/BaseLink/tn__Arm1_j8"])
        self.assertEqual(revolute_joint.GetBody1Rel().GetTargets(), ["/simple_fixed_continuous_joints/Geometry/BaseLink/tn__Arm1_j8/tn__Arm2_j8"])
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1.1, 0, 0), self.tolerance))
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), self.tolerance))
        self.assertEqual(revolute_joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
        self.assertEqual(revolute_joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
        self.assertEqual(revolute_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.x)
        self.assertEqual(revolute_joint.GetLowerLimitAttr().Get(), -math.inf)
        self.assertEqual(revolute_joint.GetUpperLimitAttr().Get(), math.inf)

    def test_fixed_prismatic_joints(self):
        input_path = "tests/data/simple_fixed_prismatic_joints.urdf"
        output_dir = self.tmpDir()

        converter = Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        physics_scene_prim = stage.GetPrimAtPath("/PhysicsScene")
        self.assertIsNotNone(physics_scene_prim)

        default_prim = stage.GetDefaultPrim()
        self.assertIsNotNone(default_prim)
        default_prim_path = default_prim.GetPath()

        physics_scope_prim = stage.GetPrimAtPath(default_prim_path.AppendChild("Physics"))
        self.assertIsNotNone(physics_scope_prim)

        physics_fixed_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_root"))
        self.assertIsNotNone(physics_fixed_joint_prim)
        self.assertTrue(physics_fixed_joint_prim.IsA(UsdPhysics.FixedJoint))
        fixed_joint = UsdPhysics.FixedJoint(physics_fixed_joint_prim)
        self.assertEqual(fixed_joint.GetBody0Rel().GetTargets(), ["/simple_fixed_prismatic_joints/Geometry/BaseLink"])
        self.assertEqual(fixed_joint.GetBody1Rel().GetTargets(), ["/simple_fixed_prismatic_joints/Geometry/BaseLink/tn__Arm1_j8"])
        self.assertTrue(Gf.IsClose(fixed_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0.15, 0, 0), self.tolerance))
        self.assertTrue(Gf.IsClose(fixed_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), self.tolerance))
        self.assertEqual(fixed_joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
        self.assertEqual(fixed_joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))

        physics_prismatic_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("tn__joint_arm1_wJ"))
        self.assertIsNotNone(physics_prismatic_joint_prim)
        self.assertTrue(physics_prismatic_joint_prim.IsA(UsdPhysics.PrismaticJoint))
        prismatic_joint = UsdPhysics.PrismaticJoint(physics_prismatic_joint_prim)
        self.assertEqual(prismatic_joint.GetBody0Rel().GetTargets(), ["/simple_fixed_prismatic_joints/Geometry/BaseLink/tn__Arm1_j8"])
        self.assertEqual(prismatic_joint.GetBody1Rel().GetTargets(), ["/simple_fixed_prismatic_joints/Geometry/BaseLink/tn__Arm1_j8/tn__Arm2_j8"])
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1.1, 0, 0), self.tolerance))
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), self.tolerance))
        self.assertEqual(prismatic_joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
        self.assertEqual(prismatic_joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
        self.assertEqual(prismatic_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.x)
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLowerLimitAttr().Get(), 0, self.tolerance))
        self.assertTrue(Gf.IsClose(prismatic_joint.GetUpperLimitAttr().Get(), 0.5, self.tolerance))

    @patch("urdf_usd_converter._impl.link.Tf.Warn")
    def test_fixed_floating_joints(self, mock_warn):
        input_path = "tests/data/simple_fixed_floating_joints.urdf"
        output_dir = self.tmpDir()

        converter = Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        # Verify that Tf.Warn was called with the expected message
        mock_warn.assert_called()

        # Check if any call contains the floating joints warning
        warning_found = False
        for call in mock_warn.call_args_list:
            if "Floating joints are not supported" in str(call):
                warning_found = True
                break

        self.assertTrue(warning_found, "Expected warning about floating joints not found.")

    @patch("urdf_usd_converter._impl.link.Tf.Warn")
    def test_fixed_planar_joints(self, mock_warn):
        input_path = "tests/data/simple_fixed_planar_joints.urdf"
        output_dir = self.tmpDir()

        converter = Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        # Verify that Tf.Warn was called with the expected message
        mock_warn.assert_called()

        # Check if any call contains the floating joints warning
        warning_found = False
        for call in mock_warn.call_args_list:
            if "Planar joints are not yet implemented" in str(call):
                warning_found = True
                break

        self.assertTrue(warning_found, "Expected warning about planar joints not found.")
