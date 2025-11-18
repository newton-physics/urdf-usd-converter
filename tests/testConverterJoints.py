# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
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

    def test_revolute_joints(self):
        input_path = "tests/data/simple_revolute_joints.urdf"
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

        physics_revolute_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_root"))
        self.assertIsNotNone(physics_revolute_joint_prim)
        self.assertTrue(physics_revolute_joint_prim.IsA(UsdPhysics.RevoluteJoint))
        revolute_joint = UsdPhysics.RevoluteJoint(physics_revolute_joint_prim)
        self.assertEqual(revolute_joint.GetBody0Rel().GetTargets(), ["/simple_revolute_joints/Geometry/BaseLink"])
        self.assertEqual(revolute_joint.GetBody1Rel().GetTargets(), ["/simple_revolute_joints/Geometry/BaseLink/tn__Arm1_j8"])
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0.15, 0, 0), self.tolerance))
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), self.tolerance))
        self.assert_rotation_almost_equal(Gf.Rotation(revolute_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)
        self.assert_rotation_almost_equal(Gf.Rotation(revolute_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)

        physics_revolute_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_arm_1"))
        self.assertIsNotNone(physics_revolute_joint_prim)
        self.assertTrue(physics_revolute_joint_prim.IsA(UsdPhysics.RevoluteJoint))
        revolute_joint = UsdPhysics.RevoluteJoint(physics_revolute_joint_prim)
        self.assertEqual(revolute_joint.GetBody0Rel().GetTargets(), ["/simple_revolute_joints/Geometry/BaseLink/tn__Arm1_j8"])
        self.assertEqual(revolute_joint.GetBody1Rel().GetTargets(), ["/simple_revolute_joints/Geometry/BaseLink/tn__Arm1_j8/tn__Arm2_j8"])
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1.1, 0, 0), self.tolerance))
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), self.tolerance))
        self.assert_rotation_almost_equal(Gf.Rotation(revolute_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)
        self.assert_rotation_almost_equal(Gf.Rotation(revolute_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)
        self.assertEqual(revolute_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.y)
        self.assertTrue(Gf.IsClose(revolute_joint.GetLowerLimitAttr().Get(), 0, self.tolerance))
        self.assertTrue(Gf.IsClose(revolute_joint.GetUpperLimitAttr().Get(), 17.188734, self.tolerance))

        physics_revolute_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_arm_2"))
        self.assertIsNotNone(physics_revolute_joint_prim)
        self.assertTrue(physics_revolute_joint_prim.IsA(UsdPhysics.RevoluteJoint))
        revolute_joint = UsdPhysics.RevoluteJoint(physics_revolute_joint_prim)
        self.assertEqual(revolute_joint.GetBody0Rel().GetTargets(), ["/simple_revolute_joints/Geometry/BaseLink/tn__Arm1_j8/tn__Arm2_j8"])
        self.assertEqual(revolute_joint.GetBody1Rel().GetTargets(), ["/simple_revolute_joints/Geometry/BaseLink/tn__Arm1_j8/tn__Arm2_j8/tn__Arm3_j8"])
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1.1, 0, 0), self.tolerance))
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), self.tolerance))
        self.assert_rotation_almost_equal(Gf.Rotation(revolute_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)
        self.assert_rotation_almost_equal(Gf.Rotation(revolute_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)
        self.assertEqual(revolute_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.y)
        self.assertTrue(Gf.IsClose(revolute_joint.GetLowerLimitAttr().Get(), -17.188734, self.tolerance))
        self.assertTrue(Gf.IsClose(revolute_joint.GetUpperLimitAttr().Get(), 0, self.tolerance))

        physics_revolute_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_arm_3"))
        self.assertIsNotNone(physics_revolute_joint_prim)
        self.assertTrue(physics_revolute_joint_prim.IsA(UsdPhysics.RevoluteJoint))
        revolute_joint = UsdPhysics.RevoluteJoint(physics_revolute_joint_prim)
        self.assertEqual(revolute_joint.GetBody0Rel().GetTargets(), ["/simple_revolute_joints/Geometry/BaseLink/tn__Arm1_j8/tn__Arm2_j8/tn__Arm3_j8"])
        self.assertEqual(
            revolute_joint.GetBody1Rel().GetTargets(), ["/simple_revolute_joints/Geometry/BaseLink/tn__Arm1_j8/tn__Arm2_j8/tn__Arm3_j8/tn__Arm4_j8"]
        )
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1.1, 0, 0), self.tolerance))
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), self.tolerance))
        self.assert_rotation_almost_equal(Gf.Rotation(revolute_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)
        self.assert_rotation_almost_equal(Gf.Rotation(revolute_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)
        self.assertEqual(revolute_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.y)
        self.assertTrue(Gf.IsClose(revolute_joint.GetLowerLimitAttr().Get(), -17.188734, self.tolerance))
        self.assertTrue(Gf.IsClose(revolute_joint.GetUpperLimitAttr().Get(), 28.64789, self.tolerance))

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
        self.assert_rotation_almost_equal(Gf.Rotation(fixed_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)
        self.assert_rotation_almost_equal(Gf.Rotation(fixed_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)

        physics_revolute_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("tn__joint_arm1_wJ"))
        self.assertIsNotNone(physics_revolute_joint_prim)
        self.assertTrue(physics_revolute_joint_prim.IsA(UsdPhysics.RevoluteJoint))
        revolute_joint = UsdPhysics.RevoluteJoint(physics_revolute_joint_prim)
        self.assertEqual(revolute_joint.GetBody0Rel().GetTargets(), ["/simple_fixed_continuous_joints/Geometry/BaseLink/tn__Arm1_j8"])
        self.assertEqual(revolute_joint.GetBody1Rel().GetTargets(), ["/simple_fixed_continuous_joints/Geometry/BaseLink/tn__Arm1_j8/tn__Arm2_j8"])
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1.1, 0, 0), self.tolerance))
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), self.tolerance))
        self.assert_rotation_almost_equal(Gf.Rotation(revolute_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)
        self.assert_rotation_almost_equal(Gf.Rotation(revolute_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)
        self.assertEqual(revolute_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.x)
        self.assertEqual(revolute_joint.GetLowerLimitAttr().Get(), -math.inf)
        self.assertEqual(revolute_joint.GetUpperLimitAttr().Get(), math.inf)

    def test_fixed_prismatic_joints(self):
        input_path = "tests/data/simple_prismatic_joints.urdf"
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

        physics_prismatic_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_root"))
        self.assertIsNotNone(physics_prismatic_joint_prim)
        self.assertTrue(physics_prismatic_joint_prim.IsA(UsdPhysics.PrismaticJoint))
        prismatic_joint = UsdPhysics.PrismaticJoint(physics_prismatic_joint_prim)
        self.assertEqual(prismatic_joint.GetBody0Rel().GetTargets(), ["/simple_prismatic_joints/Geometry/BaseLink"])
        self.assertEqual(prismatic_joint.GetBody1Rel().GetTargets(), ["/simple_prismatic_joints/Geometry/BaseLink/tn__Arm1_j8"])
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0.15, 0, 0), self.tolerance))
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), self.tolerance))
        self.assert_rotation_almost_equal(Gf.Rotation(prismatic_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)
        self.assert_rotation_almost_equal(Gf.Rotation(prismatic_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)
        self.assertEqual(prismatic_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.x)
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLowerLimitAttr().Get(), 0, self.tolerance))
        self.assertTrue(Gf.IsClose(prismatic_joint.GetUpperLimitAttr().Get(), 0, self.tolerance))

        physics_prismatic_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_arm_1"))
        self.assertIsNotNone(physics_prismatic_joint_prim)
        self.assertTrue(physics_prismatic_joint_prim.IsA(UsdPhysics.PrismaticJoint))
        prismatic_joint = UsdPhysics.PrismaticJoint(physics_prismatic_joint_prim)
        self.assertEqual(prismatic_joint.GetBody0Rel().GetTargets(), ["/simple_prismatic_joints/Geometry/BaseLink/tn__Arm1_j8"])
        self.assertEqual(prismatic_joint.GetBody1Rel().GetTargets(), ["/simple_prismatic_joints/Geometry/BaseLink/tn__Arm1_j8/tn__Arm2_j8"])
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1.1, 0, 0), self.tolerance))
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), self.tolerance))
        self.assert_rotation_almost_equal(Gf.Rotation(prismatic_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)
        self.assert_rotation_almost_equal(Gf.Rotation(prismatic_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)
        self.assertEqual(prismatic_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.x)
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLowerLimitAttr().Get(), 0, self.tolerance))
        self.assertTrue(Gf.IsClose(prismatic_joint.GetUpperLimitAttr().Get(), 0.5, self.tolerance))

        physics_prismatic_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_arm_2"))
        self.assertIsNotNone(physics_prismatic_joint_prim)
        self.assertTrue(physics_prismatic_joint_prim.IsA(UsdPhysics.PrismaticJoint))
        prismatic_joint = UsdPhysics.PrismaticJoint(physics_prismatic_joint_prim)
        self.assertEqual(prismatic_joint.GetBody0Rel().GetTargets(), ["/simple_prismatic_joints/Geometry/BaseLink/tn__Arm1_j8/tn__Arm2_j8"])
        self.assertEqual(
            prismatic_joint.GetBody1Rel().GetTargets(), ["/simple_prismatic_joints/Geometry/BaseLink/tn__Arm1_j8/tn__Arm2_j8/tn__Arm3_j8"]
        )
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1.1, 0, 0), self.tolerance))
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), self.tolerance))
        self.assert_rotation_almost_equal(Gf.Rotation(prismatic_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)
        self.assert_rotation_almost_equal(Gf.Rotation(prismatic_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)
        self.assertEqual(prismatic_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.x)
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLowerLimitAttr().Get(), -0.2, self.tolerance))
        self.assertTrue(Gf.IsClose(prismatic_joint.GetUpperLimitAttr().Get(), 0, self.tolerance))

        physics_prismatic_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_arm_3"))
        self.assertIsNotNone(physics_prismatic_joint_prim)
        self.assertTrue(physics_prismatic_joint_prim.IsA(UsdPhysics.PrismaticJoint))
        prismatic_joint = UsdPhysics.PrismaticJoint(physics_prismatic_joint_prim)
        self.assertEqual(
            prismatic_joint.GetBody0Rel().GetTargets(), ["/simple_prismatic_joints/Geometry/BaseLink/tn__Arm1_j8/tn__Arm2_j8/tn__Arm3_j8"]
        )
        self.assertEqual(
            prismatic_joint.GetBody1Rel().GetTargets(), ["/simple_prismatic_joints/Geometry/BaseLink/tn__Arm1_j8/tn__Arm2_j8/tn__Arm3_j8/tn__Arm4_j8"]
        )
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1.1, 0, 0), self.tolerance))
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), self.tolerance))
        self.assert_rotation_almost_equal(Gf.Rotation(prismatic_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)
        self.assert_rotation_almost_equal(Gf.Rotation(prismatic_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)
        self.assertEqual(prismatic_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.x)
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLowerLimitAttr().Get(), 0, self.tolerance))
        self.assertTrue(Gf.IsClose(prismatic_joint.GetUpperLimitAttr().Get(), 0.5, self.tolerance))

    def test_fixed_planar_joints(self):
        input_path = "tests/data/simple_fixed_planar_joints.urdf"
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

        # Planar joint(UsdPhysics.Joint): Axis (0, 0, 1).
        physics_planar_axis_z_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_planar_axis_z"))
        self.assertIsNotNone(physics_planar_axis_z_prim)
        self.assertTrue(physics_planar_axis_z_prim.IsA(UsdPhysics.Joint))
        physics_joint = UsdPhysics.Joint(physics_planar_axis_z_prim)
        self.assertEqual(physics_joint.GetBody0Rel().GetTargets(), ["/simple_fixed_planar_joints/Geometry/BaseLink/link1"])
        self.assertEqual(physics_joint.GetBody1Rel().GetTargets(), ["/simple_fixed_planar_joints/Geometry/BaseLink/link1/planar_z"])
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0.35), self.tolerance))
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), self.tolerance))
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)

        self.assertFalse(physics_planar_axis_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transX))
        self.assertFalse(physics_planar_axis_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transY))
        self.assertTrue(physics_planar_axis_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transZ))
        self.assertTrue(physics_planar_axis_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotX))
        self.assertTrue(physics_planar_axis_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotY))
        self.assertFalse(physics_planar_axis_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotZ))

        limit_api_trans_z: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_z_prim.GetPrim(), UsdPhysics.Tokens.transZ)
        self.assertEqual(limit_api_trans_z.GetLowAttr().Get(), 0.0)
        self.assertEqual(limit_api_trans_z.GetHighAttr().Get(), 0.0)
        limit_api_rot_x: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_z_prim.GetPrim(), UsdPhysics.Tokens.rotX)
        self.assertEqual(limit_api_rot_x.GetLowAttr().Get(), 0.0)
        self.assertEqual(limit_api_rot_x.GetHighAttr().Get(), 0.0)
        limit_api_rot_y: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_z_prim.GetPrim(), UsdPhysics.Tokens.rotY)
        self.assertEqual(limit_api_rot_y.GetLowAttr().Get(), 0.0)
        self.assertEqual(limit_api_rot_y.GetHighAttr().Get(), 0.0)

        # Planar joint(UsdPhysics.Joint): Axis (0, 0, -1).
        physics_planar_axis_negative_z_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_planar_axis_negative_z"))
        self.assertIsNotNone(physics_planar_axis_negative_z_prim)
        self.assertTrue(physics_planar_axis_negative_z_prim.IsA(UsdPhysics.Joint))
        physics_joint = UsdPhysics.Joint(physics_planar_axis_negative_z_prim)
        self.assertEqual(physics_joint.GetBody0Rel().GetTargets(), ["/simple_fixed_planar_joints/Geometry/BaseLink/link1"])
        self.assertEqual(physics_joint.GetBody1Rel().GetTargets(), ["/simple_fixed_planar_joints/Geometry/BaseLink/link1/planar_negative_z"])
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0.7), self.tolerance))
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), self.tolerance))
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(0, Gf.Vec3f(-1, 0, 0))), 1e-4)
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(0, Gf.Vec3f(-1, 0, 0))), 1e-4)

        self.assertFalse(physics_planar_axis_negative_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transX))
        self.assertFalse(physics_planar_axis_negative_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transY))
        self.assertTrue(physics_planar_axis_negative_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transZ))
        self.assertTrue(physics_planar_axis_negative_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotX))
        self.assertTrue(physics_planar_axis_negative_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotY))
        self.assertFalse(physics_planar_axis_negative_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotZ))

        limit_api_trans_z: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_z_prim.GetPrim(), UsdPhysics.Tokens.transZ)
        self.assertEqual(limit_api_trans_z.GetLowAttr().Get(), 0.0)
        self.assertEqual(limit_api_trans_z.GetHighAttr().Get(), 0.0)
        limit_api_rot_x: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_z_prim.GetPrim(), UsdPhysics.Tokens.rotX)
        self.assertEqual(limit_api_rot_x.GetLowAttr().Get(), 0.0)
        self.assertEqual(limit_api_rot_x.GetHighAttr().Get(), 0.0)
        limit_api_rot_y: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_z_prim.GetPrim(), UsdPhysics.Tokens.rotY)
        self.assertEqual(limit_api_rot_y.GetLowAttr().Get(), 0.0)
        self.assertEqual(limit_api_rot_y.GetHighAttr().Get(), 0.0)

        # Planar joint(UsdPhysics.Joint): Axis (0, 1, 0).
        physics_planar_axis_y_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_planar_axis_y"))
        self.assertIsNotNone(physics_planar_axis_y_prim)
        self.assertTrue(physics_planar_axis_y_prim.IsA(UsdPhysics.Joint))
        physics_joint = UsdPhysics.Joint(physics_planar_axis_y_prim)
        self.assertEqual(physics_joint.GetBody0Rel().GetTargets(), ["/simple_fixed_planar_joints/Geometry/BaseLink/link1"])
        self.assertEqual(physics_joint.GetBody1Rel().GetTargets(), ["/simple_fixed_planar_joints/Geometry/BaseLink/link1/planar_y"])
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 1, 0), self.tolerance))
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), self.tolerance))
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)

        self.assertFalse(physics_planar_axis_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transX))
        self.assertTrue(physics_planar_axis_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transY))
        self.assertFalse(physics_planar_axis_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transZ))
        self.assertTrue(physics_planar_axis_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotX))
        self.assertFalse(physics_planar_axis_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotY))
        self.assertTrue(physics_planar_axis_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotZ))

        limit_api_trans_y: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_y_prim.GetPrim(), UsdPhysics.Tokens.transY)
        self.assertEqual(limit_api_trans_y.GetLowAttr().Get(), 0.0)
        self.assertEqual(limit_api_trans_y.GetHighAttr().Get(), 0.0)
        limit_api_rot_x: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_y_prim.GetPrim(), UsdPhysics.Tokens.rotX)
        self.assertEqual(limit_api_rot_x.GetLowAttr().Get(), 0.0)
        self.assertEqual(limit_api_rot_x.GetHighAttr().Get(), 0.0)
        limit_api_rot_z: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_y_prim.GetPrim(), UsdPhysics.Tokens.rotZ)
        self.assertEqual(limit_api_rot_z.GetLowAttr().Get(), 0.0)
        self.assertEqual(limit_api_rot_z.GetHighAttr().Get(), 0.0)

        # Planar joint(UsdPhysics.Joint): Axis (0, -1, 0).
        physics_planar_axis_negative_y_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_planar_axis_negative_y"))
        self.assertIsNotNone(physics_planar_axis_negative_y_prim)
        self.assertTrue(physics_planar_axis_negative_y_prim.IsA(UsdPhysics.Joint))
        physics_joint = UsdPhysics.Joint(physics_planar_axis_negative_y_prim)
        self.assertEqual(physics_joint.GetBody0Rel().GetTargets(), ["/simple_fixed_planar_joints/Geometry/BaseLink/link1"])
        self.assertEqual(physics_joint.GetBody1Rel().GetTargets(), ["/simple_fixed_planar_joints/Geometry/BaseLink/link1/planar_negative_y"])
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 2, 0), self.tolerance))
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), self.tolerance))
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(0, Gf.Vec3f(-1, 0, 0))), 1e-4)
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(0, Gf.Vec3f(-1, 0, 0))), 1e-4)

        self.assertFalse(physics_planar_axis_negative_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transX))
        self.assertTrue(physics_planar_axis_negative_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transY))
        self.assertFalse(physics_planar_axis_negative_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transZ))
        self.assertTrue(physics_planar_axis_negative_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotX))
        self.assertFalse(physics_planar_axis_negative_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotY))
        self.assertTrue(physics_planar_axis_negative_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotZ))

        limit_api_trans_y: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_y_prim.GetPrim(), UsdPhysics.Tokens.transY)
        self.assertEqual(limit_api_trans_y.GetLowAttr().Get(), 0.0)
        self.assertEqual(limit_api_trans_y.GetHighAttr().Get(), 0.0)
        limit_api_rot_x: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_y_prim.GetPrim(), UsdPhysics.Tokens.rotX)
        self.assertEqual(limit_api_rot_x.GetLowAttr().Get(), 0.0)
        self.assertEqual(limit_api_rot_x.GetHighAttr().Get(), 0.0)
        limit_api_rot_z: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_y_prim.GetPrim(), UsdPhysics.Tokens.rotZ)
        self.assertEqual(limit_api_rot_z.GetLowAttr().Get(), 0.0)
        self.assertEqual(limit_api_rot_z.GetHighAttr().Get(), 0.0)

        # Planar joint(UsdPhysics.Joint): Axis (1, 0, 0).
        physics_planar_axis_x_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_planar_axis_x"))
        self.assertIsNotNone(physics_planar_axis_x_prim)
        self.assertTrue(physics_planar_axis_x_prim.IsA(UsdPhysics.Joint))
        physics_joint = UsdPhysics.Joint(physics_planar_axis_x_prim)
        self.assertEqual(physics_joint.GetBody0Rel().GetTargets(), ["/simple_fixed_planar_joints/Geometry/BaseLink/link1"])
        self.assertEqual(physics_joint.GetBody1Rel().GetTargets(), ["/simple_fixed_planar_joints/Geometry/BaseLink/link1/planar_x"])
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(-1, 0, 0), self.tolerance))
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), self.tolerance))
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)

        self.assertTrue(physics_planar_axis_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transX))
        self.assertFalse(physics_planar_axis_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transY))
        self.assertFalse(physics_planar_axis_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transZ))
        self.assertFalse(physics_planar_axis_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotX))
        self.assertTrue(physics_planar_axis_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotY))
        self.assertTrue(physics_planar_axis_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotZ))

        limit_api_trans_x: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_x_prim.GetPrim(), UsdPhysics.Tokens.transX)
        self.assertEqual(limit_api_trans_x.GetLowAttr().Get(), 0.0)
        self.assertEqual(limit_api_trans_x.GetHighAttr().Get(), 0.0)
        limit_api_rot_y: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_x_prim.GetPrim(), UsdPhysics.Tokens.rotY)
        self.assertEqual(limit_api_rot_y.GetLowAttr().Get(), 0.0)
        self.assertEqual(limit_api_rot_y.GetHighAttr().Get(), 0.0)
        limit_api_rot_z: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_x_prim.GetPrim(), UsdPhysics.Tokens.rotZ)
        self.assertEqual(limit_api_rot_z.GetLowAttr().Get(), 0.0)
        self.assertEqual(limit_api_rot_z.GetHighAttr().Get(), 0.0)

        # Planar joint(UsdPhysics.Joint): Axis (-1, 0, 0).
        physics_planar_axis_negative_x_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_planar_axis_negative_x"))
        self.assertIsNotNone(physics_planar_axis_negative_x_prim)
        self.assertTrue(physics_planar_axis_negative_x_prim.IsA(UsdPhysics.Joint))
        physics_joint = UsdPhysics.Joint(physics_planar_axis_negative_x_prim)
        self.assertEqual(physics_joint.GetBody0Rel().GetTargets(), ["/simple_fixed_planar_joints/Geometry/BaseLink/link1"])
        self.assertEqual(physics_joint.GetBody1Rel().GetTargets(), ["/simple_fixed_planar_joints/Geometry/BaseLink/link1/planar_negative_x"])
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(-2, 0, 0), self.tolerance))
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), self.tolerance))
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(0, Gf.Vec3f(0, -1, 0))), 1e-4)
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(0, Gf.Vec3f(0, -1, 0))), 1e-4)

        self.assertTrue(physics_planar_axis_negative_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transX))
        self.assertFalse(physics_planar_axis_negative_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transY))
        self.assertFalse(physics_planar_axis_negative_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transZ))
        self.assertFalse(physics_planar_axis_negative_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotX))
        self.assertTrue(physics_planar_axis_negative_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotY))
        self.assertTrue(physics_planar_axis_negative_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotZ))

        limit_api_trans_x: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_x_prim.GetPrim(), UsdPhysics.Tokens.transX)
        self.assertEqual(limit_api_trans_x.GetLowAttr().Get(), 0.0)
        self.assertEqual(limit_api_trans_x.GetHighAttr().Get(), 0.0)
        limit_api_rot_y: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_x_prim.GetPrim(), UsdPhysics.Tokens.rotY)
        self.assertEqual(limit_api_rot_y.GetLowAttr().Get(), 0.0)
        self.assertEqual(limit_api_rot_y.GetHighAttr().Get(), 0.0)
        limit_api_rot_z: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_x_prim.GetPrim(), UsdPhysics.Tokens.rotZ)
        self.assertEqual(limit_api_rot_z.GetLowAttr().Get(), 0.0)
        self.assertEqual(limit_api_rot_z.GetHighAttr().Get(), 0.0)

        # Planar joint(UsdPhysics.Joint): Axis (0.2 0.3 1).
        physics_planar_axis_tilt_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_planar_axis_tilt"))
        self.assertIsNotNone(physics_planar_axis_tilt_prim)
        self.assertTrue(physics_planar_axis_tilt_prim.IsA(UsdPhysics.Joint))
        physics_joint = UsdPhysics.Joint(physics_planar_axis_tilt_prim)
        self.assertEqual(physics_joint.GetBody0Rel().GetTargets(), ["/simple_fixed_planar_joints/Geometry/BaseLink/link1"])
        self.assertEqual(physics_joint.GetBody1Rel().GetTargets(), ["/simple_fixed_planar_joints/Geometry/BaseLink/link1/planar_tilt"])
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1, 0, 0), self.tolerance))
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), self.tolerance))
        self.assert_rotation_almost_equal(
            Gf.Rotation(physics_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(0.7707607, Gf.Vec3f(0, -0.6102548, 0.18307644))), 1e-4
        )
        self.assert_rotation_almost_equal(
            Gf.Rotation(physics_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(0.7707607, Gf.Vec3f(0, -0.6102548, 0.18307644))), 1e-4
        )

        self.assertTrue(physics_planar_axis_tilt_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transX))
        self.assertFalse(physics_planar_axis_tilt_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transY))
        self.assertFalse(physics_planar_axis_tilt_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transZ))
        self.assertFalse(physics_planar_axis_tilt_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotX))
        self.assertTrue(physics_planar_axis_tilt_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotY))
        self.assertTrue(physics_planar_axis_tilt_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotZ))

        # Planar joint(UsdPhysics.Joint): Axis (0, 0, 0).
        physics_planar_axis_tilt_axis_len_0_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_planar_axis_tilt_axis_len_0"))
        self.assertIsNotNone(physics_planar_axis_tilt_axis_len_0_prim)
        self.assertTrue(physics_planar_axis_tilt_axis_len_0_prim.IsA(UsdPhysics.Joint))
        physics_joint = UsdPhysics.Joint(physics_planar_axis_tilt_axis_len_0_prim)
        self.assertEqual(physics_joint.GetBody0Rel().GetTargets(), ["/simple_fixed_planar_joints/Geometry/BaseLink/link1"])
        self.assertEqual(physics_joint.GetBody1Rel().GetTargets(), ["/simple_fixed_planar_joints/Geometry/BaseLink/link1/planar_tilt_axis_len_0"])
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1, 0, 0.5), self.tolerance))
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), self.tolerance))
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-4)

        self.assertTrue(physics_planar_axis_tilt_axis_len_0_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transX))
        self.assertFalse(physics_planar_axis_tilt_axis_len_0_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transY))
        self.assertFalse(physics_planar_axis_tilt_axis_len_0_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transZ))
        self.assertFalse(physics_planar_axis_tilt_axis_len_0_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotX))
        self.assertTrue(physics_planar_axis_tilt_axis_len_0_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotY))
        self.assertTrue(physics_planar_axis_tilt_axis_len_0_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotZ))

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
