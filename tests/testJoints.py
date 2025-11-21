# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import usdex.test
from pxr import Gf, Tf, Usd, UsdPhysics

import urdf_usd_converter
from tests.util.ConverterTestCase import ConverterTestCase


class TestJoints(ConverterTestCase):
    def test_revolute_joints(self):
        input_path = "tests/data/revolute_joints.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        default_prim = stage.GetDefaultPrim()
        self.assertTrue(default_prim.IsValid())
        default_prim_path = default_prim.GetPath()

        physics_scope_prim = stage.GetPrimAtPath(default_prim_path.AppendChild("Physics"))
        self.assertTrue(physics_scope_prim.IsValid())

        physics_revolute_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_root"))
        self.assertTrue(physics_revolute_joint_prim.IsValid())
        self.assertTrue(physics_revolute_joint_prim.IsA(UsdPhysics.RevoluteJoint))
        revolute_joint = UsdPhysics.RevoluteJoint(physics_revolute_joint_prim)
        self.assertEqual(revolute_joint.GetBody0Rel().GetTargets(), ["/revolute_joints/Geometry/BaseLink"])
        self.assertEqual(revolute_joint.GetBody1Rel().GetTargets(), ["/revolute_joints/Geometry/BaseLink/Arm_1"])
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0.15, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assert_rotation_almost_equal(Gf.Rotation(revolute_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)
        self.assert_rotation_almost_equal(Gf.Rotation(revolute_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)
        self.assertEqual(revolute_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.y)

        physics_revolute_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_arm_1"))
        self.assertTrue(physics_revolute_joint_prim.IsValid())
        self.assertTrue(physics_revolute_joint_prim.IsA(UsdPhysics.RevoluteJoint))
        revolute_joint = UsdPhysics.RevoluteJoint(physics_revolute_joint_prim)
        self.assertEqual(revolute_joint.GetBody0Rel().GetTargets(), ["/revolute_joints/Geometry/BaseLink/Arm_1"])
        self.assertEqual(revolute_joint.GetBody1Rel().GetTargets(), ["/revolute_joints/Geometry/BaseLink/Arm_1/Arm_2"])
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1.1, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assert_rotation_almost_equal(Gf.Rotation(revolute_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)
        self.assert_rotation_almost_equal(Gf.Rotation(revolute_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)
        self.assertEqual(revolute_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.y)
        self.assertAlmostEqual(revolute_joint.GetLowerLimitAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(revolute_joint.GetUpperLimitAttr().Get(), 17.188734, places=6)

        physics_revolute_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_arm_2"))
        self.assertTrue(physics_revolute_joint_prim.IsValid())
        self.assertTrue(physics_revolute_joint_prim.IsA(UsdPhysics.RevoluteJoint))
        revolute_joint = UsdPhysics.RevoluteJoint(physics_revolute_joint_prim)
        self.assertEqual(revolute_joint.GetBody0Rel().GetTargets(), ["/revolute_joints/Geometry/BaseLink/Arm_1/Arm_2"])
        self.assertEqual(revolute_joint.GetBody1Rel().GetTargets(), ["/revolute_joints/Geometry/BaseLink/Arm_1/Arm_2/Arm_3"])
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1.1, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assert_rotation_almost_equal(Gf.Rotation(revolute_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)
        self.assert_rotation_almost_equal(Gf.Rotation(revolute_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)
        self.assertEqual(revolute_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.y)
        self.assertAlmostEqual(revolute_joint.GetLowerLimitAttr().Get(), -17.188734, places=6)
        self.assertAlmostEqual(revolute_joint.GetUpperLimitAttr().Get(), 0.0, places=6)

        physics_revolute_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_arm_3"))
        self.assertTrue(physics_revolute_joint_prim.IsValid())
        self.assertTrue(physics_revolute_joint_prim.IsA(UsdPhysics.RevoluteJoint))
        revolute_joint = UsdPhysics.RevoluteJoint(physics_revolute_joint_prim)
        self.assertEqual(revolute_joint.GetBody0Rel().GetTargets(), ["/revolute_joints/Geometry/BaseLink/Arm_1/Arm_2/Arm_3"])
        self.assertEqual(revolute_joint.GetBody1Rel().GetTargets(), ["/revolute_joints/Geometry/BaseLink/Arm_1/Arm_2/Arm_3/Arm_4"])
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1.1, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assert_rotation_almost_equal(Gf.Rotation(revolute_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)
        self.assert_rotation_almost_equal(Gf.Rotation(revolute_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)
        self.assertEqual(revolute_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.y)
        self.assertAlmostEqual(revolute_joint.GetLowerLimitAttr().Get(), -17.188734, places=6)
        self.assertAlmostEqual(revolute_joint.GetUpperLimitAttr().Get(), 28.64789, places=6)

    def test_fixed_continuous_joints(self):
        input_path = "tests/data/fixed_continuous_joints.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        default_prim = stage.GetDefaultPrim()
        self.assertTrue(default_prim.IsValid())
        default_prim_path = default_prim.GetPath()

        physics_scope_prim = stage.GetPrimAtPath(default_prim_path.AppendChild("Physics"))
        self.assertIsNotNone(physics_scope_prim)

        physics_fixed_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_root"))
        self.assertTrue(physics_fixed_joint_prim.IsValid())
        self.assertTrue(physics_fixed_joint_prim.IsA(UsdPhysics.FixedJoint))
        fixed_joint = UsdPhysics.FixedJoint(physics_fixed_joint_prim)
        self.assertEqual(fixed_joint.GetBody0Rel().GetTargets(), ["/fixed_continuous_joints/Geometry/BaseLink"])
        self.assertEqual(fixed_joint.GetBody1Rel().GetTargets(), ["/fixed_continuous_joints/Geometry/BaseLink/Arm_1"])
        self.assertTrue(Gf.IsClose(fixed_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0.15, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(fixed_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assert_rotation_almost_equal(Gf.Rotation(fixed_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)
        self.assert_rotation_almost_equal(Gf.Rotation(fixed_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)

        physics_revolute_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_arm_1"))
        self.assertTrue(physics_revolute_joint_prim.IsValid())
        self.assertTrue(physics_revolute_joint_prim.IsA(UsdPhysics.RevoluteJoint))
        revolute_joint = UsdPhysics.RevoluteJoint(physics_revolute_joint_prim)
        self.assertEqual(revolute_joint.GetBody0Rel().GetTargets(), ["/fixed_continuous_joints/Geometry/BaseLink/Arm_1"])
        self.assertEqual(revolute_joint.GetBody1Rel().GetTargets(), ["/fixed_continuous_joints/Geometry/BaseLink/Arm_1/Arm_2"])
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1.1, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assert_rotation_almost_equal(Gf.Rotation(revolute_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)
        self.assert_rotation_almost_equal(Gf.Rotation(revolute_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)
        self.assertEqual(revolute_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.x)
        self.assertFalse(revolute_joint.GetLowerLimitAttr().HasAuthoredValue())
        self.assertFalse(revolute_joint.GetUpperLimitAttr().HasAuthoredValue())

    def test_fixed_prismatic_joints(self):
        input_path = "tests/data/prismatic_joints.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        default_prim = stage.GetDefaultPrim()
        self.assertTrue(default_prim.IsValid())
        default_prim_path = default_prim.GetPath()

        physics_scope_prim = stage.GetPrimAtPath(default_prim_path.AppendChild("Physics"))
        self.assertTrue(physics_scope_prim.IsValid())

        physics_prismatic_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_root"))
        self.assertTrue(physics_prismatic_joint_prim.IsValid())
        self.assertTrue(physics_prismatic_joint_prim.IsA(UsdPhysics.PrismaticJoint))
        prismatic_joint = UsdPhysics.PrismaticJoint(physics_prismatic_joint_prim)
        self.assertEqual(prismatic_joint.GetBody0Rel().GetTargets(), ["/prismatic_joints/Geometry/BaseLink"])
        self.assertEqual(prismatic_joint.GetBody1Rel().GetTargets(), ["/prismatic_joints/Geometry/BaseLink/Arm_1"])
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0.15, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assert_rotation_almost_equal(Gf.Rotation(prismatic_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)
        self.assert_rotation_almost_equal(Gf.Rotation(prismatic_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)
        self.assertEqual(prismatic_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.x)
        self.assertAlmostEqual(prismatic_joint.GetLowerLimitAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(prismatic_joint.GetUpperLimitAttr().Get(), 0.0, places=6)

        physics_prismatic_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_arm_1"))
        self.assertTrue(physics_prismatic_joint_prim.IsValid())
        self.assertTrue(physics_prismatic_joint_prim.IsA(UsdPhysics.PrismaticJoint))
        prismatic_joint = UsdPhysics.PrismaticJoint(physics_prismatic_joint_prim)
        self.assertEqual(prismatic_joint.GetBody0Rel().GetTargets(), ["/prismatic_joints/Geometry/BaseLink/Arm_1"])
        self.assertEqual(prismatic_joint.GetBody1Rel().GetTargets(), ["/prismatic_joints/Geometry/BaseLink/Arm_1/Arm_2"])
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1.1, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assert_rotation_almost_equal(Gf.Rotation(prismatic_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)
        self.assert_rotation_almost_equal(Gf.Rotation(prismatic_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)
        self.assertEqual(prismatic_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.x)
        self.assertAlmostEqual(prismatic_joint.GetLowerLimitAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(prismatic_joint.GetUpperLimitAttr().Get(), 0.5, places=6)

        physics_prismatic_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_arm_2"))
        self.assertTrue(physics_prismatic_joint_prim.IsValid())
        self.assertTrue(physics_prismatic_joint_prim.IsA(UsdPhysics.PrismaticJoint))
        prismatic_joint = UsdPhysics.PrismaticJoint(physics_prismatic_joint_prim)
        self.assertEqual(prismatic_joint.GetBody0Rel().GetTargets(), ["/prismatic_joints/Geometry/BaseLink/Arm_1/Arm_2"])
        self.assertEqual(prismatic_joint.GetBody1Rel().GetTargets(), ["/prismatic_joints/Geometry/BaseLink/Arm_1/Arm_2/Arm_3"])
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1.1, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assert_rotation_almost_equal(Gf.Rotation(prismatic_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)
        self.assert_rotation_almost_equal(Gf.Rotation(prismatic_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)
        self.assertEqual(prismatic_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.x)
        self.assertAlmostEqual(prismatic_joint.GetLowerLimitAttr().Get(), -0.2, places=6)
        self.assertAlmostEqual(prismatic_joint.GetUpperLimitAttr().Get(), 0.0, places=6)

        physics_prismatic_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_arm_3"))
        self.assertTrue(physics_prismatic_joint_prim.IsValid())
        self.assertTrue(physics_prismatic_joint_prim.IsA(UsdPhysics.PrismaticJoint))
        prismatic_joint = UsdPhysics.PrismaticJoint(physics_prismatic_joint_prim)
        self.assertEqual(prismatic_joint.GetBody0Rel().GetTargets(), ["/prismatic_joints/Geometry/BaseLink/Arm_1/Arm_2/Arm_3"])
        self.assertEqual(prismatic_joint.GetBody1Rel().GetTargets(), ["/prismatic_joints/Geometry/BaseLink/Arm_1/Arm_2/Arm_3/Arm_4"])
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1.1, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assert_rotation_almost_equal(Gf.Rotation(prismatic_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)
        self.assert_rotation_almost_equal(Gf.Rotation(prismatic_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)
        self.assertEqual(prismatic_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.x)
        self.assertAlmostEqual(prismatic_joint.GetLowerLimitAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(prismatic_joint.GetUpperLimitAttr().Get(), 0.5, places=6)

    def test_fixed_planar_joints(self):
        input_path = "tests/data/fixed_planar_joints.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        default_prim = stage.GetDefaultPrim()
        self.assertTrue(default_prim.IsValid())
        default_prim_path = default_prim.GetPath()

        physics_scope_prim = stage.GetPrimAtPath(default_prim_path.AppendChild("Physics"))
        self.assertTrue(physics_scope_prim.IsValid())

        # Planar joint(UsdPhysics.Joint): Axis (0, 0, 1).
        physics_planar_axis_z_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_planar_axis_z"))
        self.assertTrue(physics_planar_axis_z_prim.IsValid())
        self.assertTrue(physics_planar_axis_z_prim.IsA(UsdPhysics.Joint))
        physics_joint = UsdPhysics.Joint(physics_planar_axis_z_prim)
        self.assertEqual(physics_joint.GetBody0Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1"])
        self.assertEqual(physics_joint.GetBody1Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1/planar_z"])
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0.35), 1e-6))
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)

        self.assertFalse(physics_planar_axis_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transX))
        self.assertFalse(physics_planar_axis_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transY))
        self.assertTrue(physics_planar_axis_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transZ))
        self.assertTrue(physics_planar_axis_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotX))
        self.assertTrue(physics_planar_axis_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotY))
        self.assertFalse(physics_planar_axis_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotZ))

        limit_api_trans_z: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_z_prim.GetPrim(), UsdPhysics.Tokens.transZ)
        self.assertAlmostEqual(limit_api_trans_z.GetLowAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(limit_api_trans_z.GetHighAttr().Get(), 0.0, places=6)
        limit_api_rot_x: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_z_prim.GetPrim(), UsdPhysics.Tokens.rotX)
        self.assertAlmostEqual(limit_api_rot_x.GetLowAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(limit_api_rot_x.GetHighAttr().Get(), 0.0, places=6)
        limit_api_rot_y: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_z_prim.GetPrim(), UsdPhysics.Tokens.rotY)
        self.assertAlmostEqual(limit_api_rot_y.GetLowAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(limit_api_rot_y.GetHighAttr().Get(), 0.0, places=6)

        # Planar joint(UsdPhysics.Joint): Axis (0, 0, -1).
        physics_planar_axis_negative_z_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_planar_axis_negative_z"))
        self.assertTrue(physics_planar_axis_negative_z_prim.IsValid())
        self.assertTrue(physics_planar_axis_negative_z_prim.IsA(UsdPhysics.Joint))
        physics_joint = UsdPhysics.Joint(physics_planar_axis_negative_z_prim)
        self.assertEqual(physics_joint.GetBody0Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1"])
        self.assertEqual(physics_joint.GetBody1Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1/planar_negative_z"])
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0.7), 1e-6))
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(0, Gf.Vec3f(-1, 0, 0))), 1e-6)
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(0, Gf.Vec3f(-1, 0, 0))), 1e-6)

        self.assertFalse(physics_planar_axis_negative_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transX))
        self.assertFalse(physics_planar_axis_negative_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transY))
        self.assertTrue(physics_planar_axis_negative_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transZ))
        self.assertTrue(physics_planar_axis_negative_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotX))
        self.assertTrue(physics_planar_axis_negative_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotY))
        self.assertFalse(physics_planar_axis_negative_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotZ))

        limit_api_trans_z: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_z_prim.GetPrim(), UsdPhysics.Tokens.transZ)
        self.assertAlmostEqual(limit_api_trans_z.GetLowAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(limit_api_trans_z.GetHighAttr().Get(), 0.0, places=6)
        limit_api_rot_x: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_z_prim.GetPrim(), UsdPhysics.Tokens.rotX)
        self.assertAlmostEqual(limit_api_rot_x.GetLowAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(limit_api_rot_x.GetHighAttr().Get(), 0.0, places=6)
        limit_api_rot_y: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_z_prim.GetPrim(), UsdPhysics.Tokens.rotY)
        self.assertAlmostEqual(limit_api_rot_y.GetLowAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(limit_api_rot_y.GetHighAttr().Get(), 0.0, places=6)

        # Planar joint(UsdPhysics.Joint): Axis (0, 1, 0).
        physics_planar_axis_y_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_planar_axis_y"))
        self.assertTrue(physics_planar_axis_y_prim.IsValid())
        self.assertTrue(physics_planar_axis_y_prim.IsA(UsdPhysics.Joint))
        physics_joint = UsdPhysics.Joint(physics_planar_axis_y_prim)
        self.assertEqual(physics_joint.GetBody0Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1"])
        self.assertEqual(physics_joint.GetBody1Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1/planar_y"])
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 1, 0), 1e-6))
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)

        self.assertFalse(physics_planar_axis_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transX))
        self.assertTrue(physics_planar_axis_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transY))
        self.assertFalse(physics_planar_axis_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transZ))
        self.assertTrue(physics_planar_axis_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotX))
        self.assertFalse(physics_planar_axis_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotY))
        self.assertTrue(physics_planar_axis_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotZ))

        limit_api_trans_y: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_y_prim.GetPrim(), UsdPhysics.Tokens.transY)
        self.assertAlmostEqual(limit_api_trans_y.GetLowAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(limit_api_trans_y.GetHighAttr().Get(), 0.0, places=6)
        limit_api_rot_x: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_y_prim.GetPrim(), UsdPhysics.Tokens.rotX)
        self.assertAlmostEqual(limit_api_rot_x.GetLowAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(limit_api_rot_x.GetHighAttr().Get(), 0.0, places=6)
        limit_api_rot_z: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_y_prim.GetPrim(), UsdPhysics.Tokens.rotZ)
        self.assertAlmostEqual(limit_api_rot_z.GetLowAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(limit_api_rot_z.GetHighAttr().Get(), 0.0, places=6)

        # Planar joint(UsdPhysics.Joint): Axis (0, -1, 0).
        physics_planar_axis_negative_y_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_planar_axis_negative_y"))
        self.assertTrue(physics_planar_axis_negative_y_prim.IsValid())
        self.assertTrue(physics_planar_axis_negative_y_prim.IsA(UsdPhysics.Joint))
        physics_joint = UsdPhysics.Joint(physics_planar_axis_negative_y_prim)
        self.assertEqual(physics_joint.GetBody0Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1"])
        self.assertEqual(physics_joint.GetBody1Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1/planar_negative_y"])
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 2, 0), 1e-6))
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(0, Gf.Vec3f(-1, 0, 0))), 1e-6)
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(0, Gf.Vec3f(-1, 0, 0))), 1e-6)

        self.assertFalse(physics_planar_axis_negative_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transX))
        self.assertTrue(physics_planar_axis_negative_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transY))
        self.assertFalse(physics_planar_axis_negative_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transZ))
        self.assertTrue(physics_planar_axis_negative_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotX))
        self.assertFalse(physics_planar_axis_negative_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotY))
        self.assertTrue(physics_planar_axis_negative_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotZ))

        limit_api_trans_y: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_y_prim.GetPrim(), UsdPhysics.Tokens.transY)
        self.assertAlmostEqual(limit_api_trans_y.GetLowAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(limit_api_trans_y.GetHighAttr().Get(), 0.0, places=6)
        limit_api_rot_x: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_y_prim.GetPrim(), UsdPhysics.Tokens.rotX)
        self.assertAlmostEqual(limit_api_rot_x.GetLowAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(limit_api_rot_x.GetHighAttr().Get(), 0.0, places=6)
        limit_api_rot_z: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_y_prim.GetPrim(), UsdPhysics.Tokens.rotZ)
        self.assertAlmostEqual(limit_api_rot_z.GetLowAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(limit_api_rot_z.GetHighAttr().Get(), 0.0, places=6)

        # Planar joint(UsdPhysics.Joint): Axis (1, 0, 0).
        physics_planar_axis_x_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_planar_axis_x"))
        self.assertTrue(physics_planar_axis_x_prim.IsValid())
        self.assertTrue(physics_planar_axis_x_prim.IsA(UsdPhysics.Joint))
        physics_joint = UsdPhysics.Joint(physics_planar_axis_x_prim)
        self.assertEqual(physics_joint.GetBody0Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1"])
        self.assertEqual(physics_joint.GetBody1Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1/planar_x"])
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(-1, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)

        self.assertTrue(physics_planar_axis_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transX))
        self.assertFalse(physics_planar_axis_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transY))
        self.assertFalse(physics_planar_axis_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transZ))
        self.assertFalse(physics_planar_axis_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotX))
        self.assertTrue(physics_planar_axis_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotY))
        self.assertTrue(physics_planar_axis_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotZ))

        limit_api_trans_x: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_x_prim.GetPrim(), UsdPhysics.Tokens.transX)
        self.assertAlmostEqual(limit_api_trans_x.GetLowAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(limit_api_trans_x.GetHighAttr().Get(), 0.0, places=6)
        limit_api_rot_y: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_x_prim.GetPrim(), UsdPhysics.Tokens.rotY)
        self.assertAlmostEqual(limit_api_rot_y.GetLowAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(limit_api_rot_y.GetHighAttr().Get(), 0.0, places=6)
        limit_api_rot_z: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_x_prim.GetPrim(), UsdPhysics.Tokens.rotZ)
        self.assertAlmostEqual(limit_api_rot_z.GetLowAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(limit_api_rot_z.GetHighAttr().Get(), 0.0, places=6)

        # Planar joint(UsdPhysics.Joint): Axis (-1, 0, 0).
        physics_planar_axis_negative_x_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_planar_axis_negative_x"))
        self.assertTrue(physics_planar_axis_negative_x_prim.IsValid())
        self.assertTrue(physics_planar_axis_negative_x_prim.IsA(UsdPhysics.Joint))
        physics_joint = UsdPhysics.Joint(physics_planar_axis_negative_x_prim)
        self.assertEqual(physics_joint.GetBody0Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1"])
        self.assertEqual(physics_joint.GetBody1Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1/planar_negative_x"])
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(-2, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(0, Gf.Vec3f(0, -1, 0))), 1e-6)
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(0, Gf.Vec3f(0, -1, 0))), 1e-6)

        self.assertTrue(physics_planar_axis_negative_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transX))
        self.assertFalse(physics_planar_axis_negative_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transY))
        self.assertFalse(physics_planar_axis_negative_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transZ))
        self.assertFalse(physics_planar_axis_negative_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotX))
        self.assertTrue(physics_planar_axis_negative_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotY))
        self.assertTrue(physics_planar_axis_negative_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotZ))

        limit_api_trans_x: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_x_prim.GetPrim(), UsdPhysics.Tokens.transX)
        self.assertAlmostEqual(limit_api_trans_x.GetLowAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(limit_api_trans_x.GetHighAttr().Get(), 0.0, places=6)
        limit_api_rot_y: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_x_prim.GetPrim(), UsdPhysics.Tokens.rotY)
        self.assertAlmostEqual(limit_api_rot_y.GetLowAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(limit_api_rot_y.GetHighAttr().Get(), 0.0, places=6)
        limit_api_rot_z: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_x_prim.GetPrim(), UsdPhysics.Tokens.rotZ)
        self.assertAlmostEqual(limit_api_rot_z.GetLowAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(limit_api_rot_z.GetHighAttr().Get(), 0.0, places=6)

        # Planar joint(UsdPhysics.Joint): Axis (0.2 0.3 1).
        physics_planar_axis_tilt_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_planar_axis_tilt"))
        self.assertTrue(physics_planar_axis_tilt_prim.IsValid())
        self.assertTrue(physics_planar_axis_tilt_prim.IsA(UsdPhysics.Joint))
        physics_joint = UsdPhysics.Joint(physics_planar_axis_tilt_prim)
        self.assertEqual(physics_joint.GetBody0Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1"])
        self.assertEqual(physics_joint.GetBody1Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1/planar_tilt"])
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assert_rotation_almost_equal(
            Gf.Rotation(physics_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(0.7707607, Gf.Vec3f(0, -0.6102548, 0.18307644))), 1e-6
        )
        self.assert_rotation_almost_equal(
            Gf.Rotation(physics_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(0.7707607, Gf.Vec3f(0, -0.6102548, 0.18307644))), 1e-6
        )

        self.assertTrue(physics_planar_axis_tilt_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transX))
        self.assertFalse(physics_planar_axis_tilt_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transY))
        self.assertFalse(physics_planar_axis_tilt_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transZ))
        self.assertFalse(physics_planar_axis_tilt_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotX))
        self.assertTrue(physics_planar_axis_tilt_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotY))
        self.assertTrue(physics_planar_axis_tilt_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotZ))

        # Planar joint(UsdPhysics.Joint): Axis (0, 0, 0).
        physics_planar_axis_tilt_axis_len_0_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_planar_axis_tilt_axis_len_0"))
        self.assertTrue(physics_planar_axis_tilt_axis_len_0_prim.IsValid())
        self.assertTrue(physics_planar_axis_tilt_axis_len_0_prim.IsA(UsdPhysics.Joint))
        physics_joint = UsdPhysics.Joint(physics_planar_axis_tilt_axis_len_0_prim)
        self.assertEqual(physics_joint.GetBody0Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1"])
        self.assertEqual(physics_joint.GetBody1Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1/planar_tilt_axis_len_0"])
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1, 0, 0.5), 1e-6))
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot0Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)
        self.assert_rotation_almost_equal(Gf.Rotation(physics_joint.GetLocalRot1Attr().Get()), Gf.Rotation(Gf.Quatf(1, Gf.Vec3f(0, 0, 0))), 1e-6)

        self.assertTrue(physics_planar_axis_tilt_axis_len_0_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transX))
        self.assertFalse(physics_planar_axis_tilt_axis_len_0_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transY))
        self.assertFalse(physics_planar_axis_tilt_axis_len_0_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transZ))
        self.assertFalse(physics_planar_axis_tilt_axis_len_0_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotX))
        self.assertTrue(physics_planar_axis_tilt_axis_len_0_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotY))
        self.assertTrue(physics_planar_axis_tilt_axis_len_0_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotZ))

    def test_fixed_floating_joints(self):
        input_path = "tests/data/fixed_floating_joints.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Floating joints are not supported.*"),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())
