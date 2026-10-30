# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import math
import pathlib

import usdex.core
import usdex.test
from pxr import Gf, Tf, Usd, UsdPhysics

import urdf_usd_converter
from tests.util.ConverterTestCase import ConverterTestCase


class TestJoints(ConverterTestCase):
    def test_revolute_joints(self):
        input_path = "tests/data/revolute_joints.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        revolute_arms = [("Arm_1", "revolute"), ("Arm_2", "revolute"), ("Arm_3", "revolute"), ("Arm_4", "revolute")]
        with self.expect_no_mass_source(revolute_arms):
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

        # root_joint
        # This is a fixed joint connecting the world to the first link.
        physics_root_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("root_joint"))
        self.assertTrue(physics_root_joint_prim.IsValid())
        self.assertTrue(physics_root_joint_prim.IsA(UsdPhysics.FixedJoint))
        root_joint = UsdPhysics.FixedJoint(physics_root_joint_prim)
        self.assertEqual(root_joint.GetBody0Rel().GetTargets(), ["/revolute_joints"])
        self.assertEqual(root_joint.GetBody1Rel().GetTargets(), ["/revolute_joints/Geometry/BaseLink"])
        self.assertTrue(Gf.IsClose(root_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(root_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(root_joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(root_joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))

        # Joint_root.
        physics_revolute_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_root"))
        self.assertTrue(physics_revolute_joint_prim.IsValid())
        self.assertTrue(physics_revolute_joint_prim.IsA(UsdPhysics.RevoluteJoint))
        revolute_joint = UsdPhysics.RevoluteJoint(physics_revolute_joint_prim)
        self.assertEqual(revolute_joint.GetBody0Rel().GetTargets(), ["/revolute_joints/Geometry/BaseLink"])
        self.assertEqual(revolute_joint.GetBody1Rel().GetTargets(), ["/revolute_joints/Geometry/BaseLink/Arm_1"])
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0.15, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(revolute_joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(revolute_joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertEqual(revolute_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.y)

        # Custom attributes. This joint does not have a limit specified.
        self.assertFalse(revolute_joint.GetPrim().GetAttribute("urdf:limit:effort").HasAuthoredValue())
        self.assertFalse(revolute_joint.GetPrim().HasAPI("NewtonJointAPI"))

        # Joint_arm_1.
        physics_revolute_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_arm_1"))
        self.assertTrue(physics_revolute_joint_prim.IsValid())
        self.assertTrue(physics_revolute_joint_prim.IsA(UsdPhysics.RevoluteJoint))
        revolute_joint = UsdPhysics.RevoluteJoint(physics_revolute_joint_prim)
        self.assertEqual(revolute_joint.GetBody0Rel().GetTargets(), ["/revolute_joints/Geometry/BaseLink/Arm_1"])
        self.assertEqual(revolute_joint.GetBody1Rel().GetTargets(), ["/revolute_joints/Geometry/BaseLink/Arm_1/Arm_2"])
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1.1, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(revolute_joint.GetLocalRot0Attr().Get(), Gf.Quatf(0.9887711, 0, 0, 0.14943811))
        self.assertRotationsAlmostEqual(revolute_joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertEqual(revolute_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.y)
        self.assertAlmostEqual(revolute_joint.GetLowerLimitAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(revolute_joint.GetUpperLimitAttr().Get(), 17.188734, places=6)

        # Custom attributes. Velocity is omitted, so no velocity limit is authored.
        joint_prim = revolute_joint.GetPrim()
        self.assertTrue(joint_prim.GetAttribute("urdf:limit:effort").HasAuthoredValue())
        self.assertTrue(joint_prim.GetAttribute("urdf:limit:effort").IsCustom())
        self.assertAlmostEqual(joint_prim.GetAttribute("urdf:limit:effort").Get(), 0.0, places=6)
        self.assertFalse(joint_prim.HasAPI("NewtonJointAPI"))
        self.assertFalse(joint_prim.GetAttribute("newton:velocityLimit").HasAuthoredValue())
        physics_revolute_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_arm_2"))
        self.assertTrue(physics_revolute_joint_prim.IsValid())
        self.assertTrue(physics_revolute_joint_prim.IsA(UsdPhysics.RevoluteJoint))
        revolute_joint = UsdPhysics.RevoluteJoint(physics_revolute_joint_prim)
        self.assertEqual(revolute_joint.GetBody0Rel().GetTargets(), ["/revolute_joints/Geometry/BaseLink/Arm_1/Arm_2"])
        self.assertEqual(revolute_joint.GetBody1Rel().GetTargets(), ["/revolute_joints/Geometry/BaseLink/Arm_1/Arm_2/Arm_3"])
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1.1, 0, 0.1), 1e-6))
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(revolute_joint.GetLocalRot0Attr().Get(), Gf.Quatf(0.9859915, -0.011197348, -0.14901796, -0.07408829))
        self.assertRotationsAlmostEqual(revolute_joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertEqual(revolute_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.y)
        self.assertAlmostEqual(revolute_joint.GetLowerLimitAttr().Get(), -17.188734, places=6)
        self.assertAlmostEqual(revolute_joint.GetUpperLimitAttr().Get(), 0.0, places=6)

        # Custom attributes. Velocity is omitted, so no velocity limit is authored.
        joint_prim = revolute_joint.GetPrim()
        self.assertTrue(joint_prim.GetAttribute("urdf:limit:effort").HasAuthoredValue())
        self.assertTrue(joint_prim.GetAttribute("urdf:limit:effort").IsCustom())
        self.assertAlmostEqual(joint_prim.GetAttribute("urdf:limit:effort").Get(), 0.0, places=6)
        self.assertFalse(joint_prim.HasAPI("NewtonJointAPI"))
        self.assertFalse(joint_prim.GetAttribute("newton:velocityLimit").HasAuthoredValue())

        # Joint_arm_3.
        physics_revolute_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_arm_3"))
        self.assertTrue(physics_revolute_joint_prim.IsValid())
        self.assertTrue(physics_revolute_joint_prim.IsA(UsdPhysics.RevoluteJoint))
        revolute_joint = UsdPhysics.RevoluteJoint(physics_revolute_joint_prim)
        self.assertEqual(revolute_joint.GetBody0Rel().GetTargets(), ["/revolute_joints/Geometry/BaseLink/Arm_1/Arm_2/Arm_3"])
        self.assertEqual(revolute_joint.GetBody1Rel().GetTargets(), ["/revolute_joints/Geometry/BaseLink/Arm_1/Arm_2/Arm_3/Arm_4"])
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1.1, 0.1, -1.110223e-16), 1e-6))
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(revolute_joint.GetLocalRot0Attr().Get(), Gf.Quatf(0.9971888, -0.07492986, 9.7144515e-17, -6.2450045e-17))
        self.assertRotationsAlmostEqual(revolute_joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertEqual(revolute_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.y)
        self.assertAlmostEqual(revolute_joint.GetLowerLimitAttr().Get(), -17.188734, places=6)
        self.assertAlmostEqual(revolute_joint.GetUpperLimitAttr().Get(), 28.64789, places=6)

        # Custom attributes and NewtonJointAPI.
        joint_prim = revolute_joint.GetPrim()
        self.assertTrue(joint_prim.GetAttribute("urdf:limit:effort").HasAuthoredValue())
        self.assertTrue(joint_prim.GetAttribute("urdf:limit:effort").IsCustom())
        self.assertAlmostEqual(joint_prim.GetAttribute("urdf:limit:effort").Get(), 0.01, places=6)
        self.assertTrue(joint_prim.HasAPI("NewtonJointAPI"))
        self.assertTrue(joint_prim.GetAttribute("newton:velocityLimit").HasAuthoredValue())
        self.assertAlmostEqual(joint_prim.GetAttribute("newton:velocityLimit").Get(), math.degrees(0.02), places=6)
        self.assertTrue(joint_prim.GetAttribute("newton:damping").HasAuthoredValue())
        self.assertAlmostEqual(joint_prim.GetAttribute("newton:damping").Get(), 1.0 * math.pi / 180.0, places=6)
        self.assertTrue(joint_prim.GetAttribute("newton:friction").HasAuthoredValue())
        self.assertAlmostEqual(joint_prim.GetAttribute("newton:friction").Get(), 0.5, places=6)

    def test_fixed_continuous_joints(self):
        input_path = "tests/data/fixed_continuous_joints.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        with self.expect_no_mass_source([("Arm_2", "continuous")]):
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

        # Joint_root.
        physics_fixed_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_root"))
        self.assertTrue(physics_fixed_joint_prim.IsValid())
        self.assertTrue(physics_fixed_joint_prim.IsA(UsdPhysics.FixedJoint))
        fixed_joint = UsdPhysics.FixedJoint(physics_fixed_joint_prim)
        self.assertEqual(fixed_joint.GetBody0Rel().GetTargets(), ["/fixed_continuous_joints/Geometry/BaseLink"])
        self.assertEqual(fixed_joint.GetBody1Rel().GetTargets(), ["/fixed_continuous_joints/Geometry/BaseLink/Arm_1"])
        self.assertTrue(Gf.IsClose(fixed_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0.15, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(fixed_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(fixed_joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(fixed_joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))

        # Joint_arm_1.
        physics_revolute_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_arm_1"))
        self.assertTrue(physics_revolute_joint_prim.IsValid())
        self.assertTrue(physics_revolute_joint_prim.IsA(UsdPhysics.RevoluteJoint))
        revolute_joint = UsdPhysics.RevoluteJoint(physics_revolute_joint_prim)
        self.assertEqual(revolute_joint.GetBody0Rel().GetTargets(), ["/fixed_continuous_joints/Geometry/BaseLink/Arm_1"])
        self.assertEqual(revolute_joint.GetBody1Rel().GetTargets(), ["/fixed_continuous_joints/Geometry/BaseLink/Arm_1/Arm_2"])
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1.1, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(revolute_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(revolute_joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(revolute_joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertEqual(revolute_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.x)
        self.assertFalse(revolute_joint.GetLowerLimitAttr().HasAuthoredValue())
        self.assertFalse(revolute_joint.GetUpperLimitAttr().HasAuthoredValue())

        # Custom attributes. This joint does not have a limit specified.
        self.assertFalse(revolute_joint.GetPrim().GetAttribute("urdf:limit:effort").HasAuthoredValue())
        self.assertFalse(revolute_joint.GetPrim().HasAPI("NewtonJointAPI"))

    def test_fixed_prismatic_joints(self):
        input_path = "tests/data/prismatic_joints.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        prismatic_arms = [("Arm_1", "prismatic"), ("Arm_2", "prismatic"), ("Arm_3", "prismatic"), ("Arm_4", "prismatic")]
        with self.expect_no_mass_source(prismatic_arms):
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

        # Joint_root.
        physics_prismatic_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_root"))
        self.assertTrue(physics_prismatic_joint_prim.IsValid())
        self.assertTrue(physics_prismatic_joint_prim.IsA(UsdPhysics.PrismaticJoint))
        prismatic_joint = UsdPhysics.PrismaticJoint(physics_prismatic_joint_prim)
        self.assertEqual(prismatic_joint.GetBody0Rel().GetTargets(), ["/prismatic_joints/Geometry/BaseLink"])
        self.assertEqual(prismatic_joint.GetBody1Rel().GetTargets(), ["/prismatic_joints/Geometry/BaseLink/Arm_1"])
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0.15, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(prismatic_joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(prismatic_joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertEqual(prismatic_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.x)
        self.assertAlmostEqual(prismatic_joint.GetLowerLimitAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(prismatic_joint.GetUpperLimitAttr().Get(), 0.0, places=6)

        # Custom attributes. This joint does not have a limit specified.
        self.assertFalse(prismatic_joint.GetPrim().GetAttribute("urdf:limit:effort").HasAuthoredValue())
        self.assertFalse(prismatic_joint.GetPrim().HasAPI("NewtonJointAPI"))

        # Joint_arm_1.
        physics_prismatic_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_arm_1"))
        self.assertTrue(physics_prismatic_joint_prim.IsValid())
        self.assertTrue(physics_prismatic_joint_prim.IsA(UsdPhysics.PrismaticJoint))
        prismatic_joint = UsdPhysics.PrismaticJoint(physics_prismatic_joint_prim)
        self.assertEqual(prismatic_joint.GetBody0Rel().GetTargets(), ["/prismatic_joints/Geometry/BaseLink/Arm_1"])
        self.assertEqual(prismatic_joint.GetBody1Rel().GetTargets(), ["/prismatic_joints/Geometry/BaseLink/Arm_1/Arm_2"])
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1.1, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(prismatic_joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(prismatic_joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertEqual(prismatic_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.x)
        self.assertAlmostEqual(prismatic_joint.GetLowerLimitAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(prismatic_joint.GetUpperLimitAttr().Get(), 0.5, places=6)

        # Custom attributes. Velocity is omitted, so no velocity limit is authored.
        joint_prim = prismatic_joint.GetPrim()
        self.assertTrue(joint_prim.GetAttribute("urdf:limit:effort").HasAuthoredValue())
        self.assertTrue(joint_prim.GetAttribute("urdf:limit:effort").IsCustom())
        self.assertAlmostEqual(joint_prim.GetAttribute("urdf:limit:effort").Get(), 0.01, places=6)
        self.assertFalse(joint_prim.HasAPI("NewtonJointAPI"))
        self.assertFalse(joint_prim.GetAttribute("newton:velocityLimit").HasAuthoredValue())

        # Joint_arm_2.
        physics_prismatic_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_arm_2"))
        self.assertTrue(physics_prismatic_joint_prim.IsValid())
        self.assertTrue(physics_prismatic_joint_prim.IsA(UsdPhysics.PrismaticJoint))
        prismatic_joint = UsdPhysics.PrismaticJoint(physics_prismatic_joint_prim)
        self.assertEqual(prismatic_joint.GetBody0Rel().GetTargets(), ["/prismatic_joints/Geometry/BaseLink/Arm_1/Arm_2"])
        self.assertEqual(prismatic_joint.GetBody1Rel().GetTargets(), ["/prismatic_joints/Geometry/BaseLink/Arm_1/Arm_2/Arm_3"])
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1.1, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(prismatic_joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(prismatic_joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertEqual(prismatic_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.x)
        self.assertAlmostEqual(prismatic_joint.GetLowerLimitAttr().Get(), -0.2, places=6)
        self.assertAlmostEqual(prismatic_joint.GetUpperLimitAttr().Get(), 0.0, places=6)

        # Custom attributes and NewtonJointAPI.
        joint_prim = prismatic_joint.GetPrim()
        self.assertTrue(joint_prim.GetAttribute("urdf:limit:effort").HasAuthoredValue())
        self.assertTrue(joint_prim.GetAttribute("urdf:limit:effort").IsCustom())
        self.assertAlmostEqual(joint_prim.GetAttribute("urdf:limit:effort").Get(), 0.02, places=6)
        self.assertTrue(joint_prim.HasAPI("NewtonJointAPI"))
        self.assertTrue(joint_prim.GetAttribute("newton:velocityLimit").HasAuthoredValue())
        self.assertAlmostEqual(joint_prim.GetAttribute("newton:velocityLimit").Get(), 0.01, places=6)

        # Joint_arm_3.
        physics_prismatic_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_arm_3"))
        self.assertTrue(physics_prismatic_joint_prim.IsValid())
        self.assertTrue(physics_prismatic_joint_prim.IsA(UsdPhysics.PrismaticJoint))
        prismatic_joint = UsdPhysics.PrismaticJoint(physics_prismatic_joint_prim)
        self.assertEqual(prismatic_joint.GetBody0Rel().GetTargets(), ["/prismatic_joints/Geometry/BaseLink/Arm_1/Arm_2/Arm_3"])
        self.assertEqual(prismatic_joint.GetBody1Rel().GetTargets(), ["/prismatic_joints/Geometry/BaseLink/Arm_1/Arm_2/Arm_3/Arm_4"])
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1.1, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(prismatic_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(prismatic_joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(prismatic_joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertEqual(prismatic_joint.GetAxisAttr().Get(), UsdPhysics.Tokens.x)
        self.assertAlmostEqual(prismatic_joint.GetLowerLimitAttr().Get(), 0.0, places=6)
        self.assertAlmostEqual(prismatic_joint.GetUpperLimitAttr().Get(), 0.5, places=6)

        # Custom attributes and NewtonJointAPI.
        joint_prim = prismatic_joint.GetPrim()
        self.assertTrue(joint_prim.GetAttribute("urdf:limit:effort").HasAuthoredValue())
        self.assertTrue(joint_prim.GetAttribute("urdf:limit:effort").IsCustom())
        self.assertAlmostEqual(joint_prim.GetAttribute("urdf:limit:effort").Get(), 0.03, places=6)
        self.assertTrue(joint_prim.HasAPI("NewtonJointAPI"))
        self.assertTrue(joint_prim.GetAttribute("newton:velocityLimit").HasAuthoredValue())
        self.assertAlmostEqual(joint_prim.GetAttribute("newton:velocityLimit").Get(), 0.01, places=6)
        self.assertTrue(joint_prim.GetAttribute("newton:damping").HasAuthoredValue())
        self.assertAlmostEqual(joint_prim.GetAttribute("newton:damping").Get(), 0.02, places=6)
        self.assertTrue(joint_prim.GetAttribute("newton:friction").HasAuthoredValue())
        self.assertAlmostEqual(joint_prim.GetAttribute("newton:friction").Get(), 0.03, places=6)

    def test_fixed_planar_joints(self):
        input_path = "tests/data/fixed_planar_joints.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        planar_links = [
            ("planar_negative_z", "planar"),
            ("planar_z", "planar"),
            ("planar_negative_y", "planar"),
            ("planar_y", "planar"),
            ("planar_negative_x", "planar"),
            ("planar_x", "planar"),
            ("planar_tilt", "planar"),
        ]
        with self.expect_no_mass_source(planar_links):
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
        self.assertRotationsAlmostEqual(physics_joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(physics_joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))

        self.assertFalse(physics_planar_axis_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transX))
        self.assertFalse(physics_planar_axis_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transY))
        self.assertTrue(physics_planar_axis_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transZ))
        self.assertTrue(physics_planar_axis_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotX))
        self.assertTrue(physics_planar_axis_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotY))
        self.assertFalse(physics_planar_axis_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotZ))

        limit_api_trans_z: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_z_prim.GetPrim(), UsdPhysics.Tokens.transZ)
        self.assertAlmostEqual(limit_api_trans_z.GetLowAttr().Get(), math.inf)
        self.assertAlmostEqual(limit_api_trans_z.GetHighAttr().Get(), -math.inf)
        limit_api_rot_x: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_z_prim.GetPrim(), UsdPhysics.Tokens.rotX)
        self.assertAlmostEqual(limit_api_rot_x.GetLowAttr().Get(), math.inf)
        self.assertAlmostEqual(limit_api_rot_x.GetHighAttr().Get(), -math.inf)
        limit_api_rot_y: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_z_prim.GetPrim(), UsdPhysics.Tokens.rotY)
        self.assertAlmostEqual(limit_api_rot_y.GetLowAttr().Get(), math.inf)
        self.assertAlmostEqual(limit_api_rot_y.GetHighAttr().Get(), -math.inf)

        # Planar joint(UsdPhysics.Joint): Axis (0, 0, -1).
        physics_planar_axis_negative_z_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_planar_axis_negative_z"))
        self.assertTrue(physics_planar_axis_negative_z_prim.IsValid())
        self.assertTrue(physics_planar_axis_negative_z_prim.IsA(UsdPhysics.Joint))
        physics_joint = UsdPhysics.Joint(physics_planar_axis_negative_z_prim)
        self.assertEqual(physics_joint.GetBody0Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1"])
        self.assertEqual(physics_joint.GetBody1Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1/planar_negative_z"])
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0.7), 1e-6))
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(physics_joint.GetLocalRot0Attr().Get(), Gf.Quatf(0, -1, 0, 0))
        self.assertRotationsAlmostEqual(physics_joint.GetLocalRot1Attr().Get(), Gf.Quatf(0, -1, 0, 0))

        self.assertFalse(physics_planar_axis_negative_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transX))
        self.assertFalse(physics_planar_axis_negative_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transY))
        self.assertTrue(physics_planar_axis_negative_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transZ))
        self.assertTrue(physics_planar_axis_negative_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotX))
        self.assertTrue(physics_planar_axis_negative_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotY))
        self.assertFalse(physics_planar_axis_negative_z_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotZ))

        limit_api_trans_z: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_z_prim.GetPrim(), UsdPhysics.Tokens.transZ)
        self.assertAlmostEqual(limit_api_trans_z.GetLowAttr().Get(), math.inf)
        self.assertAlmostEqual(limit_api_trans_z.GetHighAttr().Get(), -math.inf)
        limit_api_rot_x: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_z_prim.GetPrim(), UsdPhysics.Tokens.rotX)
        self.assertAlmostEqual(limit_api_rot_x.GetLowAttr().Get(), math.inf)
        self.assertAlmostEqual(limit_api_rot_x.GetHighAttr().Get(), -math.inf)
        limit_api_rot_y: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_z_prim.GetPrim(), UsdPhysics.Tokens.rotY)
        self.assertAlmostEqual(limit_api_rot_y.GetLowAttr().Get(), math.inf)
        self.assertAlmostEqual(limit_api_rot_y.GetHighAttr().Get(), -math.inf)

        # Planar joint(UsdPhysics.Joint): Axis (0, 1, 0).
        physics_planar_axis_y_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_planar_axis_y"))
        self.assertTrue(physics_planar_axis_y_prim.IsValid())
        self.assertTrue(physics_planar_axis_y_prim.IsA(UsdPhysics.Joint))
        physics_joint = UsdPhysics.Joint(physics_planar_axis_y_prim)
        self.assertEqual(physics_joint.GetBody0Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1"])
        self.assertEqual(physics_joint.GetBody1Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1/planar_y"])
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 1, 0), 1e-6))
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(physics_joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(physics_joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))

        self.assertFalse(physics_planar_axis_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transX))
        self.assertTrue(physics_planar_axis_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transY))
        self.assertFalse(physics_planar_axis_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transZ))
        self.assertTrue(physics_planar_axis_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotX))
        self.assertFalse(physics_planar_axis_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotY))
        self.assertTrue(physics_planar_axis_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotZ))

        limit_api_trans_y: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_y_prim.GetPrim(), UsdPhysics.Tokens.transY)
        self.assertAlmostEqual(limit_api_trans_y.GetLowAttr().Get(), math.inf)
        self.assertAlmostEqual(limit_api_trans_y.GetHighAttr().Get(), -math.inf)
        limit_api_rot_x: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_y_prim.GetPrim(), UsdPhysics.Tokens.rotX)
        self.assertAlmostEqual(limit_api_rot_x.GetLowAttr().Get(), math.inf)
        self.assertAlmostEqual(limit_api_rot_x.GetHighAttr().Get(), -math.inf)
        limit_api_rot_z: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_y_prim.GetPrim(), UsdPhysics.Tokens.rotZ)
        self.assertAlmostEqual(limit_api_rot_z.GetLowAttr().Get(), math.inf)
        self.assertAlmostEqual(limit_api_rot_z.GetHighAttr().Get(), -math.inf)

        # Planar joint(UsdPhysics.Joint): Axis (0, -1, 0).
        physics_planar_axis_negative_y_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_planar_axis_negative_y"))
        self.assertTrue(physics_planar_axis_negative_y_prim.IsValid())
        self.assertTrue(physics_planar_axis_negative_y_prim.IsA(UsdPhysics.Joint))
        physics_joint = UsdPhysics.Joint(physics_planar_axis_negative_y_prim)
        self.assertEqual(physics_joint.GetBody0Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1"])
        self.assertEqual(physics_joint.GetBody1Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1/planar_negative_y"])
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 2, 0), 1e-6))
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(physics_joint.GetLocalRot0Attr().Get(), Gf.Quatf(0, -1, 0, 0))
        self.assertRotationsAlmostEqual(physics_joint.GetLocalRot1Attr().Get(), Gf.Quatf(0, -1, 0, 0))

        self.assertFalse(physics_planar_axis_negative_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transX))
        self.assertTrue(physics_planar_axis_negative_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transY))
        self.assertFalse(physics_planar_axis_negative_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transZ))
        self.assertTrue(physics_planar_axis_negative_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotX))
        self.assertFalse(physics_planar_axis_negative_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotY))
        self.assertTrue(physics_planar_axis_negative_y_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotZ))

        limit_api_trans_y: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_y_prim.GetPrim(), UsdPhysics.Tokens.transY)
        self.assertAlmostEqual(limit_api_trans_y.GetLowAttr().Get(), math.inf)
        self.assertAlmostEqual(limit_api_trans_y.GetHighAttr().Get(), -math.inf)
        limit_api_rot_x: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_y_prim.GetPrim(), UsdPhysics.Tokens.rotX)
        self.assertAlmostEqual(limit_api_rot_x.GetLowAttr().Get(), math.inf)
        self.assertAlmostEqual(limit_api_rot_x.GetHighAttr().Get(), -math.inf)
        limit_api_rot_z: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_y_prim.GetPrim(), UsdPhysics.Tokens.rotZ)
        self.assertAlmostEqual(limit_api_rot_z.GetLowAttr().Get(), math.inf)
        self.assertAlmostEqual(limit_api_rot_z.GetHighAttr().Get(), -math.inf)

        # Planar joint(UsdPhysics.Joint): Axis (1, 0, 0).
        physics_planar_axis_x_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_planar_axis_x"))
        self.assertTrue(physics_planar_axis_x_prim.IsValid())
        self.assertTrue(physics_planar_axis_x_prim.IsA(UsdPhysics.Joint))
        physics_joint = UsdPhysics.Joint(physics_planar_axis_x_prim)
        self.assertEqual(physics_joint.GetBody0Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1"])
        self.assertEqual(physics_joint.GetBody1Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1/planar_x"])
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(-1, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(physics_joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
        self.assertRotationsAlmostEqual(physics_joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))

        self.assertTrue(physics_planar_axis_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transX))
        self.assertFalse(physics_planar_axis_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transY))
        self.assertFalse(physics_planar_axis_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transZ))
        self.assertFalse(physics_planar_axis_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotX))
        self.assertTrue(physics_planar_axis_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotY))
        self.assertTrue(physics_planar_axis_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotZ))

        limit_api_trans_x: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_x_prim.GetPrim(), UsdPhysics.Tokens.transX)
        self.assertAlmostEqual(limit_api_trans_x.GetLowAttr().Get(), math.inf)
        self.assertAlmostEqual(limit_api_trans_x.GetHighAttr().Get(), -math.inf)
        limit_api_rot_y: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_x_prim.GetPrim(), UsdPhysics.Tokens.rotY)
        self.assertAlmostEqual(limit_api_rot_y.GetLowAttr().Get(), math.inf)
        self.assertAlmostEqual(limit_api_rot_y.GetHighAttr().Get(), -math.inf)
        limit_api_rot_z: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_x_prim.GetPrim(), UsdPhysics.Tokens.rotZ)
        self.assertAlmostEqual(limit_api_rot_z.GetLowAttr().Get(), math.inf)
        self.assertAlmostEqual(limit_api_rot_z.GetHighAttr().Get(), -math.inf)

        # Planar joint(UsdPhysics.Joint): Axis (-1, 0, 0).
        physics_planar_axis_negative_x_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_planar_axis_negative_x"))
        self.assertTrue(physics_planar_axis_negative_x_prim.IsValid())
        self.assertTrue(physics_planar_axis_negative_x_prim.IsA(UsdPhysics.Joint))
        physics_joint = UsdPhysics.Joint(physics_planar_axis_negative_x_prim)
        self.assertEqual(physics_joint.GetBody0Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1"])
        self.assertEqual(physics_joint.GetBody1Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1/planar_negative_x"])
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(-2, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(physics_joint.GetLocalRot0Attr().Get(), Gf.Quatf(0, Gf.Vec3f(0, -1, 0)))
        self.assertRotationsAlmostEqual(physics_joint.GetLocalRot1Attr().Get(), Gf.Quatf(0, Gf.Vec3f(0, -1, 0)))

        self.assertTrue(physics_planar_axis_negative_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transX))
        self.assertFalse(physics_planar_axis_negative_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transY))
        self.assertFalse(physics_planar_axis_negative_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transZ))
        self.assertFalse(physics_planar_axis_negative_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotX))
        self.assertTrue(physics_planar_axis_negative_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotY))
        self.assertTrue(physics_planar_axis_negative_x_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotZ))

        limit_api_trans_x: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_x_prim.GetPrim(), UsdPhysics.Tokens.transX)
        self.assertAlmostEqual(limit_api_trans_x.GetLowAttr().Get(), math.inf)
        self.assertAlmostEqual(limit_api_trans_x.GetHighAttr().Get(), -math.inf)
        limit_api_rot_y: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_x_prim.GetPrim(), UsdPhysics.Tokens.rotY)
        self.assertAlmostEqual(limit_api_rot_y.GetLowAttr().Get(), math.inf)
        self.assertAlmostEqual(limit_api_rot_y.GetHighAttr().Get(), -math.inf)
        limit_api_rot_z: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_negative_x_prim.GetPrim(), UsdPhysics.Tokens.rotZ)
        self.assertAlmostEqual(limit_api_rot_z.GetLowAttr().Get(), math.inf)
        self.assertAlmostEqual(limit_api_rot_z.GetHighAttr().Get(), -math.inf)

        # Planar joint(UsdPhysics.Joint): Axis (0.2 0.3 1).
        physics_planar_axis_tilt_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_planar_axis_tilt"))
        self.assertTrue(physics_planar_axis_tilt_prim.IsValid())
        self.assertTrue(physics_planar_axis_tilt_prim.IsA(UsdPhysics.Joint))
        physics_joint = UsdPhysics.Joint(physics_planar_axis_tilt_prim)
        self.assertEqual(physics_joint.GetBody0Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1"])
        self.assertEqual(physics_joint.GetBody1Rel().GetTargets(), ["/fixed_planar_joints/Geometry/BaseLink/link1/planar_tilt"])
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(1, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(physics_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(physics_joint.GetLocalRot0Attr().Get(), Gf.Quatf(0.7707607, Gf.Vec3f(0, -0.6102548, 0.18307644)))
        self.assertRotationsAlmostEqual(physics_joint.GetLocalRot1Attr().Get(), Gf.Quatf(0.7707607, Gf.Vec3f(0, -0.6102548, 0.18307644)))

        self.assertTrue(physics_planar_axis_tilt_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transX))
        self.assertFalse(physics_planar_axis_tilt_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transY))
        self.assertFalse(physics_planar_axis_tilt_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.transZ))
        self.assertFalse(physics_planar_axis_tilt_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotX))
        self.assertTrue(physics_planar_axis_tilt_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotY))
        self.assertTrue(physics_planar_axis_tilt_prim.HasAPI(UsdPhysics.LimitAPI, UsdPhysics.Tokens.rotZ))

        limit_api_trans_x: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_tilt_prim.GetPrim(), UsdPhysics.Tokens.transX)
        self.assertAlmostEqual(limit_api_trans_x.GetLowAttr().Get(), math.inf)
        self.assertAlmostEqual(limit_api_trans_x.GetHighAttr().Get(), -math.inf)
        limit_api_rot_y: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_tilt_prim.GetPrim(), UsdPhysics.Tokens.rotY)
        self.assertAlmostEqual(limit_api_rot_y.GetLowAttr().Get(), math.inf)
        self.assertAlmostEqual(limit_api_rot_y.GetHighAttr().Get(), -math.inf)
        limit_api_rot_z: UsdPhysics.LimitAPI = UsdPhysics.LimitAPI(physics_planar_axis_tilt_prim.GetPrim(), UsdPhysics.Tokens.rotZ)
        self.assertAlmostEqual(limit_api_rot_z.GetLowAttr().Get(), math.inf)
        self.assertAlmostEqual(limit_api_rot_z.GetHighAttr().Get(), -math.inf)

    def test_fixed_floating_joints(self):
        input_path = "tests/data/fixed_floating_joints.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        with self.expect_no_mass_source([("Arm_2", "floating")]):
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

        # root_joint
        # This is a fixed joint connecting the world to the first link.
        physics_root_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("root_joint"))
        self.assertTrue(physics_root_joint_prim.IsValid())
        self.assertTrue(physics_root_joint_prim.IsA(UsdPhysics.FixedJoint))
        root_joint = UsdPhysics.FixedJoint(physics_root_joint_prim)
        self.assertEqual(root_joint.GetBody0Rel().GetTargets(), ["/fixed_floating_joints"])
        self.assertEqual(root_joint.GetBody1Rel().GetTargets(), ["/fixed_floating_joints/Geometry/BaseLink"])
        self.assertTrue(Gf.IsClose(root_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(root_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(root_joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(root_joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))

        # Joint_root.
        physics_fixed_joint_prim = stage.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_root"))
        self.assertTrue(physics_fixed_joint_prim.IsValid())
        self.assertTrue(physics_fixed_joint_prim.IsA(UsdPhysics.FixedJoint))
        fixed_joint = UsdPhysics.FixedJoint(physics_fixed_joint_prim)
        self.assertEqual(fixed_joint.GetBody0Rel().GetTargets(), ["/fixed_floating_joints/Geometry/BaseLink"])
        self.assertEqual(fixed_joint.GetBody1Rel().GetTargets(), ["/fixed_floating_joints/Geometry/BaseLink/Arm_1"])
        self.assertTrue(Gf.IsClose(fixed_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0.15, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(fixed_joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(fixed_joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(fixed_joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))

        # The number of children of physics_scope_prim is 2.
        self.assertEqual(len(physics_scope_prim.GetChildren()), 2)

        # Confirm that Arm_2 is assigned as a rigid body.
        # This is a rigid body that is not participating in the joint structure.
        geometry_scope_prim = stage.GetPrimAtPath(default_prim_path.AppendChild("Geometry"))
        self.assertTrue(geometry_scope_prim.IsValid())

        arm2_prim_path = geometry_scope_prim.GetPath().AppendChild("BaseLink").AppendChild("Arm_1").AppendChild("Arm_2")
        arm_2_prim = stage.GetPrimAtPath(arm2_prim_path)
        self.assertTrue(arm_2_prim.IsValid())
        self.assertTrue(arm_2_prim.HasAPI(UsdPhysics.RigidBodyAPI))

    def test_mimic_joints(self):
        input_path = "tests/data/mimic_joint.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        mimic_links = [("linkA", "revolute"), ("linkB", "revolute"), ("linkC", "prismatic"), ("linkD", "prismatic")]
        with self.expect_no_mass_source(mimic_links):
            asset_path = converter.convert(input_path, output_dir)

        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        default_prim = stage.GetDefaultPrim()
        physics_scope_prim = stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Physics"))
        self.assertTrue(physics_scope_prim.IsValid())

        joint_b_prim = physics_scope_prim.GetChild("jointB")
        self.assertTrue(joint_b_prim.IsValid())
        self.assertTrue(joint_b_prim.IsA(UsdPhysics.RevoluteJoint))

        self.assertTrue(joint_b_prim.HasAPI("NewtonMimicAPI"))

        # 'newton:mimicEnabled' returns False because it has a default value.
        self.assertFalse(joint_b_prim.GetAttribute("newton:mimicEnabled").HasAuthoredValue())

        self.assertTrue(joint_b_prim.GetAttribute("newton:mimicCoef0").HasAuthoredValue())
        self.assertTrue(joint_b_prim.GetAttribute("newton:mimicCoef1").HasAuthoredValue())
        self.assertTrue(joint_b_prim.GetAttribute("newton:mimicEnabled").Get())
        # URDF authors the offset in the follower's position units -- radians for a
        # revolute joint -- while NewtonMimicAPI documents coef0 in degrees.
        self.assertAlmostEqual(joint_b_prim.GetAttribute("newton:mimicCoef0").Get(), math.degrees(0.5), places=4)
        # The multiplier is dimensionless and is authored unscaled.
        self.assertAlmostEqual(joint_b_prim.GetAttribute("newton:mimicCoef1").Get(), 2.0)

        self.assertTrue(joint_b_prim.HasRelationship("newton:mimicJoint"))
        rel = joint_b_prim.GetRelationship("newton:mimicJoint")
        self.assertEqual(len(rel.GetTargets()), 1)
        ref_joint = rel.GetTargets()[0]
        self.assertEqual(ref_joint, "/mimic_joint/Physics/jointA")

        # A prismatic follower carries distance units, so the offset is not converted.
        joint_d_prim = physics_scope_prim.GetChild("jointD")
        self.assertTrue(joint_d_prim.IsValid())
        self.assertTrue(joint_d_prim.IsA(UsdPhysics.PrismaticJoint))
        self.assertTrue(joint_d_prim.HasAPI("NewtonMimicAPI"))
        self.assertAlmostEqual(joint_d_prim.GetAttribute("newton:mimicCoef0").Get(), 0.25, places=6)
        self.assertAlmostEqual(joint_d_prim.GetAttribute("newton:mimicCoef1").Get(), 2.0)

    def test_velocity_limit(self):
        input_path = "tests/data/velocity_limits.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        mass_warnings = [
            (
                Tf.TF_DIAGNOSTIC_WARNING_TYPE,
                (
                    f".*Link '{name}' has no collision or positive-mass inertial, including its descendants, "
                    f"and is connected by a {joint_type} joint.*"
                ),
            )
            for name, joint_type in [
                ("positive_link", "revolute"),
                ("omitted_link", "revolute"),
                ("zero_link", "revolute"),
                ("negative_link", "revolute"),
                ("damped_link", "revolute"),
                ("slide_link", "prismatic"),
            ]
        ]
        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                *mass_warnings,
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, '.*Joint "negative" has negative <limit velocity> "-1.0".*'),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        physics_scope_prim = stage.GetPrimAtPath(stage.GetDefaultPrim().GetPath().AppendChild("Physics"))
        self.assertTrue(physics_scope_prim.IsValid())

        positive = physics_scope_prim.GetChild("positive")
        self.assertTrue(positive.HasAPI("NewtonJointAPI"))
        self.assertTrue(positive.GetAttribute("newton:velocityLimit").HasAuthoredValue())
        self.assertAlmostEqual(positive.GetAttribute("newton:velocityLimit").Get(), math.degrees(0.5), places=6)

        omitted = physics_scope_prim.GetChild("omitted")
        self.assertFalse(omitted.HasAPI("NewtonJointAPI"))
        self.assertFalse(omitted.GetAttribute("newton:velocityLimit").HasAuthoredValue())

        zero = physics_scope_prim.GetChild("zero")
        self.assertFalse(zero.HasAPI("NewtonJointAPI"))
        self.assertFalse(zero.GetAttribute("newton:velocityLimit").HasAuthoredValue())

        negative = physics_scope_prim.GetChild("negative")
        self.assertFalse(negative.HasAPI("NewtonJointAPI"))
        self.assertFalse(negative.GetAttribute("newton:velocityLimit").HasAuthoredValue())

        # Dynamics are still authored when the velocity limit is dropped.
        damped = physics_scope_prim.GetChild("damped")
        self.assertTrue(damped.HasAPI("NewtonJointAPI"))
        self.assertFalse(damped.GetAttribute("newton:velocityLimit").HasAuthoredValue())
        self.assertTrue(damped.GetAttribute("newton:damping").HasAuthoredValue())
        self.assertAlmostEqual(damped.GetAttribute("newton:damping").Get(), math.pi / 180.0, places=6)
        self.assertAlmostEqual(damped.GetAttribute("newton:friction").Get(), 0.5, places=6)

        slide = physics_scope_prim.GetChild("slide")
        self.assertTrue(slide.HasAPI("NewtonJointAPI"))
        self.assertTrue(slide.GetAttribute("newton:velocityLimit").HasAuthoredValue())
        self.assertAlmostEqual(slide.GetAttribute("newton:velocityLimit").Get(), 0.2, places=6)
