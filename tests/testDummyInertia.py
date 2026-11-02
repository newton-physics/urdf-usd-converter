# SPDX-FileCopyrightText: Copyright (c) 2025-2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

from pxr import Gf, Usd, UsdPhysics

import urdf_usd_converter
from tests.util.ConverterTestCase import ConverterTestCase


class TestDummyInertiaMerge(ConverterTestCase):
    def test_fixed_dummy_inertia_is_merged_into_the_parent(self):
        """
        An inertial-only child on a fixed joint is copied onto the parent.

        The child link is not authored, and a sibling with collision stays its own rigid body.
        """
        input_path = "tests/data/dummy_inertia_fixed.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        geometry = stage.GetDefaultPrim().GetChild("Geometry")
        base = geometry.GetChild("base")
        dummy = base.GetChild("base_inertia")
        wheel = base.GetChild("wheel")

        self.assertTrue(base.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(base.HasAPI(UsdPhysics.MassAPI))
        mass_api = UsdPhysics.MassAPI(base)
        self.assertAlmostEqual(mass_api.GetMassAttr().Get(), 16.8, places=5)
        self.assertTrue(Gf.IsClose(mass_api.GetCenterOfMassAttr().Get(), Gf.Vec3f(0.1, 0.0, 0.05), 1e-6))
        inertia = base.GetAttribute("newton:inertia").Get()
        for actual, expected in zip(inertia, [0.2, 0.3, 0.4, 0.01, 0.0, 0.0]):
            self.assertAlmostEqual(actual, expected, places=6)

        self.assertFalse(dummy.IsValid())

        self.assertTrue(wheel.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertAlmostEqual(UsdPhysics.MassAPI(wheel).GetMassAttr().Get(), 1.1, places=6)

        physics = stage.GetDefaultPrim().GetChild("Physics")
        self.assertEqual({child.GetName() for child in physics.GetChildren()}, {"root_joint", "wheel_joint"})
        wheel_joint = UsdPhysics.Joint(physics.GetChild("wheel_joint"))
        self.assertEqual(wheel_joint.GetBody0Rel().GetTargets(), ["/dummy_inertia_fixed/Geometry/base"])
        self.assertEqual(wheel_joint.GetBody1Rel().GetTargets(), ["/dummy_inertia_fixed/Geometry/base/wheel"])

    def test_revolute_dummy_inertia_is_not_merged_into_the_parent(self):
        """
        The same inertial-only child on a revolute joint is not copied onto the parent.

        The child keeps its own rigid body, and the revolute joint is authored.
        """
        input_path = "tests/data/dummy_inertia_revolute.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        geometry = stage.GetDefaultPrim().GetChild("Geometry")
        base = geometry.GetChild("base")
        dummy = base.GetChild("base_inertia")
        wheel = base.GetChild("wheel")

        self.assertTrue(base.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(base.HasAPI(UsdPhysics.MassAPI))

        self.assertTrue(dummy.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(dummy.HasAPI(UsdPhysics.MassAPI))
        mass_api = UsdPhysics.MassAPI(dummy)
        self.assertAlmostEqual(mass_api.GetMassAttr().Get(), 16.8, places=5)
        self.assertTrue(Gf.IsClose(mass_api.GetCenterOfMassAttr().Get(), Gf.Vec3f(0.1, 0.0, 0.05), 1e-6))
        inertia = dummy.GetAttribute("newton:inertia").Get()
        for actual, expected in zip(inertia, [0.2, 0.3, 0.4, 0.01, 0.0, 0.0]):
            self.assertAlmostEqual(actual, expected, places=6)

        self.assertTrue(wheel.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertAlmostEqual(UsdPhysics.MassAPI(wheel).GetMassAttr().Get(), 1.1, places=6)

        physics = stage.GetDefaultPrim().GetChild("Physics")
        self.assertEqual({child.GetName() for child in physics.GetChildren()}, {"root_joint", "base_to_base_inertia", "wheel_joint"})
        inertia_joint = UsdPhysics.RevoluteJoint(physics.GetChild("base_to_base_inertia"))
        self.assertEqual(inertia_joint.GetBody0Rel().GetTargets(), ["/dummy_inertia_revolute/Geometry/base"])
        self.assertEqual(inertia_joint.GetBody1Rel().GetTargets(), ["/dummy_inertia_revolute/Geometry/base/base_inertia"])
        wheel_joint = UsdPhysics.Joint(physics.GetChild("wheel_joint"))
        self.assertEqual(wheel_joint.GetBody0Rel().GetTargets(), ["/dummy_inertia_revolute/Geometry/base"])
        self.assertEqual(wheel_joint.GetBody1Rel().GetTargets(), ["/dummy_inertia_revolute/Geometry/base/wheel"])

    def test_parent_with_inertial_is_not_merged(self):
        """
        A parent that already has inertial is not merged with its inertial-only fixed child.

        Both keep their own mass, and the fixed joint is authored.
        """
        input_path = "tests/data/dummy_inertia_parent_mass.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        geometry = stage.GetDefaultPrim().GetChild("Geometry")
        arm = geometry.GetChild("base").GetChild("arm")
        dummy = arm.GetChild("arm_inertia")

        self.assertTrue(arm.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(arm.HasAPI(UsdPhysics.MassAPI))
        arm_mass = UsdPhysics.MassAPI(arm)
        self.assertAlmostEqual(arm_mass.GetMassAttr().Get(), 4.0, places=6)
        self.assertTrue(Gf.IsClose(arm_mass.GetCenterOfMassAttr().Get(), Gf.Vec3f(0.0, 0.0, 0.0), 1e-6))
        arm_inertia = arm.GetAttribute("newton:inertia").Get()
        for actual, expected in zip(arm_inertia, [0.05, 0.06, 0.07, 0.0, 0.0, 0.0]):
            self.assertAlmostEqual(actual, expected, places=6)

        self.assertTrue(dummy.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(dummy.HasAPI(UsdPhysics.MassAPI))
        dummy_mass = UsdPhysics.MassAPI(dummy)
        self.assertAlmostEqual(dummy_mass.GetMassAttr().Get(), 0.2, places=6)
        self.assertTrue(Gf.IsClose(dummy_mass.GetCenterOfMassAttr().Get(), Gf.Vec3f(0.1, 0.0, 0.05), 1e-6))

        physics = stage.GetDefaultPrim().GetChild("Physics")
        self.assertEqual({child.GetName() for child in physics.GetChildren()}, {"root_joint", "arm_joint", "arm_to_arm_inertia"})
        inertia_joint = UsdPhysics.Joint(physics.GetChild("arm_to_arm_inertia"))
        self.assertEqual(inertia_joint.GetBody0Rel().GetTargets(), ["/dummy_inertia_parent_mass/Geometry/base/arm"])
        self.assertEqual(inertia_joint.GetBody1Rel().GetTargets(), ["/dummy_inertia_parent_mass/Geometry/base/arm/arm_inertia"])

    def test_joint_origin_is_composed_into_the_parent_inertial(self):
        """
        A non-identity fixed joint origin is applied before the child inertial is written on the parent.

        The joint origin uses a translation and a right angle on every axis.
        Child CoM (0, 0, 0.05) becomes (1.05, 2, 3), and diag(1, 2, 3) becomes (3, 2, 1).
        The child link is not authored.
        """
        input_path = "tests/data/dummy_inertia_offset.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        base = stage.GetDefaultPrim().GetChild("Geometry").GetChild("base")
        mass_api = UsdPhysics.MassAPI(base)
        self.assertAlmostEqual(mass_api.GetMassAttr().Get(), 2.0, places=6)
        self.assertTrue(Gf.IsClose(mass_api.GetCenterOfMassAttr().Get(), Gf.Vec3f(1.05, 2.0, 3.0), 1e-5))
        inertia = base.GetAttribute("newton:inertia").Get()
        for actual, expected in zip(inertia, [3.0, 2.0, 1.0, 0.0, 0.0, 0.0]):
            self.assertAlmostEqual(actual, expected, places=5)

        self.assertFalse(base.GetChild("inertia_link").IsValid())
        physics = stage.GetDefaultPrim().GetChild("Physics")
        self.assertEqual({child.GetName() for child in physics.GetChildren()}, {"root_joint", "side_joint"})

    def test_missing_joint_origin_copies_the_child_inertial_unchanged(self):
        """
        A fixed joint with no origin is the URDF identity.

        The child's center of mass and inertia are copied onto the parent unchanged.
        """
        input_path = "tests/data/dummy_inertia_offset.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        side = stage.GetDefaultPrim().GetChild("Geometry").GetChild("base").GetChild("side")
        mass_api = UsdPhysics.MassAPI(side)
        self.assertAlmostEqual(mass_api.GetMassAttr().Get(), 4.0, places=6)
        self.assertTrue(Gf.IsClose(mass_api.GetCenterOfMassAttr().Get(), Gf.Vec3f(0.25, 0.5, 0.125), 1e-6))
        inertia = side.GetAttribute("newton:inertia").Get()
        for actual, expected in zip(inertia, [1.0, 2.0, 3.0, 0.0, 0.0, 0.0]):
            self.assertAlmostEqual(actual, expected, places=6)

        self.assertFalse(side.GetChild("side_inertia").IsValid())
        physics = stage.GetDefaultPrim().GetChild("Physics")
        self.assertNotIn("side_inertia_joint", {child.GetName() for child in physics.GetChildren()})
