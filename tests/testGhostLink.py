# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

from pxr import Gf, Usd, UsdPhysics

import urdf_usd_converter
from tests.util.ConverterTestCase import ConverterTestCase


class TestGhostLink(ConverterTestCase):
    def test_link_first_ghost_link_fixed(self):
        input_path = "tests/data/link_first_ghost_link_fixed.urdf"
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
        self.assertTrue(geometry_scope_prim.IsValid())

        # Check physics rigid bodies.
        base_link_prim = geometry_scope_prim.GetChild("BaseLink")
        self.assertTrue(base_link_prim.IsValid())
        self.assertFalse(base_link_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(base_link_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(base_link_prim.HasAPI("NewtonArticulationRootAPI"))

        link1_prim = base_link_prim.GetChild("link1")
        self.assertTrue(link1_prim.IsValid())
        self.assertTrue(link1_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(link1_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertTrue(link1_prim.HasAPI("NewtonArticulationRootAPI"))

        link_box_prim = link1_prim.GetChild("link_box")
        self.assertTrue(link_box_prim.IsValid())
        self.assertTrue(link_box_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(link_box_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(link_box_prim.HasAPI("NewtonArticulationRootAPI"))

        # Check physics joint.
        physics_scope_prim = default_prim.GetChild("Physics")
        self.assertTrue(physics_scope_prim.IsValid())
        self.assertEqual(len(physics_scope_prim.GetChildren()), 2)

        joint1_prim = physics_scope_prim.GetChild("joint1")
        self.assertTrue(joint1_prim.IsValid())
        self.assertTrue(joint1_prim.IsA(UsdPhysics.FixedJoint))
        joint = UsdPhysics.FixedJoint(joint1_prim)
        self.assertEqual(joint.GetBody0Rel().GetTargets(), ["/link_first_ghost_link_fixed"])
        self.assertEqual(joint.GetBody1Rel().GetTargets(), ["/link_first_ghost_link_fixed/Geometry/BaseLink/link1"])
        self.assertTrue(Gf.IsClose(joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 1), 1e-6))
        self.assertTrue(Gf.IsClose(joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))

        joint_box_prim = physics_scope_prim.GetChild("joint_box")
        self.assertTrue(joint_box_prim.IsValid())
        self.assertTrue(joint_box_prim.IsA(UsdPhysics.FixedJoint))
        joint = UsdPhysics.FixedJoint(joint_box_prim)
        self.assertEqual(joint.GetBody0Rel().GetTargets(), ["/link_first_ghost_link_fixed/Geometry/BaseLink/link1"])
        self.assertEqual(joint.GetBody1Rel().GetTargets(), ["/link_first_ghost_link_fixed/Geometry/BaseLink/link1/link_box"])
        self.assertTrue(Gf.IsClose(joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0.8), 1e-6))
        self.assertTrue(Gf.IsClose(joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))

    def test_link_first_ghost_link_floating(self):
        input_path = "tests/data/link_first_ghost_link_floating.urdf"
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
        self.assertTrue(geometry_scope_prim.IsValid())

        # Check physics rigid bodies.
        base_link_prim = geometry_scope_prim.GetChild("BaseLink")
        self.assertTrue(base_link_prim.IsValid())
        self.assertFalse(base_link_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(base_link_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(base_link_prim.HasAPI("NewtonArticulationRootAPI"))

        link1_prim = base_link_prim.GetChild("link1")
        self.assertTrue(link1_prim.IsValid())
        self.assertTrue(link1_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(link1_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertTrue(link1_prim.HasAPI("NewtonArticulationRootAPI"))

        link_box_prim = link1_prim.GetChild("link_box")
        self.assertTrue(link_box_prim.IsValid())
        self.assertTrue(link_box_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(link_box_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(link_box_prim.HasAPI("NewtonArticulationRootAPI"))

        # Check physics joint.
        # There is only one physics joint, which is the joint between the link1 and the link_box.
        physics_scope_prim = default_prim.GetChild("Physics")
        self.assertTrue(physics_scope_prim.IsValid())
        self.assertEqual(len(physics_scope_prim.GetChildren()), 1)

        joint_box_prim = physics_scope_prim.GetChild("joint_box")
        self.assertTrue(joint_box_prim.IsValid())
        self.assertTrue(joint_box_prim.IsA(UsdPhysics.FixedJoint))
        joint = UsdPhysics.FixedJoint(joint_box_prim)
        self.assertEqual(joint.GetBody0Rel().GetTargets(), ["/link_first_ghost_link_floating/Geometry/BaseLink/link1"])
        self.assertEqual(joint.GetBody1Rel().GetTargets(), ["/link_first_ghost_link_floating/Geometry/BaseLink/link1/link_box"])
        self.assertTrue(Gf.IsClose(joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0.8), 1e-6))
        self.assertTrue(Gf.IsClose(joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))
