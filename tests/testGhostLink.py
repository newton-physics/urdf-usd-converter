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

        link1_prim = base_link_prim.GetChild("link1")
        self.assertTrue(link1_prim.IsValid())
        self.assertTrue(link1_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(link1_prim.HasAPI(UsdPhysics.ArticulationRootAPI))

        # Check physics joint.
        physics_scope_prim = default_prim.GetChild("Physics")
        self.assertTrue(physics_scope_prim.IsValid())

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

        link1_prim = base_link_prim.GetChild("link1")
        self.assertTrue(link1_prim.IsValid())
        self.assertTrue(link1_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(link1_prim.HasAPI(UsdPhysics.ArticulationRootAPI))

        # The physics joint does not exist.
        physics_scope_prim = default_prim.GetChild("Physics")
        self.assertTrue(physics_scope_prim.IsValid())
        self.assertEqual(len(physics_scope_prim.GetChildren()), 0)
