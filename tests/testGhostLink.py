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

    def test_link_skip_ghost_link(self):
        input_path = "tests/data/link_skip_ghost_link.urdf"
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

        # Check physics rigid bodies. This is a root link, and Articulation has been assigned to it.
        base_link_prim = geometry_scope_prim.GetChild("BaseLink")
        self.assertTrue(base_link_prim.IsValid())
        self.assertTrue(base_link_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(base_link_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertTrue(base_link_prim.HasAPI("NewtonArticulationRootAPI"))

        # This ghost link does not have a rigid body.
        ghost_link_prim = base_link_prim.GetChild("ghost_link")
        self.assertTrue(ghost_link_prim.IsValid())
        self.assertFalse(ghost_link_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(ghost_link_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(ghost_link_prim.HasAPI("NewtonArticulationRootAPI"))

        # Check physics rigid bodies.
        link_box_prim = ghost_link_prim.GetChild("link_box")
        self.assertTrue(link_box_prim.IsValid())
        self.assertTrue(link_box_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(link_box_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(link_box_prim.HasAPI("NewtonArticulationRootAPI"))

        # Check physics joint.
        # "root_joint" and "joint_box" are created.
        # "joint1" is not created because it is a fixed joint and body1 is a ghost link without a rigid body.
        physics_scope_prim = default_prim.GetChild("Physics")
        self.assertTrue(physics_scope_prim.IsValid())
        self.assertEqual(len(physics_scope_prim.GetChildren()), 2)

        root_joint_prim = physics_scope_prim.GetChild("root_joint")
        self.assertTrue(root_joint_prim.IsValid())
        self.assertTrue(root_joint_prim.IsA(UsdPhysics.FixedJoint))
        joint = UsdPhysics.FixedJoint(root_joint_prim)
        self.assertEqual(joint.GetBody0Rel().GetTargets(), ["/link_skip_ghost_link"])
        self.assertEqual(joint.GetBody1Rel().GetTargets(), ["/link_skip_ghost_link/Geometry/BaseLink"])
        self.assertTrue(Gf.IsClose(joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))

        joint_box_prim = physics_scope_prim.GetChild("joint_box")
        self.assertTrue(joint_box_prim.IsValid())
        self.assertTrue(joint_box_prim.IsA(UsdPhysics.RevoluteJoint))
        joint = UsdPhysics.RevoluteJoint(joint_box_prim)
        self.assertEqual(joint.GetBody0Rel().GetTargets(), ["/link_skip_ghost_link/Geometry/BaseLink"])
        self.assertEqual(joint.GetBody1Rel().GetTargets(), ["/link_skip_ghost_link/Geometry/BaseLink/ghost_link/link_box"])
        self.assertTrue(Gf.IsClose(joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0.6), 1e-6))
        self.assertTrue(Gf.IsClose(joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))

        joint1_prim = physics_scope_prim.GetChild("joint1")
        self.assertFalse(joint1_prim.IsValid())

    def test_link_skip_ghost_link_chain(self):
        input_path = "tests/data/link_skip_ghost_link_chain.urdf"
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

        # Check physics rigid bodies. This is a root link, and Articulation has been assigned to it.
        base_link_prim = geometry_scope_prim.GetChild("BaseLink")
        self.assertTrue(base_link_prim.IsValid())
        self.assertTrue(base_link_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(base_link_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertTrue(base_link_prim.HasAPI("NewtonArticulationRootAPI"))

        # This ghost link does not have a rigid body.
        ghost_link_prim = base_link_prim.GetChild("ghost_link")
        self.assertTrue(ghost_link_prim.IsValid())
        self.assertFalse(ghost_link_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(ghost_link_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(ghost_link_prim.HasAPI("NewtonArticulationRootAPI"))

        # This ghost link does not have a rigid body.
        ghost_link_2_prim = ghost_link_prim.GetChild("ghost_link_2")
        self.assertTrue(ghost_link_2_prim.IsValid())
        self.assertFalse(ghost_link_2_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(ghost_link_2_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(ghost_link_2_prim.HasAPI("NewtonArticulationRootAPI"))

        # This ghost link does not have a rigid body.
        ghost_link_3_prim = ghost_link_2_prim.GetChild("ghost_link_3")
        self.assertTrue(ghost_link_3_prim.IsValid())
        self.assertFalse(ghost_link_3_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(ghost_link_3_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(ghost_link_3_prim.HasAPI("NewtonArticulationRootAPI"))

        # Check physics rigid bodies.
        link_box_prim = ghost_link_3_prim.GetChild("link_box")
        self.assertTrue(link_box_prim.IsValid())
        self.assertTrue(link_box_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(link_box_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(link_box_prim.HasAPI("NewtonArticulationRootAPI"))

        # Check physics joint.
        # "root_joint" and "joint_box" are created.
        # "joint1" is not created because it is a fixed joint and body1 is a ghost link without a rigid body.
        physics_scope_prim = default_prim.GetChild("Physics")
        self.assertTrue(physics_scope_prim.IsValid())
        self.assertEqual(len(physics_scope_prim.GetChildren()), 2)

        root_joint_prim = physics_scope_prim.GetChild("root_joint")
        self.assertTrue(root_joint_prim.IsValid())
        self.assertTrue(root_joint_prim.IsA(UsdPhysics.FixedJoint))
        joint = UsdPhysics.FixedJoint(root_joint_prim)
        self.assertEqual(joint.GetBody0Rel().GetTargets(), ["/link_skip_ghost_link_chain"])
        self.assertEqual(joint.GetBody1Rel().GetTargets(), ["/link_skip_ghost_link_chain/Geometry/BaseLink"])
        self.assertTrue(Gf.IsClose(joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))

        joint_box_prim = physics_scope_prim.GetChild("joint_box")
        self.assertTrue(joint_box_prim.IsValid())
        self.assertTrue(joint_box_prim.IsA(UsdPhysics.RevoluteJoint))
        joint = UsdPhysics.RevoluteJoint(joint_box_prim)
        self.assertEqual(joint.GetBody0Rel().GetTargets(), ["/link_skip_ghost_link_chain/Geometry/BaseLink"])
        self.assertEqual(
            joint.GetBody1Rel().GetTargets(), ["/link_skip_ghost_link_chain/Geometry/BaseLink/ghost_link/ghost_link_2/ghost_link_3/link_box"]
        )
        self.assertTrue(Gf.IsClose(joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0.5), 1e-6))
        self.assertTrue(Gf.IsClose(joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))

        joint1_prim = physics_scope_prim.GetChild("joint1")
        self.assertFalse(joint1_prim.IsValid())
