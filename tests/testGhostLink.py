# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import usdex.core
import usdex.test
from pxr import Gf, Tf, Usd, UsdGeom, UsdPhysics

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

    def test_two_robots_under_an_empty_world_stay_separate_articulations(self):
        """
        Two robots under an empty world each get an articulation root.

        Putting ArticulationRootAPI on the world would merge both robots into one articulation.
        """
        input_path = "tests/data/two_robots_world.urdf"
        output_dir = self.tmpDir()

        with usdex.test.ScopedDiagnosticChecker(self, [], level=usdex.core.DiagnosticsLevel.eWarning):
            asset_path = urdf_usd_converter.Converter().convert(input_path, output_dir)

        stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)
        default_prim = stage.GetDefaultPrim()
        self.assertFalse(default_prim.HasAPI(UsdPhysics.ArticulationRootAPI))

        world = default_prim.GetChild("Geometry").GetChild("world")
        self.assertFalse(world.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(world.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(world.HasAPI("NewtonArticulationRootAPI"))

        robots = (
            ("left_base", "left_arm", "left_fixed", "left_arm_joint", Gf.Vec3f(0, 0.2, 0)),
            ("right_base", "right_arm", "right_fixed", "right_arm_joint", Gf.Vec3f(0, -0.2, 0)),
        )
        physics = default_prim.GetChild("Physics")
        self.assertEqual({child.GetName() for child in physics.GetChildren()}, {"left_fixed", "left_arm_joint", "right_fixed", "right_arm_joint"})
        for base_name, arm_name, fixed_name, revolute_name, fixed_pos in robots:
            base = world.GetChild(base_name)
            arm = base.GetChild(arm_name)
            self.assertTrue(base.HasAPI(UsdPhysics.RigidBodyAPI))
            self.assertTrue(base.HasAPI(UsdPhysics.MassAPI))
            self.assertTrue(base.HasAPI(UsdPhysics.ArticulationRootAPI))
            self.assertTrue(base.HasAPI("NewtonArticulationRootAPI"))
            self.assertTrue(arm.HasAPI(UsdPhysics.RigidBodyAPI))
            self.assertTrue(arm.HasAPI(UsdPhysics.MassAPI))
            self.assertFalse(arm.HasAPI(UsdPhysics.ArticulationRootAPI))
            self.assertFalse(arm.HasAPI("NewtonArticulationRootAPI"))
            self.assertTrue(arm.GetChild("box").IsA(UsdGeom.Cube))

            fixed = UsdPhysics.FixedJoint(physics.GetChild(fixed_name))
            self.assertEqual(fixed.GetBody0Rel().GetTargets(), ["/two_robots_world"])
            self.assertEqual(fixed.GetBody1Rel().GetTargets(), [f"/two_robots_world/Geometry/world/{base_name}"])
            self.assertTrue(Gf.IsClose(fixed.GetLocalPos0Attr().Get(), fixed_pos, 1e-6))
            joint = UsdPhysics.RevoluteJoint(physics.GetChild(revolute_name))
            self.assertEqual(joint.GetBody0Rel().GetTargets(), [f"/two_robots_world/Geometry/world/{base_name}"])
            self.assertEqual(joint.GetBody1Rel().GetTargets(), [f"/two_robots_world/Geometry/world/{base_name}/{arm_name}"])
            self.assertTrue(Gf.IsClose(joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0.2), 1e-6))

    def test_revolute_and_floating_robots_under_an_empty_world_stay_separate_articulations(self):
        """
        A revolute robot and a floating robot under an empty world stay separate articulations.

        ArticulationRootAPI is applied to arm and free, not to world. The floating joint is not
        authored, so the two articulations do not overlap.
        """
        input_path = "tests/data/world_revolute_and_floating.urdf"
        output_dir = self.tmpDir()

        with usdex.test.ScopedDiagnosticChecker(self, [], level=usdex.core.DiagnosticsLevel.eWarning):
            asset_path = urdf_usd_converter.Converter().convert(input_path, output_dir)

        stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)
        default_prim = stage.GetDefaultPrim()
        self.assertFalse(default_prim.HasAPI(UsdPhysics.ArticulationRootAPI))

        world = default_prim.GetChild("Geometry").GetChild("world")
        arm = world.GetChild("arm")
        free = world.GetChild("free")
        freechild = free.GetChild("freechild")
        self.assertFalse(world.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(world.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(world.HasAPI("NewtonArticulationRootAPI"))

        for link in (arm, free):
            self.assertTrue(link.HasAPI(UsdPhysics.RigidBodyAPI))
            self.assertTrue(link.HasAPI(UsdPhysics.MassAPI))
            self.assertTrue(link.HasAPI(UsdPhysics.ArticulationRootAPI))
            self.assertTrue(link.HasAPI("NewtonArticulationRootAPI"))
            self.assertTrue(link.GetChild("box").IsA(UsdGeom.Cube))
        self.assertTrue(freechild.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(freechild.HasAPI(UsdPhysics.MassAPI))
        self.assertFalse(freechild.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(freechild.HasAPI("NewtonArticulationRootAPI"))
        self.assertTrue(freechild.GetChild("box").IsA(UsdGeom.Cube))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(arm).GetTranslation(), Gf.Vec3d(0, 0.25, 0), 1e-6))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(free).GetTranslation(), Gf.Vec3d(0, -0.25, 0), 1e-6))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(freechild).GetTranslation(), Gf.Vec3d(0, 0, 0.3), 1e-6))

        physics = default_prim.GetChild("Physics")
        self.assertEqual({child.GetName() for child in physics.GetChildren()}, {"arm_joint", "freechild_joint"})
        arm_joint = UsdPhysics.RevoluteJoint(physics.GetChild("arm_joint"))
        self.assertEqual(arm_joint.GetBody0Rel().GetTargets(), ["/world_revolute_and_floating"])
        self.assertEqual(arm_joint.GetBody1Rel().GetTargets(), ["/world_revolute_and_floating/Geometry/world/arm"])
        self.assertTrue(Gf.IsClose(arm_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0.25, 0), 1e-6))
        freechild_joint = UsdPhysics.RevoluteJoint(physics.GetChild("freechild_joint"))
        self.assertEqual(freechild_joint.GetBody0Rel().GetTargets(), ["/world_revolute_and_floating/Geometry/world/free"])
        self.assertEqual(freechild_joint.GetBody1Rel().GetTargets(), ["/world_revolute_and_floating/Geometry/world/free/freechild"])
        self.assertTrue(Gf.IsClose(freechild_joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0.3), 1e-6))

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

        # Check physics joint.
        # "root_joint" and "joint_box" are created.
        # "joint1" is not created because it is a fixed joint and body1 is a ghost link without a rigid body.
        physics_scope_prim = default_prim.GetChild("Physics")
        self.assertTrue(physics_scope_prim.IsValid())
        self.assertEqual(len(physics_scope_prim.GetChildren()), 1)

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

        joint1_prim = physics_scope_prim.GetChild("joint1")
        self.assertFalse(joint1_prim.IsValid())

        joint2_prim = physics_scope_prim.GetChild("joint2")
        self.assertFalse(joint2_prim.IsValid())

        joint3_prim = physics_scope_prim.GetChild("joint3")
        self.assertFalse(joint3_prim.IsValid())

        joint_box_prim = physics_scope_prim.GetChild("joint_box")
        self.assertTrue(joint_box_prim.IsValid())
        self.assertTrue(joint_box_prim.IsA(UsdPhysics.RevoluteJoint))
        joint = UsdPhysics.RevoluteJoint(joint_box_prim)
        # body0 is authored as "BaseLink" (resolved from "ghost_link_3") so consumers
        # reading body0 directly see the rigid-body ancestor without walking the hierarchy.
        self.assertEqual(joint.GetBody0Rel().GetTargets(), ["/link_skip_ghost_link_chain/Geometry/BaseLink"])
        self.assertEqual(
            joint.GetBody1Rel().GetTargets(), ["/link_skip_ghost_link_chain/Geometry/BaseLink/ghost_link/ghost_link_2/ghost_link_3/link_box"]
        )
        # Authored localPos0 is relative to BaseLink (0.1+0.2+0.2), matching the world-space joint.
        self.assertTrue(Gf.IsClose(joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0.5), 1e-6))
        self.assertTrue(Gf.IsClose(joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))

        # Resolved parse agrees with the authored retargeting (same body0 / localPose0).
        parsed = UsdPhysics.LoadUsdPhysicsFromRange(stage, ["/"])
        revolute_paths, revolute_descs = parsed[UsdPhysics.ObjectType.RevoluteJoint]
        self.assertEqual(list(revolute_paths), [joint_box_prim.GetPath()])
        self.assertEqual(str(revolute_descs[0].body0), "/link_skip_ghost_link_chain/Geometry/BaseLink")
        self.assertTrue(Gf.IsClose(revolute_descs[0].localPose0Position, Gf.Vec3f(0, 0, 0.5), 1e-6))

    def test_link_skip_ghost_link_chain_end(self):
        input_path = "tests/data/link_skip_ghost_link_chain_end.urdf"
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

        # Check physics joint.
        # "root_joint" is created.
        # "joint1", "joint2", and "joint3" are not created because they are fixed joints and body1 is a ghost link without a rigid body.
        physics_scope_prim = default_prim.GetChild("Physics")
        self.assertTrue(physics_scope_prim.IsValid())
        self.assertEqual(len(physics_scope_prim.GetChildren()), 1)

        root_joint_prim = physics_scope_prim.GetChild("root_joint")
        self.assertTrue(root_joint_prim.IsValid())
        self.assertTrue(root_joint_prim.IsA(UsdPhysics.FixedJoint))
        joint = UsdPhysics.FixedJoint(root_joint_prim)
        self.assertEqual(joint.GetBody0Rel().GetTargets(), ["/link_skip_ghost_link_chain_end"])
        self.assertEqual(joint.GetBody1Rel().GetTargets(), ["/link_skip_ghost_link_chain_end/Geometry/BaseLink"])
        self.assertTrue(Gf.IsClose(joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))

    def test_link_skip_ghost_link_chain_branch(self):
        input_path = "tests/data/link_skip_ghost_link_chain_branch.urdf"
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

        # This link has a rigid body.
        link_box_prim = ghost_link_3_prim.GetChild("link_box")
        self.assertTrue(link_box_prim.IsValid())
        self.assertTrue(link_box_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(link_box_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(link_box_prim.HasAPI("NewtonArticulationRootAPI"))

        # This ghost link does not have a rigid body.
        ghost_link_4_prim = ghost_link_2_prim.GetChild("ghost_link_4")
        self.assertTrue(ghost_link_4_prim.IsValid())
        self.assertFalse(ghost_link_4_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(ghost_link_4_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(ghost_link_4_prim.HasAPI("NewtonArticulationRootAPI"))

        # This ghost link does not have a rigid body.
        ghost_link_5_prim = ghost_link_4_prim.GetChild("ghost_link_5")
        self.assertTrue(ghost_link_5_prim.IsValid())
        self.assertFalse(ghost_link_5_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(ghost_link_5_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(ghost_link_5_prim.HasAPI("NewtonArticulationRootAPI"))

        # Check physics joint.
        # "root_joint" is created.
        # "ghost_link_4", and "ghost_link_5" are not created because they are fixed joints and body1 is a ghost link without a rigid body.
        physics_scope_prim = default_prim.GetChild("Physics")
        self.assertTrue(physics_scope_prim.IsValid())
        self.assertEqual(len(physics_scope_prim.GetChildren()), 2)

        root_joint_prim = physics_scope_prim.GetChild("root_joint")
        self.assertTrue(root_joint_prim.IsValid())
        self.assertTrue(root_joint_prim.IsA(UsdPhysics.FixedJoint))
        joint = UsdPhysics.FixedJoint(root_joint_prim)
        self.assertEqual(joint.GetBody0Rel().GetTargets(), ["/link_skip_ghost_link_chain_branch"])
        self.assertEqual(joint.GetBody1Rel().GetTargets(), ["/link_skip_ghost_link_chain_branch/Geometry/BaseLink"])
        self.assertTrue(Gf.IsClose(joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))

        joint1_prim = physics_scope_prim.GetChild("joint1")
        self.assertFalse(joint1_prim.IsValid())

        joint2_prim = physics_scope_prim.GetChild("joint2")
        self.assertFalse(joint2_prim.IsValid())

        joint3_prim = physics_scope_prim.GetChild("joint3")
        self.assertFalse(joint3_prim.IsValid())

        joint_box_prim = physics_scope_prim.GetChild("joint_box")
        self.assertTrue(joint_box_prim.IsValid())
        self.assertTrue(joint_box_prim.IsA(UsdPhysics.RevoluteJoint))
        joint = UsdPhysics.RevoluteJoint(joint_box_prim)
        # body0 is authored as "BaseLink" (resolved from "ghost_link_3") so consumers
        # reading body0 directly see the rigid-body ancestor without walking the hierarchy.
        self.assertEqual(joint.GetBody0Rel().GetTargets(), ["/link_skip_ghost_link_chain_branch/Geometry/BaseLink"])
        self.assertEqual(
            joint.GetBody1Rel().GetTargets(), ["/link_skip_ghost_link_chain_branch/Geometry/BaseLink/ghost_link/ghost_link_2/ghost_link_3/link_box"]
        )
        self.assertTrue(Gf.IsClose(joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0.5), 1e-6))
        self.assertTrue(Gf.IsClose(joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))

        joint4_prim = physics_scope_prim.GetChild("joint4")
        self.assertFalse(joint4_prim.IsValid())

        joint5_prim = physics_scope_prim.GetChild("joint5")
        self.assertFalse(joint5_prim.IsValid())

    def test_link_skip_ghost_link_with_mimic(self):
        input_path = "tests/data/link_skip_ghost_link_with_mimic.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        link_a = ".*Link 'linkA' has no collision or positive-mass inertial, including its descendants, and is the child of fixed joint 'jointA'.*"
        link_b = ".*Link 'linkB' has no collision or positive-mass inertial, including its descendants, and is connected by a revolute joint.*"
        with usdex.test.ScopedDiagnosticChecker(
            self,
            [(Tf.TF_DIAGNOSTIC_WARNING_TYPE, link_a), (Tf.TF_DIAGNOSTIC_WARNING_TYPE, link_b)],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
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
        base_link_a_prim = geometry_scope_prim.GetChild("BaseLinkA")
        self.assertTrue(base_link_a_prim.IsValid())
        self.assertTrue(base_link_a_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(base_link_a_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertTrue(base_link_a_prim.HasAPI("NewtonArticulationRootAPI"))

        base_link_b_prim = base_link_a_prim.GetChild("BaseLinkB")
        self.assertTrue(base_link_b_prim.IsValid())
        self.assertTrue(base_link_b_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(base_link_b_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(base_link_b_prim.HasAPI("NewtonArticulationRootAPI"))

        # "linkA" is a Ghost Link, but since it is referenced from a mimic joint, the rigid body will not be deleted.
        link_a_prim = base_link_a_prim.GetChild("linkA")
        self.assertTrue(link_a_prim.IsValid())
        self.assertTrue(link_a_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(link_a_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(link_a_prim.HasAPI("NewtonArticulationRootAPI"))

        link_b_prim = base_link_b_prim.GetChild("linkB")
        self.assertTrue(link_b_prim.IsValid())
        self.assertTrue(link_b_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(link_b_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(link_b_prim.HasAPI("NewtonArticulationRootAPI"))

        # Check physics joint.
        # "root_joint" is created.
        physics_scope_prim = default_prim.GetChild("Physics")
        self.assertTrue(physics_scope_prim.IsValid())
        self.assertEqual(len(physics_scope_prim.GetChildren()), 4)

        root_joint_prim = physics_scope_prim.GetChild("root_joint")
        self.assertTrue(root_joint_prim.IsValid())
        self.assertTrue(root_joint_prim.IsA(UsdPhysics.FixedJoint))
        joint = UsdPhysics.FixedJoint(root_joint_prim)
        self.assertEqual(joint.GetBody0Rel().GetTargets(), ["/link_skip_ghost_link_with_mimic"])
        self.assertEqual(joint.GetBody1Rel().GetTargets(), ["/link_skip_ghost_link_with_mimic/Geometry/BaseLinkA"])
        self.assertTrue(Gf.IsClose(joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))

        joint_base_prim = physics_scope_prim.GetChild("joint_base")
        self.assertTrue(joint_base_prim.IsValid())
        self.assertTrue(joint_base_prim.IsA(UsdPhysics.FixedJoint))
        joint = UsdPhysics.FixedJoint(joint_base_prim)
        self.assertEqual(joint.GetBody0Rel().GetTargets(), ["/link_skip_ghost_link_with_mimic/Geometry/BaseLinkA"])
        self.assertEqual(joint.GetBody1Rel().GetTargets(), ["/link_skip_ghost_link_with_mimic/Geometry/BaseLinkA/BaseLinkB"])
        self.assertTrue(Gf.IsClose(joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0.4, 0), 1e-6))
        self.assertTrue(Gf.IsClose(joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))

        joint_a_prim = physics_scope_prim.GetChild("jointA")
        self.assertTrue(joint_a_prim.IsValid())
        self.assertTrue(joint_a_prim.IsA(UsdPhysics.FixedJoint))
        joint = UsdPhysics.FixedJoint(joint_a_prim)
        self.assertEqual(joint.GetBody0Rel().GetTargets(), ["/link_skip_ghost_link_with_mimic/Geometry/BaseLinkA"])
        self.assertEqual(joint.GetBody1Rel().GetTargets(), ["/link_skip_ghost_link_with_mimic/Geometry/BaseLinkA/linkA"])
        self.assertTrue(Gf.IsClose(joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))

        joint_b_prim = physics_scope_prim.GetChild("jointB")
        self.assertTrue(joint_b_prim.IsValid())
        self.assertTrue(joint_b_prim.IsA(UsdPhysics.RevoluteJoint))
        joint = UsdPhysics.RevoluteJoint(joint_b_prim)
        self.assertEqual(joint.GetBody0Rel().GetTargets(), ["/link_skip_ghost_link_with_mimic/Geometry/BaseLinkA/BaseLinkB"])
        self.assertEqual(joint.GetBody1Rel().GetTargets(), ["/link_skip_ghost_link_with_mimic/Geometry/BaseLinkA/BaseLinkB/linkB"])
        self.assertTrue(Gf.IsClose(joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertTrue(joint_b_prim.HasAPI("NewtonMimicAPI"))
        self.assertTrue(joint_b_prim.GetAttribute("newton:mimicEnabled").Get())
        self.assertAlmostEqual(joint_b_prim.GetAttribute("newton:mimicCoef0").Get(), 0.0)
        self.assertAlmostEqual(joint_b_prim.GetAttribute("newton:mimicCoef1").Get(), 2.0)
        self.assertTrue(joint_b_prim.HasRelationship("newton:mimicJoint"))
        rel = joint_b_prim.GetRelationship("newton:mimicJoint")
        self.assertEqual(len(rel.GetTargets()), 1)
        ref_joint = rel.GetTargets()[0]
        self.assertEqual(ref_joint, "/link_skip_ghost_link_with_mimic/Physics/jointA")

    def test_link_ghost_link_inertial_zero(self):
        input_path = "tests/data/link_ghost_link_inertial_zero.urdf"
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
        self.assertTrue(base_link_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(base_link_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertTrue(base_link_prim.HasAPI("NewtonArticulationRootAPI"))

        box_link_prim = base_link_prim.GetChild("boxLink")
        self.assertTrue(box_link_prim.IsValid())
        self.assertTrue(box_link_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(box_link_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(box_link_prim.HasAPI("NewtonArticulationRootAPI"))

        # This ghost link does not have a rigid body.
        ghost_link_zero_mass_prim = box_link_prim.GetChild("ghost_link_zero_mass")
        self.assertTrue(ghost_link_zero_mass_prim.IsValid())
        self.assertFalse(ghost_link_zero_mass_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(ghost_link_zero_mass_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(ghost_link_zero_mass_prim.HasAPI("NewtonArticulationRootAPI"))

        # This ghost link does not have a rigid body.
        ghost_link_no_inertia_prim = box_link_prim.GetChild("ghost_link_no_inertia")
        self.assertTrue(ghost_link_no_inertia_prim.IsValid())
        self.assertFalse(ghost_link_no_inertia_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(ghost_link_no_inertia_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(ghost_link_no_inertia_prim.HasAPI("NewtonArticulationRootAPI"))

        # Check physics joint.
        physics_scope_prim = default_prim.GetChild("Physics")
        self.assertTrue(physics_scope_prim.IsValid())

        root_joint_prim = physics_scope_prim.GetChild("root_joint")
        self.assertTrue(root_joint_prim.IsValid())
        self.assertTrue(root_joint_prim.IsA(UsdPhysics.FixedJoint))
        joint = UsdPhysics.FixedJoint(root_joint_prim)
        self.assertEqual(joint.GetBody0Rel().GetTargets(), ["/link_ghost_link_inertial_zero"])
        self.assertEqual(joint.GetBody1Rel().GetTargets(), ["/link_ghost_link_inertial_zero/Geometry/BaseLink"])
        self.assertTrue(Gf.IsClose(joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))

        joint1_prim = physics_scope_prim.GetChild("joint1")
        self.assertTrue(joint1_prim.IsValid())
        self.assertTrue(joint1_prim.IsA(UsdPhysics.FixedJoint))
        joint = UsdPhysics.FixedJoint(joint1_prim)
        self.assertEqual(joint.GetBody0Rel().GetTargets(), ["/link_ghost_link_inertial_zero/Geometry/BaseLink"])
        self.assertEqual(joint.GetBody1Rel().GetTargets(), ["/link_ghost_link_inertial_zero/Geometry/BaseLink/boxLink"])
        self.assertTrue(Gf.IsClose(joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0.1), 1e-6))
        self.assertTrue(Gf.IsClose(joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))

        joint_zero_mass_prim = physics_scope_prim.GetChild("joint_zero_mass")
        self.assertFalse(joint_zero_mass_prim.IsValid())

        joint_no_inertia_prim = physics_scope_prim.GetChild("joint_no_inertia")
        self.assertFalse(joint_no_inertia_prim.IsValid())

    def test_link_ghost_link_children(self):
        input_path = "tests/data/link_ghost_link_children.urdf"
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

        # This link has a rigid body.
        box_link_prim = ghost_link_prim.GetChild("box_link")
        self.assertTrue(box_link_prim.IsValid())
        self.assertTrue(box_link_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(box_link_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(box_link_prim.HasAPI("NewtonArticulationRootAPI"))

        # Check physics joint.
        # "root_joint" and "joint2" are created.
        # "joint1" is not created because it is a fixed joint and body1 is a ghost link without a rigid body.
        physics_scope_prim = default_prim.GetChild("Physics")
        self.assertTrue(physics_scope_prim.IsValid())
        self.assertEqual(len(physics_scope_prim.GetChildren()), 2)

        root_joint_prim = physics_scope_prim.GetChild("root_joint")
        self.assertTrue(root_joint_prim.IsValid())
        self.assertTrue(root_joint_prim.IsA(UsdPhysics.FixedJoint))
        joint = UsdPhysics.FixedJoint(root_joint_prim)
        self.assertEqual(joint.GetBody0Rel().GetTargets(), ["/link_ghost_link_children"])
        self.assertEqual(joint.GetBody1Rel().GetTargets(), ["/link_ghost_link_children/Geometry/BaseLink"])
        self.assertTrue(Gf.IsClose(joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))

        joint1_prim = physics_scope_prim.GetChild("joint1")
        self.assertFalse(joint1_prim.IsValid())

        joint2_prim = physics_scope_prim.GetChild("joint2")
        self.assertTrue(joint2_prim.IsValid())
        self.assertTrue(joint2_prim.IsA(UsdPhysics.RevoluteJoint))
        joint = UsdPhysics.RevoluteJoint(joint2_prim)
        # body0 is authored as "BaseLink" (resolved from "ghost_link") so consumers
        # reading body0 directly see the rigid-body ancestor without walking the hierarchy.
        self.assertEqual(joint.GetBody0Rel().GetTargets(), ["/link_ghost_link_children/Geometry/BaseLink"])
        self.assertEqual(joint.GetBody1Rel().GetTargets(), ["/link_ghost_link_children/Geometry/BaseLink/ghost_link/box_link"])
        # The origin of "joint1" and "joint2" are accumulated into the joint frame of "BaseLink".
        self.assertTrue(Gf.IsClose(joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0.4, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(joint.GetLocalPos1Attr().Get(), Gf.Vec3f(0, 0, 0), 1e-6))
        self.assertRotationsAlmostEqual(joint.GetLocalRot0Attr().Get(), Gf.Quatf(1, 0, 0, 0))
        self.assertRotationsAlmostEqual(joint.GetLocalRot1Attr().Get(), Gf.Quatf(1, 0, 0, 0))

        # Resolved parse agrees with the authored retargeting (same body0 / localPose0).
        parsed = UsdPhysics.LoadUsdPhysicsFromRange(stage, ["/"])
        revolute_paths, revolute_descs = parsed[UsdPhysics.ObjectType.RevoluteJoint]
        self.assertEqual(list(revolute_paths), [joint2_prim.GetPath()])
        self.assertEqual(str(revolute_descs[0].body0), "/link_ghost_link_children/Geometry/BaseLink")
        self.assertTrue(Gf.IsClose(revolute_descs[0].localPose0Position, Gf.Vec3f(0.4, 0, 0), 1e-6))
