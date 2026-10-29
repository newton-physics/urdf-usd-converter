# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import usdex.core
import usdex.test
from pxr import Gf, Tf, Usd, UsdGeom, UsdPhysics

import urdf_usd_converter
from tests.util.ConverterTestCase import ConverterTestCase


class TestVisualOnlyLink(ConverterTestCase):
    def test_fixed_visual_only_link_is_not_a_rigid_body(self):
        """
        A visual-only link on a fixed joint keeps its visual and gets no rigid body.

        A link with a collision still gets a rigid body.
        Explicit zero mass is the same case as a missing inertial.
        """
        input_path = "tests/data/visual_only_fixed.urdf"
        output_dir = self.tmpDir()

        with usdex.test.ScopedDiagnosticChecker(self, [], level=usdex.core.DiagnosticsLevel.eWarning):
            asset_path = urdf_usd_converter.Converter().convert(input_path, output_dir)

        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)
        geometry = stage.GetDefaultPrim().GetChild("Geometry")
        base = geometry.GetChild("base")
        drawn = base.GetChild("drawn")
        zero = base.GetChild("zero")
        solid = base.GetChild("solid")
        bare = base.GetChild("bare")

        self.assertTrue(base.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(base.HasAPI(UsdPhysics.MassAPI))

        self.assertTrue(drawn.IsValid())
        self.assertFalse(drawn.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(drawn.HasAPI(UsdPhysics.MassAPI))
        self.assertTrue(drawn.GetChild("box").IsValid())
        self.assertTrue(drawn.GetChild("box").IsA(UsdGeom.Cube))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(drawn).GetTranslation(), Gf.Vec3d(0.1, 0, 0), 1e-6))

        self.assertFalse(zero.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(zero.GetChild("box").IsValid())

        self.assertTrue(solid.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(solid.HasAPI(UsdPhysics.MassAPI))

        self.assertTrue(bare.IsValid())
        self.assertFalse(bare.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertEqual(len(bare.GetChildren()), 0)

        physics = stage.GetDefaultPrim().GetChild("Physics")
        self.assertEqual({child.GetName() for child in physics.GetChildren()}, {"root_joint", "solid_joint"})

    def test_visual_only_fixed_link_retargets_a_later_revolute_joint(self):
        """
        A revolute joint below a visual-only fixed link targets the rigid-body ancestor.
        """
        input_path = "tests/data/visual_only_chain.urdf"
        output_dir = self.tmpDir()

        with usdex.test.ScopedDiagnosticChecker(self, [], level=usdex.core.DiagnosticsLevel.eWarning):
            asset_path = urdf_usd_converter.Converter().convert(input_path, output_dir)

        stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)
        geometry = stage.GetDefaultPrim().GetChild("Geometry")
        base = geometry.GetChild("base")
        drawn = base.GetChild("drawn")
        link_box = drawn.GetChild("link_box")
        self.assertFalse(drawn.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(link_box.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(link_box.HasAPI(UsdPhysics.MassAPI))

        physics = stage.GetDefaultPrim().GetChild("Physics")
        self.assertFalse(physics.GetChild("drawn_joint").IsValid())
        joint = UsdPhysics.RevoluteJoint(physics.GetChild("box_joint"))
        self.assertTrue(joint.GetPrim().IsValid())
        self.assertEqual(joint.GetBody0Rel().GetTargets(), ["/visual_only_chain/Geometry/base"])
        self.assertEqual(joint.GetBody1Rel().GetTargets(), ["/visual_only_chain/Geometry/base/drawn/link_box"])
        self.assertTrue(Gf.IsClose(joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0.3), 1e-6))

    def test_root_visual_only_link_stays_a_rigid_body(self):
        """
        A visual-only kinematic root stays a rigid body without mass.

        `root_joint` welds it to the world, so its mass does not matter.
        """
        input_path = "tests/data/visual_only_root.urdf"
        output_dir = self.tmpDir()

        with usdex.test.ScopedDiagnosticChecker(self, [], level=usdex.core.DiagnosticsLevel.eWarning):
            asset_path = urdf_usd_converter.Converter().convert(input_path, output_dir)

        stage = Usd.Stage.Open(asset_path.path)
        base = stage.GetDefaultPrim().GetChild("Geometry").GetChild("base")
        self.assertTrue(base.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(base.HasAPI(UsdPhysics.MassAPI))

    def test_revolute_visual_only_link_warns(self):
        """
        A visual-only link on a movable joint warns and stays a rigid body without mass.

        Revolute stands for every non-fixed joint. The check only distinguishes fixed from the rest.
        """
        input_path = "tests/data/visual_only_revolute.urdf"
        output_dir = self.tmpDir()
        warning = ".*Link 'arm' has no collision or positive-mass inertial, including its descendants, and is connected by a revolute joint.*"

        with usdex.test.ScopedDiagnosticChecker(
            self,
            [(Tf.TF_DIAGNOSTIC_WARNING_TYPE, warning)],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            asset_path = urdf_usd_converter.Converter().convert(input_path, output_dir)

        stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)
        arm = stage.GetDefaultPrim().GetChild("Geometry").GetChild("base").GetChild("arm")
        self.assertTrue(arm.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(arm.HasAPI(UsdPhysics.MassAPI))
        self.assertTrue(arm.GetChild("box").IsA(UsdGeom.Cube))
        self.assertTrue(stage.GetDefaultPrim().GetChild("Physics").GetChild("arm_joint").IsValid())

    def test_revolute_visual_only_link_fails_when_strict(self):
        """
        strict fails conversion for a visual-only link on a movable joint.
        """
        input_path = "tests/data/visual_only_revolute.urdf"
        with self.assertRaisesRegex(
            ValueError,
            "Link 'arm' has no collision or positive-mass inertial, including its descendants, and is connected by a revolute joint",
        ):
            urdf_usd_converter.Converter(strict=True).convert(input_path, self.tmpDir())

    def test_mimic_referenced_visual_only_fixed_link_warns(self):
        """
        A mimic-referenced visual-only link on a fixed joint warns and stays a rigid body.
        """
        input_path = "tests/data/visual_only_mimic.urdf"
        output_dir = self.tmpDir()
        warning = ".*Link 'pad' has no collision or positive-mass inertial, including its descendants, and is the child of fixed joint 'pad_joint'.*"

        with usdex.test.ScopedDiagnosticChecker(
            self,
            [(Tf.TF_DIAGNOSTIC_WARNING_TYPE, warning)],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            asset_path = urdf_usd_converter.Converter().convert(input_path, output_dir)

        stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)
        pad = stage.GetDefaultPrim().GetChild("Geometry").GetChild("base").GetChild("pad")
        self.assertTrue(pad.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(pad.HasAPI(UsdPhysics.MassAPI))
        self.assertTrue(stage.GetDefaultPrim().GetChild("Physics").GetChild("pad_joint").IsValid())

    def test_mimic_referenced_visual_only_fixed_link_fails_when_strict(self):
        """
        strict fails conversion for a mimic-referenced visual-only link on a fixed joint.
        """
        input_path = "tests/data/visual_only_mimic.urdf"
        with self.assertRaisesRegex(
            ValueError,
            "Link 'pad' has no collision or positive-mass inertial, including its descendants, and is the child of fixed joint 'pad_joint'",
        ):
            urdf_usd_converter.Converter(strict=True).convert(input_path, self.tmpDir())

    def test_fixed_inertial_child_supplies_mass_for_a_visual_only_link(self):
        """
        A visual-only link keeps its rigid body when a fixed inertial-only child provides the mass.
        """
        input_path = "tests/data/visual_only_dummy.urdf"
        output_dir = self.tmpDir()

        with usdex.test.ScopedDiagnosticChecker(self, [], level=usdex.core.DiagnosticsLevel.eWarning):
            asset_path = urdf_usd_converter.Converter().convert(input_path, output_dir)

        stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)
        pad = stage.GetDefaultPrim().GetChild("Geometry").GetChild("base").GetChild("pad")
        self.assertTrue(pad.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(pad.HasAPI(UsdPhysics.MassAPI))
        self.assertAlmostEqual(UsdPhysics.MassAPI(pad).GetMassAttr().Get(), 0.2, places=6)
        self.assertFalse(pad.GetChild("pad_inertia").IsValid())
        physics = stage.GetDefaultPrim().GetChild("Physics")
        self.assertTrue(physics.GetChild("pad_joint").IsValid())
        self.assertFalse(physics.GetChild("pad_inertia_joint").IsValid())

    def test_empty_link_on_a_revolute_joint_warns(self):
        """
        An empty link on a movable joint warns the same way a visual-only link does.
        """
        input_path = "tests/data/mass_source_empty_revolute.urdf"
        output_dir = self.tmpDir()
        warning = ".*Link 'empty' has no collision or positive-mass inertial, including its descendants, and is connected by a revolute joint.*"

        with usdex.test.ScopedDiagnosticChecker(
            self,
            [(Tf.TF_DIAGNOSTIC_WARNING_TYPE, warning)],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            asset_path = urdf_usd_converter.Converter().convert(input_path, output_dir)

        stage = Usd.Stage.Open(asset_path.path)
        empty = stage.GetDefaultPrim().GetChild("Geometry").GetChild("base").GetChild("empty")
        self.assertTrue(empty.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(empty.HasAPI(UsdPhysics.MassAPI))
        self.assertTrue(stage.GetDefaultPrim().GetChild("Physics").GetChild("empty_joint").IsValid())

    def test_empty_link_on_a_revolute_joint_fails_when_strict(self):
        """
        strict fails conversion for an empty link on a movable joint.
        """
        input_path = "tests/data/mass_source_empty_revolute.urdf"
        with self.assertRaisesRegex(
            ValueError,
            "Link 'empty' has no collision or positive-mass inertial, including its descendants, and is connected by a revolute joint",
        ):
            urdf_usd_converter.Converter(strict=True).convert(input_path, self.tmpDir())

    def test_empty_link_between_joints_stays_silent_when_a_descendant_has_mass(self):
        """
        An empty link on a revolute joint stays silent when a descendant has positive mass.
        """
        input_path = "tests/data/mass_source_empty_chain.urdf"
        output_dir = self.tmpDir()

        with usdex.test.ScopedDiagnosticChecker(self, [], level=usdex.core.DiagnosticsLevel.eWarning):
            asset_path = urdf_usd_converter.Converter().convert(input_path, output_dir)

        stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)
        empty = stage.GetDefaultPrim().GetChild("Geometry").GetChild("base").GetChild("empty")
        arm = empty.GetChild("arm")
        self.assertTrue(empty.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(empty.HasAPI(UsdPhysics.MassAPI))
        self.assertTrue(arm.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(arm.HasAPI(UsdPhysics.MassAPI))
        physics = stage.GetDefaultPrim().GetChild("Physics")
        self.assertTrue(physics.GetChild("empty_joint").IsValid())
        self.assertTrue(physics.GetChild("arm_joint").IsValid())

    def test_empty_link_warns_when_its_fixed_visual_child_supplies_no_mass(self):
        """
        A visual-only child on a fixed joint does not give its movable parent a mass source.
        """
        input_path = "tests/data/mass_source_empty_visual.urdf"
        output_dir = self.tmpDir()
        warning = ".*Link 'empty' has no collision or positive-mass inertial, including its descendants, and is connected by a revolute joint.*"

        with usdex.test.ScopedDiagnosticChecker(
            self,
            [(Tf.TF_DIAGNOSTIC_WARNING_TYPE, warning)],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            asset_path = urdf_usd_converter.Converter().convert(input_path, output_dir)

        stage = Usd.Stage.Open(asset_path.path)
        empty = stage.GetDefaultPrim().GetChild("Geometry").GetChild("base").GetChild("empty")
        drawn = empty.GetChild("drawn")
        self.assertTrue(empty.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(empty.HasAPI(UsdPhysics.MassAPI))
        self.assertFalse(drawn.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(drawn.GetChild("box").IsA(UsdGeom.Cube))
        physics = stage.GetDefaultPrim().GetChild("Physics")
        self.assertTrue(physics.GetChild("empty_joint").IsValid())
        self.assertFalse(physics.GetChild("drawn_joint").IsValid())

    def test_visual_only_link_stays_silent_when_a_descendant_has_mass(self):
        """
        A visual-only link on a revolute joint stays silent when a fixed child has positive mass.

        The child also has a visual, so its mass stays on the child rather than being copied onto the parent.
        """
        input_path = "tests/data/mass_source_descendant.urdf"
        output_dir = self.tmpDir()

        with usdex.test.ScopedDiagnosticChecker(self, [], level=usdex.core.DiagnosticsLevel.eWarning):
            asset_path = urdf_usd_converter.Converter().convert(input_path, output_dir)

        stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)
        drawn = stage.GetDefaultPrim().GetChild("Geometry").GetChild("base").GetChild("drawn")
        tool = drawn.GetChild("tool")
        self.assertTrue(drawn.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(drawn.HasAPI(UsdPhysics.MassAPI))
        self.assertTrue(tool.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(tool.HasAPI(UsdPhysics.MassAPI))
        self.assertAlmostEqual(UsdPhysics.MassAPI(tool).GetMassAttr().Get(), 0.4, places=6)
        physics = stage.GetDefaultPrim().GetChild("Physics")
        self.assertTrue(physics.GetChild("drawn_joint").IsValid())
        self.assertTrue(physics.GetChild("tool_joint").IsValid())

    def test_visual_only_link_between_world_and_arm_keeps_the_articulation(self):
        """
        A visual-only link stays without a rigid body, and the arm stays in the articulation.

        ArticulationRootAPI is applied to the non-body root. Putting it on the arm would
        exclude the revolute joint to the world, and Newton would build no articulation.
        """
        input_path = "tests/data/articulation_root_visual_world.urdf"
        output_dir = self.tmpDir()

        with usdex.test.ScopedDiagnosticChecker(self, [], level=usdex.core.DiagnosticsLevel.eWarning):
            asset_path = urdf_usd_converter.Converter().convert(input_path, output_dir)

        stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)
        default_prim = stage.GetDefaultPrim()
        self.assertFalse(default_prim.HasAPI(UsdPhysics.ArticulationRootAPI))

        world = default_prim.GetChild("Geometry").GetChild("world")
        drawn = world.GetChild("drawn")
        arm = drawn.GetChild("arm")

        self.assertFalse(world.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(world.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertTrue(world.HasAPI("NewtonArticulationRootAPI"))

        self.assertFalse(drawn.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(drawn.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertTrue(drawn.GetChild("box").IsA(UsdGeom.Cube))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(drawn).GetTranslation(), Gf.Vec3d(0, 0, 0.1), 1e-6))

        self.assertTrue(arm.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(arm.HasAPI(UsdPhysics.MassAPI))
        self.assertFalse(arm.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(arm.HasAPI("NewtonArticulationRootAPI"))

        physics = default_prim.GetChild("Physics")
        self.assertEqual({child.GetName() for child in physics.GetChildren()}, {"j2"})
        joint = UsdPhysics.RevoluteJoint(physics.GetChild("j2"))
        self.assertEqual(joint.GetBody0Rel().GetTargets(), ["/articulation_root_visual_world"])
        self.assertEqual(joint.GetBody1Rel().GetTargets(), ["/articulation_root_visual_world/Geometry/world/drawn/arm"])
        self.assertTrue(Gf.IsClose(joint.GetLocalPos0Attr().Get(), Gf.Vec3f(0, 0, 0.3), 1e-6))
