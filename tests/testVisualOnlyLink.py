# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import usdex.core
import usdex.test
from pxr import Gf, Usd, UsdGeom, UsdPhysics

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
        input_path = "tests/data/visual_only_root.urdf"
        output_dir = self.tmpDir()

        with usdex.test.ScopedDiagnosticChecker(self, [], level=usdex.core.DiagnosticsLevel.eWarning):
            asset_path = urdf_usd_converter.Converter().convert(input_path, output_dir)

        stage = Usd.Stage.Open(asset_path.path)
        base = stage.GetDefaultPrim().GetChild("Geometry").GetChild("base")
        self.assertTrue(base.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(base.HasAPI(UsdPhysics.MassAPI))

    def test_revolute_visual_only_link_fails(self):
        """
        A visual-only link on a movable joint fails conversion.

        Revolute stands for every non-fixed joint. The check only distinguishes fixed from the rest.
        """
        input_path = "tests/data/visual_only_revolute.urdf"
        with self.assertRaisesRegex(ValueError, "Link 'arm' has a visual but no inertial or collision and is connected by a revolute joint"):
            urdf_usd_converter.Converter().convert(input_path, self.tmpDir())

    def test_mimic_referenced_visual_only_fixed_link_fails(self):
        input_path = "tests/data/visual_only_mimic.urdf"
        output_dir = self.tmpDir()

        with self.assertRaisesRegex(
            ValueError,
            "Link 'pad' has a visual but no inertial or collision and is the child of fixed joint 'pad_joint'",
        ):
            urdf_usd_converter.Converter().convert(input_path, output_dir)

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
