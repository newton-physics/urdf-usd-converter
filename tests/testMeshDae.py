# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import usdex.core
import usdex.test
from pxr import Tf, Usd, UsdGeom

import urdf_usd_converter
from tests.util.ConverterTestCase import ConverterTestCase


class TestMeshDae(ConverterTestCase):
    def test_dae_box_with_line(self):
        input_path = "tests/data/dae_box_with_line.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Unsupported primitive type:.*LineSet for geometry:.*"),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            asset_path = converter.convert(input_path, output_dir)

        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        default_prim = stage.GetDefaultPrim()
        geometry_scope_prim = stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Geometry"))
        self.assertTrue(geometry_scope_prim.IsValid())

        box_with_line_path = geometry_scope_prim.GetPath().AppendChild("link1").AppendChild("box_with_line")
        box_with_line_prim = stage.GetPrimAtPath(box_with_line_path)
        self.assertTrue(box_with_line_prim.IsValid())
        self.assertTrue(box_with_line_prim.IsA(UsdGeom.Xform))
        self.assertTrue(box_with_line_prim.HasAuthoredReferences())

        cube_prim_path = box_with_line_path.AppendChild("Cube")
        cube_prim = stage.GetPrimAtPath(cube_prim_path)
        self.assertTrue(cube_prim.IsValid())

        # Check whether the prim for the line is not output and only one mesh exists.
        child_count = len(box_with_line_prim.GetChildren())
        self.assertEqual(child_count, 1)
