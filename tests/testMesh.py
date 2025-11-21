# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import usdex.test
from pxr import Tf

import urdf_usd_converter
from tests.util.ConverterTestCase import ConverterTestCase


class TestMesh(ConverterTestCase):
    def test_mesh_conversion(self):
        input_path = "tests/data/meshes.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*The stl format is not yet supported:.*"),
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*The obj format is not yet supported:.*"),
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*The dae format is not yet supported:.*"),
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Unsupported mesh format:.*"),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())
