# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import urdf_usd_converter
from tests.util.ConverterTestCase import ConverterTestCase


class TestConverter(ConverterTestCase):
    def test_invalid_input(self):
        # input_path is a path that does not exist (should fail).
        input_path = "tests/data/non_existent.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        with self.assertRaisesRegex(ValueError, r".*Input file tests/data/non_existent.urdf is not a readable file.*"):
            converter.convert(input_path, output_dir)

    def test_output_path_is_file(self):
        # Specify a file instead of a directory (should fail).
        input_path = "tests/data/simple_box.urdf"
        output_dir = "tests/data/simple_box.urdf"

        converter = urdf_usd_converter.Converter()
        with self.assertRaisesRegex(ValueError, r".*Output directory tests/data/simple_box.urdf is not a directory.*"):
            converter.convert(input_path, output_dir)

    def test_output_directory_does_not_exist(self):
        # If the output directory does not exist.
        input_path = "tests/data/simple_box.urdf"
        output_dir = str(pathlib.Path(self.tmpDir()) / "non_existent_directory")

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

    def test_joint_loop_structure(self):
        # Joint loop structure error check.
        input_path = "tests/data/error_convert_loop_joint_structure.urdf"
        output_dir = str(pathlib.Path(self.tmpDir()) / "error_convert_loop_joint_structure")

        converter = urdf_usd_converter.Converter()
        with self.assertRaisesRegex(ValueError, r".*Closed loop articulations are not supported.*"):
            converter.convert(input_path, output_dir)
