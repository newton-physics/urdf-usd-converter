# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib
import shutil
from unittest.mock import patch

import usdex.test
from pxr import Sdf, Tf, Usd

from tests.util.ConverterTestCase import ConverterTestCase
from urdf_usd_converter._impl.cli import run


class TestCli(ConverterTestCase):

    def test_run(self):
        input_path = "tests/data/simple-primitives.urdf"
        with (
            patch("sys.argv", ["urdf_usd_converter", input_path, self.tmpDir()]),
            usdex.test.ScopedDiagnosticChecker(
                self,
                [
                    (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Calibration is not supported"),
                    (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Dynamics is not supported"),
                ],
                level=usdex.core.DiagnosticsLevel.eWarning,
            ),
        ):
            self.assertEqual(run(), 0, f"Failed to convert {input_path}")
        self.assertTrue((pathlib.Path(self.tmpDir()) / "simple-primitives.usda").exists())

    def test_no_layer_structure(self):
        model = "tests/data/simple_box.urdf"
        robot_name = pathlib.Path(model).stem
        with patch("sys.argv", ["urdf_usd_converter", model, self.tmpDir(), "--no-layer-structure"]):
            self.assertEqual(run(), 0, f"Failed to convert {model}")
            self.assertFalse((pathlib.Path(self.tmpDir()) / "Payload").exists())
            self.assertFalse((pathlib.Path(self.tmpDir()) / f"{robot_name}.usda").exists())
            self.assertTrue((pathlib.Path(self.tmpDir()) / f"{robot_name}.usdc").exists())

    def test_no_physics_scene(self):
        model = "tests/data/simple_box.urdf"
        robot_name = pathlib.Path(model).stem
        with patch("sys.argv", ["urdf_usd_converter", model, self.tmpDir(), "--no-physics-scene"]):
            self.assertEqual(run(), 0, f"Failed to convert {model}")
            self.assertTrue((pathlib.Path(self.tmpDir()) / f"{robot_name}.usda").exists())
            stage = Usd.Stage.Open((pathlib.Path(self.tmpDir()) / f"{robot_name}.usda").as_posix())
            self.assertFalse(stage.GetPrimAtPath("/PhysicsScene").IsValid())

    def test_comment(self):
        model = "tests/data/simple_box.urdf"
        robot_name = pathlib.Path(model).stem
        with patch("sys.argv", ["urdf_usd_converter", model, self.tmpDir(), "--comment", "from the unittests"]):
            self.assertEqual(run(), 0, f"Failed to convert {model}")
            self.assertTrue((pathlib.Path(self.tmpDir()) / f"{robot_name}.usda").exists())
            layer = Sdf.Layer.FindOrOpen((pathlib.Path(self.tmpDir()) / f"{robot_name}.usda").as_posix())
            self.assertEqual(layer.comment, "from the unittests")

    def test_invalid_input(self):
        with (
            patch("sys.argv", ["urdf_usd_converter", "tests/data/invalid.xml", self.tmpDir()]),
            usdex.test.ScopedDiagnosticChecker(self, [(Tf.TF_DIAGNOSTIC_WARNING_TYPE, "Input file does not exist.*")]),
        ):
            self.assertEqual(run(), 1, "Expected non-zero exit code for invalid input")

    def test_invalid_output(self):
        # create a file that is not a directory
        pathlib.Path("tests/output").mkdir(parents=True, exist_ok=True)
        pathlib.Path("tests/output/invalid").touch()
        with (
            patch("sys.argv", ["urdf_usd_converter", "tests/data/simple_box.urdf", "tests/output/invalid"]),
            usdex.test.ScopedDiagnosticChecker(self, [(Tf.TF_DIAGNOSTIC_WARNING_TYPE, "Output path exists but is not a directory.*")]),
        ):
            self.assertEqual(run(), 1, "Expected non-zero exit code for invalid output")

    def test_input_path_is_directory(self):
        # Create a directory as input_file (should fail)
        input_dir = pathlib.Path("tests/data/input_dir")
        input_dir.mkdir(parents=True, exist_ok=True)
        try:
            with (
                patch("sys.argv", ["urdf_usd_converter", str(input_dir), self.tmpDir()]),
                usdex.test.ScopedDiagnosticChecker(self, [(Tf.TF_DIAGNOSTIC_WARNING_TYPE, "Input path is not a file.*")]),
            ):
                self.assertEqual(run(), 1, "Expected non-zero exit code for input path as directory")
        finally:
            shutil.rmtree(input_dir)

    def test_input_file_not_xml(self):
        # Create a non-xml file as input_file (should fail)
        not_xml = pathlib.Path("tests/data/not_xml.txt")
        not_xml.write_text("dummy content")
        try:
            with (
                patch("sys.argv", ["urdf_usd_converter", str(not_xml), self.tmpDir()]),
                usdex.test.ScopedDiagnosticChecker(self, [(Tf.TF_DIAGNOSTIC_WARNING_TYPE, "Only URDF.*are supported as input.*")]),
            ):
                self.assertEqual(run(), 1, "Expected non-zero exit code for non-xml input file")
        finally:
            not_xml.unlink()

    def test_output_dir_cannot_create(self):
        # Simulate output_dir.mkdir raising an exception (should fail)
        robot = "tests/data/simple_box.urdf"
        output_dir = pathlib.Path("tests/output/cannot_create")
        with (
            patch("pathlib.Path.mkdir", side_effect=OSError("Permission denied")),
            patch("sys.argv", ["urdf_usd_converter", robot, str(output_dir)]),
            usdex.test.ScopedDiagnosticChecker(self, [(Tf.TF_DIAGNOSTIC_WARNING_TYPE, "Failed to create output directory.*")]),
        ):
            self.assertEqual(run(), 1, "Expected non-zero exit code when output dir cannot be created")

    def test_conversion_returns_none(self):
        # Test the case where converter.convert() returns None/false value
        robot = "tests/data/simple_box.urdf"
        with (
            patch("urdf_usd_converter.Converter.convert", return_value=None),
            patch("sys.argv", ["urdf_usd_converter", robot, self.tmpDir()]),
            usdex.test.ScopedDiagnosticChecker(
                self,
                [(Tf.TF_DIAGNOSTIC_WARNING_TYPE, "Conversion failed for unknown reason.*")],
                level=usdex.core.DiagnosticsLevel.eWarning,
            ),
        ):
            self.assertEqual(run(), 1, "Expected non-zero exit code when conversion returns None")

    def test_conversion_exception_non_verbose(self):
        # Test exception handling when verbose=False (should not re-raise)
        robot = "tests/data/simple_box.urdf"
        with (
            patch("urdf_usd_converter.Converter.convert", side_effect=RuntimeError("Test conversion error")),
            patch("sys.argv", ["urdf_usd_converter", robot, self.tmpDir()]),
            usdex.test.ScopedDiagnosticChecker(
                self,
                [(Tf.TF_DIAGNOSTIC_WARNING_TYPE, "Conversion failed: Test conversion error.*")],
                level=usdex.core.DiagnosticsLevel.eWarning,
            ),
        ):
            self.assertEqual(run(), 1, "Expected non-zero exit code when conversion raises exception")

    def test_conversion_exception_verbose(self):
        # Test exception handling when verbose=True (should re-raise)
        robot = "tests/data/simple_box.urdf"
        with (
            patch("urdf_usd_converter.Converter.convert", side_effect=RuntimeError("Test conversion error")),
            patch("sys.argv", ["urdf_usd_converter", robot, self.tmpDir(), "--verbose"]),
            self.assertRaises(RuntimeError),
            usdex.test.ScopedDiagnosticChecker(self, [], level=usdex.core.DiagnosticsLevel.eWarning),
        ):
            run()

    def test_conversion_exception_ros_package_name_format(self):
        robot = "tests/data/ros_packages.urdf"
        output_dir = self.tmpDir()

        # Run the converter with the specified ROS package names.
        # Here, we will specify an incorrect CLI argument.
        package_1 = "test_package"  # Error: If no path is specified
        with (
            self.assertRaisesRegex(RuntimeError, r"Invalid format: .*. Expected format: <name>=<path>"),
            patch("sys.argv", ["urdf_usd_converter", robot, str(output_dir), "--package", package_1]),
        ):
            self.assertEqual(run(), 1, "Expected non-zero exit code for invalid input")

    def test_conversion_exception_multiple_ros_packages_name_format(self):
        robot = "tests/data/ros_packages.urdf"
        output_dir = self.tmpDir()

        # Run the converter with the specified ROS package names.
        # Here, we will specify an incorrect CLI argument.
        package_1 = "test_package=/home/foo"  # Correct specification
        package_2 = "test_texture_package="  # Error: If no path is specified
        with (
            self.assertRaisesRegex(RuntimeError, r"Invalid format: .*. Expected format: <name>=<path>"),
            patch("sys.argv", ["urdf_usd_converter", robot, str(output_dir), "--package", package_1, "--package", package_2]),
        ):
            self.assertEqual(run(), 1, "Expected non-zero exit code for invalid input")

    def test_conversion_warning_multiple_ros_packages_invalid(self):
        robot = "tests/data/ros_packages.urdf"
        robot_name = pathlib.Path(robot).stem
        output_dir = self.tmpDir()

        # Run the converter with the specified ROS package names.
        # Here, we will specify an incorrect CLI argument.
        package_1 = "test_package=package://foo=bar"  # Error: Invalid path

        with (
            patch("sys.argv", ["urdf_usd_converter", robot, str(output_dir), "--package", package_1]),
            usdex.test.ScopedDiagnosticChecker(
                self,
                [(Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Failed to convert mesh:.*")],
                level=usdex.core.DiagnosticsLevel.eWarning,
            ),
        ):
            self.assertEqual(run(), 0, "Expected non-zero exit code for invalid input")
            self.assertTrue((pathlib.Path(self.tmpDir()) / f"{robot_name}.usda").exists())

    def test_conversion_warning_https(self):
        robot = "tests/data/mesh_https.urdf"
        robot_name = pathlib.Path(robot).stem
        output_dir = self.tmpDir()

        with (
            patch("sys.argv", ["urdf_usd_converter", robot, str(output_dir)]),
            usdex.test.ScopedDiagnosticChecker(
                self,
                [(Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*'https' mesh is not yet supported:.*")],
                level=usdex.core.DiagnosticsLevel.eWarning,
            ),
        ):
            self.assertEqual(run(), 0, "Expected non-zero exit code for invalid input")
            self.assertTrue((pathlib.Path(self.tmpDir()) / f"{robot_name}.usda").exists())
