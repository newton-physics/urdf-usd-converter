# SPDX-FileCopyrightText: Copyright (c) 2025-2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib
import shutil

import usdex.test
from pxr import Tf, Usd, UsdGeom

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

    def test_load_warning_obj_no_exist_filename(self):
        # A non-existent obj file is specified.

        input_path = "tests/data/warning_obj_no_exist_filename.urdf"
        output_dir = str(pathlib.Path(self.tmpDir()) / "warning_obj_no_exist_filename")

        converter = urdf_usd_converter.Converter()
        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*could not be parsed. Cannot open file.*"),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            converter.convert(input_path, output_dir)

    def test_load_warning_obj_no_shape(self):
        # There is no shape.

        input_path = "tests/data/warning_obj_no_shape.urdf"
        output_dir = str(pathlib.Path(self.tmpDir()) / "error_obj_no_shape")

        converter = urdf_usd_converter.Converter()
        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*contains no meshes.*"),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            converter.convert(input_path, output_dir)

    def test_load_warning_ros_package_name_without_relative_path(self):
        # When an invalid path is specified in the ROS package URI.

        input_path = "tests/data/warning_ref_ros_package.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Invalid ROS package URI. No relative path specified: package://test_package.*"),
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*No file has been specified. It is a directory:*"),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            converter.convert(input_path, output_dir)

    def test_ros_packages(self):
        # Specify ROS package arguments as Converter constructor.

        input_path = "tests/data/ros_packages.urdf"
        output_dir = self.tmpDir()

        test_package_dir = output_dir + "/temp"
        test_texture_package_dir = output_dir + "/temp/textures"
        pathlib.Path(test_package_dir + "/assets").mkdir(parents=True, exist_ok=True)
        pathlib.Path(test_texture_package_dir + "/assets").mkdir(parents=True, exist_ok=True)

        # Copy "tests/data/assets/box.stl" to test_package_dir
        shutil.copy("tests/data/assets/box.stl", test_package_dir + "/assets")

        # Copy "tests/data/assets/grid.png" to test_texture_package_dir
        shutil.copy("tests/data/assets/grid.png", test_texture_package_dir + "/assets")

        temp_stl_file_path = test_package_dir + "/assets/box.stl"
        temp_texture_file_path = test_texture_package_dir + "/assets/grid.png"
        self.assertTrue(pathlib.Path(temp_stl_file_path).exists())
        self.assertTrue(pathlib.Path(temp_texture_file_path).exists())

        packages = [
            {"name": "test_package", "path": test_package_dir},
            {"name": "test_texture_package", "path": test_texture_package_dir},
        ]
        converter = urdf_usd_converter.Converter(ros_packages=packages)
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        # If the URI specified in converter.convert is invalid, a warning will be displayed and processing will continue.
        # Therefore, this process opens the USD file to verify that the mesh has been loaded correctly.
        output_path = pathlib.Path(output_dir) / "ros_packages.usda"
        self.assertTrue(output_path.exists())

        self.stage: Usd.Stage = Usd.Stage.Open(str(output_path))
        self.assertIsValidUsd(self.stage)

        # Check geometry.
        default_prim = self.stage.GetDefaultPrim()
        geometry_scope_prim = self.stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Geometry"))
        self.assertTrue(geometry_scope_prim.IsValid())

        link_mesh_stl_path = geometry_scope_prim.GetPath().AppendChild("BaseLink").AppendChild("link_mesh_stl")
        link_stl_prim = self.stage.GetPrimAtPath(link_mesh_stl_path)
        self.assertTrue(link_stl_prim.IsValid())
        self.assertTrue(link_stl_prim.IsA(UsdGeom.Xform))

        stl_mesh_prim = self.stage.GetPrimAtPath(link_mesh_stl_path.AppendChild("box"))
        self.assertTrue(stl_mesh_prim.IsValid())
        self.assertTrue(stl_mesh_prim.IsA(UsdGeom.Mesh))
        self.assertTrue(stl_mesh_prim.HasAuthoredReferences())

        # Check material texture.
        # TODO: Here we need to make sure that the reference to the usd file is correct after the texture is loaded.

    def test_load_warning_dae_no_exist_filename(self):
        # A non-existent dae file is specified.

        input_path = "tests/data/warning_dae_no_exist_filename.urdf"
        output_dir = str(pathlib.Path(self.tmpDir()) / "warning_dae_no_exist_filename")

        converter = urdf_usd_converter.Converter()
        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*No such file or directory:.*"),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            converter.convert(input_path, output_dir)

    def test_asset_identifer(self):
        model = pathlib.Path("tests/data/prismatic_joints.urdf")
        model_name = model.stem
        output_dir = pathlib.Path(self.tmpDir()) / model_name
        usdc_path = output_dir / f"{model_name}.usdc"

        asset_identifier = urdf_usd_converter.Converter(layer_structure=False).convert(model, output_dir)
        self.assertTrue(usdc_path.exists())

        # check that the asset identifier returned from convert() is the same as the usdc path
        flattened_usdc_path = pathlib.Path(asset_identifier.path).absolute().as_posix()
        self.assertEqual(flattened_usdc_path, usdc_path.absolute().as_posix())
