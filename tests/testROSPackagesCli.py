# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib
import shutil
from unittest.mock import patch

import usdex.test
from pxr import Tf, Usd, UsdGeom, UsdShade

from tests.util.ConverterTestCase import ConverterTestCase
from urdf_usd_converter._impl.cli import run


class TestROSPackagesCli(ConverterTestCase):
    def test_do_not_specify_ros_package_name(self):
        """
        If the `package` argument is not specified in `converter.convert`.
        In this case, if the mesh or texture URI specifies "package://PackageName/foo/test.png",
        and the relative path "foo/test.png" exists, PackageName="" is assigned and automatically resolved.
        """
        input_path = "tests/data/ros_packages.urdf"
        output_dir = self.tmpDir()

        with (
            patch("sys.argv", ["urdf_usd_converter", input_path, output_dir]),
            usdex.test.ScopedDiagnosticChecker(
                self,
                [
                    (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Textures are not projection mapped for Cube, Sphere, and Cylinder:.*"),
                ],
                level=usdex.core.DiagnosticsLevel.eWarning,
            ),
        ):
            self.assertEqual(run(), 0, f"Failed to convert {input_path}")

        # Check the USD file after converting ros_packages.urdf.
        output_path = pathlib.Path(output_dir) / "ros_packages.usda"
        self.check_usd_converted_from_urdf(output_path)

    def test_specify_ros_package_names(self):
        """
        Specify ROS package arguments as CLI
        """
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

        with (
            patch(
                "sys.argv",
                [
                    "urdf_usd_converter",
                    input_path,
                    output_dir,
                    "--package",
                    "test_package=" + test_package_dir,
                    "--package",
                    "test_texture_package=" + test_texture_package_dir,
                ],
            ),
            usdex.test.ScopedDiagnosticChecker(
                self,
                [
                    (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Textures are not projection mapped for Cube, Sphere, and Cylinder:.*"),
                ],
                level=usdex.core.DiagnosticsLevel.eWarning,
            ),
        ):
            self.assertEqual(run(), 0, f"Failed to convert {input_path}")

        # Check the USD file after converting ros_packages.urdf.
        output_path = pathlib.Path(output_dir) / "ros_packages.usda"
        self.check_usd_converted_from_urdf(output_path)

    def test_do_not_specify_ros_package_with_relative_path(self):
        """
        If the `package` argument is not specified in `converter.convert`.
        ROS package name resolution is performed automatically.

        Search for each mesh and texture from the relative path one directory up from the current directory,
        starting from `ros_packages.urdf` within the following directory structure.

        [temp]
          [urdf]
            ros_packages.urdf
          [assets]
            box.stl
            grid.png
        """
        temp_path = pathlib.Path(self.tmpDir())
        urdf_dir = temp_path / "urdf"
        mesh_dir = temp_path / "assets"
        texture_dir = temp_path / "assets"
        output_dir = temp_path / "output"
        input_path = urdf_dir / "ros_packages.urdf"

        urdf_dir.mkdir(parents=True, exist_ok=True)
        mesh_dir.mkdir(parents=True, exist_ok=True)
        texture_dir.mkdir(parents=True, exist_ok=True)
        output_dir.mkdir(parents=True, exist_ok=True)

        shutil.copy("tests/data/ros_packages.urdf", urdf_dir)
        shutil.copy("tests/data/assets/box.stl", mesh_dir)
        shutil.copy("tests/data/assets/grid.png", texture_dir)

        with (
            patch(
                "sys.argv",
                [
                    "urdf_usd_converter",
                    str(input_path),
                    str(output_dir),
                ],
            ),
            usdex.test.ScopedDiagnosticChecker(
                self,
                [
                    (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Textures are not projection mapped for Cube, Sphere, and Cylinder:.*"),
                ],
                level=usdex.core.DiagnosticsLevel.eWarning,
            ),
        ):
            self.assertEqual(run(), 0, f"Failed to convert {input_path}")

        # Check the USD file after converting ros_packages.urdf.
        output_path = output_dir / "ros_packages.usda"
        self.check_usd_converted_from_urdf(output_path)

    def check_usd_converted_from_urdf(self, usd_path: pathlib.Path):
        """
        Perform checks on the USD file after converting ros_packages.urdf.

        Args:
            usd_path: The path to the USD file.
        """
        self.assertTrue(usd_path.exists())

        self.stage: Usd.Stage = Usd.Stage.Open(str(usd_path))
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
        material_scope_prim = default_prim.GetChild("Materials")
        self.assertTrue(material_scope_prim.IsValid())
        self.assertTrue(material_scope_prim.IsA(UsdGeom.Scope))

        texture_material_prim = material_scope_prim.GetChild("texture_material")
        self.assertTrue(texture_material_prim.IsValid())
        self.assertTrue(texture_material_prim.IsA(UsdShade.Material))

        texture_material = UsdShade.Material(texture_material_prim)
        self.assertTrue(texture_material)
        self.assertTrue(texture_material.GetPrim().HasAuthoredReferences())

        texture_path = self.get_material_texture_path(texture_material, "diffuseColor")
        self.assertEqual(texture_path, pathlib.Path("./Textures/grid.png"))
        diffuse_color = self.get_material_diffuse_color(texture_material)
        self.assertEqual(diffuse_color, None)
        opacity = self.get_material_opacity(texture_material)
        self.assertAlmostEqual(opacity, 1.0, places=6)
