# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib
import shutil

import usdex.test
from pxr import Gf, Tf, Usd, UsdGeom, UsdShade

import urdf_usd_converter
from tests.util.ConverterTestCase import ConverterTestCase


class TestMaterial(ConverterTestCase):
    def test_material_color(self):
        input_path = "tests/data/material_color.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)

        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        # Check materials.
        default_prim = stage.GetDefaultPrim()
        material_scope_prim = default_prim.GetChild("Materials")
        self.assertTrue(material_scope_prim.IsValid())
        self.assertTrue(material_scope_prim.IsA(UsdGeom.Scope))

        red_material_prim = material_scope_prim.GetChild("red")
        self.assertTrue(red_material_prim.IsValid())
        self.assertTrue(red_material_prim.IsA(UsdShade.Material))

        red_material = UsdShade.Material(red_material_prim)
        self.assertTrue(red_material)
        self.assertTrue(red_material.GetPrim().HasAuthoredReferences())

        diffuse_color = self.get_material_diffuse_color(red_material)
        self.assertEqual(diffuse_color, Gf.Vec3f(1, 0, 0))
        opacity = self.get_material_opacity(red_material)
        self.assertEqual(opacity, 1.0)

        green_material_prim = material_scope_prim.GetChild("green")
        self.assertTrue(green_material_prim.IsValid())
        self.assertTrue(green_material_prim.IsA(UsdShade.Material))

        green_material = UsdShade.Material(green_material_prim)
        self.assertTrue(green_material)
        self.assertTrue(green_material.GetPrim().HasAuthoredReferences())

        diffuse_color = self.get_material_diffuse_color(green_material)
        self.assertEqual(diffuse_color, Gf.Vec3f(0, 1, 0))
        opacity = self.get_material_opacity(green_material)
        self.assertEqual(opacity, 1.0)

        opacity_half_material_prim = material_scope_prim.GetChild("opacity_half")
        self.assertTrue(opacity_half_material_prim.IsValid())
        self.assertTrue(opacity_half_material_prim.IsA(UsdShade.Material))

        opacity_half_material = UsdShade.Material(opacity_half_material_prim)
        self.assertTrue(opacity_half_material)
        self.assertTrue(opacity_half_material.GetPrim().HasAuthoredReferences())

        # Diffuse Color is stored in Linear format, so it is converted from Linear to sRGB.
        diffuse_color = self.get_material_diffuse_color(opacity_half_material)
        diffuse_color = usdex.core.linearToSrgb(diffuse_color)
        self.assertTrue(Gf.IsClose(diffuse_color, Gf.Vec3f(0.2, 0.5, 1), 1e-6))
        opacity = self.get_material_opacity(opacity_half_material)
        self.assertEqual(opacity, 0.5)

        # Check the material bindings.
        geometry_scope_prim = default_prim.GetChild("Geometry")
        self.assertTrue(geometry_scope_prim.IsValid())
        self.assertTrue(geometry_scope_prim.IsA(UsdGeom.Scope))

        link_box_red_prim = geometry_scope_prim.GetChild("link_box_red")
        self.assertTrue(link_box_red_prim.IsValid())
        self.assertTrue(link_box_red_prim.IsA(UsdGeom.Xform))

        box_prim = link_box_red_prim.GetChild("box")
        self.assertTrue(box_prim.IsValid())
        self.assertTrue(box_prim.IsA(UsdGeom.Cube))
        self.check_material_binding(box_prim, red_material)

        link_box_green_prim = link_box_red_prim.GetChild("link_box_green")
        self.assertTrue(link_box_green_prim.IsValid())
        self.assertTrue(link_box_green_prim.IsA(UsdGeom.Xform))

        box_prim = link_box_green_prim.GetChild("box")
        self.assertTrue(box_prim.IsValid())
        self.assertTrue(box_prim.IsA(UsdGeom.Cube))
        self.check_material_binding(box_prim, green_material)

        link_box_opacity_half_prim = link_box_green_prim.GetChild("link_box_opacity_half")
        self.assertTrue(link_box_opacity_half_prim.IsValid())
        self.assertTrue(link_box_opacity_half_prim.IsA(UsdGeom.Xform))

        box_prim = link_box_opacity_half_prim.GetChild("box")
        self.assertTrue(box_prim.IsValid())
        self.assertTrue(box_prim.IsA(UsdGeom.Cube))
        self.check_material_binding(box_prim, opacity_half_material)

    def test_material_texture(self):
        input_path = "tests/data/material_texture.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()

        # A warning will appear when performing texture mapping on cubes, spheres, and cylinders.
        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Textures are not projection mapped for Cube, Sphere, and Cylinder:.*"),
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Textures are not projection mapped for Cube, Sphere, and Cylinder:.*"),
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Textures are not projection mapped for Cube, Sphere, and Cylinder:.*"),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            asset_path = converter.convert(input_path, output_dir)

        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        # Check texture.
        output_texture_path = pathlib.Path(output_dir) / "Payload" / "Textures" / "grid.png"
        self.assertTrue(output_texture_path.exists())

        # Check materials.
        default_prim = stage.GetDefaultPrim()
        material_scope_prim = default_prim.GetChild("Materials")
        self.assertTrue(material_scope_prim.IsValid())
        self.assertTrue(material_scope_prim.IsA(UsdGeom.Scope))

        texture_material_prim = material_scope_prim.GetChild("texture_material")
        self.assertTrue(texture_material_prim.IsValid())
        self.assertTrue(texture_material_prim.IsA(UsdShade.Material))

        texture_material = UsdShade.Material(texture_material_prim)
        self.assertTrue(texture_material)
        self.assertTrue(texture_material.GetPrim().HasAuthoredReferences())

        diffuse_color = self.get_material_diffuse_color(texture_material)
        self.assertEqual(diffuse_color, None)
        opacity = self.get_material_opacity(texture_material)
        self.assertEqual(opacity, 1.0)
        diffuse_color_texture_path = self.get_material_diffuse_color_texture_path(texture_material)
        self.assertEqual(diffuse_color_texture_path, pathlib.Path("./Textures/grid.png"))

        color_texture_material_prim = material_scope_prim.GetChild("color_texture_material")
        self.assertTrue(color_texture_material_prim.IsValid())
        self.assertTrue(color_texture_material_prim.IsA(UsdShade.Material))

        color_texture_material = UsdShade.Material(color_texture_material_prim)
        self.assertTrue(color_texture_material)
        self.assertTrue(color_texture_material.GetPrim().HasAuthoredReferences())

        opacity = self.get_material_opacity(color_texture_material)
        self.assertEqual(opacity, 1.0)

        # This texture is multiplied, so the color value is obtained from the scale.
        diffuse_color = self.get_material_diffuse_color_texture_scale(color_texture_material)
        diffuse_color = Gf.Vec3f(*diffuse_color[:3])
        diffuse_color = usdex.core.linearToSrgb(diffuse_color)
        self.assertTrue(Gf.IsClose(diffuse_color, Gf.Vec3f(0.5, 0.2, 0.5), 1e-6))

        geometry_scope_prim = default_prim.GetChild("Geometry")
        self.assertTrue(geometry_scope_prim.IsValid())
        self.assertTrue(geometry_scope_prim.IsA(UsdGeom.Scope))

        link_box_prim = geometry_scope_prim.GetChild("link_box")
        self.assertTrue(link_box_prim.IsValid())
        self.assertTrue(link_box_prim.IsA(UsdGeom.Xform))

        box_prim = link_box_prim.GetChild("box")
        self.assertTrue(box_prim.IsValid())
        self.assertTrue(box_prim.IsA(UsdGeom.Cube))
        self.check_material_binding(box_prim, texture_material)

        link_sphere_prim = link_box_prim.GetChild("link_sphere")
        self.assertTrue(link_sphere_prim.IsValid())
        self.assertTrue(link_sphere_prim.IsA(UsdGeom.Xform))

        sphere_prim = link_sphere_prim.GetChild("sphere")
        self.assertTrue(sphere_prim.IsValid())
        self.assertTrue(sphere_prim.IsA(UsdGeom.Sphere))
        self.check_material_binding(sphere_prim, texture_material)

        link_cylinder_prim = link_sphere_prim.GetChild("link_cylinder")
        self.assertTrue(link_cylinder_prim.IsValid())
        self.assertTrue(link_cylinder_prim.IsA(UsdGeom.Xform))

        cylinder_prim = link_cylinder_prim.GetChild("cylinder")
        self.assertTrue(cylinder_prim.IsValid())
        self.assertTrue(cylinder_prim.IsA(UsdGeom.Cylinder))
        self.check_material_binding(cylinder_prim, texture_material)

        link_obj_texture_prim = link_cylinder_prim.GetChild("link_obj_texture")
        self.assertTrue(link_obj_texture_prim.IsValid())
        self.assertTrue(link_obj_texture_prim.IsA(UsdGeom.Xform))

        obj_prim = link_obj_texture_prim.GetChild("box")
        self.assertTrue(obj_prim.IsValid())
        self.assertTrue(obj_prim.IsA(UsdGeom.Mesh))
        self.check_material_binding(obj_prim, texture_material)

        link_obj_color_texture_prim = link_obj_texture_prim.GetChild("link_obj_color_texture")
        self.assertTrue(link_obj_color_texture_prim.IsValid())
        self.assertTrue(link_obj_color_texture_prim.IsA(UsdGeom.Xform))

        obj_prim = link_obj_color_texture_prim.GetChild("box")
        self.assertTrue(obj_prim.IsValid())
        self.assertTrue(obj_prim.IsA(UsdGeom.Mesh))
        self.check_material_binding(obj_prim, color_texture_material)

    def test_material_texture_name_duplication_missing_texture(self):
        input_path = "tests/data/material_texture_name_duplication.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Texture file not found:.*"),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            asset_path = converter.convert(input_path, output_dir)

        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        # Check texture.
        output_texture_path = pathlib.Path(output_dir) / "Payload" / "Textures" / "grid.png"
        self.assertTrue(output_texture_path.exists())

        # Confirm that this non-existent texture is not being output.
        output_texture_path = pathlib.Path(output_dir) / "Payload" / "Textures" / "grid_1.png"
        self.assertFalse(output_texture_path.exists())

    def test_material_texture_name_duplication(self):
        """
        Place a structure containing the same-named texture "grid.png" in the temporary directory.
        In this case, the converted textures will be placed as "grid.png" and "grid_1.png" in the "Textures" directory.

        [temp]
          material_texture_name_duplication.urdf
          [assets]
            box.obj
            grid.png
            [textures]
              grid.png
        """
        temp_path = pathlib.Path(self.tmpDir())
        input_path = temp_path / "material_texture_name_duplication.urdf"
        assets_dir = temp_path / "assets"
        assets_textures_dir = temp_path / "assets" / "textures"
        output_dir = temp_path / "output"

        assets_dir.mkdir(parents=True, exist_ok=True)
        assets_textures_dir.mkdir(parents=True, exist_ok=True)
        output_dir.mkdir(parents=True, exist_ok=True)

        shutil.copy("tests/data/material_texture_name_duplication.urdf", temp_path)
        shutil.copy("tests/data/assets/box.obj", assets_dir)
        shutil.copy("tests/data/assets/grid.png", assets_dir)
        shutil.copy("tests/data/assets/grid.png", assets_textures_dir)

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)

        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        # Check texture.
        output_texture_path = pathlib.Path(output_dir) / "Payload" / "Textures" / "grid.png"
        self.assertTrue(output_texture_path.exists())

        output_texture_path = pathlib.Path(output_dir) / "Payload" / "Textures" / "grid_1.png"
        self.assertTrue(output_texture_path.exists())

    def test_material_mesh_color(self):
        input_path = "tests/data/material_mesh_color.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)

        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        default_prim = stage.GetDefaultPrim()
        geometry_scope_prim = default_prim.GetChild("Geometry")
        self.assertTrue(geometry_scope_prim.IsValid())
        self.assertTrue(geometry_scope_prim.IsA(UsdGeom.Scope))

        link_box_prim = geometry_scope_prim.GetChild("link_box")
        self.assertTrue(link_box_prim.IsValid())
        self.assertTrue(link_box_prim.IsA(UsdGeom.Xform))

        link_obj_prim = link_box_prim.GetChild("link_obj")
        self.assertTrue(link_obj_prim.IsValid())
        self.assertTrue(link_obj_prim.IsA(UsdGeom.Xform))

        two_boxes_prim = link_obj_prim.GetChild("two_boxes")
        self.assertTrue(two_boxes_prim.IsValid())
        self.assertTrue(two_boxes_prim.IsA(UsdGeom.Xform))
        self.assertTrue(two_boxes_prim.HasAuthoredReferences())

        # Check the material in the geometry.
        material_scope_prim = two_boxes_prim.GetChild("Materials")
        self.assertTrue(material_scope_prim.IsValid())

        green_material_prim = material_scope_prim.GetChild("green_mat")
        self.assertTrue(green_material_prim.IsValid())
        self.assertTrue(green_material_prim.IsA(UsdShade.Material))
        green_material = UsdShade.Material(green_material_prim)
        self.assertTrue(green_material)

        diffuse_color = self.get_material_diffuse_color(green_material)
        diffuse_color = usdex.core.linearToSrgb(diffuse_color)
        self.assertTrue(Gf.IsClose(diffuse_color, Gf.Vec3f(0, 1, 0), 1e-6))
        opacity = self.get_material_opacity(green_material)
        self.assertEqual(opacity, 1.0)

        red_material_prim = material_scope_prim.GetChild("red_mat")
        self.assertTrue(red_material_prim.IsValid())
        self.assertTrue(red_material_prim.IsA(UsdShade.Material))
        red_material = UsdShade.Material(red_material_prim)
        self.assertTrue(red_material)

        diffuse_color = self.get_material_diffuse_color(red_material)
        diffuse_color = usdex.core.linearToSrgb(diffuse_color)
        self.assertTrue(Gf.IsClose(diffuse_color, Gf.Vec3f(1, 0, 0), 1e-6))
        opacity = self.get_material_opacity(red_material)
        self.assertEqual(opacity, 1.0)

        mesh_prim = two_boxes_prim.GetChild("Cube_Green")
        self.assertTrue(mesh_prim.IsValid())
        self.assertTrue(mesh_prim.IsA(UsdGeom.Mesh))
        self.check_material_binding(mesh_prim, green_material)

        mesh_prim = two_boxes_prim.GetChild("Cube_Red")
        self.assertTrue(mesh_prim.IsValid())
        self.assertTrue(mesh_prim.IsA(UsdGeom.Mesh))
        self.check_material_binding(mesh_prim, red_material)

    def test_material_mesh_texture(self):
        input_path = "tests/data/material_mesh_texture.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)

        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        # Check texture.
        output_texture_path = pathlib.Path(output_dir) / "Payload" / "Textures" / "grid.png"
        self.assertTrue(output_texture_path.exists())

        default_prim = stage.GetDefaultPrim()
        geometry_scope_prim = default_prim.GetChild("Geometry")
        self.assertTrue(geometry_scope_prim.IsValid())
        self.assertTrue(geometry_scope_prim.IsA(UsdGeom.Scope))

        link_box_prim = geometry_scope_prim.GetChild("link_box")
        self.assertTrue(link_box_prim.IsValid())
        self.assertTrue(link_box_prim.IsA(UsdGeom.Xform))

        link_obj_prim = link_box_prim.GetChild("link_obj")
        self.assertTrue(link_obj_prim.IsValid())
        self.assertTrue(link_obj_prim.IsA(UsdGeom.Xform))

        box_with_texture_prim = link_obj_prim.GetChild("box_with_texture")
        self.assertTrue(box_with_texture_prim.IsValid())
        self.assertTrue(box_with_texture_prim.IsA(UsdGeom.Xform))
        self.assertTrue(box_with_texture_prim.HasAuthoredReferences())

        # Check the material in the geometry.
        material_scope_prim = box_with_texture_prim.GetChild("Materials")
        self.assertTrue(material_scope_prim.IsValid())

        texture_material_prim = material_scope_prim.GetChild("texture_mat")
        self.assertTrue(texture_material_prim.IsValid())
        self.assertTrue(texture_material_prim.IsA(UsdShade.Material))
        texture_material = UsdShade.Material(texture_material_prim)
        self.assertTrue(texture_material)

        diffuse_color = self.get_material_diffuse_color(texture_material)
        self.assertIsNone(diffuse_color)
        opacity = self.get_material_opacity(texture_material)
        self.assertEqual(opacity, 1.0)
        texture_path = self.get_material_diffuse_color_texture_path(texture_material)
        self.assertEqual(texture_path, pathlib.Path("./Textures/grid.png"))

        mesh_prim = box_with_texture_prim.GetChild("box_with_texture")
        self.assertTrue(mesh_prim.IsValid())
        self.assertTrue(mesh_prim.IsA(UsdGeom.Mesh))
        self.check_material_binding(mesh_prim, texture_material)

    def test_material_mesh_override(self):
        input_path = "tests/data/material_mesh_override.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)

        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        # Check materials.
        default_prim = stage.GetDefaultPrim()
        material_scope_prim = default_prim.GetChild("Materials")
        self.assertTrue(material_scope_prim.IsValid())
        self.assertTrue(material_scope_prim.IsA(UsdGeom.Scope))

        blue_material_prim = material_scope_prim.GetChild("blue")
        self.assertTrue(blue_material_prim.IsValid())
        self.assertTrue(blue_material_prim.IsA(UsdShade.Material))

        blue_material = UsdShade.Material(blue_material_prim)
        self.assertTrue(blue_material)
        self.assertTrue(blue_material.GetPrim().HasAuthoredReferences())

        diffuse_color = self.get_material_diffuse_color(blue_material)
        self.assertEqual(diffuse_color, Gf.Vec3f(0, 0, 1))
        opacity = self.get_material_opacity(blue_material)
        self.assertEqual(opacity, 1.0)

        default_prim = stage.GetDefaultPrim()
        geometry_scope_prim = default_prim.GetChild("Geometry")
        self.assertTrue(geometry_scope_prim.IsValid())
        self.assertTrue(geometry_scope_prim.IsA(UsdGeom.Scope))

        two_boxes_prim = geometry_scope_prim.GetChild("link_box").GetChild("link_obj").GetChild("two_boxes")
        self.assertTrue(two_boxes_prim.IsValid())
        self.assertTrue(two_boxes_prim.IsA(UsdGeom.Xform))
        self.assertTrue(two_boxes_prim.HasAuthoredReferences())

        # Check that the material bind is overwritten with blue_material.
        mesh_prim = two_boxes_prim.GetChild("Cube_Red")
        self.assertTrue(mesh_prim.IsValid())
        self.assertTrue(mesh_prim.IsA(UsdGeom.Mesh))
        self.check_material_binding(mesh_prim, blue_material)

        mesh_prim = two_boxes_prim.GetChild("Cube_Green")
        self.assertTrue(mesh_prim.IsValid())
        self.assertTrue(mesh_prim.IsA(UsdGeom.Mesh))
        self.check_material_binding(mesh_prim, blue_material)
