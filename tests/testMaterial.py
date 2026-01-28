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
        wrap_mode = self.get_material_wrap_mode(texture_material)
        self.assertEqual(wrap_mode, "repeat")

        diffuse_color = self.get_material_diffuse_color(texture_material)
        self.assertEqual(diffuse_color, None)
        opacity = self.get_material_opacity(texture_material)
        self.assertEqual(opacity, 1.0)
        diffuse_color_texture_path = self.get_material_texture_path(texture_material, "diffuseColor")
        self.assertEqual(diffuse_color_texture_path, pathlib.Path("./Textures/grid.png"))

        color_texture_material_prim = material_scope_prim.GetChild("color_texture_material")
        self.assertTrue(color_texture_material_prim.IsValid())
        self.assertTrue(color_texture_material_prim.IsA(UsdShade.Material))

        color_texture_material = UsdShade.Material(color_texture_material_prim)
        self.assertTrue(color_texture_material)
        self.assertTrue(color_texture_material.GetPrim().HasAuthoredReferences())
        wrap_mode = self.get_material_wrap_mode(color_texture_material)
        self.assertEqual(wrap_mode, "repeat")

        opacity = self.get_material_opacity(color_texture_material)
        self.assertEqual(opacity, 1.0)

        # This texture is multiplied, so the color value is obtained from the fallback.
        diffuse_color = self.get_material_diffuse_color_texture_fallback(color_texture_material)
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

        link_obj_specular_workflow_prim = link_obj_prim.GetChild("link_obj_specular_workflow")
        self.assertTrue(link_obj_specular_workflow_prim.IsValid())
        self.assertTrue(link_obj_specular_workflow_prim.IsA(UsdGeom.Xform))

        # Check the materials.
        material_scope_prim = default_prim.GetChild("Materials")
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
        ior = self.get_material_ior(green_material)
        self.assertAlmostEqual(ior, 1.5, places=6)

        red_material_prim = material_scope_prim.GetChild("red_mat")
        self.assertTrue(red_material_prim.IsValid())
        self.assertTrue(red_material_prim.IsA(UsdShade.Material))
        red_material = UsdShade.Material(red_material_prim)
        self.assertTrue(red_material)

        diffuse_color = self.get_material_diffuse_color(red_material)
        diffuse_color = usdex.core.linearToSrgb(diffuse_color)
        self.assertTrue(Gf.IsClose(diffuse_color, Gf.Vec3f(1, 0, 0), 1e-6))
        opacity = self.get_material_opacity(red_material)
        self.assertAlmostEqual(opacity, 1.0, places=6)
        ior = self.get_material_ior(red_material)
        self.assertAlmostEqual(ior, 1.45, places=6)
        roughness = self.get_material_roughness(red_material)
        self.assertAlmostEqual(roughness, 0.3, places=6)
        metallic = self.get_material_metallic(red_material)
        self.assertAlmostEqual(metallic, 0.05, places=6)

        box_specular_workflow_prim = material_scope_prim.GetChild("specular_workflow_mat")
        self.assertTrue(box_specular_workflow_prim.IsValid())
        self.assertTrue(box_specular_workflow_prim.IsA(UsdShade.Material))
        box_specular_workflow_material = UsdShade.Material(box_specular_workflow_prim)
        self.assertTrue(box_specular_workflow_material)
        ior = self.get_material_ior(box_specular_workflow_material)
        self.assertAlmostEqual(ior, 1.45, places=6)

        mesh_prim = two_boxes_prim.GetChild("Cube_Green")
        self.assertTrue(mesh_prim.IsValid())
        self.assertTrue(mesh_prim.IsA(UsdGeom.Mesh))
        self.check_material_binding(mesh_prim, green_material)

        mesh_prim = two_boxes_prim.GetChild("Cube_Red")
        self.assertTrue(mesh_prim.IsValid())
        self.assertTrue(mesh_prim.IsA(UsdGeom.Mesh))
        self.check_material_binding(mesh_prim, red_material)

        box_specular_workflow_prim = link_obj_specular_workflow_prim.GetChild("box_specular_workflow")
        self.assertTrue(box_specular_workflow_prim.IsValid())
        self.assertTrue(box_specular_workflow_prim.IsA(UsdGeom.Mesh))
        self.assertTrue(box_specular_workflow_prim.HasAuthoredReferences())
        self.check_material_binding(box_specular_workflow_prim, box_specular_workflow_material)

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
        self.assertTrue(box_with_texture_prim.IsA(UsdGeom.Mesh))
        self.assertTrue(box_with_texture_prim.HasAuthoredReferences())

        link_obj_opacity_prim = link_obj_prim.GetChild("link_obj_opacity")
        self.assertTrue(link_obj_opacity_prim.IsValid())
        self.assertTrue(link_obj_opacity_prim.IsA(UsdGeom.Xform))

        box_with_texture_opacity_prim = link_obj_opacity_prim.GetChild("box_with_texture_opacity")
        self.assertTrue(box_with_texture_opacity_prim.IsValid())
        self.assertTrue(box_with_texture_opacity_prim.IsA(UsdGeom.Mesh))
        self.assertTrue(box_with_texture_opacity_prim.HasAuthoredReferences())

        link_obj_specular_workflow_with_texture_prim = link_obj_opacity_prim.GetChild("link_obj_specular_workflow_with_texture")
        self.assertTrue(link_obj_specular_workflow_with_texture_prim.IsValid())
        self.assertTrue(link_obj_specular_workflow_with_texture_prim.IsA(UsdGeom.Xform))

        box_specular_workflow_with_texture_prim = link_obj_specular_workflow_with_texture_prim.GetChild("box_specular_workflow_with_texture")
        self.assertTrue(box_specular_workflow_with_texture_prim.IsValid())
        self.assertTrue(box_specular_workflow_with_texture_prim.IsA(UsdGeom.Mesh))
        self.assertTrue(box_specular_workflow_with_texture_prim.HasAuthoredReferences())

        # Check the materials.
        material_scope_prim = default_prim.GetChild("Materials")
        self.assertTrue(material_scope_prim.IsValid())

        texture_material_prim = material_scope_prim.GetChild("texture_mat")
        self.assertTrue(texture_material_prim.IsValid())
        self.assertTrue(texture_material_prim.IsA(UsdShade.Material))
        texture_material = UsdShade.Material(texture_material_prim)
        self.assertTrue(texture_material)
        wrap_mode = self.get_material_wrap_mode(texture_material)
        self.assertEqual(wrap_mode, "repeat")

        diffuse_color = self.get_material_diffuse_color(texture_material)
        self.assertIsNone(diffuse_color)
        opacity = self.get_material_opacity(texture_material)
        self.assertAlmostEqual(opacity, 1.0, places=6)
        ior = self.get_material_ior(texture_material)
        self.assertAlmostEqual(ior, 1.45, places=6)
        diffuse_color_texture_path = self.get_material_texture_path(texture_material, "diffuseColor")
        self.assertEqual(diffuse_color_texture_path, pathlib.Path("./Textures/grid.png"))
        normal_texture_path = self.get_material_texture_path(texture_material, "normal")
        self.assertEqual(normal_texture_path, pathlib.Path("./Textures/normal.png"))
        roughness_texture_path = self.get_material_texture_path(texture_material, "roughness")
        self.assertEqual(roughness_texture_path, pathlib.Path("./Textures/roughness.png"))
        metallic_texture_path = self.get_material_texture_path(texture_material, "metallic")
        self.assertEqual(metallic_texture_path, pathlib.Path("./Textures/metallic.png"))

        texture_opacity_material_prim = material_scope_prim.GetChild("texture_opacity_mat")
        self.assertTrue(texture_opacity_material_prim.IsValid())
        self.assertTrue(texture_opacity_material_prim.IsA(UsdShade.Material))
        texture_opacity_material = UsdShade.Material(texture_opacity_material_prim)
        self.assertTrue(texture_opacity_material)
        wrap_mode = self.get_material_wrap_mode(texture_opacity_material)
        self.assertEqual(wrap_mode, "repeat")

        diffuse_color = self.get_material_diffuse_color(texture_opacity_material)
        self.assertIsNone(diffuse_color)
        ior = self.get_material_ior(texture_opacity_material)
        self.assertAlmostEqual(ior, 1.0, places=6)
        opacity_texture_path = self.get_material_texture_path(texture_opacity_material, "opacity")
        self.assertEqual(opacity_texture_path, pathlib.Path("./Textures/opacity.png"))

        texture_specular_workflow_material_prim = material_scope_prim.GetChild("specular_workflow_with_texture_mat")
        self.assertTrue(texture_specular_workflow_material_prim.IsValid())
        self.assertTrue(texture_specular_workflow_material_prim.IsA(UsdShade.Material))
        texture_specular_workflow_material = UsdShade.Material(texture_specular_workflow_material_prim)
        self.assertTrue(texture_specular_workflow_material)
        wrap_mode = self.get_material_wrap_mode(texture_specular_workflow_material)
        self.assertEqual(wrap_mode, "repeat")

        # Specular Workflow is currently disabled.
        diffuse_color = self.get_material_diffuse_color(texture_specular_workflow_material)
        diffuse_color = usdex.core.linearToSrgb(diffuse_color)
        self.assertTrue(Gf.IsClose(diffuse_color, Gf.Vec3f(0.4, 0.4, 0.4), 1e-6))
        ior = self.get_material_ior(texture_specular_workflow_material)
        self.assertAlmostEqual(ior, 1.45, places=6)

        self.check_material_binding(box_with_texture_prim, texture_material)
        self.check_material_binding(box_with_texture_opacity_prim, texture_opacity_material)
        self.check_material_binding(box_specular_workflow_with_texture_prim, texture_specular_workflow_material)

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
        self.assertTrue(Gf.IsClose(diffuse_color, Gf.Vec3f(0, 0, 1), 1e-6))
        opacity = self.get_material_opacity(blue_material)
        self.assertAlmostEqual(opacity, 1.0, places=6)

        red_material_prim = material_scope_prim.GetChild("red_mat")
        self.assertTrue(red_material_prim.IsValid())
        self.assertTrue(red_material_prim.IsA(UsdShade.Material))

        red_material = UsdShade.Material(red_material_prim)
        self.assertTrue(red_material)
        self.assertTrue(red_material.GetPrim().HasAuthoredReferences())

        diffuse_color = self.get_material_diffuse_color(red_material)
        self.assertTrue(Gf.IsClose(diffuse_color, Gf.Vec3f(1, 0, 0), 1e-6))
        opacity = self.get_material_opacity(red_material)
        self.assertAlmostEqual(opacity, 1.0, places=6)

        green_material_prim = material_scope_prim.GetChild("green_mat")
        self.assertTrue(green_material_prim.IsValid())
        self.assertTrue(green_material_prim.IsA(UsdShade.Material))

        green_material = UsdShade.Material(green_material_prim)
        self.assertTrue(green_material)
        self.assertTrue(green_material.GetPrim().HasAuthoredReferences())

        diffuse_color = self.get_material_diffuse_color(green_material)
        self.assertTrue(Gf.IsClose(diffuse_color, Gf.Vec3f(0, 1, 0), 1e-6))
        opacity = self.get_material_opacity(green_material)
        self.assertAlmostEqual(opacity, 1.0, places=6)

        dae_material_prim = material_scope_prim.GetChild("Material")
        self.assertTrue(dae_material_prim.IsValid())
        self.assertTrue(dae_material_prim.IsA(UsdShade.Material))
        dae_material = UsdShade.Material(dae_material_prim)
        self.assertTrue(dae_material)
        self.assertTrue(dae_material.GetPrim().HasAuthoredReferences())

        diffuse_color = self.get_material_diffuse_color(dae_material)
        diffuse_color = usdex.core.linearToSrgb(diffuse_color)
        self.assertTrue(Gf.IsClose(diffuse_color, Gf.Vec3f(0.8, 0.8, 0.8), 1e-6))
        opacity = self.get_material_opacity(dae_material)
        self.assertAlmostEqual(opacity, 1.0, places=6)

        # Check material bindings.
        default_prim = stage.GetDefaultPrim()
        geometry_scope_prim = default_prim.GetChild("Geometry")
        self.assertTrue(geometry_scope_prim.IsValid())
        self.assertTrue(geometry_scope_prim.IsA(UsdGeom.Scope))

        link_obj_prim = geometry_scope_prim.GetChild("link_box").GetChild("link_obj")
        self.assertTrue(link_obj_prim.IsValid())

        two_boxes_prim = link_obj_prim.GetChild("two_boxes")
        self.assertTrue(two_boxes_prim.IsValid())
        self.assertTrue(two_boxes_prim.IsA(UsdGeom.Xform))
        self.assertTrue(two_boxes_prim.HasAuthoredReferences())

        # Check that the material bind is overwritten with blue_material.
        self.check_material_binding(two_boxes_prim, blue_material)

        # Check that the material bind is overwritten with red_material.
        cube_red_prim = two_boxes_prim.GetChild("Cube_Red")
        self.assertTrue(cube_red_prim.IsValid())
        self.assertTrue(cube_red_prim.IsA(UsdGeom.Mesh))
        self.check_material_binding(cube_red_prim, red_material)

        # Check that the material bind is dae_material.
        # Since the material by dae is already assigned, it will not be overwritten by blue_material.
        dae_box_prim = link_obj_prim.GetChild("link_dae").GetChild("box")
        self.assertTrue(dae_box_prim.IsValid())
        self.assertTrue(dae_box_prim.IsA(UsdGeom.Mesh))
        self.check_material_binding(dae_box_prim, dae_material)

    def test_dae_materials(self):
        input_path = "tests/data/dae_materials.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)

        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        default_prim = stage.GetDefaultPrim()

        # Check the materials.
        material_scope_prim = default_prim.GetChild("Materials")
        self.assertTrue(material_scope_prim.IsValid())

        material_prim = material_scope_prim.GetChild("texture_mat")
        self.assertTrue(material_prim.IsValid())
        self.assertTrue(material_prim.IsA(UsdShade.Material))
        texture_material = UsdShade.Material(material_prim)

        diffuse_color_texture_path = self.get_material_texture_path(texture_material, "diffuseColor")
        self.assertEqual(diffuse_color_texture_path, pathlib.Path("./Textures/grid.png"))
        opacity = self.get_material_opacity(texture_material)
        self.assertAlmostEqual(opacity, 1.0, places=6)

        material_prim = material_scope_prim.GetChild("emissive_mat")
        self.assertTrue(material_prim.IsValid())
        self.assertTrue(material_prim.IsA(UsdShade.Material))
        emissive_material = UsdShade.Material(material_prim)

        diffuse_color = self.get_material_diffuse_color(emissive_material)
        diffuse_color = usdex.core.linearToSrgb(diffuse_color)
        self.assertTrue(Gf.IsClose(diffuse_color, Gf.Vec3f(0, 0, 0), 1e-6))
        emissive_color = self.get_material_emissive_color(emissive_material)
        emissive_color = usdex.core.linearToSrgb(emissive_color)
        self.assertTrue(Gf.IsClose(emissive_color, Gf.Vec3f(1, 1, 0), 1e-6))
        opacity = self.get_material_opacity(emissive_material)
        self.assertAlmostEqual(opacity, 1.0, places=6)

        material_prim = material_scope_prim.GetChild("emissive_color_tex_mat")
        self.assertTrue(material_prim.IsValid())
        self.assertTrue(material_prim.IsA(UsdShade.Material))
        emissive_color_tex_material = UsdShade.Material(material_prim)

        diffuse_color = self.get_material_diffuse_color(emissive_color_tex_material)
        self.assertTrue(Gf.IsClose(diffuse_color, Gf.Vec3f(0, 0, 0), 1e-6))
        emissive_color_texture_path = self.get_material_texture_path(emissive_color_tex_material, "emissiveColor")
        self.assertEqual(emissive_color_texture_path, pathlib.Path("./Textures/emissive.png"))

        material_prim = material_scope_prim.GetChild("opacity_mat")
        self.assertTrue(material_prim.IsValid())
        self.assertTrue(material_prim.IsA(UsdShade.Material))
        opacity_material = UsdShade.Material(material_prim)

        diffuse_texture_path = self.get_material_texture_path(opacity_material, "diffuseColor")
        self.assertEqual(diffuse_texture_path, pathlib.Path("./Textures/grid.png"))
        opacity = self.get_material_opacity(opacity_material)
        self.assertAlmostEqual(opacity, 0.4, places=6)

        material_prim = material_scope_prim.GetChild("opacity_texture_mat")
        self.assertTrue(material_prim.IsValid())
        self.assertTrue(material_prim.IsA(UsdShade.Material))
        opacity_texture_material = UsdShade.Material(material_prim)

        diffuse_texture_path = self.get_material_texture_path(opacity_texture_material, "diffuseColor")
        self.assertEqual(diffuse_texture_path, pathlib.Path("./Textures/grid.png"))
        opacity_texture_path = self.get_material_texture_path(opacity_texture_material, "opacity")
        self.assertEqual(opacity_texture_path, pathlib.Path("./Textures/opacity.png"))

        material_prim = material_scope_prim.GetChild("specular_mat")
        self.assertTrue(material_prim.IsValid())
        self.assertTrue(material_prim.IsA(UsdShade.Material))
        specular_material = UsdShade.Material(material_prim)

        diffuse_color = self.get_material_diffuse_color(specular_material)
        diffuse_color = usdex.core.linearToSrgb(diffuse_color)
        self.assertTrue(Gf.IsClose(diffuse_color, Gf.Vec3f(0.2, 0.2, 0.2), 1e-6))
        opacity = self.get_material_opacity(specular_material)
        self.assertAlmostEqual(opacity, 1.0, places=6)

        material_prim = material_scope_prim.GetChild("specular_texture_mat")
        self.assertTrue(material_prim.IsValid())
        self.assertTrue(material_prim.IsA(UsdShade.Material))
        specular_texture_material = UsdShade.Material(material_prim)

        diffuse_color = self.get_material_diffuse_color(specular_texture_material)
        diffuse_color = usdex.core.linearToSrgb(diffuse_color)
        self.assertTrue(Gf.IsClose(diffuse_color, Gf.Vec3f(0.2, 0.2, 0.2), 1e-6))
        opacity = self.get_material_opacity(specular_texture_material)
        self.assertAlmostEqual(opacity, 1.0, places=6)

        material_prim = material_scope_prim.GetChild("Material_red")
        self.assertTrue(material_prim.IsValid())
        self.assertTrue(material_prim.IsA(UsdShade.Material))
        material_red = UsdShade.Material(material_prim)

        diffuse_color = self.get_material_diffuse_color(material_red)
        diffuse_color = usdex.core.linearToSrgb(diffuse_color)
        self.assertTrue(Gf.IsClose(diffuse_color, Gf.Vec3f(1, 0, 0), 1e-6))
        opacity = self.get_material_opacity(material_red)
        self.assertAlmostEqual(opacity, 1.0, places=6)

        material_prim = material_scope_prim.GetChild("Material_green")
        self.assertTrue(material_prim.IsValid())
        self.assertTrue(material_prim.IsA(UsdShade.Material))
        material_green = UsdShade.Material(material_prim)

        diffuse_color = self.get_material_diffuse_color(material_green)
        diffuse_color = usdex.core.linearToSrgb(diffuse_color)
        self.assertTrue(Gf.IsClose(diffuse_color, Gf.Vec3f(0, 1, 0), 1e-6))
        opacity = self.get_material_opacity(material_green)
        self.assertAlmostEqual(opacity, 1.0, places=6)

        material_prim = material_scope_prim.GetChild("transparent_mat")
        self.assertTrue(material_prim.IsValid())
        self.assertTrue(material_prim.IsA(UsdShade.Material))
        material_transparent = UsdShade.Material(material_prim)

        diffuse_color = self.get_material_diffuse_color(material_transparent)
        diffuse_color = usdex.core.linearToSrgb(diffuse_color)
        self.assertTrue(Gf.IsClose(diffuse_color, Gf.Vec3f(0.1, 0.8, 0.1), 1e-6))
        opacity = self.get_material_opacity(material_transparent)
        self.assertAlmostEqual(opacity, 0.8, places=6)

        # Check the bindings.
        default_prim = stage.GetDefaultPrim()
        geometry_scope_prim = default_prim.GetChild("Geometry")
        self.assertTrue(geometry_scope_prim.IsValid())
        self.assertTrue(geometry_scope_prim.IsA(UsdGeom.Scope))
        link1_prim = geometry_scope_prim.GetChild("link1")
        box_materials_prim = link1_prim.GetChild("box_materials")
        self.assertTrue(box_materials_prim.IsValid())
        self.assertTrue(box_materials_prim.IsA(UsdGeom.Xform))
        self.assertTrue(box_materials_prim.HasAuthoredReferences())

        cube_prim = box_materials_prim.GetChild("Cube")
        self.assertTrue(cube_prim.IsValid())
        self.assertTrue(cube_prim.IsA(UsdGeom.Mesh))
        self.check_material_binding(cube_prim, texture_material)

        cube_002_prim = box_materials_prim.GetChild("tn__Cube002_VB")
        self.assertTrue(cube_002_prim.IsValid())
        self.assertTrue(cube_002_prim.IsA(UsdGeom.Mesh))
        self.check_material_binding(cube_002_prim, emissive_material)

        cube_003_prim = box_materials_prim.GetChild("tn__Cube003_VB")
        self.assertTrue(cube_003_prim.IsValid())
        self.assertTrue(cube_003_prim.IsA(UsdGeom.Mesh))
        self.check_material_binding(cube_003_prim, emissive_color_tex_material)

        cube_004_prim = box_materials_prim.GetChild("tn__Cube004_VB")
        self.assertTrue(cube_004_prim.IsValid())
        self.assertTrue(cube_004_prim.IsA(UsdGeom.Mesh))
        self.check_material_binding(cube_004_prim, opacity_material)

        cube_005_prim = box_materials_prim.GetChild("tn__Cube005_VB")
        self.assertTrue(cube_005_prim.IsValid())
        self.assertTrue(cube_005_prim.IsA(UsdGeom.Mesh))
        self.check_material_binding(cube_005_prim, opacity_texture_material)

        cube_001_prim = box_materials_prim.GetChild("tn__Cube001_VB")
        self.assertTrue(cube_001_prim.IsValid())
        self.assertTrue(cube_001_prim.IsA(UsdGeom.Mesh))
        self.check_material_binding(cube_001_prim, specular_material)

        cube_006_prim = box_materials_prim.GetChild("tn__Cube006_VB")
        self.assertTrue(cube_006_prim.IsValid())
        self.assertTrue(cube_006_prim.IsA(UsdGeom.Mesh))
        self.check_material_binding(cube_006_prim, specular_texture_material)

        # Check material binding to GeomSubset.
        link2_prim = link1_prim.GetChild("link2")
        box_materials_prim = link2_prim.GetChild("box_two_materials")
        self.assertTrue(box_materials_prim.IsValid())
        self.assertTrue(box_materials_prim.IsA(UsdGeom.Mesh))
        self.assertTrue(box_materials_prim.HasAuthoredReferences())

        subset_001_prim = box_materials_prim.GetChild("GeomSubset_001")
        self.assertTrue(subset_001_prim.IsValid())
        self.assertTrue(subset_001_prim.IsA(UsdGeom.Subset))
        self.check_material_binding(subset_001_prim, material_red)

        subset_002_prim = box_materials_prim.GetChild("GeomSubset_002")
        self.assertTrue(subset_002_prim.IsValid())
        self.assertTrue(subset_002_prim.IsA(UsdGeom.Subset))
        self.check_material_binding(subset_002_prim, material_green)

        link_transparent_prim = link2_prim.GetChild("link_transparent")
        box_transparent_prim = link_transparent_prim.GetChild("box_transparent_material")
        self.assertTrue(box_transparent_prim.IsValid())
        self.assertTrue(box_transparent_prim.IsA(UsdGeom.Mesh))
        self.assertTrue(box_transparent_prim.HasAuthoredReferences())
        self.check_material_binding(box_transparent_prim, material_transparent)

    def test_mesh_subsets_materials(self):
        input_path = "tests/data/mesh_subsets.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)

        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        default_prim = stage.GetDefaultPrim()

        # Check the materials.
        material_scope_prim = default_prim.GetChild("Materials")
        self.assertTrue(material_scope_prim.IsValid())

        material_green_prim = material_scope_prim.GetChild("Material_green")
        self.assertTrue(material_green_prim.IsValid())
        self.assertTrue(material_green_prim.IsA(UsdShade.Material))
        material_green = UsdShade.Material(material_green_prim)

        diffuse_color = self.get_material_diffuse_color(material_green)
        self.assertTrue(Gf.IsClose(diffuse_color, Gf.Vec3f(0, 1, 0), 1e-6))
        opacity = self.get_material_opacity(material_green)
        self.assertAlmostEqual(opacity, 1.0, places=6)

        material_red_prim = material_scope_prim.GetChild("Material_red")
        self.assertTrue(material_red_prim.IsValid())
        self.assertTrue(material_red_prim.IsA(UsdShade.Material))
        material_red = UsdShade.Material(material_red_prim)

        diffuse_color = self.get_material_diffuse_color(material_red)
        self.assertTrue(Gf.IsClose(diffuse_color, Gf.Vec3f(1, 0, 0), 1e-6))
        opacity = self.get_material_opacity(material_red)
        self.assertAlmostEqual(opacity, 1.0, places=6)

        material_red_1_prim = material_scope_prim.GetChild("Material_red_1")
        self.assertTrue(material_red_1_prim.IsValid())
        self.assertTrue(material_red_1_prim.IsA(UsdShade.Material))
        material_red_1 = UsdShade.Material(material_red_1_prim)

        diffuse_color = self.get_material_diffuse_color(material_red_1)
        self.assertTrue(Gf.IsClose(diffuse_color, Gf.Vec3f(1, 0, 0), 1e-6))
        opacity = self.get_material_opacity(material_red_1)
        self.assertAlmostEqual(opacity, 1.0, places=6)

        material_green_1_prim = material_scope_prim.GetChild("Material_green_1")
        self.assertTrue(material_green_1_prim.IsValid())
        self.assertTrue(material_green_1_prim.IsA(UsdShade.Material))
        material_green_1 = UsdShade.Material(material_green_1_prim)

        diffuse_color = self.get_material_diffuse_color(material_green_1)
        self.assertTrue(Gf.IsClose(diffuse_color, Gf.Vec3f(0, 1, 0), 1e-6))
        opacity = self.get_material_opacity(material_green_1)
        self.assertAlmostEqual(opacity, 1.0, places=6)

        # Check the bindings.
        default_prim = stage.GetDefaultPrim()
        geometry_scope_prim = default_prim.GetChild("Geometry")
        self.assertTrue(geometry_scope_prim.IsValid())
        self.assertTrue(geometry_scope_prim.IsA(UsdGeom.Scope))
        link_mesh_obj_prim = geometry_scope_prim.GetChild("link_mesh_obj")
        box_materials_prim = link_mesh_obj_prim.GetChild("box_two_materials")
        self.assertTrue(box_materials_prim.IsValid())
        self.assertTrue(box_materials_prim.IsA(UsdGeom.Mesh))
        self.assertTrue(box_materials_prim.HasAuthoredReferences())

        subset_001_prim = box_materials_prim.GetChild("GeomSubset_001")
        self.assertTrue(subset_001_prim.IsValid())
        self.assertTrue(subset_001_prim.IsA(UsdGeom.Subset))
        self.check_material_binding(subset_001_prim, material_green)

        mesh = UsdGeom.Mesh(box_materials_prim)
        subset = UsdGeom.Subset(subset_001_prim)
        element_type = subset.GetElementTypeAttr().Get()
        self.assertEqual(element_type, UsdGeom.Tokens.face)
        family_name = subset.GetFamilyNameAttr().Get()
        self.assertEqual(family_name, UsdShade.Tokens.materialBind)
        family_type = subset.GetFamilyType(mesh, family_name)
        self.assertEqual(family_type, UsdGeom.Tokens.partition)
        self.assertTrue(subset.ValidateFamily(mesh, element_type, family_name))

        subset_002_prim = box_materials_prim.GetChild("GeomSubset_002")
        self.assertTrue(subset_002_prim.IsValid())
        self.assertTrue(subset_002_prim.IsA(UsdGeom.Subset))
        self.check_material_binding(subset_002_prim, material_red)

        subset = UsdGeom.Subset(subset_002_prim)
        element_type = subset.GetElementTypeAttr().Get()
        self.assertEqual(element_type, UsdGeom.Tokens.face)
        family_name = subset.GetFamilyNameAttr().Get()
        self.assertEqual(family_name, UsdShade.Tokens.materialBind)
        family_type = subset.GetFamilyType(mesh, family_name)
        self.assertEqual(family_type, UsdGeom.Tokens.partition)
        self.assertTrue(subset.ValidateFamily(mesh, element_type, family_name))

        link_mesh_dae_prim = link_mesh_obj_prim.GetChild("link_mesh_dae")
        box_materials_prim = link_mesh_dae_prim.GetChild("box_two_materials")
        self.assertTrue(box_materials_prim.IsValid())
        self.assertTrue(box_materials_prim.IsA(UsdGeom.Mesh))
        self.assertTrue(box_materials_prim.HasAuthoredReferences())

        mesh = UsdGeom.Mesh(box_materials_prim)
        subset_001_prim = box_materials_prim.GetChild("GeomSubset_001")
        self.assertTrue(subset_001_prim.IsValid())
        self.assertTrue(subset_001_prim.IsA(UsdGeom.Subset))
        self.check_material_binding(subset_001_prim, material_red_1)

        subset = UsdGeom.Subset(subset_001_prim)
        element_type = subset.GetElementTypeAttr().Get()
        self.assertEqual(element_type, UsdGeom.Tokens.face)
        family_name = subset.GetFamilyNameAttr().Get()
        self.assertEqual(family_name, UsdShade.Tokens.materialBind)
        family_type = subset.GetFamilyType(mesh, family_name)
        self.assertEqual(family_type, UsdGeom.Tokens.partition)
        self.assertTrue(subset.ValidateFamily(mesh, element_type, family_name))

        subset_002_prim = box_materials_prim.GetChild("GeomSubset_002")
        self.assertTrue(subset_002_prim.IsValid())
        self.assertTrue(subset_002_prim.IsA(UsdGeom.Subset))
        self.check_material_binding(subset_002_prim, material_green_1)

        subset = UsdGeom.Subset(subset_002_prim)
        element_type = subset.GetElementTypeAttr().Get()
        self.assertEqual(element_type, UsdGeom.Tokens.face)
        family_name = subset.GetFamilyNameAttr().Get()
        self.assertEqual(family_name, UsdShade.Tokens.materialBind)
        family_type = subset.GetFamilyType(mesh, family_name)
        self.assertEqual(family_type, UsdGeom.Tokens.partition)
        self.assertTrue(subset.ValidateFamily(mesh, element_type, family_name))
