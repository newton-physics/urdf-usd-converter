# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
import pathlib

import usdex.core
from pxr import Gf, Usd, UsdGeom

from tests.util.ConverterTestCase import ConverterTestCase
from urdf_usd_converter._impl.convert import Converter


class TestConverterGeometries(ConverterTestCase):
    def setUp(self):
        super().setUp()
        self.tolerance = 1e-6

    def test_geometries(self):
        input_path = "tests/data/simple-primitives.urdf"
        output_dir = self.tmpDir()

        converter = Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        default_prim = stage.GetDefaultPrim()
        geometry_scope_prim = stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Geometry"))
        self.assertIsNotNone(geometry_scope_prim)

        link_box_prim = stage.GetPrimAtPath(geometry_scope_prim.GetPath().AppendChild("link_box"))
        self.assertIsNotNone(link_box_prim)
        self.assertTrue(link_box_prim.IsA(UsdGeom.Xform))

        box_prim = stage.GetPrimAtPath(link_box_prim.GetPath().AppendChild("box"))
        self.assertIsNotNone(box_prim)
        self.assertTrue(box_prim.IsA(UsdGeom.Cube))
        cube = UsdGeom.Cube(box_prim)
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(box_prim.GetPrim()).GetTranslation(), (-0.5, 0.0, 0.5), self.tolerance))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(box_prim.GetPrim()).GetScale(), (0.5, 0.5, 1.0), 1e-6))
        self.assertEqual(cube.GetSizeAttr().Get(), 1.0)
        self.assertEqual(UsdGeom.Imageable(box_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.default_)

        box_1_prim = stage.GetPrimAtPath(link_box_prim.GetPath().AppendChild("box_1"))
        self.assertIsNotNone(box_1_prim)
        self.assertTrue(box_1_prim.IsA(UsdGeom.Cube))
        box_1 = UsdGeom.Cube(box_1_prim)
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(box_1_prim.GetPrim()).GetTranslation(), (-0.5, 0.0, 0.5), self.tolerance))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(box_1_prim.GetPrim()).GetScale(), (0.5, 0.5, 1.0), self.tolerance))
        self.assert_rotation_almost_equal(usdex.core.getLocalTransform(box_1_prim.GetPrim()).GetRotation(), Gf.Rotation(Gf.Quatf(1, 0, 0, 0)), 1e-4)
        self.assertEqual(box_1.GetSizeAttr().Get(), 1.0)
        self.assertEqual(UsdGeom.Imageable(box_1_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.guide)

        link_cylinder_prim = stage.GetPrimAtPath(link_box_prim.GetPath().AppendChild("link_cylinder"))
        self.assertIsNotNone(link_cylinder_prim)
        self.assertTrue(link_cylinder_prim.IsA(UsdGeom.Xform))

        cylinder_prim = stage.GetPrimAtPath(link_cylinder_prim.GetPath().AppendChild("cylinder"))
        self.assertIsNotNone(cylinder_prim)
        self.assertTrue(cylinder_prim.IsA(UsdGeom.Cylinder))
        cylinder = UsdGeom.Cylinder(cylinder_prim)
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(cylinder_prim.GetPrim()).GetTranslation(), (0.5, 0.0, 0.5), self.tolerance))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(cylinder_prim.GetPrim()).GetScale(), (1.0, 1.0, 1.0), self.tolerance))
        self.assert_rotation_almost_equal(
            usdex.core.getLocalTransform(cylinder_prim.GetPrim()).GetRotation(), Gf.Rotation(Gf.Quatf(0.96592593, 0.25881866, 0, 0)), 1e-4
        )
        self.assertEqual(cylinder.GetAxisAttr().Get(), UsdGeom.Tokens.z)
        self.assertEqual(cylinder.GetRadiusAttr().Get(), 0.3)
        self.assertEqual(cylinder.GetHeightAttr().Get(), 1.0)
        self.assertEqual(UsdGeom.Imageable(cylinder_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.default_)

        cylinder_1_prim = stage.GetPrimAtPath(link_cylinder_prim.GetPath().AppendChild("cylinder_1"))
        self.assertIsNotNone(cylinder_1_prim)
        self.assertTrue(cylinder_1_prim.IsA(UsdGeom.Cylinder))
        cylinder_1 = UsdGeom.Cylinder(cylinder_1_prim)
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(cylinder_1_prim.GetPrim()).GetTranslation(), (0.5, 0.0, 0.5), self.tolerance))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(cylinder_1_prim.GetPrim()).GetScale(), (1.0, 1.0, 1.0), self.tolerance))
        self.assert_rotation_almost_equal(
            usdex.core.getLocalTransform(cylinder_1_prim.GetPrim()).GetRotation(), Gf.Rotation(Gf.Quatf(0.96592593, 0.25881866, 0, 0)), 1e-4
        )
        self.assertEqual(cylinder_1.GetAxisAttr().Get(), UsdGeom.Tokens.z)
        self.assertEqual(cylinder_1.GetRadiusAttr().Get(), 0.3)
        self.assertEqual(cylinder_1.GetHeightAttr().Get(), 1.0)
        self.assertEqual(UsdGeom.Imageable(cylinder_1_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.guide)

        link_sphere_prim = stage.GetPrimAtPath(link_cylinder_prim.GetPath().AppendChild("tn__linksphere_rJ"))
        self.assertIsNotNone(link_sphere_prim)
        self.assertTrue(link_sphere_prim.IsA(UsdGeom.Xform))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(link_sphere_prim.GetPrim()).GetTranslation(), (1.5, 0.0, 0.5), self.tolerance))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(link_sphere_prim.GetPrim()).GetScale(), (1.0, 1.0, 1.0), self.tolerance))

        sphere_prim = stage.GetPrimAtPath(link_sphere_prim.GetPath().AppendChild("sphere"))
        self.assertIsNotNone(sphere_prim)
        self.assertTrue(sphere_prim.IsA(UsdGeom.Sphere))
        sphere = UsdGeom.Sphere(sphere_prim)
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(sphere_prim.GetPrim()).GetTranslation(), (0.0, 0.0, 0.0), self.tolerance))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(sphere_prim.GetPrim()).GetScale(), (1.0, 1.0, 1.0), self.tolerance))
        self.assert_rotation_almost_equal(usdex.core.getLocalTransform(sphere_prim.GetPrim()).GetRotation(), Gf.Rotation(Gf.Quatf(1, 0, 0, 0)), 1e-4)
        self.assertEqual(sphere.GetRadiusAttr().Get(), 0.5)
        self.assertEqual(UsdGeom.Imageable(sphere_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.default_)

    def test_visuals_collisions_in_link(self):
        input_path = "tests/data/multiple_visuals_collisions_in_link.urdf"
        output_dir = self.tmpDir()

        converter = Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        default_prim = stage.GetDefaultPrim()
        geometry_scope_prim = stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Geometry"))
        self.assertIsNotNone(geometry_scope_prim)

        root_link_prim = stage.GetPrimAtPath(geometry_scope_prim.GetPath().AppendChild("RootLink"))
        self.assertIsNotNone(root_link_prim)
        self.assertTrue(root_link_prim.IsA(UsdGeom.Xform))

        link_prim = stage.GetPrimAtPath(root_link_prim.GetPath().AppendChild("link"))
        self.assertIsNotNone(link_prim)
        self.assertTrue(link_prim.IsA(UsdGeom.Xform))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(link_prim.GetPrim()).GetTranslation(), (0.0, 0.0, 0.25), self.tolerance))

        box_prim = stage.GetPrimAtPath(link_prim.GetPath().AppendChild("box"))
        self.assertIsNotNone(box_prim)
        self.assertTrue(box_prim.IsA(UsdGeom.Cube))
        cube = UsdGeom.Cube(box_prim)
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(box_prim.GetPrim()).GetTranslation(), (0.0, 0.0, 0.0), self.tolerance))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(box_prim.GetPrim()).GetScale(), (0.5, 0.5, 0.5), self.tolerance))
        self.assert_rotation_almost_equal(usdex.core.getLocalTransform(box_prim.GetPrim()).GetRotation(), Gf.Rotation(Gf.Quatf(1, 0, 0, 0)), 1e-4)
        self.assertEqual(cube.GetSizeAttr().Get(), 1.0)
        self.assertEqual(UsdGeom.Imageable(box_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.default_)

        sphere_prim = stage.GetPrimAtPath(link_prim.GetPath().AppendChild("sphere"))
        self.assertIsNotNone(sphere_prim)
        self.assertTrue(sphere_prim.IsA(UsdGeom.Sphere))
        sphere = UsdGeom.Sphere(sphere_prim)
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(sphere_prim.GetPrim()).GetTranslation(), (0.0, 0.0, 0.45), self.tolerance))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(sphere_prim.GetPrim()).GetScale(), (1.0, 1.0, 1.0), self.tolerance))
        self.assert_rotation_almost_equal(usdex.core.getLocalTransform(sphere_prim.GetPrim()).GetRotation(), Gf.Rotation(Gf.Quatf(1, 0, 0, 0)), 1e-4)
        self.assertEqual(sphere.GetRadiusAttr().Get(), 0.2)
        self.assertEqual(UsdGeom.Imageable(sphere_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.default_)

        collision_box_prim = stage.GetPrimAtPath(link_prim.GetPath().AppendChild("box_1"))
        self.assertIsNotNone(collision_box_prim)
        self.assertTrue(collision_box_prim.IsA(UsdGeom.Cube))
        collision_box = UsdGeom.Cube(collision_box_prim)
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(collision_box_prim.GetPrim()).GetTranslation(), (0.0, 0.0, 0.0), self.tolerance))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(collision_box_prim.GetPrim()).GetScale(), (0.5, 0.5, 0.5), self.tolerance))
        self.assert_rotation_almost_equal(
            usdex.core.getLocalTransform(collision_box_prim.GetPrim()).GetRotation(), Gf.Rotation(Gf.Quatf(1, 0, 0, 0)), 1e-4
        )
        self.assertEqual(collision_box.GetSizeAttr().Get(), 1.0)
        self.assertEqual(UsdGeom.Imageable(collision_box_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.guide)

        collision_sphere_prim = stage.GetPrimAtPath(link_prim.GetPath().AppendChild("sphere_1"))
        self.assertIsNotNone(collision_sphere_prim)
        self.assertTrue(collision_sphere_prim.IsA(UsdGeom.Sphere))
        collision_sphere = UsdGeom.Sphere(collision_sphere_prim)
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(collision_sphere_prim.GetPrim()).GetTranslation(), (0.0, 0.0, 0.45), self.tolerance))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(collision_sphere_prim.GetPrim()).GetScale(), (1.0, 1.0, 1.0), self.tolerance))
        self.assert_rotation_almost_equal(
            usdex.core.getLocalTransform(collision_sphere_prim.GetPrim()).GetRotation(), Gf.Rotation(Gf.Quatf(1, 0, 0, 0)), 1e-4
        )
        self.assertEqual(collision_sphere.GetRadiusAttr().Get(), 0.2)
        self.assertEqual(UsdGeom.Imageable(collision_sphere_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.guide)
