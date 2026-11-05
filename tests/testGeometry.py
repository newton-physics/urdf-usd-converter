# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import usdex.core
from pxr import Gf, Usd, UsdGeom

import urdf_usd_converter
from tests.util.ConverterTestCase import ConverterTestCase


class TestGeometry(ConverterTestCase):
    def test_geometries(self):
        input_path = "tests/data/simple-primitives.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        default_prim = stage.GetDefaultPrim()
        geometry_scope_prim = stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Geometry"))
        self.assertTrue(geometry_scope_prim.IsValid())

        link_box_prim = stage.GetPrimAtPath(geometry_scope_prim.GetPath().AppendChild("link_box"))
        self.assertTrue(link_box_prim.IsValid())
        self.assertTrue(link_box_prim.IsA(UsdGeom.Xform))

        box_prim = stage.GetPrimAtPath(link_box_prim.GetPath().AppendChild("box"))
        self.assertTrue(box_prim.IsValid())
        self.assertTrue(box_prim.IsA(UsdGeom.Cube))
        cube = UsdGeom.Cube(box_prim)
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(box_prim.GetPrim()).GetTranslation(), Gf.Vec3d(-0.5, 0.0, 0.5), 1e-6))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(box_prim.GetPrim()).GetScale(), Gf.Vec3d(0.5, 0.5, 1.0), 1e-6))
        self.assertRotationsAlmostEqual(usdex.core.getLocalTransform(box_prim.GetPrim()).GetRotation(), Gf.Rotation(Gf.Quatf(1, 0, 0, 0)))
        self.assertEqual(cube.GetSizeAttr().Get(), 1.0)
        self.assertEqual(UsdGeom.Imageable(box_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.default_)

        collision_box_prim = stage.GetPrimAtPath(link_box_prim.GetPath().AppendChild("box_1"))
        self.assertTrue(collision_box_prim.IsValid())
        self.assertTrue(collision_box_prim.IsA(UsdGeom.Cube))
        collision_box = UsdGeom.Cube(collision_box_prim)
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(collision_box_prim.GetPrim()).GetTranslation(), Gf.Vec3d(-0.5, 0.0, 0.5), 1e-6))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(collision_box_prim.GetPrim()).GetScale(), Gf.Vec3d(0.5, 0.5, 1.0), 1e-6))
        self.assertRotationsAlmostEqual(usdex.core.getLocalTransform(collision_box_prim.GetPrim()).GetRotation(), Gf.Rotation(Gf.Quatf(1, 0, 0, 0)))
        self.assertEqual(collision_box.GetSizeAttr().Get(), 1.0)
        self.assertEqual(UsdGeom.Imageable(collision_box_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.guide)

        link_cylinder_prim = stage.GetPrimAtPath(link_box_prim.GetPath().AppendChild("link_cylinder"))
        self.assertTrue(link_cylinder_prim.IsValid())
        self.assertTrue(link_cylinder_prim.IsA(UsdGeom.Xform))

        cylinder_prim = stage.GetPrimAtPath(link_cylinder_prim.GetPath().AppendChild("cylinder"))
        self.assertTrue(cylinder_prim.IsValid())
        self.assertTrue(cylinder_prim.IsA(UsdGeom.Cylinder))
        cylinder = UsdGeom.Cylinder(cylinder_prim)
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(cylinder_prim.GetPrim()).GetTranslation(), Gf.Vec3d(0.5, 0.0, 0.5), 1e-6))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(cylinder_prim.GetPrim()).GetScale(), Gf.Vec3d(1.0, 1.0, 1.0), 1e-6))
        self.assertRotationsAlmostEqual(
            usdex.core.getLocalTransform(cylinder_prim.GetPrim()).GetRotation(), Gf.Rotation(Gf.Quatf(0.96592593, 0.25881866, 0, 0))
        )
        self.assertEqual(cylinder.GetAxisAttr().Get(), UsdGeom.Tokens.z)
        self.assertEqual(cylinder.GetRadiusAttr().Get(), 0.3)
        self.assertEqual(cylinder.GetHeightAttr().Get(), 1.0)
        self.assertEqual(UsdGeom.Imageable(cylinder_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.default_)

        collision_cylinder_prim = stage.GetPrimAtPath(link_cylinder_prim.GetPath().AppendChild("cylinder_1"))
        self.assertTrue(collision_cylinder_prim.IsValid())
        self.assertTrue(collision_cylinder_prim.IsA(UsdGeom.Cylinder))
        collision_cylinder = UsdGeom.Cylinder(collision_cylinder_prim)
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(collision_cylinder_prim.GetPrim()).GetTranslation(), Gf.Vec3d(0.5, 0.0, 0.5), 1e-6))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(collision_cylinder_prim.GetPrim()).GetScale(), Gf.Vec3d(1.0, 1.0, 1.0), 1e-6))
        self.assertRotationsAlmostEqual(
            usdex.core.getLocalTransform(collision_cylinder_prim.GetPrim()).GetRotation(), Gf.Rotation(Gf.Quatf(0.96592593, 0.25881866, 0, 0))
        )
        self.assertEqual(collision_cylinder.GetAxisAttr().Get(), UsdGeom.Tokens.z)
        self.assertEqual(collision_cylinder.GetRadiusAttr().Get(), 0.3)
        self.assertEqual(collision_cylinder.GetHeightAttr().Get(), 1.0)
        self.assertEqual(UsdGeom.Imageable(collision_cylinder_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.guide)

        link_sphere_prim = stage.GetPrimAtPath(link_cylinder_prim.GetPath().AppendChild("link_sphere"))
        self.assertTrue(link_sphere_prim.IsValid())
        self.assertTrue(link_sphere_prim.IsA(UsdGeom.Xform))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(link_sphere_prim.GetPrim()).GetTranslation(), Gf.Vec3d(1.5, 0.0, 0.5), 1e-6))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(link_sphere_prim.GetPrim()).GetScale(), Gf.Vec3d(1.0, 1.0, 1.0), 1e-6))
        self.assertRotationsAlmostEqual(usdex.core.getLocalTransform(link_sphere_prim.GetPrim()).GetRotation(), Gf.Rotation(Gf.Quatf(1, 0, 0, 0)))

        sphere_prim = stage.GetPrimAtPath(link_sphere_prim.GetPath().AppendChild("sphere"))
        self.assertTrue(sphere_prim.IsValid())
        self.assertTrue(sphere_prim.IsA(UsdGeom.Sphere))
        sphere = UsdGeom.Sphere(sphere_prim)
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(sphere_prim.GetPrim()).GetTranslation(), Gf.Vec3d(0.0, 0.0, 0.0), 1e-6))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(sphere_prim.GetPrim()).GetScale(), Gf.Vec3d(1.0, 1.0, 1.0), 1e-6))
        self.assertRotationsAlmostEqual(usdex.core.getLocalTransform(sphere_prim.GetPrim()).GetRotation(), Gf.Rotation(Gf.Quatf(1, 0, 0, 0)))
        self.assertEqual(sphere.GetRadiusAttr().Get(), 0.5)
        self.assertEqual(UsdGeom.Imageable(sphere_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.default_)

    def test_urdf11_capsule(self):
        input_path = "tests/data/urdf_11_capsule.urdf"
        output_dir = self.tmpDir()

        asset_path = urdf_usd_converter.Converter().convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        geometry_scope_prim = stage.GetDefaultPrim().GetChild("Geometry")
        base_link_prim = geometry_scope_prim.GetChild("base_link")
        arm_link_prim = base_link_prim.GetChild("arm_link")
        self.assertTrue(arm_link_prim.IsA(UsdGeom.Xform))

        expected_rotation = Gf.Rotation(Gf.Quatf(0.70710678, 0.0, 0.70710678, 0.0))
        for name, purpose in (("capsule", UsdGeom.Tokens.default_), ("collision_capsule", UsdGeom.Tokens.guide)):
            capsule_prim = arm_link_prim.GetChild(name)
            self.assertTrue(capsule_prim.IsA(UsdGeom.Capsule))

            transform = usdex.core.getLocalTransform(capsule_prim)
            self.assertTrue(Gf.IsClose(transform.GetTranslation(), Gf.Vec3d(0.3, 0.0, 0.05), 1e-6))
            self.assertTrue(Gf.IsClose(transform.GetScale(), Gf.Vec3d(1.0), 1e-6))
            self.assertRotationsAlmostEqual(transform.GetRotation(), expected_rotation)

            capsule = UsdGeom.Capsule(capsule_prim)
            self.assertEqual(capsule.GetAxisAttr().Get(), UsdGeom.Tokens.z)
            self.assertEqual(capsule.GetRadiusAttr().Get(), 0.05)
            self.assertEqual(capsule.GetHeightAttr().Get(), 0.5)
            self.assertEqual(UsdGeom.Imageable(capsule_prim).GetPurposeAttr().Get(), purpose)

        tip_link_prim = arm_link_prim.GetChild("tip_link")
        self.assertTrue(tip_link_prim.IsA(UsdGeom.Xform))
        tip_transform = usdex.core.getLocalTransform(tip_link_prim)
        self.assertTrue(Gf.IsClose(tip_transform.GetTranslation(), Gf.Vec3d(0.68, 0.0, 0.05), 1e-6))
        self.assertRotationsAlmostEqual(tip_transform.GetRotation(), expected_rotation)

        for name, purpose in (("capsule", UsdGeom.Tokens.default_), ("collision_capsule2", UsdGeom.Tokens.guide)):
            capsule_prim = tip_link_prim.GetChild(name)
            self.assertTrue(capsule_prim.IsA(UsdGeom.Capsule))
            capsule = UsdGeom.Capsule(capsule_prim)
            self.assertEqual(capsule.GetAxisAttr().Get(), UsdGeom.Tokens.z)
            self.assertEqual(capsule.GetRadiusAttr().Get(), 0.03)
            self.assertEqual(capsule.GetHeightAttr().Get(), 0.1)
            self.assertEqual(UsdGeom.Imageable(capsule_prim).GetPurposeAttr().Get(), purpose)

    def test_visuals_collisions_in_link(self):
        input_path = "tests/data/multiple_visuals_collisions_in_link.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        default_prim = stage.GetDefaultPrim()
        geometry_scope_prim = stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Geometry"))
        self.assertTrue(geometry_scope_prim.IsValid())

        root_link_prim = stage.GetPrimAtPath(geometry_scope_prim.GetPath().AppendChild("RootLink"))
        self.assertTrue(root_link_prim.IsValid())
        self.assertTrue(root_link_prim.IsA(UsdGeom.Xform))

        link_prim = stage.GetPrimAtPath(root_link_prim.GetPath().AppendChild("link"))
        self.assertTrue(link_prim.IsValid())
        self.assertTrue(link_prim.IsA(UsdGeom.Xform))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(link_prim.GetPrim()).GetTranslation(), Gf.Vec3d(0.0, 0.0, 0.25), 1e-6))

        box_prim = stage.GetPrimAtPath(link_prim.GetPath().AppendChild("box"))
        self.assertTrue(box_prim.IsValid())
        self.assertTrue(box_prim.IsA(UsdGeom.Cube))
        cube = UsdGeom.Cube(box_prim)
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(box_prim.GetPrim()).GetTranslation(), Gf.Vec3d(0.0, 0.0, 0.0), 1e-6))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(box_prim.GetPrim()).GetScale(), Gf.Vec3d(0.5, 0.5, 0.5), 1e-6))
        self.assertRotationsAlmostEqual(usdex.core.getLocalTransform(box_prim.GetPrim()).GetRotation(), Gf.Rotation(Gf.Quatf(1, 0, 0, 0)))
        self.assertEqual(cube.GetSizeAttr().Get(), 1.0)
        self.assertEqual(UsdGeom.Imageable(box_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.default_)

        sphere_prim = stage.GetPrimAtPath(link_prim.GetPath().AppendChild("sphere"))
        self.assertTrue(sphere_prim.IsValid())
        self.assertTrue(sphere_prim.IsA(UsdGeom.Sphere))
        sphere = UsdGeom.Sphere(sphere_prim)
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(sphere_prim.GetPrim()).GetTranslation(), Gf.Vec3d(0.0, 0.0, 0.45), 1e-6))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(sphere_prim.GetPrim()).GetScale(), Gf.Vec3d(1.0, 1.0, 1.0), 1e-6))
        self.assertRotationsAlmostEqual(usdex.core.getLocalTransform(sphere_prim.GetPrim()).GetRotation(), Gf.Rotation(Gf.Quatf(1, 0, 0, 0)))
        self.assertEqual(sphere.GetRadiusAttr().Get(), 0.2)
        self.assertEqual(UsdGeom.Imageable(sphere_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.default_)

        collision_box_prim = stage.GetPrimAtPath(link_prim.GetPath().AppendChild("box_1"))
        self.assertTrue(collision_box_prim.IsValid())
        self.assertTrue(collision_box_prim.IsA(UsdGeom.Cube))
        collision_box = UsdGeom.Cube(collision_box_prim)
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(collision_box_prim.GetPrim()).GetTranslation(), Gf.Vec3d(0.0, 0.0, 0.0), 1e-6))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(collision_box_prim.GetPrim()).GetScale(), Gf.Vec3d(0.5, 0.5, 0.5), 1e-6))
        self.assertRotationsAlmostEqual(usdex.core.getLocalTransform(collision_box_prim.GetPrim()).GetRotation(), Gf.Rotation(Gf.Quatf(1, 0, 0, 0)))
        self.assertEqual(collision_box.GetSizeAttr().Get(), 1.0)
        self.assertEqual(UsdGeom.Imageable(collision_box_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.guide)

        collision_sphere_prim = stage.GetPrimAtPath(link_prim.GetPath().AppendChild("sphere_1"))
        self.assertTrue(collision_sphere_prim.IsValid())
        self.assertTrue(collision_sphere_prim.IsA(UsdGeom.Sphere))
        collision_sphere = UsdGeom.Sphere(collision_sphere_prim)
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(collision_sphere_prim.GetPrim()).GetTranslation(), Gf.Vec3d(0.0, 0.0, 0.45), 1e-6))
        self.assertTrue(Gf.IsClose(usdex.core.getLocalTransform(collision_sphere_prim.GetPrim()).GetScale(), Gf.Vec3d(1.0, 1.0, 1.0), 1e-6))
        self.assertRotationsAlmostEqual(
            usdex.core.getLocalTransform(collision_sphere_prim.GetPrim()).GetRotation(), Gf.Rotation(Gf.Quatf(1, 0, 0, 0))
        )
        self.assertEqual(collision_sphere.GetRadiusAttr().Get(), 0.2)
        self.assertEqual(UsdGeom.Imageable(collision_sphere_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.guide)

    def test_visual_collision_name(self):
        input_path = "tests/data/visual_collision_name.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        default_prim = stage.GetDefaultPrim()
        geometry_scope_prim = stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Geometry"))
        self.assertTrue(geometry_scope_prim.IsValid())

        root_link_prim = stage.GetPrimAtPath(geometry_scope_prim.GetPath().AppendChild("RootLink"))
        self.assertTrue(root_link_prim.IsValid())
        self.assertTrue(root_link_prim.IsA(UsdGeom.Xform))

        box_prim = stage.GetPrimAtPath(root_link_prim.GetPath().AppendChild("box"))
        self.assertTrue(box_prim.IsValid())
        self.assertTrue(box_prim.IsA(UsdGeom.Cube))

        collision_box_prim = stage.GetPrimAtPath(root_link_prim.GetPath().AppendChild("box_1"))
        self.assertTrue(collision_box_prim.IsValid())
        self.assertTrue(collision_box_prim.IsA(UsdGeom.Cube))

        link_prim = stage.GetPrimAtPath(root_link_prim.GetPath().AppendChild("link"))
        self.assertTrue(root_link_prim.IsValid())
        self.assertTrue(root_link_prim.IsA(UsdGeom.Xform))

        box_name_visual_prim = stage.GetPrimAtPath(link_prim.GetPath().AppendChild("box_name_visual"))
        self.assertTrue(box_name_visual_prim.IsValid())
        self.assertTrue(box_name_visual_prim.IsA(UsdGeom.Cube))

        box_name_collision_prim = stage.GetPrimAtPath(link_prim.GetPath().AppendChild("box_name_collision"))
        self.assertTrue(box_name_collision_prim.IsValid())
        self.assertTrue(box_name_collision_prim.IsA(UsdGeom.Cube))
