# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

from pxr import Gf, Usd, UsdPhysics

import urdf_usd_converter
from tests.util.ConverterTestCase import ConverterTestCase


class TestPhysics(ConverterTestCase):
    def setUp(self):
        super().setUp()

        input_path = "tests/data/simple-primitives.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        self.stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(self.stage)

        physics_scene_prim = self.stage.GetPrimAtPath("/PhysicsScene")
        self.assertTrue(physics_scene_prim.IsValid())

    def test_physics_link_box(self):
        default_prim = self.stage.GetDefaultPrim()
        self.assertTrue(default_prim.IsValid())
        default_prim_path = default_prim.GetPath()

        geometry_scope_prim = self.stage.GetPrimAtPath(default_prim_path.AppendChild("Geometry"))
        self.assertTrue(geometry_scope_prim.IsValid())

        # Rigid body.
        link_box_prim = self.stage.GetPrimAtPath(geometry_scope_prim.GetPath().AppendChild("link_box"))
        self.assertTrue(link_box_prim.IsValid())
        self.assertTrue(link_box_prim.HasAPI(UsdPhysics.RigidBodyAPI))

        # Mass.
        self.assertTrue(link_box_prim.HasAPI(UsdPhysics.MassAPI))
        mass_api: UsdPhysics.MassAPI = UsdPhysics.MassAPI(link_box_prim)
        self.assertTrue(Gf.IsClose(mass_api.GetCenterOfMassAttr().Get(), Gf.Vec3f(0, 0, 0.5), 1e-6))
        self.assertTrue(Gf.IsClose(mass_api.GetDiagonalInertiaAttr().Get(), Gf.Vec3f(100, 100, 100), 1e-6))
        self.assertAlmostEqual(mass_api.GetMassAttr().Get(), 0.8, places=6)
        self.assertRotationsAlmostEqual(mass_api.GetPrincipalAxesAttr().Get(), Gf.Quatf(1, 0, 0, 0))

        # Collision.
        collision_link_box_prim = self.stage.GetPrimAtPath(link_box_prim.GetPath().AppendChild("box_1"))
        self.assertTrue(collision_link_box_prim.HasAPI(UsdPhysics.CollisionAPI))
        collision_api: UsdPhysics.CollisionAPI = UsdPhysics.CollisionAPI(collision_link_box_prim)
        self.assertTrue(collision_api.GetCollisionEnabledAttr().Get())

    def test_physics_link_cylinder(self):
        default_prim = self.stage.GetDefaultPrim()
        self.assertIsNotNone(default_prim)
        default_prim_path = default_prim.GetPath()

        geometry_scope_prim = self.stage.GetPrimAtPath(default_prim_path.AppendChild("Geometry"))
        self.assertTrue(geometry_scope_prim.IsValid())

        link_box_prim = self.stage.GetPrimAtPath(geometry_scope_prim.GetPath().AppendChild("link_box"))
        self.assertTrue(link_box_prim.IsValid())

        # Rigid body.
        link_cylinder_prim = self.stage.GetPrimAtPath(link_box_prim.GetPath().AppendChild("link_cylinder"))
        self.assertTrue(link_cylinder_prim.IsValid())
        self.assertTrue(link_cylinder_prim.HasAPI(UsdPhysics.RigidBodyAPI))

        # Collision.
        collision_cylinder_prim = self.stage.GetPrimAtPath(link_cylinder_prim.GetPath().AppendChild("cylinder_1"))
        self.assertTrue(collision_cylinder_prim.IsValid())
        self.assertTrue(collision_cylinder_prim.HasAPI(UsdPhysics.CollisionAPI))
        collision_api: UsdPhysics.CollisionAPI = UsdPhysics.CollisionAPI(collision_cylinder_prim)
        self.assertTrue(collision_api.GetCollisionEnabledAttr().Get())

    def test_physics_link_sphere(self):
        default_prim = self.stage.GetDefaultPrim()
        self.assertIsNotNone(default_prim)
        default_prim_path = default_prim.GetPath()

        geometry_scope_prim = self.stage.GetPrimAtPath(default_prim_path.AppendChild("Geometry"))
        self.assertTrue(geometry_scope_prim.IsValid())

        link_box_prim = self.stage.GetPrimAtPath(geometry_scope_prim.GetPath().AppendChild("link_box"))
        self.assertTrue(link_box_prim.IsValid())

        link_cylinder_prim = self.stage.GetPrimAtPath(link_box_prim.GetPath().AppendChild("link_cylinder"))
        self.assertTrue(link_cylinder_prim.IsValid())

        # Rigid body.
        link_sphere_prim = self.stage.GetPrimAtPath(link_cylinder_prim.GetPath().AppendChild("link_sphere"))
        self.assertTrue(link_sphere_prim.IsValid())
        self.assertTrue(link_sphere_prim.HasAPI(UsdPhysics.RigidBodyAPI))

        # It has no collision prim.
        child_prims = link_sphere_prim.GetChildren()
        self.assertEqual(len(child_prims), 1)
        self.assertEqual(child_prims[0].GetName(), "sphere")
        self.assertFalse(child_prims[0].HasAPI(UsdPhysics.CollisionAPI))
