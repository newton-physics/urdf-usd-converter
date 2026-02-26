# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

from pxr import Gf, Usd, UsdGeom, UsdPhysics

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
        self.assertTrue(link_box_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertTrue(link_box_prim.HasAPI("NewtonArticulationRootAPI"))

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
        self.assertTrue(collision_link_box_prim.HasAPI("NewtonCollisionAPI"))
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
        self.assertTrue(link_box_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(link_box_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertTrue(link_box_prim.HasAPI("NewtonArticulationRootAPI"))

        # Rigid body.
        link_cylinder_prim = self.stage.GetPrimAtPath(link_box_prim.GetPath().AppendChild("link_cylinder"))
        self.assertTrue(link_cylinder_prim.IsValid())
        self.assertTrue(link_cylinder_prim.HasAPI(UsdPhysics.RigidBodyAPI))

        # Collision.
        collision_cylinder_prim = self.stage.GetPrimAtPath(link_cylinder_prim.GetPath().AppendChild("cylinder_1"))
        self.assertTrue(collision_cylinder_prim.IsValid())
        self.assertTrue(collision_cylinder_prim.HasAPI(UsdPhysics.CollisionAPI))
        self.assertTrue(collision_cylinder_prim.HasAPI("NewtonCollisionAPI"))
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
        self.assertTrue(link_box_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(link_box_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertTrue(link_box_prim.HasAPI("NewtonArticulationRootAPI"))

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
        self.assertFalse(child_prims[0].HasAPI("NewtonCollisionAPI"))


class TestPhysicsMesh(ConverterTestCase):
    def test_physics_mesh_collision(self):
        input_path = "tests/data/meshes.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)

        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        default_prim = stage.GetDefaultPrim()
        self.assertTrue(default_prim.IsValid())

        geometry_scope_prim = stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Geometry"))
        self.assertTrue(geometry_scope_prim.IsValid())

        link_mesh_stl_prim = stage.GetPrimAtPath(geometry_scope_prim.GetPath().AppendChild("link_mesh_stl"))
        self.assertTrue(link_mesh_stl_prim.IsValid())
        self.assertTrue(link_mesh_stl_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(link_mesh_stl_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertTrue(link_mesh_stl_prim.HasAPI("NewtonArticulationRootAPI"))

        link_mesh_obj_prim = stage.GetPrimAtPath(link_mesh_stl_prim.GetPath().AppendChild("link_mesh_obj"))
        self.assertTrue(link_mesh_obj_prim.IsValid())

        # Check Collision on mesh.
        collision_box_prim = stage.GetPrimAtPath(link_mesh_obj_prim.GetPath().AppendChild("collision_box"))
        self.assertTrue(collision_box_prim.IsValid())
        self.assertTrue(collision_box_prim.IsA(UsdGeom.Mesh))
        self.assertTrue(collision_box_prim.HasAPI(UsdPhysics.CollisionAPI))
        self.assertTrue(collision_box_prim.HasAPI("NewtonCollisionAPI"))
        collision_api: UsdPhysics.CollisionAPI = UsdPhysics.CollisionAPI(collision_box_prim)
        self.assertTrue(collision_api.GetCollisionEnabledAttr().Get())

        # Check MeshCollision on mesh.
        self.assertTrue(collision_box_prim.HasAPI(UsdPhysics.MeshCollisionAPI))
        self.assertTrue(collision_box_prim.HasAPI("NewtonMeshCollisionAPI"))
        mesh_collision_api: UsdPhysics.MeshCollisionAPI = UsdPhysics.MeshCollisionAPI(collision_box_prim)
        self.assertEqual(mesh_collision_api.GetApproximationAttr().Get(), UsdPhysics.Tokens.convexHull)

        link_mesh_multi_objs_prim = stage.GetPrimAtPath(link_mesh_stl_prim.GetPath().AppendChild("link_mesh_multi_objs"))
        self.assertTrue(link_mesh_multi_objs_prim.IsValid())
        self.assertTrue(link_mesh_multi_objs_prim.HasAPI(UsdPhysics.RigidBodyAPI))

        collision_two_boxes_prim = stage.GetPrimAtPath(link_mesh_multi_objs_prim.GetPath().AppendChild("two_collision_boxes"))
        self.assertTrue(collision_two_boxes_prim.IsValid())
        self.assertTrue(collision_two_boxes_prim.IsA(UsdGeom.Xform))

        # Check Collision on mesh.
        cube_red_prim = stage.GetPrimAtPath(collision_two_boxes_prim.GetPath().AppendChild("Cube_Red"))
        self.assertTrue(cube_red_prim.IsValid())
        self.assertTrue(cube_red_prim.IsA(UsdGeom.Mesh))
        self.assertTrue(cube_red_prim.HasAPI(UsdPhysics.CollisionAPI))
        self.assertTrue(cube_red_prim.HasAPI("NewtonCollisionAPI"))
        collision_api: UsdPhysics.CollisionAPI = UsdPhysics.CollisionAPI(cube_red_prim)
        self.assertTrue(collision_api.GetCollisionEnabledAttr().Get())

        # Check MeshCollision on mesh.
        self.assertTrue(cube_red_prim.HasAPI(UsdPhysics.MeshCollisionAPI))
        self.assertTrue(cube_red_prim.HasAPI("NewtonMeshCollisionAPI"))
        mesh_collision_api: UsdPhysics.MeshCollisionAPI = UsdPhysics.MeshCollisionAPI(cube_red_prim)
        self.assertEqual(mesh_collision_api.GetApproximationAttr().Get(), UsdPhysics.Tokens.convexHull)

        # Check Collision on mesh.
        cube_green_prim = stage.GetPrimAtPath(collision_two_boxes_prim.GetPath().AppendChild("Cube_Green"))
        self.assertTrue(cube_green_prim.IsValid())
        self.assertTrue(cube_green_prim.IsA(UsdGeom.Mesh))
        self.assertTrue(cube_green_prim.HasAPI(UsdPhysics.CollisionAPI))
        self.assertTrue(cube_green_prim.HasAPI("NewtonCollisionAPI"))
        collision_api: UsdPhysics.CollisionAPI = UsdPhysics.CollisionAPI(cube_green_prim)
        self.assertTrue(collision_api.GetCollisionEnabledAttr().Get())

        # Check MeshCollision on mesh.
        self.assertTrue(cube_green_prim.HasAPI(UsdPhysics.MeshCollisionAPI))
        self.assertTrue(cube_green_prim.HasAPI("NewtonMeshCollisionAPI"))
        mesh_collision_api: UsdPhysics.MeshCollisionAPI = UsdPhysics.MeshCollisionAPI(cube_green_prim)
        self.assertEqual(mesh_collision_api.GetApproximationAttr().Get(), UsdPhysics.Tokens.convexHull)

        link_mesh_dae_prim = stage.GetPrimAtPath(link_mesh_stl_prim.GetPath().AppendChild("link_mesh_dae"))
        self.assertTrue(link_mesh_dae_prim.IsValid())
        self.assertTrue(link_mesh_dae_prim.HasAPI(UsdPhysics.RigidBodyAPI))

        # Check Collision on mesh.
        collision_box_prim = stage.GetPrimAtPath(link_mesh_dae_prim.GetPath().AppendChild("collision_box"))
        self.assertTrue(collision_box_prim.IsValid())
        self.assertTrue(collision_box_prim.IsA(UsdGeom.Mesh))
        self.assertTrue(collision_box_prim.HasAPI(UsdPhysics.CollisionAPI))
        self.assertTrue(collision_box_prim.HasAPI("NewtonCollisionAPI"))
        collision_api: UsdPhysics.CollisionAPI = UsdPhysics.CollisionAPI(collision_box_prim)
        self.assertTrue(collision_api.GetCollisionEnabledAttr().Get())

        # Check MeshCollision on mesh.
        self.assertTrue(collision_box_prim.HasAPI(UsdPhysics.MeshCollisionAPI))
        self.assertTrue(collision_box_prim.HasAPI("NewtonMeshCollisionAPI"))
        mesh_collision_api: UsdPhysics.MeshCollisionAPI = UsdPhysics.MeshCollisionAPI(collision_box_prim)
        self.assertEqual(mesh_collision_api.GetApproximationAttr().Get(), UsdPhysics.Tokens.convexHull)

    def test_physics_inertia(self):
        input_path = "tests/data/inertia.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)

        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        default_prim = stage.GetDefaultPrim()
        self.assertTrue(default_prim.IsValid())

        geometry_scope_prim = stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Geometry"))
        self.assertTrue(geometry_scope_prim.IsValid())

        # link_box1
        link_box1_prim = stage.GetPrimAtPath(geometry_scope_prim.GetPath().AppendChild("link_box1"))
        self.assertTrue(link_box1_prim.IsValid())
        self.assertTrue(link_box1_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(link_box1_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertTrue(link_box1_prim.HasAPI("NewtonArticulationRootAPI"))

        # Mass.
        self.assertTrue(link_box1_prim.HasAPI(UsdPhysics.MassAPI))
        mass_api: UsdPhysics.MassAPI = UsdPhysics.MassAPI(link_box1_prim)
        self.assertTrue(Gf.IsClose(mass_api.GetCenterOfMassAttr().Get(), Gf.Vec3f(0, 0, 0.5), 1e-6))
        self.assertAlmostEqual(mass_api.GetMassAttr().Get(), 0.8, places=6)
        self.assertTrue(Gf.IsClose(mass_api.GetDiagonalInertiaAttr().Get(), Gf.Vec3f(1, 1, 1), 1e-6))
        self.assertRotationsAlmostEqual(mass_api.GetPrincipalAxesAttr().Get(), Gf.Quatf(1, 0, 0, 0))

        # link_box2
        link_box2_prim = stage.GetPrimAtPath(link_box1_prim.GetPath().AppendChild("link_box2"))
        self.assertTrue(link_box2_prim.IsValid())
        self.assertTrue(link_box2_prim.HasAPI(UsdPhysics.RigidBodyAPI))

        # Mass.
        self.assertTrue(link_box2_prim.HasAPI(UsdPhysics.MassAPI))
        mass_api: UsdPhysics.MassAPI = UsdPhysics.MassAPI(link_box2_prim)
        self.assertTrue(Gf.IsClose(mass_api.GetCenterOfMassAttr().Get(), Gf.Vec3f(0, 0, 0.5), 1e-6))
        self.assertAlmostEqual(mass_api.GetMassAttr().Get(), 0.8, places=6)
        self.assertTrue(Gf.IsClose(mass_api.GetDiagonalInertiaAttr().Get(), Gf.Vec3f(1, 1, 2), 1e-6))
        self.assertRotationsAlmostEqual(mass_api.GetPrincipalAxesAttr().Get(), Gf.Quatf(-0.5, 0.5, 0.5, 0.5))

        # link_box3
        link_box3_prim = stage.GetPrimAtPath(link_box2_prim.GetPath().AppendChild("link_box3"))
        self.assertTrue(link_box3_prim.IsValid())
        self.assertTrue(link_box3_prim.HasAPI(UsdPhysics.RigidBodyAPI))

        # Mass.
        self.assertTrue(link_box3_prim.HasAPI(UsdPhysics.MassAPI))
        mass_api: UsdPhysics.MassAPI = UsdPhysics.MassAPI(link_box3_prim)
        self.assertTrue(Gf.IsClose(mass_api.GetCenterOfMassAttr().Get(), Gf.Vec3f(0, 0, 0.5), 1e-6))
        self.assertAlmostEqual(mass_api.GetMassAttr().Get(), 0.8, places=6)
        self.assertTrue(Gf.IsClose(mass_api.GetDiagonalInertiaAttr().Get(), Gf.Vec3f(1, 2, 2), 1e-6))
        self.assertRotationsAlmostEqual(mass_api.GetPrincipalAxesAttr().Get(), Gf.Quatf(1, 0, 0, 0))

        # link_box4
        link_box4_prim = stage.GetPrimAtPath(link_box3_prim.GetPath().AppendChild("link_box4"))
        self.assertTrue(link_box4_prim.IsValid())
        self.assertTrue(link_box4_prim.HasAPI(UsdPhysics.RigidBodyAPI))

        # Mass.
        self.assertTrue(link_box4_prim.HasAPI(UsdPhysics.MassAPI))
        mass_api: UsdPhysics.MassAPI = UsdPhysics.MassAPI(link_box4_prim)
        self.assertTrue(Gf.IsClose(mass_api.GetCenterOfMassAttr().Get(), Gf.Vec3f(0, 0, 0.5), 1e-6))
        self.assertAlmostEqual(mass_api.GetMassAttr().Get(), 0.8, places=6)
        self.assertTrue(Gf.IsClose(mass_api.GetDiagonalInertiaAttr().Get(), Gf.Vec3f(1, 1, 2), 1e-6))
        self.assertRotationsAlmostEqual(mass_api.GetPrincipalAxesAttr().Get(), Gf.Quatf(-0.4632976, -0.4632976, -0.5341866, 0.5341866))

        # link_box5
        link_box5_prim = stage.GetPrimAtPath(link_box4_prim.GetPath().AppendChild("link_box5"))
        self.assertTrue(link_box5_prim.IsValid())
        self.assertTrue(link_box5_prim.HasAPI(UsdPhysics.RigidBodyAPI))

        # Mass.
        self.assertTrue(link_box5_prim.HasAPI(UsdPhysics.MassAPI))
        mass_api: UsdPhysics.MassAPI = UsdPhysics.MassAPI(link_box5_prim)
        self.assertTrue(Gf.IsClose(mass_api.GetCenterOfMassAttr().Get(), Gf.Vec3f(0, 0, 0.5), 1e-6))
        self.assertAlmostEqual(mass_api.GetMassAttr().Get(), 0.8, places=6)
        self.assertTrue(Gf.IsClose(mass_api.GetDiagonalInertiaAttr().Get(), Gf.Vec3f(1, 2, 2), 1e-6))
        self.assertRotationsAlmostEqual(mass_api.GetPrincipalAxesAttr().Get(), Gf.Quatf(6.123234e-17, -0.07088902, 0.9974842, 0))

        # link_box6
        link_box6_prim = stage.GetPrimAtPath(link_box5_prim.GetPath().AppendChild("link_box6"))
        self.assertTrue(link_box6_prim.IsValid())
        self.assertTrue(link_box6_prim.HasAPI(UsdPhysics.RigidBodyAPI))

        # Mass.
        self.assertTrue(link_box6_prim.HasAPI(UsdPhysics.MassAPI))
        mass_api: UsdPhysics.MassAPI = UsdPhysics.MassAPI(link_box6_prim)
        self.assertTrue(Gf.IsClose(mass_api.GetCenterOfMassAttr().Get(), Gf.Vec3f(0, 0, 0.5), 1e-6))
        self.assertAlmostEqual(mass_api.GetMassAttr().Get(), 0.8, places=6)
        self.assertTrue(Gf.IsClose(mass_api.GetDiagonalInertiaAttr().Get(), Gf.Vec3f(0.79289323, 2.2071068, 3), 1e-6))
        self.assertRotationsAlmostEqual(mass_api.GetPrincipalAxesAttr().Get(), Gf.Quatf(6.123234e-17, 0.55557024, 0.8314696, 0))
