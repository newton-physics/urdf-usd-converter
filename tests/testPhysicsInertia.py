# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

from pxr import Gf, Usd, UsdPhysics

import urdf_usd_converter
from tests.util.ConverterTestCase import ConverterTestCase


class TestPhysicsInertia(ConverterTestCase):
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
        self.assertRotationsAlmostEqual(mass_api.GetPrincipalAxesAttr().Get(), Gf.Quatf(-0.4632976, 0.4632976, 0.5341866, 0.5341866))

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
        self.assertRotationsAlmostEqual(mass_api.GetPrincipalAxesAttr().Get(), Gf.Quatf(0.9974842, 0, 0, -0.07088902))

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
        self.assertRotationsAlmostEqual(mass_api.GetPrincipalAxesAttr().Get(), Gf.Quatf(0, 0.55557024, 0.8314696, 0))
