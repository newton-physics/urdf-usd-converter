# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import math
import pathlib

import numpy as np
from pxr import Gf, Usd, UsdPhysics

import urdf_usd_converter
from tests.util.ConverterTestCase import ConverterTestCase


class TestPhysicsInertia(ConverterTestCase):
    def _assert_newton_inertia(self, prim, ixx, iyy, izz, ixy, ixz, iyz):
        self.assertTrue(prim.HasAPI("NewtonMassAPI"))
        inertia = prim.GetAttribute("newton:inertia").Get()
        expected = [ixx, iyy, izz, ixy, ixz, iyz]
        for actual, expected_value in zip(inertia, expected):
            self.assertAlmostEqual(actual, expected_value, places=6)

    def _assert_inertia_reconstructs(self, prim, ixx, iyy, izz, ixy, ixz, iyz, rpy=(0.0, 0.0, 0.0)):
        """
        The authored principal axes and diagonal inertia must reconstruct the URDF inertia tensor:
        R(principalAxes) @ diag(diagonalInertia) @ R(principalAxes)^T == R(rpy) @ I_urdf @ R(rpy)^T
        The rotation matrix is built from the quaternion components directly, so this check is
        independent of the Gf matrix/vector conventions used by the converter.
        """
        mass_api = UsdPhysics.MassAPI(prim)
        quat = mass_api.GetPrincipalAxesAttr().Get()
        w = quat.GetReal()
        x, y, z = quat.GetImaginary()
        rotation = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ]
        )
        reconstructed = rotation @ np.diag(mass_api.GetDiagonalInertiaAttr().Get()) @ rotation.T

        tensor = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
        # URDF rpy is a fixed-axis (extrinsic) XYZ rotation: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
        roll, pitch, yaw = rpy
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rotation_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        rotation_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        rotation_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        rotation_rpy = rotation_z @ rotation_y @ rotation_x
        expected = rotation_rpy @ tensor @ rotation_rpy.T

        self.assertTrue(
            np.allclose(reconstructed, expected, atol=1e-5),
            msg=f"reconstructed inertia tensor\n{reconstructed}\ndoes not match the URDF inertia tensor\n{expected}",
        )

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
        self._assert_newton_inertia(link_box1_prim, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
        self._assert_inertia_reconstructs(link_box1_prim, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0)

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
        self.assertRotationsAlmostEqual(mass_api.GetPrincipalAxesAttr().Get(), Gf.Quatf(0.5, 0.5, 0.5, 0.5))
        self._assert_newton_inertia(link_box2_prim, 2.0, 1.0, 1.0, 0.0, 0.0, 0.0)
        self._assert_inertia_reconstructs(link_box2_prim, 2.0, 1.0, 1.0, 0.0, 0.0, 0.0)

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
        self._assert_newton_inertia(link_box3_prim, 1.0, 2.0, 2.0, 0.0, 0.0, 0.0)
        self._assert_inertia_reconstructs(link_box3_prim, 1.0, 2.0, 2.0, 0.0, 0.0, 0.0)

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
        self.assertRotationsAlmostEqual(mass_api.GetPrincipalAxesAttr().Get(), Gf.Quatf(0.4632976, 0.4632976, 0.5341866, 0.5341866))
        self._assert_newton_inertia(link_box4_prim, 1.98, 1.02, 1.0, 0.14, 0.0, 0.0)
        self._assert_inertia_reconstructs(link_box4_prim, 1.98, 1.02, 1.0, 0.14, 0.0, 0.0)

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
        self.assertRotationsAlmostEqual(mass_api.GetPrincipalAxesAttr().Get(), Gf.Quatf(0.9974842, 0, 0, 0.07088902))
        self._assert_newton_inertia(link_box5_prim, 1.02, 1.98, 2.0, -0.14, 0.0, 0.0)
        self._assert_inertia_reconstructs(link_box5_prim, 1.02, 1.98, 2.0, -0.14, 0.0, 0.0)

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
        self._assert_newton_inertia(link_box6_prim, 2.0, 1.0, 3.0, 0.5, 0.0, 0.0)
        self._assert_inertia_reconstructs(link_box6_prim, 2.0, 1.0, 3.0, 0.5, 0.0, 0.0)

        # link_box7
        link_box7_prim = stage.GetPrimAtPath(link_box6_prim.GetPath().AppendChild("link_box7"))
        self.assertTrue(link_box7_prim.IsValid())
        self.assertTrue(link_box7_prim.HasAPI(UsdPhysics.RigidBodyAPI))

        # Mass.
        self.assertTrue(link_box7_prim.HasAPI(UsdPhysics.MassAPI))
        mass_api: UsdPhysics.MassAPI = UsdPhysics.MassAPI(link_box7_prim)
        self.assertTrue(Gf.IsClose(mass_api.GetCenterOfMassAttr().Get(), Gf.Vec3f(0, 0, 0.5), 1e-6))
        self.assertAlmostEqual(mass_api.GetMassAttr().Get(), 0.8, places=6)
        self.assertTrue(Gf.IsClose(mass_api.GetDiagonalInertiaAttr().Get(), Gf.Vec3f(0.77679752, 2.1640169, 3.0591856), 1e-6))
        self.assertRotationsAlmostEqual(mass_api.GetPrincipalAxesAttr().Get(), Gf.Quatf(-0.0463736, 0.2748650, 0.9481796, 0.1524932))
        self._assert_newton_inertia(link_box7_prim, 2.0, 1.0, 3.0, 0.5, 0.25, -0.1)
        self._assert_inertia_reconstructs(link_box7_prim, 2.0, 1.0, 3.0, 0.5, 0.25, -0.1, rpy=(0.3, -0.4, 0.5))
