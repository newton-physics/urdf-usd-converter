# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import math
import pathlib

import numpy as np
from pxr import Gf, Usd, UsdPhysics

import urdf_usd_converter
from tests.util.ConverterTestCase import ConverterTestCase
from urdf_usd_converter._impl.link import _extract_inertia, _inertia_tensor_in_body_frame
from urdf_usd_converter._impl.urdf_parser.elements import ElementInertia, ElementPose


class TestPhysicsInertia(ConverterTestCase):
    def _rotation_from_rpy(self, rpy):
        """
        Build the rotation matrix for a URDF `origin.rpy`, which is a fixed-axis
        (extrinsic) XYZ rotation: R = Rz(yaw) @ Ry(pitch) @ Rx(roll).
        """
        roll, pitch, yaw = rpy
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rotation_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        rotation_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        rotation_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        return rotation_z @ rotation_y @ rotation_x

    def _assert_newton_inertia(self, prim, ixx, iyy, izz, ixy, ixz, iyz, rpy=(0.0, 0.0, 0.0)):
        """
        `newton:inertia` must be the URDF tensor expressed in the body/link frame,
        so it is compared against `I_body = R(rpy) @ I_urdf @ R(rpy)^T` rebuilt here
        from the URDF values. Deriving the expectation rather than hardcoding the
        rotated components keeps a flipped transform direction reported as a
        convention mismatch. The attribute is `double[]`, so the tolerance is tight
        enough to catch a rotation built in single precision.
        """
        self.assertTrue(prim.HasAPI("NewtonMassAPI"))
        tensor = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
        rotation = self._rotation_from_rpy(rpy)
        i_body = rotation @ tensor @ rotation.T

        inertia = prim.GetAttribute("newton:inertia").Get()
        expected = [i_body[0, 0], i_body[1, 1], i_body[2, 2], i_body[0, 1], i_body[0, 2], i_body[1, 2]]
        for actual, expected_value in zip(inertia, expected):
            self.assertAlmostEqual(actual, expected_value, places=12)

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
        rotation_rpy = self._rotation_from_rpy(rpy)
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
        # principalAxes come from eigendecomposition of I_body (not I_urdf composed with rpy).
        self.assertRotationsAlmostEqual(mass_api.GetPrincipalAxesAttr().Get(), Gf.Quatf(0.9481796, 0.15249318, 0.04637361, -0.27486506))
        # `newton:inertia` is in the body/link frame (URDF inertia rotated by origin.rpy).
        self._assert_newton_inertia(link_box7_prim, 2.0, 1.0, 3.0, 0.5, 0.25, -0.1, rpy=(0.3, -0.4, 0.5))
        self._assert_inertia_reconstructs(link_box7_prim, 2.0, 1.0, 3.0, 0.5, 0.25, -0.1, rpy=(0.3, -0.4, 0.5))

    def _principal_axes(self, scale: float, rpy: tuple[float, float, float], *, izz_factor: float = 2.0) -> Gf.Quatf:
        """
        Principal axes from a pair-degenerate (default) or isotropic URDF tensor
        after rotation into the body frame, via the shipped `_extract_inertia`.
        """
        inertia = ElementInertia()
        inertia.ixx = inertia.iyy = scale
        inertia.izz = izz_factor * scale
        inertia.ixy = inertia.ixz = inertia.iyz = 0.0
        origin = ElementPose()
        origin.rpy = rpy
        return _extract_inertia(_inertia_tensor_in_body_frame(inertia, origin))[0]

    def test_principal_axes_are_scale_invariant(self):
        """
        Assert principalAxes depend on inertia shape, not magnitude.

        Same rpy and tensor shape must yield the same quaternion at any scale.
        Uses pair-degenerate tensors (ixx == iyy != izz) so the shipped
        `_canonicalize_eigenvectors` / `_fix_degenerate_plane` path is exercised
        through `_extract_inertia`.
        """
        for rpy in [(0.37, 1.02, -2.9), (0.8606, -1.4465, -2.8841), (0.1, 0.2, 0.3)]:
            reference = self._principal_axes(1.0, rpy)
            for scale in (1e3, 1e5, 1e6, 1e7):
                self.assertRotationsAlmostEqual(self._principal_axes(scale, rpy), reference)

    def test_large_isotropic_extract_inertia_is_canonical(self):
        """
        Assert `_extract_inertia` returns identity axes for a large isotropic tensor.

        Large isotropic I=2e6 with non-identity rpy must still canonicalize to
        identity principalAxes when eigenvalue degeneracy uses a relative tolerance.
        """
        inertia = ElementInertia()
        inertia.ixx = inertia.iyy = inertia.izz = 2e6
        inertia.ixy = inertia.ixz = inertia.iyz = 0.0
        origin = ElementPose()
        origin.rpy = (0.37, 1.02, -2.9)
        i_body = _inertia_tensor_in_body_frame(inertia, origin)
        orientation, diag_inertia = _extract_inertia(i_body)
        self.assertRotationsAlmostEqual(orientation, Gf.Quatf(1, 0, 0, 0))
        # Gf.IsClose epsilon is absolute: |a-b| < 1e-3. At scale 2e6 that is still
        # a tight relative check (~5e-10); looser than the O(1) tests' 1e-6 only
        # to absorb float32 Vec3f / eigh rounding at large magnitude.
        self.assertTrue(Gf.IsClose(diag_inertia, Gf.Vec3f(2e6, 2e6, 2e6), 1e-3))

    def test_large_isotropic_urdf_principal_axes_are_canonical(self):
        """
        Assert URDF conversion authors identity principalAxes for a large isotropic link.

        Same large isotropic case as above, end-to-end through URDF conversion:
        authored physics:principalAxes must be the canonical identity.
        """
        input_path = "tests/data/inertia_large_degenerate.urdf"
        asset_path = urdf_usd_converter.Converter().convert(input_path, self.tmpDir())
        self.assertIsNotNone(asset_path)

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)
        default_prim = stage.GetDefaultPrim()
        geometry_scope = stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Geometry"))
        link_prim = stage.GetPrimAtPath(geometry_scope.GetPath().AppendChild("link_isotropic_large"))
        self.assertTrue(link_prim.IsValid())

        mass_api = UsdPhysics.MassAPI(link_prim)
        self.assertRotationsAlmostEqual(mass_api.GetPrincipalAxesAttr().Get(), Gf.Quatf(1, 0, 0, 0))
        # Absolute epsilon (see test_large_isotropic_extract_inertia_is_canonical).
        self.assertTrue(Gf.IsClose(mass_api.GetDiagonalInertiaAttr().Get(), Gf.Vec3f(2e6, 2e6, 2e6), 1e-3))
        # Isotropic I_body ≈ s*I; Gf vs NumPy rpy paths differ at ~1e-9 abs, so use relative tol.
        inertia = link_prim.GetAttribute("newton:inertia").Get()
        self.assertTrue(
            np.allclose(inertia[:3], [2e6, 2e6, 2e6], rtol=1e-12, atol=1e-3),
            msg=f"diagonal newton:inertia {inertia[:3]} not isotropic at 2e6",
        )
        self.assertTrue(
            np.allclose(inertia[3:], [0.0, 0.0, 0.0], atol=1e-6),
            msg=f"off-diagonal newton:inertia {inertia[3:]} should be ~0",
        )
        self._assert_inertia_reconstructs(link_prim, 2e6, 2e6, 2e6, 0.0, 0.0, 0.0, rpy=(0.37, 1.02, -2.9))

    def test_large_pair_degenerate_urdf_principal_axes(self):
        """
        Assert URDF conversion handles large pair-degenerate inertia (ixx == iyy != izz).

        Exercises `_fix_degenerate_plane` end-to-end: authored principalAxes must
        match the scale-invariant result from `_extract_inertia`, and reconstruct I_body.
        """
        rpy = (0.37, 1.02, -2.9)
        scale = 2e6
        input_path = "tests/data/inertia_large_degenerate.urdf"
        asset_path = urdf_usd_converter.Converter().convert(input_path, self.tmpDir())
        self.assertIsNotNone(asset_path)

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)
        default_prim = stage.GetDefaultPrim()
        geometry_scope = stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Geometry"))
        parent_prim = stage.GetPrimAtPath(geometry_scope.GetPath().AppendChild("link_isotropic_large"))
        link_prim = stage.GetPrimAtPath(parent_prim.GetPath().AppendChild("link_pair_degenerate_large"))
        self.assertTrue(link_prim.IsValid())

        mass_api = UsdPhysics.MassAPI(link_prim)
        # Same shape at unit scale must yield the same axes (relative degeneracy tolerance).
        self.assertRotationsAlmostEqual(mass_api.GetPrincipalAxesAttr().Get(), self._principal_axes(1.0, rpy))
        # Absolute epsilon (see test_large_isotropic_extract_inertia_is_canonical).
        self.assertTrue(Gf.IsClose(mass_api.GetDiagonalInertiaAttr().Get(), Gf.Vec3f(scale, scale, 2.0 * scale), 1e-3))
        # Gf vs NumPy rpy paths differ at ~1e-9 abs for large |I|; use relative tol.
        expected_body = self._rotation_from_rpy(rpy) @ np.diag([scale, scale, 2.0 * scale]) @ self._rotation_from_rpy(rpy).T
        expected = [
            expected_body[0, 0],
            expected_body[1, 1],
            expected_body[2, 2],
            expected_body[0, 1],
            expected_body[0, 2],
            expected_body[1, 2],
        ]
        inertia = link_prim.GetAttribute("newton:inertia").Get()
        self.assertTrue(
            np.allclose(inertia, expected, rtol=1e-12, atol=1e-3),
            msg=f"newton:inertia {inertia} does not match expected body-frame tensor {expected}",
        )
        self._assert_inertia_reconstructs(link_prim, scale, scale, 2.0 * scale, 0.0, 0.0, 0.0, rpy=rpy)

    def test_origin_mass_without_inertia_skips_principal_axes(self):
        """
        Assert principalAxes is not authored when <inertial> lacks <inertia>.

        mass and centerOfMass should still be authored from the present elements;
        principalAxes, diagonalInertia, and newton:inertia require an inertia tensor.
        """
        input_path = "tests/data/inertia_origin_mass_no_inertia.urdf"
        asset_path = urdf_usd_converter.Converter().convert(input_path, self.tmpDir())
        self.assertIsNotNone(asset_path)

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)
        default_prim = stage.GetDefaultPrim()
        geometry_scope = stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Geometry"))
        link_prim = stage.GetPrimAtPath(geometry_scope.GetPath().AppendChild("link_origin_mass_only"))
        self.assertTrue(link_prim.IsValid())
        self.assertTrue(link_prim.HasAPI(UsdPhysics.MassAPI))

        mass_api = UsdPhysics.MassAPI(link_prim)
        self.assertAlmostEqual(mass_api.GetMassAttr().Get(), 1.0, places=6)
        self.assertTrue(Gf.IsClose(mass_api.GetCenterOfMassAttr().Get(), Gf.Vec3f(0, 0, 0.1), 1e-6))
        self.assertFalse(
            mass_api.GetPrincipalAxesAttr().HasAuthoredValue(),
            msg="physics:principalAxes must not be authored without <inertia>",
        )
        self.assertFalse(
            mass_api.GetDiagonalInertiaAttr().HasAuthoredValue(),
            msg="physics:diagonalInertia must not be authored without <inertia>",
        )
        newton_inertia = link_prim.GetAttribute("newton:inertia")
        self.assertTrue(newton_inertia)
        self.assertFalse(
            newton_inertia.HasAuthoredValue(),
            msg="newton:inertia must not be authored without <inertia>",
        )
