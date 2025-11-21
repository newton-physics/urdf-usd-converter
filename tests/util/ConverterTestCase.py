# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import typing

import omni.asset_validator
import usdex.test
from pxr import Gf, UsdGeom


class ConverterTestCase(usdex.test.TestCase):

    defaultUpAxis = UsdGeom.Tokens.z  # noqa: N815

    defaultValidationIssuePredicates: typing.ClassVar[list[omni.asset_validator.IssuePredicates]] = [  # noqa: N815
        # URDF USD Converter uses nested bodies, as it more faithfully matches the kinematic tree in URDF.
        #
        # Once adopted, the asset validator should be updated to support nested bodies within articulations. For now, we just ignore the issues.
        omni.asset_validator.IssuePredicates.ContainsMessage("Enabled rigid body is missing xformstack reset, when a child of a rigid body"),
        omni.asset_validator.IssuePredicates.ContainsMessage("ArticulationRootAPI definition on a kinematic rigid body is not allowed"),
    ]

    def assert_rotation_almost_equal(self, rot1: Gf.Rotation, rot2: Gf.Rotation, tolerance: float = 1e-6):
        """Assert that two rotations are approximately equal.

        Two quaternions q and -q represent the same rotation, so we check both possibilities.
        """
        q1 = rot1.GetQuat()
        q2 = rot2.GetQuat()

        # Dot product of quaternions: if |dot| ~= 1, they represent the same rotation
        dot = abs(q1.GetReal() * q2.GetReal() + Gf.Dot(q1.GetImaginary(), q2.GetImaginary()))
        self.assertTrue(Gf.IsClose(dot, 1.0, tolerance), f"Rotation mismatch: dot={dot}, q1={q1}, q2={q2}")

    def setUp(self):
        super().setUp()
        # All conversion results should be valid atomic assets
        self.validationEngine.enable_rule(omni.asset_validator.AnchoredAssetPathsChecker)
        self.validationEngine.enable_rule(omni.asset_validator.SupportedFileTypesChecker)
