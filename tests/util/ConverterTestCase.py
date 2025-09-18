# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import omni.asset_validator
import usdex.test
from pxr import Gf, UsdGeom


class ConverterTestCase(usdex.test.TestCase):

    defaultUpAxis = UsdGeom.Tokens.z  # noqa: N815

    def assert_rotation_almost_equal(self, rot1: Gf.Rotation, rot2: Gf.Rotation, tolerance: float = 1e-6):
        self.assertTrue(Gf.IsClose(rot1.GetAxis(), rot2.GetAxis(), tolerance), f"Axis mismatch: {rot1.GetAxis()} != {rot2.GetAxis()}")
        self.assertTrue(Gf.IsClose(rot1.GetAngle(), rot2.GetAngle(), tolerance), f"Angle mismatch: {rot1.GetAngle()} != {rot2.GetAngle()}")

    def setUp(self):
        super().setUp()
        # All conversion results should be valid atomic assets
        self.validationEngine.enable_rule(omni.asset_validator.AnchoredAssetPathsChecker)
        self.validationEngine.enable_rule(omni.asset_validator.SupportedFileTypesChecker)
