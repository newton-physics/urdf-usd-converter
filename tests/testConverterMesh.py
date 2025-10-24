# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
import pathlib
from unittest.mock import patch

from pxr import Usd, UsdGeom

from tests.util.ConverterTestCase import ConverterTestCase
from urdf_usd_converter._impl.convert import Converter


class TestMeshes(ConverterTestCase):
    @patch("urdf_usd_converter._impl.mesh.Tf.Warn")
    def test_mesh_conversion(self, mock_warn):
        input_path = "tests/data/simple_meshes.urdf"
        output_dir = self.tmpDir()

        converter = Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        # Verify that Tf.Warn was called with the expected message
        mock_warn.assert_called()

        # Check if any call contains the mesh format warning
        warning_obj_found = False
        warning_dae_found = False
        warning_dxf_found = False
        for call in mock_warn.call_args_list:
            if "The obj format is not yet supported: .obj" in str(call):
                warning_obj_found = True
            if "The dae format is not yet supported: .dae" in str(call):
                warning_dae_found = True
            if "Unsupported mesh format: .dxf" in str(call):
                warning_dxf_found = True

        self.assertTrue(warning_obj_found, "Expected warning about obj format not found.")
        self.assertTrue(warning_dae_found, "Expected warning about dae format not found.")
        self.assertTrue(warning_dxf_found, "Expected warning about dxf format not found.")

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        # Test STL mesh conversion
        stl_mesh_prim = stage.GetPrimAtPath("/simple_meshes/Geometry/link_mesh_stl/box")
        self.assertIsNotNone(stl_mesh_prim)
        self.assertTrue(stl_mesh_prim.IsA(UsdGeom.Mesh))
        self.assertTrue(stl_mesh_prim.GetReferences())

        usd_mesh_stl = UsdGeom.Mesh(stl_mesh_prim)
        self.assertTrue(usd_mesh_stl.GetPointsAttr().HasAuthoredValue())
        self.assertTrue(usd_mesh_stl.GetFaceVertexCountsAttr().HasAuthoredValue())
        self.assertTrue(usd_mesh_stl.GetFaceVertexIndicesAttr().HasAuthoredValue())
        # The sample box.stl has normals and they are authored as a primvar
        self.assertFalse(usd_mesh_stl.GetNormalsAttr().HasAuthoredValue())
        normals_primvar: UsdGeom.Primvar = UsdGeom.PrimvarsAPI(usd_mesh_stl).GetPrimvar("normals")
        self.assertTrue(normals_primvar.IsDefined())
        self.assertTrue(normals_primvar.HasAuthoredValue())
        self.assertTrue(normals_primvar.GetIndicesAttr().HasAuthoredValue())
