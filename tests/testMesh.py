# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import usdex.test
from pxr import Tf, Usd, UsdGeom

import urdf_usd_converter
from tests.util.ConverterTestCase import ConverterTestCase


class TestMesh(ConverterTestCase):
    def setUp(self):
        super().setUp()

        input_path = "tests/data/meshes.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*The obj format is not yet supported:.*"),
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*The dae format is not yet supported:.*"),
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Unsupported mesh format:.*"),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            asset_path = converter.convert(input_path, output_dir)

        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        self.stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(self.stage)

    def test_stl_mesh(self):
        default_prim = self.stage.GetDefaultPrim()
        geometry_scope_prim = self.stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Geometry"))
        self.assertTrue(geometry_scope_prim.IsValid())

        # Test STL mesh conversion
        link_stl_prim = self.stage.GetPrimAtPath(geometry_scope_prim.GetPath().AppendChild("link_mesh_stl"))
        self.assertTrue(link_stl_prim.IsValid())
        self.assertTrue(link_stl_prim.IsA(UsdGeom.Xform))

        stl_mesh_prim = self.stage.GetPrimAtPath(link_stl_prim.GetPath().AppendChild("box"))
        self.assertTrue(stl_mesh_prim.IsValid())
        self.assertTrue(stl_mesh_prim.IsA(UsdGeom.Mesh))
        self.assertTrue(stl_mesh_prim.HasAuthoredReferences())

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
        self.assertEqual(UsdGeom.Imageable(stl_mesh_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.default_)
