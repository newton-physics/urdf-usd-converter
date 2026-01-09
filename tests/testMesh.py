# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import usdex.core
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

        mesh_stl = UsdGeom.Mesh(stl_mesh_prim)
        self.assertTrue(mesh_stl.GetPointsAttr().HasAuthoredValue())
        self.assertTrue(mesh_stl.GetFaceVertexCountsAttr().HasAuthoredValue())
        self.assertTrue(mesh_stl.GetFaceVertexIndicesAttr().HasAuthoredValue())
        # The sample box.stl has normals and they are authored as a primvar
        self.assertFalse(mesh_stl.GetNormalsAttr().HasAuthoredValue())
        normals_primvar: UsdGeom.Primvar = UsdGeom.PrimvarsAPI(mesh_stl).GetPrimvar("normals")
        self.assertTrue(normals_primvar.IsDefined())
        self.assertTrue(normals_primvar.HasAuthoredValue())
        self.assertTrue(normals_primvar.GetIndicesAttr().HasAuthoredValue())
        self.assertEqual(UsdGeom.Imageable(stl_mesh_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.default_)

    def test_obj_single_mesh(self):
        default_prim = self.stage.GetDefaultPrim()
        self.assertTrue(default_prim.IsValid())

        geom_scope_prim = self.stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Geometry"))
        self.assertTrue(geom_scope_prim.IsValid())

        link_prim_path = geom_scope_prim.GetPath().AppendChild("link_mesh_stl").AppendChild("link_mesh_obj")
        link_prim = self.stage.GetPrimAtPath(link_prim_path)
        self.assertTrue(link_prim.IsValid())
        self.assertTrue(link_prim.IsA(UsdGeom.Xform))

        box_prim_path = link_prim_path.AppendChild("box")
        box_prim = self.stage.GetPrimAtPath(box_prim_path)
        self.assertTrue(box_prim.IsValid())
        self.assertTrue(box_prim.IsA(UsdGeom.Mesh))
        self.assertTrue(box_prim.HasAuthoredReferences())
        self.assertEqual(UsdGeom.Imageable(box_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.default_)

        box_collision_prim_path = link_prim_path.AppendChild("collision_box")
        box_collision_prim = self.stage.GetPrimAtPath(box_collision_prim_path)
        self.assertTrue(box_collision_prim.IsValid())
        self.assertTrue(box_collision_prim.IsA(UsdGeom.Mesh))
        self.assertTrue(box_collision_prim.HasAuthoredReferences())
        self.assertEqual(UsdGeom.Imageable(box_collision_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.guide)

    def test_obj_two_meshes(self):
        default_prim = self.stage.GetDefaultPrim()
        self.assertIsNotNone(default_prim)

        geom_scope_prim = self.stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Geometry"))
        self.assertTrue(geom_scope_prim.IsValid())

        link_prim_path = geom_scope_prim.GetPath().AppendChild("link_mesh_stl").AppendChild("link_mesh_multi_objs")
        link_prim = self.stage.GetPrimAtPath(link_prim_path)
        self.assertTrue(link_prim.IsValid())
        self.assertTrue(link_prim.IsA(UsdGeom.Xform))

        # When an obj file contains multiple meshes, each mesh exists as a child of the Xform.
        two_boxes_prim_path = link_prim_path.AppendChild("two_boxes")

        two_boxes_prim = self.stage.GetPrimAtPath(two_boxes_prim_path)
        self.assertTrue(two_boxes_prim.IsValid())
        self.assertTrue(two_boxes_prim.IsA(UsdGeom.Xform))
        self.assertTrue(two_boxes_prim.HasAuthoredReferences())
        self.assertEqual(UsdGeom.Imageable(two_boxes_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.default_)

        cube_red_prim = two_boxes_prim.GetChild("Cube_Red")
        self.assertTrue(cube_red_prim.IsValid())
        self.assertTrue(cube_red_prim.IsA(UsdGeom.Mesh))

        mesh_red = UsdGeom.Mesh(cube_red_prim)
        self.assertTrue(mesh_red.GetPointsAttr().HasAuthoredValue())
        self.assertTrue(mesh_red.GetFaceVertexCountsAttr().HasAuthoredValue())
        self.assertTrue(mesh_red.GetFaceVertexIndicesAttr().HasAuthoredValue())

        cube_green_prim = two_boxes_prim.GetChild("Cube_Green")
        self.assertTrue(cube_green_prim.IsValid())
        self.assertTrue(cube_green_prim.IsA(UsdGeom.Mesh))

        mesh_green = UsdGeom.Mesh(cube_green_prim)
        self.assertTrue(mesh_green.GetPointsAttr().HasAuthoredValue())
        self.assertTrue(mesh_green.GetFaceVertexCountsAttr().HasAuthoredValue())
        self.assertTrue(mesh_green.GetFaceVertexIndicesAttr().HasAuthoredValue())

        two_boxes_collision_prim_path = link_prim_path.AppendChild("two_collision_boxes")
        two_boxes_collision_prim = self.stage.GetPrimAtPath(two_boxes_collision_prim_path)
        self.assertTrue(two_boxes_collision_prim.IsValid())
        self.assertTrue(two_boxes_collision_prim.IsA(UsdGeom.Xform))
        self.assertTrue(two_boxes_collision_prim.HasAuthoredReferences())
        self.assertEqual(UsdGeom.Imageable(two_boxes_collision_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.guide)

        cube_red_prim = two_boxes_collision_prim.GetChild("Cube_Red")
        self.assertTrue(cube_red_prim.IsValid())
        self.assertTrue(cube_red_prim.IsA(UsdGeom.Mesh))

        mesh_red = UsdGeom.Mesh(cube_red_prim)
        self.assertTrue(mesh_red.GetPointsAttr().HasAuthoredValue())
        self.assertTrue(mesh_red.GetFaceVertexCountsAttr().HasAuthoredValue())
        self.assertTrue(mesh_red.GetFaceVertexIndicesAttr().HasAuthoredValue())

        cube_green_prim = two_boxes_collision_prim.GetChild("Cube_Green")
        self.assertTrue(cube_green_prim.IsValid())
        self.assertTrue(cube_green_prim.IsA(UsdGeom.Mesh))

        mesh_green = UsdGeom.Mesh(cube_green_prim)
        self.assertTrue(mesh_green.GetPointsAttr().HasAuthoredValue())
        self.assertTrue(mesh_green.GetFaceVertexCountsAttr().HasAuthoredValue())
        self.assertTrue(mesh_green.GetFaceVertexIndicesAttr().HasAuthoredValue())

    def test_dae_single_mesh(self):
        default_prim = self.stage.GetDefaultPrim()
        self.assertTrue(default_prim.IsValid())

        geom_scope_prim = self.stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Geometry"))
        self.assertTrue(geom_scope_prim.IsValid())

        link_prim_path = geom_scope_prim.GetPath().AppendChild("link_mesh_stl").AppendChild("link_mesh_dae")
        link_prim = self.stage.GetPrimAtPath(link_prim_path)
        self.assertTrue(link_prim.IsValid())
        self.assertTrue(link_prim.IsA(UsdGeom.Xform))

        box_prim = link_prim.GetChild("box")
        self.assertTrue(box_prim.IsValid())
        self.assertTrue(box_prim.IsA(UsdGeom.Mesh))
        self.assertTrue(box_prim.HasAuthoredReferences())
        self.assertEqual(UsdGeom.Imageable(box_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.default_)

        mesh = UsdGeom.Mesh(box_prim)
        self.assertTrue(mesh.GetFaceVertexCountsAttr().HasAuthoredValue())
        self.assertTrue(mesh.GetFaceVertexIndicesAttr().HasAuthoredValue())

        box_collision_prim = link_prim.GetChild("collision_box")
        self.assertTrue(box_collision_prim.IsValid())
        self.assertTrue(box_collision_prim.IsA(UsdGeom.Mesh))
        self.assertTrue(box_collision_prim.HasAuthoredReferences())
        self.assertEqual(UsdGeom.Imageable(box_collision_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.guide)

        mesh = UsdGeom.Mesh(box_collision_prim)
        self.assertTrue(mesh.GetFaceVertexCountsAttr().HasAuthoredValue())
        self.assertTrue(mesh.GetFaceVertexIndicesAttr().HasAuthoredValue())

    def test_dae_unit_cm(self):
        default_prim = self.stage.GetDefaultPrim()
        self.assertTrue(default_prim.IsValid())

        geom_scope_prim = self.stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Geometry"))
        self.assertTrue(geom_scope_prim.IsValid())

        link_prim_path = geom_scope_prim.GetPath().AppendChild("link_mesh_stl").AppendChild("link_mesh_dae").AppendChild("link_mesh_dae_unit_cm")
        link_prim = self.stage.GetPrimAtPath(link_prim_path)
        self.assertTrue(link_prim.IsValid())
        self.assertTrue(link_prim.IsA(UsdGeom.Xform))

        box_prim = link_prim.GetChild("box_unit_cm")
        self.assertTrue(box_prim.IsValid())
        self.assertTrue(box_prim.IsA(UsdGeom.Mesh))
        self.assertTrue(box_prim.HasAuthoredReferences())
        self.assertEqual(UsdGeom.Imageable(box_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.default_)

        mesh = UsdGeom.Mesh(box_prim)
        self.assertTrue(mesh.GetPointsAttr().HasAuthoredValue())
        self.assertTrue(mesh.GetFaceVertexCountsAttr().HasAuthoredValue())
        self.assertTrue(mesh.GetFaceVertexIndicesAttr().HasAuthoredValue())

    def test_dae_two_meshes(self):
        default_prim = self.stage.GetDefaultPrim()
        self.assertTrue(default_prim.IsValid())

        geom_scope_prim = self.stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Geometry"))
        self.assertTrue(geom_scope_prim.IsValid())

        link_prim_path = geom_scope_prim.GetPath().AppendChild("link_mesh_stl").AppendChild("link_mesh_dae").AppendChild("link_two_meshes_dae")
        link_prim = self.stage.GetPrimAtPath(link_prim_path)
        self.assertTrue(link_prim.IsValid())
        self.assertTrue(link_prim.IsA(UsdGeom.Xform))

        two_meshes_prim = link_prim.GetChild("two_meshes")
        self.assertTrue(two_meshes_prim.IsValid())
        self.assertTrue(two_meshes_prim.IsA(UsdGeom.Xform))
        self.assertTrue(two_meshes_prim.HasAuthoredReferences())
        self.assertEqual(UsdGeom.Imageable(two_meshes_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.default_)

        cube_red_prim = two_meshes_prim.GetChild("Cube_Red")
        self.assertTrue(cube_red_prim.IsValid())
        self.assertTrue(cube_red_prim.IsA(UsdGeom.Mesh))

        mesh_red = UsdGeom.Mesh(cube_red_prim)
        self.assertTrue(mesh_red.GetPointsAttr().HasAuthoredValue())
        self.assertTrue(mesh_red.GetFaceVertexCountsAttr().HasAuthoredValue())
        self.assertTrue(mesh_red.GetFaceVertexIndicesAttr().HasAuthoredValue())

        cube_green_prim = two_meshes_prim.GetChild("Cube_Green")
        self.assertTrue(cube_green_prim.IsValid())
        self.assertTrue(cube_green_prim.IsA(UsdGeom.Mesh))

        mesh_green = UsdGeom.Mesh(cube_green_prim)
        self.assertTrue(mesh_green.GetPointsAttr().HasAuthoredValue())
        self.assertTrue(mesh_green.GetFaceVertexCountsAttr().HasAuthoredValue())
        self.assertTrue(mesh_green.GetFaceVertexIndicesAttr().HasAuthoredValue())

    def test_dae_two_triangle_meshes(self):
        default_prim = self.stage.GetDefaultPrim()
        self.assertTrue(default_prim.IsValid())

        geom_scope_prim = self.stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Geometry"))
        self.assertTrue(geom_scope_prim.IsValid())

        link_prim_path = (
            geom_scope_prim.GetPath()
            .AppendChild("link_mesh_stl")
            .AppendChild("link_mesh_dae")
            .AppendChild("link_two_meshes_dae")
            .AppendChild("link_two_meshes_triangle_dae")
        )
        link_prim = self.stage.GetPrimAtPath(link_prim_path)
        self.assertTrue(link_prim.IsValid())
        self.assertTrue(link_prim.IsA(UsdGeom.Xform))

        two_meshes_triangle_prim = link_prim.GetChild("two_meshes_triangle")
        self.assertTrue(two_meshes_triangle_prim.IsValid())
        self.assertTrue(two_meshes_triangle_prim.IsA(UsdGeom.Xform))
        self.assertTrue(two_meshes_triangle_prim.HasAuthoredReferences())
        self.assertEqual(UsdGeom.Imageable(two_meshes_triangle_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.default_)

        cube_red_prim = two_meshes_triangle_prim.GetChild("Cube_Red")
        self.assertTrue(cube_red_prim.IsValid())
        self.assertTrue(cube_red_prim.IsA(UsdGeom.Mesh))

        mesh_red = UsdGeom.Mesh(cube_red_prim)
        self.assertTrue(mesh_red.GetPointsAttr().HasAuthoredValue())
        self.assertTrue(mesh_red.GetFaceVertexCountsAttr().HasAuthoredValue())
        self.assertTrue(mesh_red.GetFaceVertexIndicesAttr().HasAuthoredValue())

        cube_green_prim = two_meshes_triangle_prim.GetChild("Cube_Green")
        self.assertTrue(cube_green_prim.IsValid())
        self.assertTrue(cube_green_prim.IsA(UsdGeom.Mesh))

        mesh_green = UsdGeom.Mesh(cube_green_prim)
        self.assertTrue(mesh_green.GetPointsAttr().HasAuthoredValue())
        self.assertTrue(mesh_green.GetFaceVertexCountsAttr().HasAuthoredValue())
        self.assertTrue(mesh_green.GetFaceVertexIndicesAttr().HasAuthoredValue())

    def test_dae_two_materials(self):
        default_prim = self.stage.GetDefaultPrim()
        self.assertTrue(default_prim.IsValid())

        geom_scope_prim = self.stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Geometry"))
        self.assertTrue(geom_scope_prim.IsValid())

        link_prim_path = (
            geom_scope_prim.GetPath()
            .AppendChild("link_mesh_stl")
            .AppendChild("link_mesh_dae")
            .AppendChild("link_two_meshes_dae")
            .AppendChild("link_two_meshes_triangle_dae")
            .AppendChild("link_box_two_materials_dae")
        )
        link_prim = self.stage.GetPrimAtPath(link_prim_path)
        self.assertTrue(link_prim.IsValid())
        self.assertTrue(link_prim.IsA(UsdGeom.Xform))

        box_two_materials_prim = link_prim.GetChild("box_two_materials")
        self.assertTrue(box_two_materials_prim.IsValid())
        self.assertTrue(box_two_materials_prim.IsA(UsdGeom.Mesh))
        self.assertTrue(box_two_materials_prim.HasAuthoredReferences())
        self.assertEqual(UsdGeom.Imageable(box_two_materials_prim).GetPurposeAttr().Get(), UsdGeom.Tokens.default_)

        mesh = UsdGeom.Mesh(box_two_materials_prim)
        self.assertTrue(mesh.GetPointsAttr().HasAuthoredValue())
        self.assertTrue(mesh.GetFaceVertexCountsAttr().HasAuthoredValue())
        self.assertTrue(mesh.GetFaceVertexIndicesAttr().HasAuthoredValue())

        material_001_prim = box_two_materials_prim.GetChild("GeomSubset_001")
        self.assertTrue(material_001_prim.IsValid())
        self.assertTrue(material_001_prim.IsA(UsdGeom.Subset))
        material_002_prim = box_two_materials_prim.GetChild("GeomSubset_002")
        self.assertTrue(material_002_prim.IsValid())
        self.assertTrue(material_002_prim.IsA(UsdGeom.Subset))
