# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import usdex.core
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

import urdf_usd_converter
from tests.util.ConverterTestCase import ConverterTestCase


class TestDisplayName(ConverterTestCase):
    def test_display_name(self):
        input_path = "tests/data/test_displayname.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        default_prim = stage.GetDefaultPrim()
        self.assertTrue(default_prim.IsValid())

        geometry_scope_prim = default_prim.GetChild("Geometry")
        self.assertIsNotNone(geometry_scope_prim)

        link_box_prim = geometry_scope_prim.GetChild("tn__linkbox_sA")
        self.assertTrue(link_box_prim.IsValid())
        self.assertTrue(link_box_prim.IsA(UsdGeom.Xform))
        self.assertEqual(usdex.core.getDisplayName(link_box_prim), "link-box")

        box_prim = link_box_prim.GetChild("box")
        self.assertTrue(box_prim.IsValid())
        self.assertTrue(box_prim.IsA(UsdGeom.Cube))

        link_box_2_prim = link_box_prim.GetChild("tn__linkbox2_bC")
        self.assertTrue(link_box_2_prim.IsValid())
        self.assertTrue(link_box_2_prim.IsA(UsdGeom.Xform))
        self.assertEqual(usdex.core.getDisplayName(link_box_2_prim), "link-box2")

        box_2_prim = link_box_2_prim.GetChild("box")
        self.assertTrue(box_2_prim.IsValid())
        self.assertTrue(box_2_prim.IsA(UsdGeom.Cube))

        # Check for obj containing two meshes.
        link_mesh_obj_prim = link_box_2_prim.GetChild("tn__linkmesh_obj_VI")
        self.assertTrue(link_mesh_obj_prim.IsValid())
        self.assertTrue(link_mesh_obj_prim.IsA(UsdGeom.Xform))
        self.assertEqual(usdex.core.getDisplayName(link_mesh_obj_prim), "link-mesh_obj")

        mesh_obj_prim = link_mesh_obj_prim.GetChild("name_test")
        self.assertTrue(mesh_obj_prim.IsValid())
        self.assertTrue(mesh_obj_prim.IsA(UsdGeom.Xform))

        mesh_obj_mesh_prim = mesh_obj_prim.GetChild("tn__CubeRed_YE")
        self.assertTrue(mesh_obj_mesh_prim.IsValid())
        self.assertTrue(mesh_obj_mesh_prim.IsA(UsdGeom.Mesh))
        self.assertEqual(usdex.core.getDisplayName(mesh_obj_mesh_prim), "Cube:Red")

        mesh_obj_mesh_prim = mesh_obj_prim.GetChild("tn__CubeGreen_vH")
        self.assertTrue(mesh_obj_mesh_prim.IsValid())
        self.assertTrue(mesh_obj_mesh_prim.IsA(UsdGeom.Mesh))
        self.assertEqual(usdex.core.getDisplayName(mesh_obj_mesh_prim), "Cube:Green")

        # Check for dae containing two meshes.
        link_mesh_dae_prim = link_box_2_prim.GetChild("tn__linkmesh_dae_VI")
        self.assertTrue(link_mesh_dae_prim.IsValid())
        self.assertTrue(link_mesh_dae_prim.IsA(UsdGeom.Xform))
        self.assertEqual(usdex.core.getDisplayName(link_mesh_dae_prim), "link-mesh_dae")

        mesh_dae_prim = link_mesh_dae_prim.GetChild("name_test")
        self.assertTrue(mesh_dae_prim.IsValid())
        self.assertTrue(mesh_dae_prim.IsA(UsdGeom.Xform))

        mesh_dae_mesh_prim = mesh_dae_prim.GetChild("tn__Cube001_VB")
        self.assertTrue(mesh_dae_mesh_prim.IsValid())
        self.assertTrue(mesh_dae_mesh_prim.IsA(UsdGeom.Mesh))
        self.assertEqual(usdex.core.getDisplayName(mesh_dae_mesh_prim), "Cube.001")

        mesh_dae_mesh_prim = mesh_dae_prim.GetChild("tn__Cube002_VB")
        self.assertTrue(mesh_dae_mesh_prim.IsValid())
        self.assertTrue(mesh_dae_mesh_prim.IsA(UsdGeom.Mesh))
        self.assertEqual(usdex.core.getDisplayName(mesh_dae_mesh_prim), "Cube.002")

        # Check for physics.
        physics_scope_prim = default_prim.GetChild("Physics")
        self.assertTrue(physics_scope_prim.IsValid())

        joint_root_prim = physics_scope_prim.GetChild("tn__jointroot_wH")
        self.assertTrue(joint_root_prim.IsValid())
        self.assertTrue(joint_root_prim.IsA(UsdPhysics.FixedJoint))
        self.assertEqual(usdex.core.getDisplayName(joint_root_prim), "joint:root")

        # Check for materials.
        material_scope_prim = default_prim.GetChild("Materials")
        self.assertTrue(material_scope_prim.IsValid())

        material_red_prim = material_scope_prim.GetChild("tn__materialred_rL")
        self.assertTrue(material_red_prim.IsValid())
        self.assertTrue(material_red_prim.IsA(UsdShade.Material))
        self.assertEqual(usdex.core.getDisplayName(material_red_prim), "material:red")
