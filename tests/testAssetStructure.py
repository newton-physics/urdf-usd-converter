# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import omni.asset_validator
import usdex.core
import usdex.test
from pxr import Kind, Sdf, Tf, Usd, UsdGeom, UsdPhysics, UsdShade

import urdf_usd_converter
from tests.util.ConverterTestCase import ConverterTestCase


class TestAssetStructure(ConverterTestCase):

    def test_display_name(self):
        input_path = "tests/data/test_displayname.urdf"
        asset_path = urdf_usd_converter.Converter().convert(input_path, self.tmpDir())
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

        material_red_prim = material_scope_prim.GetChild("red_mat")
        self.assertTrue(material_red_prim.IsValid())
        self.assertTrue(material_red_prim.IsA(UsdShade.Material))

        material_blue_prim = material_scope_prim.GetChild("blue_mat")
        self.assertTrue(material_blue_prim.IsValid())
        self.assertTrue(material_blue_prim.IsA(UsdShade.Material))

        material_green_prim = material_scope_prim.GetChild("green_mat")
        self.assertTrue(material_green_prim.IsValid())
        self.assertTrue(material_green_prim.IsA(UsdShade.Material))

        material_red_prim = material_scope_prim.GetChild("tn__Material_redmaterial_wT")
        self.assertTrue(material_red_prim.IsValid())
        self.assertTrue(material_red_prim.IsA(UsdShade.Material))
        self.assertEqual(usdex.core.getDisplayName(material_red_prim), "Material_same")

        material_green_prim = material_scope_prim.GetChild("tn__Material_greenmaterial_vW0")
        self.assertTrue(material_green_prim.IsValid())
        self.assertTrue(material_green_prim.IsA(UsdShade.Material))
        self.assertEqual(usdex.core.getDisplayName(material_green_prim), "Material_same")

    def test_interface_layer(self):
        input_path = "tests/data/simple_box.urdf"
        robot_name = pathlib.Path(input_path).stem
        asset_path = urdf_usd_converter.Converter().convert(input_path, self.tmpDir())
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        self.assertEqual(stage.GetRootLayer().identifier, (pathlib.Path(self.tmpDir()) / f"{robot_name}.usda").absolute().as_posix())

        # Test stage metrics
        self.assertEqual(UsdGeom.GetStageUpAxis(stage), UsdGeom.Tokens.z)
        self.assertEqual(UsdGeom.GetStageMetersPerUnit(stage), 1.0)
        self.assertEqual(UsdPhysics.GetStageKilogramsPerUnit(stage), 1.0)
        self.assertEqual(usdex.core.getLayerAuthoringMetadata(stage.GetRootLayer()), f"URDF USD Converter v{urdf_usd_converter.__version__}")

        # Test default prim structure
        default_prim: Usd.Prim = stage.GetDefaultPrim()
        self.assertTrue(default_prim.IsValid())
        self.assertEqual(default_prim.GetName(), robot_name)
        self.assertEqual(default_prim.GetAssetInfoByKey("name"), robot_name)

        self.assertEqual(Usd.ModelAPI(default_prim).GetKind(), Kind.Tokens.component)

        self.assertTrue(default_prim.HasAPI(UsdGeom.ModelAPI))
        self.assertTrue(UsdGeom.ModelAPI(default_prim).GetExtentsHintAttr().HasAuthoredValue())

        payloads: Usd.Payloads = default_prim.GetPayloads()
        self.assertTrue(payloads)

    def test_prim_stack(self):
        input_path = "tests/data/material_mesh_color.urdf"
        asset: Sdf.AssetPath = urdf_usd_converter.Converter().convert(input_path, self.tmpDir())
        parent_path = pathlib.Path(asset.path).parent
        self.assertTrue(pathlib.Path(asset.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset.path)
        self.assertIsValidUsd(stage)

        prim_stack: list[Sdf.PrimSpec] = stage.GetDefaultPrim().GetPrimStack()
        self.assertEqual(len(prim_stack), 5)

        interface_spec: Sdf.PrimSpec = prim_stack[0]
        contents_spec: Sdf.PrimSpec = prim_stack[1]
        physics_spec: Sdf.PrimSpec = prim_stack[2]
        geometry_spec: Sdf.PrimSpec = prim_stack[3]
        materials_spec: Sdf.PrimSpec = prim_stack[4]

        self.assertEqual(interface_spec.layer.identifier, pathlib.Path(asset.path).as_posix())
        self.assertEqual(contents_spec.layer.identifier, (parent_path / pathlib.Path("./Payload/Contents.usda")).as_posix())
        self.assertEqual(physics_spec.layer.identifier, (parent_path / pathlib.Path("./Payload/Physics.usda")).as_posix())
        self.assertEqual(geometry_spec.layer.identifier, (parent_path / pathlib.Path("./Payload/Geometry.usda")).as_posix())
        self.assertEqual(materials_spec.layer.identifier, (parent_path / pathlib.Path("./Payload/Materials.usda")).as_posix())

    def test_prim_stack_no_materials(self):
        input_path = "tests/data/revolute_joints.urdf"
        asset: Sdf.AssetPath = urdf_usd_converter.Converter().convert(input_path, self.tmpDir())
        parent_path = pathlib.Path(asset.path).parent
        self.assertTrue(pathlib.Path(asset.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset.path)
        self.assertIsValidUsd(stage)

        prim_stack: list[Sdf.PrimSpec] = stage.GetDefaultPrim().GetPrimStack()
        self.assertEqual(len(prim_stack), 4)

        interface_spec: Sdf.PrimSpec = prim_stack[0]
        contents_spec: Sdf.PrimSpec = prim_stack[1]
        physics_spec: Sdf.PrimSpec = prim_stack[2]
        geometry_spec: Sdf.PrimSpec = prim_stack[3]

        self.assertEqual(interface_spec.layer.identifier, pathlib.Path(asset.path).as_posix())
        self.assertEqual(contents_spec.layer.identifier, (parent_path / pathlib.Path("./Payload/Contents.usda")).as_posix())
        self.assertEqual(physics_spec.layer.identifier, (parent_path / pathlib.Path("./Payload/Physics.usda")).as_posix())
        self.assertEqual(geometry_spec.layer.identifier, (parent_path / pathlib.Path("./Payload/Geometry.usda")).as_posix())

    def test_contents_layer(self):
        input_path = "tests/data/material_mesh_color.urdf"
        robot_name = pathlib.Path(input_path).stem
        urdf_usd_converter.Converter().convert(input_path, self.tmpDir())

        contents_layer_path = pathlib.Path(self.tmpDir()) / "Payload" / "Contents.usda"
        self.assertTrue(contents_layer_path.exists(), msg=f"Contents layer not found at {contents_layer_path}")
        contents_stage: Usd.Stage = Usd.Stage.Open(contents_layer_path.as_posix())
        self.assertIsValidUsd(contents_stage)

        # Test stage metrics
        self.assertEqual(UsdGeom.GetStageUpAxis(contents_stage), UsdGeom.Tokens.z)
        self.assertEqual(UsdGeom.GetStageMetersPerUnit(contents_stage), 1.0)
        self.assertEqual(UsdPhysics.GetStageKilogramsPerUnit(contents_stage), 1.0)
        self.assertEqual(usdex.core.getLayerAuthoringMetadata(contents_stage.GetRootLayer()), f"URDF USD Converter v{urdf_usd_converter.__version__}")

        # Test default prim structure
        default_prim: Usd.Prim = contents_stage.GetDefaultPrim()
        self.assertTrue(default_prim.IsValid())
        self.assertEqual(default_prim.GetName(), robot_name)

        self.assertEqual(contents_stage.GetRootLayer().subLayerPaths, ["./Physics.usda", "./Geometry.usda", "./Materials.usda"])

    def test_geometry_layer(self):
        input_path = "tests/data/material_mesh_color.urdf"
        robot_name = pathlib.Path(input_path).stem
        asset: Sdf.AssetPath = urdf_usd_converter.Converter().convert(input_path, self.tmpDir())
        parent_path = pathlib.Path(asset.path).parent

        geometry_layer_path = pathlib.Path(self.tmpDir()) / "Payload" / "Geometry.usda"
        self.assertTrue(geometry_layer_path.exists(), msg=f"Geometry layer not found at {geometry_layer_path}")
        geometry_stage: Usd.Stage = Usd.Stage.Open(geometry_layer_path.as_posix())
        self.assertIsValidUsd(geometry_stage)

        # Test stage metrics
        self.assertEqual(UsdGeom.GetStageUpAxis(geometry_stage), UsdGeom.Tokens.z)
        self.assertEqual(UsdGeom.GetStageMetersPerUnit(geometry_stage), 1.0)
        self.assertEqual(UsdPhysics.GetStageKilogramsPerUnit(geometry_stage), 1.0)
        self.assertEqual(usdex.core.getLayerAuthoringMetadata(geometry_stage.GetRootLayer()), f"URDF USD Converter v{urdf_usd_converter.__version__}")

        # Test default prim structure
        default_prim: Usd.Prim = geometry_stage.GetDefaultPrim()
        self.assertTrue(default_prim.IsValid())
        self.assertEqual(default_prim.GetName(), robot_name)

        self.assertEqual(len(geometry_stage.GetDefaultPrim().GetAllChildren()), 1)
        geom_scope = UsdGeom.Scope(geometry_stage.GetDefaultPrim().GetChild("Geometry"))
        self.assertTrue(geom_scope.GetPrim().IsValid())

        # When converted from obj or dae files, the resulting structure may contain multiple meshes.
        # Therefore, here we check whether to reference the geometry library when there is a reference in Xform or mesh.
        for prim in geometry_stage.TraverseAll():
            if prim.HasAuthoredReferences() and (prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Xform)):
                prim_specs: list[Sdf.PrimSpec] = prim.GetPrimStack()
                self.assertEqual(len(prim_specs), 2)
                self.assertEqual(prim_specs[0].layer.identifier, (parent_path / pathlib.Path("./Payload/Geometry.usda")).as_posix())
                self.assertEqual(prim_specs[0].path, prim.GetPath())
                self.assertEqual(prim_specs[1].layer.identifier, (parent_path / pathlib.Path("./Payload/GeometryLibrary.usdc")).as_posix())
                self.assertEqual(prim_specs[1].path, f"/Geometry/{prim.GetName()}")

    def test_materials_layer(self):
        input_path = "tests/data/material_mesh_color.urdf"
        robot_name = pathlib.Path(input_path).stem
        asset: Sdf.AssetPath = urdf_usd_converter.Converter().convert(input_path, self.tmpDir())
        parent_path = pathlib.Path(asset.path).parent

        materials_layer_path = pathlib.Path(self.tmpDir()) / "Payload" / "Materials.usda"
        self.assertTrue(materials_layer_path.exists(), msg=f"Materials layer not found at {materials_layer_path}")
        materials_stage: Usd.Stage = Usd.Stage.Open(materials_layer_path.as_posix())
        # overrides are expected in the material layer
        self.validationEngine.disable_rule(omni.asset_validator.DanglingOverPrimChecker)
        self.assertIsValidUsd(materials_stage)

        # Test stage metrics
        self.assertEqual(UsdGeom.GetStageUpAxis(materials_stage), UsdGeom.Tokens.z)
        self.assertEqual(UsdGeom.GetStageMetersPerUnit(materials_stage), 1.0)
        self.assertEqual(UsdPhysics.GetStageKilogramsPerUnit(materials_stage), 1.0)
        self.assertEqual(
            usdex.core.getLayerAuthoringMetadata(materials_stage.GetRootLayer()), f"URDF USD Converter v{urdf_usd_converter.__version__}"
        )

        # Test default prim structure
        default_prim: Usd.Prim = materials_stage.GetDefaultPrim()
        self.assertTrue(default_prim.IsValid())
        self.assertEqual(default_prim.GetName(), robot_name)

        self.assertEqual(len(materials_stage.GetDefaultPrim().GetAllChildren()), 2)
        materials_scope = UsdGeom.Scope(materials_stage.GetDefaultPrim().GetChild("Materials"))
        self.assertTrue(materials_scope.GetPrim().IsValid())

        for prim in materials_scope.GetPrim().GetAllChildren():
            self.assertTrue(prim.IsA(UsdShade.Material), f"Material {prim.GetPath()} should be a material")
            prim_specs: list[Sdf.PrimSpec] = prim.GetPrimStack()
            self.assertEqual(len(prim_specs), 2)
            self.assertEqual(prim_specs[0].layer.identifier, (parent_path / pathlib.Path("./Payload/Materials.usda")).as_posix())
            self.assertEqual(prim_specs[0].path, prim.GetPath())
            self.assertEqual(prim_specs[1].layer.identifier, (parent_path / pathlib.Path("./Payload/MaterialsLibrary.usdc")).as_posix())
            self.assertEqual(prim_specs[1].path, f"/Materials/{prim.GetName()}")

        geom_scope: Usd.Prim = materials_stage.GetDefaultPrim().GetChild("Geometry")
        for prim in Usd.PrimRange(geom_scope, Usd.PrimAllPrimsPredicate):
            # all prims in the geometry scope are overrides in this layer
            self.assertEqual(prim.GetSpecifier(), Sdf.SpecifierOver)
            if prim.HasAPI(UsdShade.MaterialBindingAPI):
                # any prim with a bound material uses a local material binding with all purposes
                self.assertEqual(prim.GetAppliedSchemas(), [UsdShade.Tokens.MaterialBindingAPI])
                material_binding: UsdShade.MaterialBindingAPI = UsdShade.MaterialBindingAPI(prim)
                self.assertEqual(material_binding.GetMaterialPurposes(), [UsdShade.Tokens.allPurpose, UsdShade.Tokens.preview, UsdShade.Tokens.full])
                self.assertEqual(len(material_binding.GetDirectBindingRel().GetTargets()), 1)
                self.assertTrue(str(material_binding.GetDirectBindingRel().GetTargets()[0]).startswith(f"/{robot_name}/Materials/"))
            else:
                # any prim without a bound material is a pure over in this layer
                self.assertEqual(prim.GetAppliedSchemas(), [])
                self.assertEqual(prim.GetPropertyNames(), [])

    def test_physics_layer(self):
        input_path = "tests/data/revolute_joints.urdf"
        robot_name = pathlib.Path(input_path).stem
        urdf_usd_converter.Converter().convert(input_path, self.tmpDir())

        # kg per unit is authored in the physics layer
        physics_layer_path = pathlib.Path(self.tmpDir()) / "Payload" / "Physics.usda"
        self.assertTrue(physics_layer_path.exists(), msg=f"Physics layer not found at {physics_layer_path}")
        physics_stage: Usd.Stage = Usd.Stage.Open(physics_layer_path.as_posix())
        self.assertIsValidUsd(physics_stage)
        self.assertEqual(UsdPhysics.GetStageKilogramsPerUnit(physics_stage), 1.0)

        # Test stage metrics
        self.assertEqual(UsdGeom.GetStageUpAxis(physics_stage), UsdGeom.Tokens.z)
        self.assertEqual(UsdGeom.GetStageMetersPerUnit(physics_stage), 1.0)
        self.assertEqual(UsdPhysics.GetStageKilogramsPerUnit(physics_stage), 1.0)
        self.assertEqual(usdex.core.getLayerAuthoringMetadata(physics_stage.GetRootLayer()), f"URDF USD Converter v{urdf_usd_converter.__version__}")

        # Test default prim structure
        default_prim: Usd.Prim = physics_stage.GetDefaultPrim()
        self.assertTrue(default_prim.IsValid())
        self.assertEqual(default_prim.GetName(), robot_name)

        self.assertEqual(len(physics_stage.GetDefaultPrim().GetAllChildren()), 2)

        # no visual materials in the physics layer
        materials_scope = UsdGeom.Scope(physics_stage.GetDefaultPrim().GetChild("Materials"))
        self.assertFalse(materials_scope)

        # The Physics scope contains PhysicsJoints.
        physics_scope: Usd.Prim = physics_stage.GetDefaultPrim().GetChild("Physics")
        self.assertTrue(physics_scope.IsA(UsdGeom.Scope))
        self.assertEqual(len(physics_scope.GetAllChildren()), 4)

        joint_root_prim: Usd.Prim = physics_scope.GetChild("joint_root")
        self.assertTrue(joint_root_prim.IsValid())
        self.assertTrue(joint_root_prim.IsA(UsdPhysics.RevoluteJoint))
        self.assertEqual(joint_root_prim.GetSpecifier(), Sdf.SpecifierDef)

        joint_arm_1_prim: Usd.Prim = physics_scope.GetChild("joint_arm_1")
        self.assertTrue(joint_arm_1_prim.IsValid())
        self.assertTrue(joint_arm_1_prim.IsA(UsdPhysics.RevoluteJoint))
        self.assertEqual(joint_root_prim.GetSpecifier(), Sdf.SpecifierDef)

        joint_arm_2_prim: Usd.Prim = physics_scope.GetChild("joint_arm_2")
        self.assertTrue(joint_arm_2_prim.IsValid())
        self.assertTrue(joint_arm_2_prim.IsA(UsdPhysics.RevoluteJoint))
        self.assertEqual(joint_root_prim.GetSpecifier(), Sdf.SpecifierDef)

        joint_arm_3_prim: Usd.Prim = physics_scope.GetChild("joint_arm_3")
        self.assertTrue(joint_arm_3_prim.IsValid())
        self.assertTrue(joint_arm_3_prim.IsA(UsdPhysics.RevoluteJoint))
        self.assertEqual(joint_root_prim.GetSpecifier(), Sdf.SpecifierDef)

        # Test the sidecar PhysicsScene prim
        physics_scene = UsdPhysics.Scene(physics_stage.GetPseudoRoot().GetChild("PhysicsScene"))
        self.assertTrue(physics_scene)
        self.assertEqual(physics_scene.GetPrim().GetAllChildren(), [])

    def test_physics_does_not_leak(self):

        def check_layer(robot_name: str):
            input_path = pathlib.Path(f"./tests/data/{robot_name}.urdf")
            output_dir = pathlib.Path(self.tmpDir()) / robot_name
            urdf_usd_converter.Converter().convert(input_path, output_dir)

            for layer in (output_dir / "Payload").iterdir():
                if layer.is_dir():
                    continue
                if layer.name in ("Contents.usda", "Physics.usda"):
                    continue

                stage: Usd.Stage = Usd.Stage.Open(layer.as_posix())
                for prim in stage.Traverse():
                    for schema in prim.GetAppliedSchemas():
                        self.assertFalse(
                            "Physics" in schema,
                            f"Prim {prim.GetPath()} in {layer.name} layer should not have Physics schemas, but found {schema}",
                        )
                    for prop in prim.GetProperties():
                        self.assertNotEqual(
                            prop.GetNamespace(),
                            "physics",
                            f"Prim {prim.GetPath()} in {layer.name} layer should not have physics properties, but found {prop.GetName()}",
                        )

        check_layer("revolute_joints")  # has bodies and joints and geoms
        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Unsupported mesh format:.*"),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            check_layer("meshes")  # has mesh geoms
        check_layer("material_mesh_texture")  # has textured materials

    def test_physics_scene(self):
        input_path = "tests/data/revolute_joints.urdf"
        asset: Sdf.AssetPath = urdf_usd_converter.Converter().convert(input_path, self.tmpDir())

        stage: Usd.Stage = Usd.Stage.Open(asset.path)
        self.assertIsValidUsd(stage)
        physics_scene: UsdPhysics.Scene = UsdPhysics.Scene(stage.GetPseudoRoot().GetChild("PhysicsScene"))
        self.assertTrue(physics_scene.GetPrim().IsValid())

        # Gravity parameters are not specified because the default values are used.
        self.assertFalse(physics_scene.GetGravityDirectionAttr().HasAuthoredValue())
        self.assertFalse(physics_scene.GetGravityMagnitudeAttr().HasAuthoredValue())

    def test_no_layer_structure_material_texture(self):
        # Test --no-layer-structure with material and textures
        input_path = "tests/data/material_mesh_texture.urdf"
        robot_name = pathlib.Path(input_path).stem
        output_dir = pathlib.Path(self.tmpDir()) / f"{robot_name}_no_layer_structure"
        usdc_path = output_dir / f"{robot_name}.usdc"
        textures_dir = output_dir / "Textures"
        diffuse_texture_file = textures_dir / "grid.png"
        normal_texture_file = textures_dir / "normal.png"
        roughness_texture_file = textures_dir / "roughness.png"
        metallic_texture_file = textures_dir / "metallic.png"
        opacity_texture_file = textures_dir / "opacity.png"
        specular_texture_file = textures_dir / "specular.png"

        # convert without layer structure
        asset_identifier = urdf_usd_converter.Converter(layer_structure=False).convert(input_path, output_dir)

        # check that the asset identifier returned from convert() is the same as the usdc path
        flattened_usdc_path = pathlib.Path(asset_identifier.path).absolute().as_posix()
        self.assertEqual(flattened_usdc_path, usdc_path.absolute().as_posix())

        # check usdc and texture
        self.assertTrue(usdc_path.exists(), f"{usdc_path} not found")
        self.assertTrue(diffuse_texture_file.exists(), f"{diffuse_texture_file} not found")
        self.assertTrue(normal_texture_file.exists(), f"{normal_texture_file} not found")
        self.assertTrue(roughness_texture_file.exists(), f"{roughness_texture_file} not found")
        self.assertTrue(metallic_texture_file.exists(), f"{metallic_texture_file} not found")
        self.assertTrue(opacity_texture_file.exists(), f"{opacity_texture_file} not found")
        self.assertTrue(specular_texture_file.exists(), f"{specular_texture_file} not found")

        # check Shader prim inputs:file
        stage = Usd.Stage.Open(str(usdc_path))
        self.assertIsValidUsd(stage)
        material_prim = stage.GetPrimAtPath(f"/{robot_name}/Materials/texture_mat")
        self.assertTrue(material_prim.IsValid())
        shader = usdex.core.computeEffectivePreviewSurfaceShader(UsdShade.Material(material_prim))
        self.assertTrue(shader)

        texture_input: UsdShade.Input = shader.GetInput("diffuseColor")
        connected_source = texture_input.GetConnectedSource()
        texture_shader_prim = UsdShade.Shader(connected_source[0].GetPrim())

        # The values are defined in the material interface, not in the shader
        value_attrs = UsdShade.Utils.GetValueProducingAttributes(texture_shader_prim.GetInput("file"))
        self.assertEqual(value_attrs[0].GetPrim(), material_prim)
        texture_file_attr = value_attrs[0]
        self.assertEqual(texture_file_attr.Get().path, "./Textures/grid.png")

        texture_input: UsdShade.Input = shader.GetInput("normal")
        connected_source = texture_input.GetConnectedSource()
        texture_shader_prim = UsdShade.Shader(connected_source[0].GetPrim())

        # The values are defined in the material interface, not in the shader
        value_attrs = UsdShade.Utils.GetValueProducingAttributes(texture_shader_prim.GetInput("file"))
        self.assertEqual(value_attrs[0].GetPrim(), material_prim)
        texture_file_attr = value_attrs[0]
        self.assertEqual(texture_file_attr.Get().path, "./Textures/normal.png")

        texture_input: UsdShade.Input = shader.GetInput("roughness")
        connected_source = texture_input.GetConnectedSource()
        texture_shader_prim = UsdShade.Shader(connected_source[0].GetPrim())

        # The values are defined in the material interface, not in the shader
        value_attrs = UsdShade.Utils.GetValueProducingAttributes(texture_shader_prim.GetInput("file"))
        self.assertEqual(value_attrs[0].GetPrim(), material_prim)
        texture_file_attr = value_attrs[0]
        self.assertEqual(texture_file_attr.Get().path, "./Textures/roughness.png")

        texture_input: UsdShade.Input = shader.GetInput("metallic")
        connected_source = texture_input.GetConnectedSource()
        texture_shader_prim = UsdShade.Shader(connected_source[0].GetPrim())

        # The values are defined in the material interface, not in the shader
        value_attrs = UsdShade.Utils.GetValueProducingAttributes(texture_shader_prim.GetInput("file"))
        self.assertEqual(value_attrs[0].GetPrim(), material_prim)
        texture_file_attr = value_attrs[0]
        self.assertEqual(texture_file_attr.Get().path, "./Textures/metallic.png")

        material_prim = stage.GetPrimAtPath(f"/{robot_name}/Materials/texture_opacity_mat")
        self.assertTrue(material_prim.IsValid())
        shader = usdex.core.computeEffectivePreviewSurfaceShader(UsdShade.Material(material_prim))
        self.assertTrue(shader)

        texture_input: UsdShade.Input = shader.GetInput("opacity")
        connected_source = texture_input.GetConnectedSource()
        texture_shader_prim = UsdShade.Shader(connected_source[0].GetPrim())

        # The values are defined in the material interface, not in the shader
        value_attrs = UsdShade.Utils.GetValueProducingAttributes(texture_shader_prim.GetInput("file"))
        self.assertEqual(value_attrs[0].GetPrim(), material_prim)
        texture_file_attr = value_attrs[0]
        self.assertEqual(texture_file_attr.Get().path, "./Textures/opacity.png")

        # Specular Workflow is currently disabled.
        material_prim = stage.GetPrimAtPath(f"/{robot_name}/Materials/specular_workflow_with_texture_mat")
        self.assertTrue(material_prim.IsValid())
        shader = usdex.core.computeEffectivePreviewSurfaceShader(UsdShade.Material(material_prim))
        self.assertTrue(shader)
