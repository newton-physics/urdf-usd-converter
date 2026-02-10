# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import usdex.core
import usdex.test
from pxr import Tf, Usd, UsdGeom, UsdPhysics, UsdShade

import urdf_usd_converter
from tests.util.ConverterTestCase import ConverterTestCase


class TestUndefined(ConverterTestCase):
    def setUp(self):
        super().setUp()

        input_path = "tests/data/undefined.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Transmission is not supported.*"),
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Gazebo is not supported.*"),
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Calibration is not supported.*"),
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Dynamics is not supported.*"),
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Safety controller is not supported.*"),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            asset_path = converter.convert(input_path, output_dir)

        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        self.stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(self.stage)

    def test_check_material(self):
        default_prim = self.stage.GetDefaultPrim()
        material_scope_prim = self.stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Materials"))
        self.assertTrue(material_scope_prim.IsValid())

        blue_prim = material_scope_prim.GetPrimAtPath(material_scope_prim.GetPath().AppendChild("blue"))
        self.assertTrue(blue_prim.IsValid())
        self.assertTrue(blue_prim.IsA(UsdShade.Material))

        # Custom attributes in "blue" Material.
        self.assertTrue(blue_prim.GetAttribute("urdf:custom_attr").HasAuthoredValue())
        attr = blue_prim.GetAttribute("urdf:custom_attr").Get()
        self.assertEqual(attr, "blue_material")

    def test_check_link(self):
        default_prim = self.stage.GetDefaultPrim()
        geometry_scope_prim = self.stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Geometry"))
        self.assertTrue(geometry_scope_prim.IsValid())

        link_box_prim = geometry_scope_prim.GetPrimAtPath(geometry_scope_prim.GetPath().AppendChild("link_box"))
        self.assertTrue(link_box_prim.IsValid())
        self.assertTrue(link_box_prim.IsA(UsdGeom.Xform))

        box_prim = link_box_prim.GetPrimAtPath(link_box_prim.GetPath().AppendChild("box"))
        self.assertTrue(box_prim.IsValid())
        self.assertTrue(box_prim.IsA(UsdGeom.Cube))

        # Custom attributes in "box".
        self.assertTrue(box_prim.GetAttribute("urdf:custom_attr").HasAuthoredValue())
        attr = box_prim.GetAttribute("urdf:custom_attr").Get()
        self.assertEqual(attr, "visual")

        collision_box_prim = link_box_prim.GetPrimAtPath(link_box_prim.GetPath().AppendChild("collision_box"))
        self.assertTrue(collision_box_prim.IsValid())
        self.assertTrue(collision_box_prim.IsA(UsdGeom.Cube))

        # Custom attributes in "collision_box".
        self.assertTrue(collision_box_prim.GetAttribute("urdf:custom_attr").HasAuthoredValue())
        attr = collision_box_prim.GetAttribute("urdf:custom_attr").Get()
        self.assertEqual(attr, "collision")

        # Since "collision_box" is a Cube, it cannot have children.
        # So the custom element is being moved to its parent.
        collision_data_prim = link_box_prim.GetPrimAtPath(link_box_prim.GetPath().AppendChild("collision_data"))
        self.assertTrue(collision_data_prim.IsValid())
        self.assertTrue(collision_data_prim.IsA(UsdGeom.Scope))
        self.assertTrue(collision_data_prim.GetAttribute("urdf:text").HasAuthoredValue())
        text = collision_data_prim.GetAttribute("urdf:text").Get()
        self.assertEqual(text, "custom collision data")

        # custom element.
        undefined_custom_prim = link_box_prim.GetPrimAtPath(link_box_prim.GetPath().AppendChild("custom"))
        self.assertTrue(undefined_custom_prim.IsValid())
        self.assertTrue(undefined_custom_prim.IsA(UsdGeom.Scope))

        # Custom element.
        item_prim = undefined_custom_prim.GetPrimAtPath(undefined_custom_prim.GetPath().AppendChild("item1"))
        self.assertTrue(item_prim.IsValid())
        self.assertTrue(item_prim.IsA(UsdGeom.Scope))

        # Custom attributes in "item1".
        self.assertTrue(item_prim.GetAttribute("urdf:name").HasAuthoredValue())
        name = item_prim.GetAttribute("urdf:name").Get()
        self.assertEqual(name, "data1")
        self.assertTrue(item_prim.GetAttribute("urdf:value").HasAuthoredValue())
        value = item_prim.GetAttribute("urdf:value").Get()
        self.assertEqual(value, "foo1")

        # Custom element item.
        item_prim = undefined_custom_prim.GetPrimAtPath(undefined_custom_prim.GetPath().AppendChild("tn__testitem_bC"))
        self.assertTrue(item_prim.IsValid())
        self.assertTrue(item_prim.IsA(UsdGeom.Scope))
        self.assertEqual(usdex.core.getDisplayName(item_prim), "test-item")

        # Custom attributes in "test-item".
        self.assertTrue(item_prim.GetAttribute("urdf:name").HasAuthoredValue())
        name = item_prim.GetAttribute("urdf:name").Get()
        self.assertEqual(name, "data2")
        self.assertTrue(item_prim.GetAttribute("urdf:value").HasAuthoredValue())
        value = item_prim.GetAttribute("urdf:value").Get()
        self.assertEqual(value, "foo2")

        # custom element. The "custom" name is a duplicate, so the prim name has been renamed.
        undefined_custom_prim = link_box_prim.GetPrimAtPath(link_box_prim.GetPath().AppendChild("custom_1"))
        self.assertTrue(undefined_custom_prim.IsValid())
        self.assertTrue(undefined_custom_prim.IsA(UsdGeom.Scope))
        self.assertEqual(usdex.core.getDisplayName(undefined_custom_prim), "custom")

        # Custom element item.
        item_prim = undefined_custom_prim.GetPrimAtPath(undefined_custom_prim.GetPath().AppendChild("item3"))
        self.assertTrue(item_prim.IsValid())
        self.assertTrue(item_prim.IsA(UsdGeom.Scope))

        # Custom attributes in "item3".
        self.assertTrue(item_prim.GetAttribute("urdf:name").HasAuthoredValue())
        name = item_prim.GetAttribute("urdf:name").Get()
        self.assertEqual(name, "data3")
        self.assertTrue(item_prim.GetAttribute("urdf:value").HasAuthoredValue())
        value = item_prim.GetAttribute("urdf:value").Get()
        self.assertEqual(value, "foo3")

    def test_check_physics(self):
        default_prim = self.stage.GetDefaultPrim()
        physics_scope_prim = self.stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Physics"))
        self.assertTrue(physics_scope_prim.IsValid())

        joint_box_prim = physics_scope_prim.GetPrimAtPath(physics_scope_prim.GetPath().AppendChild("joint_box"))
        self.assertTrue(joint_box_prim.IsValid())
        self.assertTrue(joint_box_prim.IsA(UsdPhysics.FixedJoint))

        # Custom attributes in "joint_box".
        self.assertTrue(joint_box_prim.GetAttribute("urdf:custom_attr").HasAuthoredValue())
        attr = joint_box_prim.GetAttribute("urdf:custom_attr").Get()
        self.assertEqual(attr, "joint")

        # Custom element.
        data_prim = joint_box_prim.GetPrimAtPath(joint_box_prim.GetPath().AppendChild("data"))
        self.assertTrue(data_prim.IsValid())
        self.assertTrue(data_prim.IsA(UsdGeom.Scope))

        # Custom attributes in "data".
        self.assertTrue(data_prim.GetAttribute("urdf:text").HasAuthoredValue())
        text = data_prim.GetAttribute("urdf:text").Get()
        self.assertEqual(text, "joint data")

        # Cutstom attributes in "calibration".
        calibration_prim = joint_box_prim.GetPrimAtPath(joint_box_prim.GetPath().AppendChild("calibration"))
        self.assertTrue(calibration_prim.IsValid())
        self.assertTrue(calibration_prim.IsA(UsdGeom.Scope))
        self.assertTrue(calibration_prim.GetAttribute("urdf:rising").HasAuthoredValue())
        rising = calibration_prim.GetAttribute("urdf:rising").Get()
        self.assertEqual(rising, "0.3")
        self.assertTrue(calibration_prim.GetAttribute("urdf:falling").HasAuthoredValue())
        falling = calibration_prim.GetAttribute("urdf:falling").Get()
        self.assertEqual(falling, "0.2")
        self.assertTrue(calibration_prim.GetAttribute("urdf:reference_position").HasAuthoredValue())
        reference_position = calibration_prim.GetAttribute("urdf:reference_position").Get()
        self.assertEqual(reference_position, "0.1")

        # Cutstom attributes in "dynamics".
        dynamics_prim = joint_box_prim.GetPrimAtPath(joint_box_prim.GetPath().AppendChild("dynamics"))
        self.assertTrue(dynamics_prim.IsValid())
        self.assertTrue(dynamics_prim.IsA(UsdGeom.Scope))
        self.assertTrue(dynamics_prim.GetAttribute("urdf:damping").HasAuthoredValue())
        damping = dynamics_prim.GetAttribute("urdf:damping").Get()
        self.assertEqual(damping, "0.0")
        self.assertTrue(dynamics_prim.GetAttribute("urdf:friction").HasAuthoredValue())
        friction = dynamics_prim.GetAttribute("urdf:friction").Get()
        self.assertEqual(friction, "0.0")

        # Cutstom attributes in "safety_controller".
        safety_controller_prim = joint_box_prim.GetPrimAtPath(joint_box_prim.GetPath().AppendChild("safety_controller"))
        self.assertTrue(safety_controller_prim.IsValid())
        self.assertTrue(safety_controller_prim.IsA(UsdGeom.Scope))
        self.assertTrue(safety_controller_prim.GetAttribute("urdf:k_velocity").HasAuthoredValue())
        k_velocity = safety_controller_prim.GetAttribute("urdf:k_velocity").Get()
        self.assertEqual(k_velocity, "10.0")
        self.assertTrue(safety_controller_prim.GetAttribute("urdf:k_position").HasAuthoredValue())
        k_position = safety_controller_prim.GetAttribute("urdf:k_position").Get()
        self.assertEqual(k_position, "15.0")
        self.assertTrue(safety_controller_prim.GetAttribute("urdf:soft_lower_limit").HasAuthoredValue())
        soft_lower_limit = safety_controller_prim.GetAttribute("urdf:soft_lower_limit").Get()
        self.assertEqual(soft_lower_limit, "-2.0")
        self.assertTrue(safety_controller_prim.GetAttribute("urdf:soft_upper_limit").HasAuthoredValue())
        soft_upper_limit = safety_controller_prim.GetAttribute("urdf:soft_upper_limit").Get()
        self.assertEqual(soft_upper_limit, "0.5")

    def test_check_custom_elements(self):
        default_prim = self.stage.GetDefaultPrim()
        geometry_scope_prim = self.stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Geometry"))
        self.assertTrue(geometry_scope_prim.IsValid())

        # If there are any custom elements other than "link", "joint", and "material",
        # they will be in the "custom" scope.
        custom_prim = geometry_scope_prim.GetPrimAtPath(geometry_scope_prim.GetPath().AppendChild("custom"))
        self.assertTrue(custom_prim.IsValid())
        self.assertTrue(custom_prim.IsA(UsdGeom.Scope))

        # Custom element.
        transmission_prim = custom_prim.GetPrimAtPath(custom_prim.GetPath().AppendChild("transmission"))
        self.assertTrue(transmission_prim.IsValid())
        self.assertTrue(transmission_prim.IsA(UsdGeom.Scope))

        # Custom attributes in "transmission".
        self.assertTrue(transmission_prim.GetAttribute("urdf:name").HasAuthoredValue())
        name = transmission_prim.GetAttribute("urdf:name").Get()
        self.assertEqual(name, "trans1")

        # Custom element.
        type_prim = transmission_prim.GetPrimAtPath(transmission_prim.GetPath().AppendChild("type"))
        self.assertTrue(type_prim.IsValid())
        self.assertTrue(type_prim.IsA(UsdGeom.Scope))

        # Custom attributes in "type".
        self.assertTrue(type_prim.GetAttribute("urdf:text").HasAuthoredValue())
        text = type_prim.GetAttribute("urdf:text").Get()
        self.assertEqual(text, "transmission_interface/SimpleTransmission")

        # Custom element.
        transmission_prim = custom_prim.GetPrimAtPath(custom_prim.GetPath().AppendChild("transmission_1"))
        self.assertTrue(transmission_prim.IsValid())
        self.assertTrue(transmission_prim.IsA(UsdGeom.Scope))
        self.assertEqual(usdex.core.getDisplayName(transmission_prim), "transmission")

        # Custom attributes in "transmission_1".
        self.assertTrue(transmission_prim.GetAttribute("urdf:name").HasAuthoredValue())
        name = transmission_prim.GetAttribute("urdf:name").Get()
        self.assertEqual(name, "trans2")

        # Custom element.
        type_prim = transmission_prim.GetPrimAtPath(transmission_prim.GetPath().AppendChild("type"))
        self.assertTrue(type_prim.IsValid())
        self.assertTrue(type_prim.IsA(UsdGeom.Scope))

        # Custom attributes in "type".
        self.assertTrue(type_prim.GetAttribute("urdf:text").HasAuthoredValue())
        text = type_prim.GetAttribute("urdf:text").Get()
        self.assertEqual(text, "transmission_interface/SimpleTransmission")

        # Custom element.
        gazebo_prim = custom_prim.GetPrimAtPath(custom_prim.GetPath().AppendChild("gazebo"))
        self.assertTrue(gazebo_prim.IsValid())
        self.assertTrue(gazebo_prim.IsA(UsdGeom.Scope))

        # Custom element.
        static_prim = gazebo_prim.GetPrimAtPath(gazebo_prim.GetPath().AppendChild("static"))
        self.assertTrue(static_prim.IsValid())
        self.assertTrue(static_prim.IsA(UsdGeom.Scope))

        # Custom attributes in "static".
        self.assertTrue(static_prim.GetAttribute("urdf:text").HasAuthoredValue())
        text = static_prim.GetAttribute("urdf:text").Get()
        self.assertEqual(text, "true")


class TestUndefinedOthers(ConverterTestCase):
    def test_check_custom_name(self):
        """
        Test case where the root name of the link is "custom" and
        the root of the custom element has the same name "custom".
        """
        input_path = "tests/data/undefined_same_name.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        default_prim = stage.GetDefaultPrim()
        geometry_scope_prim = stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Geometry"))
        self.assertTrue(geometry_scope_prim.IsValid())

        # This is the prim of link.
        link_prim = geometry_scope_prim.GetPrimAtPath(geometry_scope_prim.GetPath().AppendChild("custom"))
        self.assertTrue(link_prim.IsValid())
        self.assertTrue(link_prim.IsA(UsdGeom.Xform))

        # If there are any custom elements other than "link", "joint", and "material",
        # they will be in the "custom" scope.
        # The "custom" name is a duplicate, so the prim name has been renamed.
        custom_prim = geometry_scope_prim.GetPrimAtPath(geometry_scope_prim.GetPath().AppendChild("custom_1"))
        self.assertTrue(custom_prim.IsValid())
        self.assertTrue(custom_prim.IsA(UsdGeom.Scope))
        self.assertEqual(usdex.core.getDisplayName(custom_prim), "custom")

        # Custom element item.
        foo_prim = custom_prim.GetPrimAtPath(custom_prim.GetPath().AppendChild("foo"))
        self.assertTrue(foo_prim.IsValid())
        self.assertTrue(foo_prim.IsA(UsdGeom.Scope))

        item_prim = foo_prim.GetPrimAtPath(foo_prim.GetPath().AppendChild("item1"))
        self.assertTrue(item_prim.IsValid())
        self.assertTrue(item_prim.IsA(UsdGeom.Scope))

        self.assertTrue(item_prim.GetAttribute("urdf:name").HasAuthoredValue())
        name = item_prim.GetAttribute("urdf:name").Get()
        self.assertEqual(name, "data1")
        self.assertTrue(item_prim.GetAttribute("urdf:value").HasAuthoredValue())
        value = item_prim.GetAttribute("urdf:value").Get()
        self.assertEqual(value, "foo1")

    def test_check_mimic_joint(self):
        """
        Test case where the joint has a mimic element.
        Currently unsupported, it stores custom elements and custom attributes of mimic joints.
        """
        input_path = "tests/data/mimic_joint.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Mimic is not supported.*"),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            asset_path = converter.convert(input_path, output_dir)

        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        stage: Usd.Stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        default_prim = stage.GetDefaultPrim()
        physics_scope_prim = stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Physics"))
        self.assertTrue(physics_scope_prim.IsValid())

        joint_b_prim = physics_scope_prim.GetChild("jointB")
        self.assertTrue(joint_b_prim.IsValid())
        self.assertTrue(joint_b_prim.IsA(UsdPhysics.RevoluteJoint))

        mimic_prim = joint_b_prim.GetChild("mimic")
        self.assertTrue(mimic_prim.IsValid())
        self.assertTrue(mimic_prim.IsA(UsdGeom.Scope))

        # Custom attributes in "mimic".
        self.assertTrue(mimic_prim.GetAttribute("urdf:joint").HasAuthoredValue())
        joint = mimic_prim.GetAttribute("urdf:joint").Get()
        self.assertEqual(joint, "jointA")
        self.assertTrue(mimic_prim.GetAttribute("urdf:multiplier").HasAuthoredValue())
        multiplier = mimic_prim.GetAttribute("urdf:multiplier").Get()
        self.assertEqual(multiplier, "2.0")
        self.assertTrue(mimic_prim.GetAttribute("urdf:offset").HasAuthoredValue())
        offset = mimic_prim.GetAttribute("urdf:offset").Get()
        self.assertEqual(offset, "0.0")
