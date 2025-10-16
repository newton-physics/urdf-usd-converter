# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
import pathlib

from tests.util.ConverterTestCase import ConverterTestCase
from urdf_usd_converter._impl.urdf_parser.parser import URDFParser


class TestURDFParser(ConverterTestCase):
    def setUp(self):
        super().setUp()

        # Parse the URDF file.
        # This file contains an XML header (<?xml version="1.0"?>).
        # We can trace the URDF structure from the root_element.
        # The root element will be "robot".
        self.parser = URDFParser(pathlib.Path("tests/data/verifying_elements.urdf"))
        try:
            self.parser.parse()
        except Exception as e:
            self.fail(f"Error parsing URDF file: {e}")

    def test_load_non_existent_file(self):
        # Load a non-existent URDF file.
        model_path = pathlib.Path("tests/data/non_existent.urdf")
        parser = URDFParser(model_path)
        try:
            parser.parse()
            self.fail("No exception occurred while loading the error XML.")
        except Exception as e:
            self.assertTrue(isinstance(e, FileNotFoundError))

    def test_load_error_xml_syntax(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_xml_syntax.urdf")
        parser = URDFParser(model_path)

        try:
            parser.parse()
            self.fail("No exception occurred while loading the error XML.")
        except Exception as e:
            self.assertTrue(isinstance(e, RuntimeError))
            self.assertTrue("5:2: mismatched tag" in e.args[0])

    def test_load_error_no_material_name(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_no_material_name.urdf")
        parser = URDFParser(model_path)

        try:
            parser.parse()
            self.fail("No exception occurred while loading the error XML.")
        except Exception as e:
            self.assertTrue(isinstance(e, RuntimeError))
            self.assertTrue("material: name is required (line: 4)" in e.args[0])

    def test_load_error_duplicate_material_names(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_dupilcate_material_names.urdf")
        parser = URDFParser(model_path)

        try:
            parser.parse()
            self.fail("No exception occurred while loading the error XML.")
        except Exception as e:
            self.assertTrue(isinstance(e, RuntimeError))
            self.assertTrue("material: Material name 'green' already exists (line: 8)" in e.args[0])

    def test_load_error_duplicate_link_names(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_duplicate_link_names.urdf")
        parser = URDFParser(model_path)

        try:
            parser.parse()
            self.fail("No exception occurred while loading the error XML.")
        except Exception as e:
            self.assertTrue(isinstance(e, RuntimeError))
            self.assertTrue("link: Link name 'link2' already exists (line: 22)" in e.args[0])

    def test_load_error_duplicate_joint_names(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_duplicate_joint_names.urdf")
        parser = URDFParser(model_path)

        try:
            parser.parse()
            self.fail("No exception occurred while loading the error XML.")
        except Exception as e:
            self.assertTrue(isinstance(e, RuntimeError))
            self.assertTrue("joint: Joint name 'JointA' already exists (line: 36)" in e.args[0])

    def test_load_error_invalid_material_name(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_invalid_material_name.urdf")
        parser = URDFParser(model_path)

        try:
            parser.parse()
            self.fail("No exception occurred while loading the error XML.")
        except Exception as e:
            self.assertTrue(isinstance(e, RuntimeError))
            self.assertTrue("Material name 'green' not found (line: 13)" in e.args[0])

    def test_load_error_missing_visual_geometry(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_missing_visual_geometry.urdf")
        parser = URDFParser(model_path)

        try:
            parser.parse()
            self.fail("No exception occurred while loading the error XML.")
        except Exception as e:
            self.assertTrue(isinstance(e, RuntimeError))
            self.assertTrue("Geometry must have one of the following: box, sphere, cylinder, or mesh (line: 6)" in e.args[0])

    def test_load_error_incorrect_visual_geometry_name(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_incorrect_visual_geometry_name.urdf")
        parser = URDFParser(model_path)

        try:
            parser.parse()
            self.fail("No exception occurred while loading the error XML.")
        except Exception as e:
            self.assertTrue(isinstance(e, RuntimeError))
            self.assertTrue("foo: Invalid geometry type (line: 7)" in e.args[0])

    def test_load_error_missing_collision_geometry(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_missing_collision_geometry.urdf")
        parser = URDFParser(model_path)

        try:
            parser.parse()
            self.fail("No exception occurred while loading the error XML.")
        except Exception as e:
            self.assertTrue(isinstance(e, RuntimeError))
            self.assertTrue("Geometry must have one of the following: box, sphere, cylinder, or mesh (line: 11)" in e.args[0])

    def test_load_error_incorrect_joint_child_link_name(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_incorrect_joint_child_link_name.urdf")
        parser = URDFParser(model_path)

        try:
            parser.parse()
            self.fail("No exception occurred while loading the error XML.")
        except Exception as e:
            self.assertTrue(isinstance(e, RuntimeError))
            self.assertTrue("joint: Child link 'link_dummy' not found (line: 24)" in e.args[0])

    def test_load_error_incorrect_joint_parent_link_name(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_incorrect_joint_parent_link_name.urdf")
        parser = URDFParser(model_path)

        try:
            parser.parse()
            self.fail("No exception occurred while loading the error XML.")
        except Exception as e:
            self.assertTrue(isinstance(e, RuntimeError))
            self.assertTrue("joint: Parent link 'BaseLink_dummy' not found (line: 23)" in e.args[0])

    def test_load_error_invalid_joint_type(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_incorrect_joint_type.urdf")
        parser = URDFParser(model_path)

        try:
            parser.parse()
            self.fail("No exception occurred while loading the error XML.")
        except Exception as e:
            self.assertTrue(isinstance(e, RuntimeError))
            self.assertTrue("joint: Invalid joint type: incorrect_type (line: 22)" in e.args[0])

    def test_get_basic_information(self):
        # Get basic information about a URDF.

        # Get the root element.
        root_element = self.parser.get_root_element()

        # Get the robot name ("robot" element's "name" attribute).
        root_robot_name = self.parser.get_robot_name()

        self.assertTrue(root_element)
        self.assertEqual(root_element.tag, "robot")
        self.assertEqual(root_element.name, "verifying_elements")
        self.assertEqual(root_robot_name, "verifying_elements")

    def test_get_materials(self):
        # Get the root element.
        root_element = self.parser.get_root_element()

        self.assertEqual(len(root_element.materials), 4)

        # Get the name from the materials list.
        materials = root_element.materials
        self.assertEqual(materials[0].name, "red")
        self.assertEqual(materials[1].name, "green")
        self.assertEqual(materials[2].name, "blue")
        self.assertEqual(materials[3].name, "texture")

        # Find materials by name.
        red_material = self.parser.find_material_by_name("red")
        self.assertTrue(red_material)
        self.assertEqual(red_material.name, "red")
        self.assertEqual(red_material.color.rgba, (1.0, 0.0, 0.0, 1.0))

        blue_material = self.parser.find_material_by_name("blue")
        self.assertTrue(blue_material)
        self.assertEqual(blue_material.name, "blue")
        self.assertEqual(blue_material.color.rgba, (0.0, 0.0, 1.0, 1.0))

        green_material = self.parser.find_material_by_name("green")
        self.assertTrue(green_material)
        self.assertEqual(green_material.name, "green")
        self.assertEqual(green_material.color.rgba, (0.0, 1.0, 0.0, 1.0))

        texture_material = self.parser.find_material_by_name("texture")
        self.assertTrue(texture_material)
        self.assertEqual(texture_material.name, "texture")
        self.assertIsNone(texture_material.color)
        self.assertEqual(texture_material.texture.filename, "assets/grid.png")

        # Get non-existent material.
        non_existent_material = self.parser.find_material_by_name("non_existent_material")
        self.assertIsNone(non_existent_material)

    def test_get_links(self):
        # Get the root element.
        root_element = self.parser.get_root_element()

        self.assertEqual(len(root_element.links), 3)

        # links[0]
        link = root_element.links[0]
        self.assertEqual(link.get_with_default("name"), "BaseLink")
        self.assertIsNone(link.type)

        visual = link.visual
        self.assertTrue(visual)
        origin = visual.origin
        self.assertTrue(origin)
        self.assertEqual(origin.get_with_default("xyz"), (-2.0, 0.0, 0.5))
        self.assertEqual(origin.get_with_default("rpy"), (0.0, 0.0, 0.0))
        geometry = visual.geometry
        self.assertTrue(geometry)
        self.assertEqual(geometry.geometry.tag, "box")
        self.assertEqual(geometry.geometry.get_with_default("size"), (1.0, 1.0, 1.0))
        material = visual.material
        self.assertTrue(material)
        self.assertEqual(material.get_with_default("name"), "red")

        # links[1]
        link = root_element.links[1]
        self.assertEqual(link.get_with_default("name"), "link2")
        self.assertIsNone(link.type)

        visual = link.visual
        self.assertTrue(visual)
        origin = visual.origin
        self.assertTrue(origin)
        self.assertEqual(origin.get_with_default("xyz"), (0.0, 0.0, 0.5))
        self.assertEqual(origin.rpy, (0.0, 0.0, 0.0))
        geometry = visual.geometry
        self.assertTrue(geometry)
        self.assertEqual(geometry.geometry.tag, "sphere")
        self.assertEqual(geometry.geometry.get_with_default("radius"), 0.5)
        material = visual.material
        self.assertTrue(material)
        self.assertEqual(material.get_with_default("name"), "green")

        # links[2]
        link = root_element.links[2]
        self.assertEqual(link.get_with_default("name"), "link3")
        self.assertIsNone(link.type)
        visual = link.visual
        self.assertTrue(visual)
        origin = visual.origin
        self.assertTrue(origin)
        self.assertEqual(origin.get_with_default("xyz"), (2.0, 0.0, 0.5))
        self.assertEqual(origin.get_with_default("rpy"), (0.0, 0.0, 0.0))
        geometry = visual.geometry
        self.assertTrue(geometry)
        self.assertEqual(geometry.geometry.tag, "cylinder")
        self.assertEqual(geometry.geometry.get_with_default("radius"), 0.5)
        self.assertEqual(geometry.geometry.get_with_default("length"), 1.0)
        material = visual.material
        self.assertTrue(material)
        self.assertEqual(material.get_with_default("name"), "blue")

        collision = link.collision
        self.assertTrue(collision)
        origin = collision.origin
        self.assertTrue(origin)
        self.assertEqual(origin.get_with_default("xyz"), (2.0, 0.0, 0.5))
        self.assertEqual(origin.get_with_default("rpy"), (0.0, 0.0, 0.0))
        geometry = collision.geometry
        self.assertTrue(geometry)
        self.assertEqual(geometry.geometry.tag, "cylinder")
        self.assertEqual(geometry.geometry.get_with_default("radius"), 0.5)
        self.assertEqual(geometry.geometry.get_with_default("length"), 1.0)
        verbose = collision.verbose
        self.assertTrue(verbose)
        self.assertEqual(verbose.get_with_default("value"), "verbose_data")

    def test_get_joints(self):
        # Get the root element.
        root_element = self.parser.get_root_element()

        self.assertEqual(len(root_element.joints), 2)

        # joints[0]
        joint = root_element.joints[0]
        self.assertEqual(joint.name, "JointA")
        self.assertEqual(joint.type, "fixed")
        self.assertTrue(joint.parent)
        self.assertEqual(joint.parent.link, "BaseLink")
        self.assertTrue(joint.child)
        self.assertEqual(joint.child.link, "link2")

        # joints[1]
        joint = root_element.joints[1]
        self.assertEqual(joint.name, "JointB")
        self.assertEqual(joint.type, "fixed")
        self.assertTrue(joint.parent)
        self.assertEqual(joint.parent.link, "link2")
        self.assertTrue(joint.child)
        self.assertEqual(joint.child.link, "link3")

    def test_get_transmissions(self):
        # Get the root element.
        root_element = self.parser.get_root_element()

        self.assertEqual(len(root_element.transmissions), 1)

        # transmissions[0]
        transmission = root_element.transmissions[0]
        self.assertEqual(transmission.get_with_default("name"), "simple_trans")
        self.assertTrue(transmission.type)
        self.assertEqual(transmission.type.text, "transmission_interface/SimpleTransmission")
        self.assertTrue(transmission.joint)
        self.assertEqual(transmission.joint.get_with_default("name"), "foo_joint")
        self.assertTrue(transmission.actuator)
        self.assertEqual(transmission.actuator.get_with_default("name"), "foo_motor")
        self.assertTrue(transmission.actuator.mechanicalReduction)
        self.assertEqual(transmission.actuator.mechanicalReduction.text, 50.0)
        self.assertTrue(transmission.actuator.hardwareInterface)
        self.assertEqual(transmission.actuator.hardwareInterface.text, "EffortJointInterface")

    def test_get_undefined_elements(self):
        # Get undefined elements and attributes in XML.

        # Get undefined elements.
        undefined_elements = self.parser.get_undefined_elements()
        self.assertEqual(len(undefined_elements), 4)

        element = undefined_elements[0]
        self.assertEqual(element.tag, "custom")
        self.assertEqual(element.path, "/robot/link/visual/custom")
        self.assertEqual(element.undefined_element, True)
        self.assertEqual(element.undefined_attributes, {})
        self.assertEqual(element.line_number, 21)

        element = undefined_elements[1]
        self.assertEqual(element.tag, "item1")
        self.assertEqual(element.path, "/robot/link/visual/custom/item1")
        self.assertEqual(element.undefined_element, True)
        self.assertEqual(element.undefined_attributes, {"name": "data1", "value": "1"})
        self.assertEqual(element.line_number, 22)

        element = undefined_elements[2]
        self.assertEqual(element.tag, "item2")
        self.assertEqual(element.path, "/robot/link/visual/custom/item2")
        self.assertEqual(element.undefined_element, True)
        self.assertEqual(element.undefined_attributes, {"name": "data2", "value": "2"})
        self.assertEqual(element.line_number, 23)

        # When adding custom attributes to an existing element.
        # In this case, element.undefined_element will be False.
        element = undefined_elements[3]
        self.assertEqual(element.tag, "visual")
        self.assertEqual(element.path, "/robot/link/visual")
        self.assertEqual(element.undefined_element, False)
        self.assertEqual(element.undefined_attributes, {"data": "custom_data"})
        self.assertEqual(element.line_number, 33)
