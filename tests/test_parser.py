# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
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

        with self.assertRaisesRegex(FileNotFoundError, ".*File not found:.*"):
            parser.parse()

    def test_load_error_xml_syntax(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_xml_syntax.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, ".*5:2: mismatched tag.*"):
            parser.parse()

    def test_load_error_no_urdf_xml(self):
        # Loading non-URDF XML.
        model_path = pathlib.Path("tests/data/error_no_urdf_xml.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*The root element must be 'robot' \(line: 2\).*"):
            parser.parse()

    def test_load_error_incorrect_vec3(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_incorrect_vec3.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*xyz: Invalid value: -2.2 0.0 0.5 1.0 \(line: 6\).*"):
            parser.parse()

    def test_load_error_incorrect_vec4(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_incorrect_vec4.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*rgba: Invalid value: 0.0 1.0 \(line: 5\).*"):
            parser.parse()

    def test_load_error_different_place(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_different_place.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(
            RuntimeError, r".*geometry: Invalid element type. This uses a reserved tag, but in the wrong place \(line: 8\).*"
        ):
            parser.parse()

    def test_load_error_no_material_name(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_no_material_name.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*material: name is required \(line: 4\).*"):
            parser.parse()

    def test_load_error_no_mesh_filename(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_no_mesh_filename.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*mesh: Filename is required \(line: 8\).*"):
            parser.parse()

    def test_load_error_duplicate_material_names(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_dupilcate_material_names.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*material: Material name 'green' already exists \(line: 8\).*"):
            parser.parse()

    def test_load_error_duplicate_link_names(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_duplicate_link_names.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*link: Link name 'link2' already exists \(line: 22\).*"):
            parser.parse()

    def test_load_error_duplicate_joint_names(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_duplicate_joint_names.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*joint: Joint name 'JointA' already exists \(line: 36\).*"):
            parser.parse()

    def test_load_error_invalid_material_name(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_invalid_material_name.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*Material name 'green' not found \(line: 13\).*"):
            parser.parse()

    def test_load_error_missing_visual_geometry(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_missing_visual_geometry.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*Geometry must have one of the following: box, sphere, cylinder, or mesh \(line: 6\).*"):
            parser.parse()

    def test_load_error_incorrect_visual_geometry_name(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_incorrect_visual_geometry_name.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*foo: Invalid geometry type \(line: 7\).*"):
            parser.parse()

    def test_load_error_missing_collision_geometry(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_missing_collision_geometry.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*Geometry must have one of the following: box, sphere, cylinder, or mesh \(line: 11\).*"):
            parser.parse()

    def test_load_error_incorrect_joint_child_link_name(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_incorrect_joint_child_link_name.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*Child link 'link_dummy' not found \(line: 24\).*"):
            parser.parse()

    def test_load_error_incorrect_joint_parent_link_name(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_incorrect_joint_parent_link_name.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*Parent link 'BaseLink_dummy' not found \(line: 23\).*"):
            parser.parse()

    def test_load_error_no_joint_type(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_no_joint_type.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*joint: Type is required \(line: 22\).*"):
            parser.parse()

    def test_load_error_no_joint_k_velocity(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_no_joint_k_velocity.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*safety_controller: k_velocity is required \(line: 26\).*"):
            parser.parse()

    def test_load_error_no_joint_mimic_joint(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_no_joint_mimic_joint.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*mimic: Joint is required \(line: 26\).*"):
            parser.parse()

    def test_load_error_invalid_joint_type(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_incorrect_joint_type.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*joint: Invalid joint type: incorrect_type \(line: 22\).*"):
            parser.parse()

    def test_load_error_no_joint_parent(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_no_joint_parent.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*Parent is required \(line: 20\).*"):
            parser.parse()

    def test_load_error_no_joint_child(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_no_joint_child.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*Child is required \(line: 20\).*"):
            parser.parse()

    def test_load_error_no_joint_parent_link(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_no_joint_parent_link.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*parent: Link is required \(line: 23\).*"):
            parser.parse()

    def test_load_error_duplicate_transmission_names(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_duplicate_transmission_names.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*transmission: Transmission name 'transmission_1' already exists \(line: 8\).*"):
            parser.parse()

    def test_has_no_material(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/simple_box_no_material.urdf")
        parser = URDFParser(model_path)

        materials = parser.get_materials()
        self.assertEqual(len(materials), 0)

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
        self.assertEqual(root_element.get_with_default("version"), "1.0")

    def test_find_materials(self):
        # Get the root element.
        root_element = self.parser.get_root_element()

        self.assertEqual(len(root_element.materials), 5)

        # Get the name from the materials list.
        materials = root_element.materials
        self.assertEqual(materials[0].name, "red")
        self.assertEqual(materials[1].name, "green")
        self.assertEqual(materials[2].name, "blue")
        self.assertEqual(materials[3].name, "black")
        self.assertEqual(materials[4].name, "texture")

        # Find materials by name.
        red_material = self.parser.find_material_by_name("red")
        self.assertTrue(red_material)
        self.assertEqual(red_material.name, "red")
        self.assertEqual(red_material.color.get_with_default("rgba"), (1.0, 0.0, 0.0, 1.0))

        blue_material = self.parser.find_material_by_name("blue")
        self.assertTrue(blue_material)
        self.assertEqual(blue_material.name, "blue")
        self.assertEqual(blue_material.color.get_with_default("rgba"), (0.0, 0.0, 1.0, 1.0))

        green_material = self.parser.find_material_by_name("green")
        self.assertTrue(green_material)
        self.assertEqual(green_material.name, "green")
        self.assertEqual(green_material.color.get_with_default("rgba"), (0.0, 1.0, 0.0, 1.0))

        texture_material = self.parser.find_material_by_name("texture")
        self.assertTrue(texture_material)
        self.assertEqual(texture_material.name, "texture")
        self.assertIsNone(texture_material.color)
        self.assertEqual(texture_material.texture.get_with_default("filename"), "assets/grid.png")

        black_material = self.parser.find_material_by_name("black")
        self.assertTrue(black_material)
        self.assertEqual(black_material.name, "black")
        self.assertEqual(black_material.color.get_with_default("rgba"), (0.0, 0.0, 0.0, 0.0))

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

        inertial = link.inertial
        self.assertTrue(inertial)
        self.assertTrue(inertial.origin)
        self.assertEqual(inertial.origin.get_with_default("xyz"), (0.0, 0.0, 0.3))
        self.assertEqual(inertial.origin.get_with_default("rpy"), (0.0, 0.0, 0.0))
        self.assertTrue(inertial.mass)
        self.assertEqual(inertial.mass.get_with_default("value"), 1.0)
        self.assertTrue(inertial.inertia)
        self.assertEqual(inertial.inertia.get_with_default("ixx"), 0.1)
        self.assertEqual(inertial.inertia.get_with_default("iyy"), 0.2)
        self.assertEqual(inertial.inertia.get_with_default("izz"), 0.3)
        self.assertEqual(inertial.inertia.get_with_default("ixy"), 0.0)
        self.assertEqual(inertial.inertia.get_with_default("ixz"), 0.0)
        self.assertEqual(inertial.inertia.get_with_default("iyz"), 0.0)

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
        self.assertEqual(geometry.geometry.tag, "mesh")
        mesh = geometry.geometry
        self.assertEqual(mesh.get_with_default("filename"), "assets/box.obj")
        self.assertEqual(mesh.get_with_default("scale"), (0.5, 0.6, 1.0))
        material = visual.material
        self.assertTrue(material)
        self.assertEqual(material.get_with_default("name"), "green")

        collision = link.collision
        self.assertTrue(collision)
        origin = collision.origin
        self.assertTrue(origin)
        self.assertEqual(origin.get_with_default("xyz"), (0.0, 0.0, 0.5))
        self.assertEqual(origin.rpy, (0.0, 0.0, 0.0))
        geometry = collision.geometry
        self.assertTrue(geometry)
        self.assertEqual(geometry.geometry.tag, "mesh")
        mesh = geometry.geometry
        self.assertEqual(mesh.get_with_default("filename"), "assets/box.stl")
        self.assertEqual(mesh.get_with_default("scale"), (0.5, 0.6, 1.0))

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
        self.assertEqual(material.get_with_default("name"), "yellow")

        collision = link.collision
        self.assertTrue(collision)
        origin = collision.origin
        self.assertTrue(origin)
        self.assertEqual(origin.get_with_default("xyz"), (2.0, 0.0, 0.5))
        self.assertEqual(origin.get_with_default("rpy"), (0.0, 0.0, 0.0))
        geometry = collision.geometry
        self.assertTrue(geometry)
        self.assertEqual(geometry.geometry.tag, "mesh")
        mesh = geometry.geometry
        self.assertEqual(mesh.get_with_default("filename"), "assets/box.stl")
        self.assertEqual(mesh.get_with_default("scale"), (0.5, 0.6, 1.0))
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
        self.assertTrue(joint.origin)
        self.assertEqual(joint.origin.get_with_default("rpy"), (0.02, 0.0, 0.0))
        self.assertEqual(joint.origin.get_with_default("xyz"), (0.0, 0.0, 0.01))
        self.assertTrue(joint.axis)
        self.assertEqual(joint.axis.get_with_default("xyz"), (0.0, 1.0, 0.0))
        self.assertTrue(joint.calibration)
        self.assertEqual(joint.calibration.get_with_default("rising"), 0.3)
        self.assertEqual(joint.calibration.get_with_default("falling"), 0.2)
        self.assertEqual(joint.calibration.get_with_default("reference_position"), 0.1)
        self.assertTrue(joint.dynamics)
        self.assertEqual(joint.dynamics.get_with_default("damping"), 0.0)
        self.assertEqual(joint.dynamics.get_with_default("friction"), 0.0)
        self.assertTrue(joint.limit)
        self.assertEqual(joint.limit.get_with_default("effort"), 30.0)
        self.assertEqual(joint.limit.get_with_default("velocity"), 1.0)
        self.assertEqual(joint.limit.get_with_default("lower"), -2.2)
        self.assertEqual(joint.limit.get_with_default("upper"), 0.7)
        self.assertTrue(joint.safety_controller)
        self.assertEqual(joint.safety_controller.get_with_default("k_velocity"), 10.0)
        self.assertEqual(joint.safety_controller.get_with_default("k_position"), 15.0)
        self.assertEqual(joint.safety_controller.get_with_default("soft_lower_limit"), -2.0)
        self.assertEqual(joint.safety_controller.get_with_default("soft_upper_limit"), 0.5)
        self.assertTrue(joint.mimic)
        self.assertEqual(joint.mimic.get_with_default("joint"), "JointA")
        self.assertEqual(joint.mimic.get_with_default("multiplier"), 2.0)
        self.assertEqual(joint.mimic.get_with_default("offset"), 1.0)

    def test_get_meshes(self):
        meshes = self.parser.get_meshes()
        self.assertEqual(len(meshes), 2)

        mesh = meshes[0]
        self.assertEqual(mesh[0], "assets/box.obj")
        self.assertEqual(mesh[1], (0.5, 0.6, 1.0))

        mesh = meshes[1]
        self.assertEqual(mesh[0], "assets/box.stl")
        self.assertEqual(mesh[1], (0.5, 0.6, 1.0))

    def test_get_materials(self):
        materials = self.parser.get_materials()

        self.assertEqual(len(materials), 6)

        material = materials[0]
        self.assertEqual(material[0], "red")
        self.assertEqual(material[1], (1.0, 0.0, 0.0, 1.0))
        self.assertEqual(material[2], None)

        material = materials[1]
        self.assertEqual(material[0], "green")
        self.assertEqual(material[1], (0.0, 1.0, 0.0, 1.0))
        self.assertEqual(material[2], None)

        material = materials[2]
        self.assertEqual(material[0], "blue")
        self.assertEqual(material[1], (0.0, 0.0, 1.0, 1.0))
        self.assertEqual(material[2], None)

        material = materials[3]
        self.assertEqual(material[0], "black")
        self.assertEqual(material[1], (0.0, 0.0, 0.0, 0.0))
        self.assertEqual(material[2], None)

        material = materials[4]
        self.assertEqual(material[0], "texture")
        self.assertEqual(material[1], (0.0, 0.0, 0.0, 0.0))
        self.assertEqual(material[2], "assets/grid.png")

        material = materials[5]
        self.assertEqual(material[0], "yellow")
        self.assertEqual(material[1], (1.0, 1.0, 0.0, 1.0))
        self.assertEqual(material[2], None)

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
        self.assertEqual(len(undefined_elements), 7)

        element = undefined_elements[0]
        self.assertEqual(element.tag, "gazebo")
        self.assertEqual(element.path, "/robot/gazebo")
        self.assertEqual(element.undefined_element, True)
        self.assertEqual(element.line_number, 102)

        element = undefined_elements[1]
        self.assertEqual(element.tag, "static")
        self.assertEqual(element.path, "/robot/gazebo/static")
        self.assertEqual(element.undefined_element, True)
        self.assertEqual(element.undefined_text, "true")
        self.assertEqual(element.line_number, 103)

        element = undefined_elements[2]
        self.assertEqual(element.tag, "custom")
        self.assertEqual(element.path, "/robot/link/visual/custom")
        self.assertEqual(element.undefined_element, True)
        self.assertEqual(element.undefined_attributes, {})
        self.assertEqual(element.line_number, 29)

        element = undefined_elements[3]
        self.assertEqual(element.tag, "item1")
        self.assertEqual(element.path, "/robot/link/visual/custom/item1")
        self.assertEqual(element.undefined_element, True)
        self.assertEqual(element.undefined_attributes, {"name": "data1", "value": "1"})
        self.assertEqual(element.line_number, 30)

        element = undefined_elements[4]
        self.assertEqual(element.tag, "item2")
        self.assertEqual(element.path, "/robot/link/visual/custom/item2")
        self.assertEqual(element.undefined_element, True)
        self.assertEqual(element.undefined_attributes, {"name": "data2", "value": "2"})
        self.assertEqual(element.line_number, 31)

        element = undefined_elements[5]
        self.assertEqual(element.tag, "data")
        self.assertEqual(element.path, "/robot/link/visual/custom/data")
        self.assertEqual(element.undefined_element, True)
        self.assertEqual(element.undefined_attributes, {"attr": "attr foo"})
        self.assertEqual(element.undefined_text, "custom text")
        self.assertEqual(element.line_number, 32)

        # When adding custom attributes to an existing element.
        # In this case, element.undefined_element will be False.
        element = undefined_elements[6]
        self.assertEqual(element.tag, "visual")
        self.assertEqual(element.path, "/robot/link/visual")
        self.assertEqual(element.undefined_element, False)
        self.assertEqual(element.undefined_attributes, {"data": "custom_data"})
        self.assertEqual(element.line_number, 42)
