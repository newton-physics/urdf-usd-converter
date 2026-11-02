# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import math
import pathlib

import usdex.core
from pxr import Tf

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

        with self.assertRaisesRegex(RuntimeError, r".*xyz: Invalid value: Parser found 4 elements.*\[-2.2 0.0 0.5 1.0\] \(line: 6\).*"):
            parser.parse()

    def test_load_error_incorrect_vec4(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_incorrect_vec4.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*rgba: Invalid value: Parser found 2 elements.*\[0.0 1.0\] \(line: 5\).*"):
            parser.parse()

    def test_load_error_no_material_name(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_no_material_name.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*material: name is required \(line: 4\).*"):
            parser.parse()

    def test_load_warning_no_mesh_filename(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/warning_no_mesh_filename.urdf")
        parser = URDFParser(model_path)

        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*mesh: Filename is required.*"),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
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

    def test_load_warning_invalid_material_name(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/warning_invalid_material_name.urdf")
        parser = URDFParser(model_path)

        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Material name 'green' not found.*"),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            parser.parse()

    def test_load_warning_missing_visual_geometry(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/warning_missing_visual_geometry.urdf")
        parser = URDFParser(model_path)

        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Geometry must have one of the following: box, sphere, cylinder, capsule, or mesh.*"),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            parser.parse()

    def test_load_warning_incorrect_visual_geometry_name(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/warning_incorrect_visual_geometry_name.urdf")
        parser = URDFParser(model_path)

        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*foo: Invalid geometry type.*"),
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*material: link: Material name 'green' not found.*"),
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Geometry must have one of the following: box, sphere, cylinder, capsule, or mesh.*"),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            parser.parse()

    def test_load_warning_missing_collision_geometry(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/warning_missing_collision_geometry.urdf")
        parser = URDFParser(model_path)

        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Geometry must have one of the following: box, sphere, cylinder, capsule, or mesh.*"),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
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

    def test_limit_velocity_default_is_unset(self):
        parser = URDFParser(pathlib.Path("tests/data/velocity_limits.urdf"))
        parser.parse()
        joints = {joint.name: joint for joint in parser.get_root_element().joints}

        self.assertEqual(joints["positive"].limit.velocity, 0.5)
        self.assertEqual(joints["positive"].limit.get_with_default("velocity"), 0.5)

        self.assertIsNone(joints["omitted"].limit.velocity)
        self.assertIsNone(joints["omitted"].limit.get_with_default("velocity"))

        self.assertEqual(joints["zero"].limit.velocity, 0.0)
        self.assertEqual(joints["negative"].limit.velocity, -1.0)

    def test_load_warning_no_joint_k_velocity(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/warning_no_joint_k_velocity.urdf")
        parser = URDFParser(model_path)

        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*safety_controller: k_velocity is required.*"),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            parser.parse()

    def test_load_error_no_joint_mimic_joint(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_no_joint_mimic_joint.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*mimic: Joint is required \(line: 26\).*"):
            parser.parse()

    def test_load_error_incorrect_mimic_joint(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_incorrect_mimic_joint.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*mimic: Mimic joint 'foo' not found \(line: 26\).*"):
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

    def test_load_error_joint_axis_0(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_joint_axis_0.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*axis: Axis xyz cannot be \(0, 0, 0\) \(line: 25\).*"):
            parser.parse()

    def test_load_warning_different_place(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/warning_different_place.urdf")
        parser = URDFParser(model_path)

        with usdex.test.ScopedDiagnosticChecker(
            self,
            [(Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*geometry: Invalid element type. This uses a reserved tag, but in the wrong place.*")],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
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
        self.assertEqual(root_element.get_with_default("version"), "1.1")

    def test_find_materials(self):
        # Get the root element.
        root_element = self.parser.get_root_element()

        self.assertEqual(len(root_element.materials), 4)

        # Get the name from the materials list.
        materials = root_element.materials
        self.assertEqual(materials[0].name, "red")
        self.assertEqual(materials[1].name, "blue")
        self.assertEqual(materials[2].name, "default")
        self.assertEqual(materials[3].name, "texture")

        # Find materials by name.
        red_material = self.parser.find_material_by_name("red")
        self.assertTrue(red_material)
        self.assertEqual(red_material.name, "red")
        self.assertEqual(red_material.color.get_with_default("rgba"), (1.0, 0.0, 0.0, 1.0))

        blue_material = self.parser.find_material_by_name("blue")
        self.assertTrue(blue_material)
        self.assertEqual(blue_material.name, "blue")
        self.assertEqual(blue_material.color.get_with_default("rgba"), (0.0, 0.0, 1.0, 1.0))

        texture_material = self.parser.find_material_by_name("texture")
        self.assertTrue(texture_material)
        self.assertEqual(texture_material.name, "texture")
        self.assertIsNone(texture_material.color)
        self.assertEqual(texture_material.texture.get_with_default("filename"), "assets/grid.png")

        default_material = self.parser.find_material_by_name("default")
        self.assertTrue(default_material)
        self.assertEqual(default_material.name, "default")
        self.assertEqual(default_material.color.get_with_default("rgba"), (1.0, 1.0, 1.0, 1.0))

        # Get non-existent material.
        non_existent_material = self.parser.find_material_by_name("non_existent_material")
        self.assertIsNone(non_existent_material)

    def test_get_links(self):
        # Get the root element.
        root_element = self.parser.get_root_element()

        self.assertEqual(len(root_element.links), 4)

        # links[0]
        link = root_element.links[0]
        self.assertEqual(link.get_with_default("name"), "BaseLink")
        self.assertIsNone(link.type)

        self.assertEqual(len(link.visuals), 1)
        visual = link.visuals[0]
        self.assertTrue(visual)
        origin = visual.origin
        self.assertTrue(origin)
        self.assertEqual(origin.get_with_default("xyz"), (-2.0, 0.0, 0.5))
        self.assertEqual(origin.get_with_default("rpy"), (0.0, 0.0, 0.0))
        geometry = visual.geometry
        self.assertTrue(geometry)
        self.assertEqual(geometry.shape.tag, "box")
        self.assertEqual(geometry.shape.get_with_default("size"), (1.0, 1.0, 1.0))
        material = visual.material
        self.assertTrue(material)
        self.assertEqual(material.get_with_default("name"), "red")

        inertial = link.inertial
        self.assertTrue(inertial)
        self.assertTrue(inertial.origin)
        self.assertEqual(inertial.origin.get_with_default("xyz"), (0.0, 0.0, 0.3))
        self.assertEqual(inertial.origin.get_with_default("rpy"), (0.0, 0.0, 0.0))
        quat_xyzw = inertial.origin.get_with_default("quat_xyzw")
        self.assertAlmostEqual(quat_xyzw[0], 0.0)
        self.assertAlmostEqual(quat_xyzw[1], 0.70710678)
        self.assertAlmostEqual(quat_xyzw[2], 0.0)
        self.assertAlmostEqual(quat_xyzw[3], 0.70710678)
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

        self.assertEqual(len(link.visuals), 1)
        visual = link.visuals[0]
        self.assertTrue(visual)
        origin = visual.origin
        self.assertTrue(origin)
        self.assertEqual(origin.get_with_default("xyz"), (0.0, 0.0, 0.5))
        self.assertEqual(origin.rpy, (0.0, 0.0, 0.0))
        geometry = visual.geometry
        self.assertTrue(geometry)
        self.assertEqual(geometry.shape.tag, "mesh")
        mesh = geometry.shape
        self.assertEqual(mesh.get_with_default("filename"), "assets/box.obj")
        self.assertEqual(mesh.get_with_default("scale"), (0.5, 0.6, 1.0))
        material = visual.material
        self.assertTrue(material)
        self.assertEqual(material.get_with_default("name"), None)  # Unnamed material
        self.assertEqual(material.get_with_default("unique_name"), "material_1")

        self.assertEqual(len(link.collisions), 1)
        collision = link.collisions[0]
        self.assertTrue(collision)
        origin = collision.origin
        self.assertTrue(origin)
        self.assertEqual(origin.get_with_default("xyz"), (0.0, 0.0, 0.5))
        self.assertEqual(origin.rpy, (0.0, 0.0, 0.0))
        geometry = collision.geometry
        self.assertTrue(geometry)
        self.assertEqual(geometry.shape.tag, "mesh")
        mesh = geometry.shape
        self.assertEqual(mesh.get_with_default("filename"), "assets/box.stl")
        self.assertEqual(mesh.get_with_default("scale"), (0.5, 0.6, 1.0))

        # links[2]
        link = root_element.links[2]
        self.assertEqual(link.get_with_default("name"), "link3")
        self.assertIsNone(link.type)
        self.assertEqual(len(link.visuals), 1)
        visual = link.visuals[0]
        self.assertTrue(visual)
        origin = visual.origin
        self.assertTrue(origin)
        self.assertEqual(origin.get_with_default("xyz"), (2.0, 0.0, 0.5))
        self.assertEqual(origin.get_with_default("rpy"), (0.0, 0.0, 0.0))
        geometry = visual.geometry
        self.assertTrue(geometry)
        self.assertEqual(geometry.shape.tag, "cylinder")
        self.assertEqual(geometry.shape.get_with_default("radius"), 0.5)
        self.assertEqual(geometry.shape.get_with_default("length"), 1.0)
        material = visual.material
        self.assertTrue(material)
        self.assertEqual(material.get_with_default("name"), "yellow")

        self.assertEqual(len(link.collisions), 1)
        collision = link.collisions[0]
        self.assertTrue(collision)
        origin = collision.origin
        self.assertTrue(origin)
        self.assertEqual(origin.get_with_default("xyz"), (2.0, 0.0, 0.5))
        self.assertEqual(origin.get_with_default("rpy"), (0.0, 0.0, 0.0))
        geometry = collision.geometry
        self.assertTrue(geometry)
        self.assertEqual(geometry.shape.tag, "mesh")
        mesh = geometry.shape
        self.assertEqual(mesh.get_with_default("filename"), "assets/box.stl")
        self.assertEqual(mesh.get_with_default("scale"), (0.5, 0.6, 1.0))
        verbose = collision.verbose
        self.assertTrue(verbose)
        self.assertEqual(verbose.get_with_default("value"), "verbose_data")

        # links[3]
        link = root_element.links[3]
        self.assertEqual(link.get_with_default("name"), "link4")
        self.assertIsNone(link.type)
        self.assertEqual(len(link.visuals), 1)
        visual = link.visuals[0]
        self.assertTrue(visual)
        geometry = visual.geometry
        self.assertTrue(geometry)
        self.assertEqual(geometry.shape.tag, "capsule")
        self.assertEqual(geometry.shape.get_with_default("radius"), 0.5)
        self.assertEqual(geometry.shape.get_with_default("length"), 1.0)
        material = visual.material
        self.assertTrue(material)
        self.assertEqual(material.get_with_default("name"), None)  # Unnamed material
        self.assertEqual(material.get_with_default("unique_name"), "material_2")

    def test_get_joints(self):
        # Get the root element.
        root_element = self.parser.get_root_element()

        self.assertEqual(len(root_element.joints), 3)

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

        # joints[2]
        joint = root_element.joints[2]
        self.assertEqual(joint.name, "JointC")
        self.assertEqual(joint.type, "fixed")
        self.assertTrue(joint.parent)
        self.assertEqual(joint.parent.link, "link3")
        self.assertTrue(joint.child)
        self.assertEqual(joint.child.link, "link4")

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

        self.assertEqual(len(materials), 7)

        material = materials[0]
        self.assertEqual(material["name"], "red")
        self.assertEqual(material["unique_name"], "red")
        self.assertEqual(material["color"], (1.0, 0.0, 0.0, 1.0))
        self.assertEqual(material["file_path"], None)

        material = materials[1]
        self.assertEqual(material["name"], "blue")
        self.assertEqual(material["unique_name"], "blue")
        self.assertEqual(material["color"], (0.0, 0.0, 1.0, 1.0))
        self.assertEqual(material["file_path"], None)

        material = materials[2]
        self.assertEqual(material["name"], "default")
        self.assertEqual(material["unique_name"], "default")
        self.assertEqual(material["color"], (1.0, 1.0, 1.0, 1.0))
        self.assertEqual(material["file_path"], None)

        material = materials[3]
        self.assertEqual(material["name"], "texture")
        self.assertEqual(material["unique_name"], "texture")
        self.assertEqual(material["color"], (1.0, 1.0, 1.0, 1.0))
        self.assertEqual(material["file_path"], "assets/grid.png")

        material = materials[4]
        self.assertEqual(material["name"], "")
        self.assertEqual(material["unique_name"], "material_1")
        self.assertEqual(material["color"], (0.0, 1.0, 0.0, 1.0))
        self.assertEqual(material["file_path"], None)

        material = materials[5]
        self.assertEqual(material["name"], "yellow")
        self.assertEqual(material["unique_name"], "yellow")
        self.assertEqual(material["color"], (1.0, 1.0, 0.0, 1.0))
        self.assertEqual(material["file_path"], None)

        material = materials[6]
        self.assertEqual(material["name"], "")
        self.assertEqual(material["unique_name"], "material_2")
        self.assertEqual(material["color"], (0.0, 1.0, 1.0, 1.0))
        self.assertEqual(material["file_path"], None)

    def test_get_undefined_elements(self):
        # Get undefined elements and attributes in XML.

        # Get undefined elements.
        undefined_elements = self.parser.get_undefined_elements()
        self.assertEqual(len(undefined_elements), 14)

        # The transmission is stored as undefined because the schema specification is unstable.
        element = undefined_elements[0]
        self.assertEqual(element.tag, "transmission")
        self.assertEqual(element.path, "/robot/transmission")
        self.assertEqual(element.undefined_element, True)
        self.assertEqual(element.undefined_attributes, {"name": "simple_trans"})
        self.assertEqual(element.line_number, 108)

        element = undefined_elements[1]
        self.assertEqual(element.tag, "type")
        self.assertEqual(element.path, "/robot/transmission/type")
        self.assertEqual(element.undefined_element, True)
        self.assertEqual(element.undefined_attributes, {})
        self.assertEqual(element.undefined_text, "transmission_interface/SimpleTransmission")
        self.assertEqual(element.line_number, 109)

        element = undefined_elements[2]
        self.assertEqual(element.tag, "joint")
        self.assertEqual(element.path, "/robot/transmission/joint")
        self.assertEqual(element.undefined_element, True)
        self.assertEqual(element.undefined_attributes, {"name": "foo_joint"})
        self.assertEqual(element.line_number, 110)

        element = undefined_elements[3]
        self.assertEqual(element.tag, "hardwareInterface")
        self.assertEqual(element.path, "/robot/transmission/joint/hardwareInterface")
        self.assertEqual(element.undefined_element, True)
        self.assertEqual(element.undefined_attributes, {})
        self.assertEqual(element.undefined_text, "EffortJointInterface")
        self.assertEqual(element.line_number, 111)

        element = undefined_elements[4]
        self.assertEqual(element.tag, "actuator")
        self.assertEqual(element.path, "/robot/transmission/actuator")
        self.assertEqual(element.undefined_element, True)
        self.assertEqual(element.undefined_attributes, {"name": "foo_motor"})
        self.assertEqual(element.line_number, 113)

        element = undefined_elements[5]
        self.assertEqual(element.tag, "mechanicalReduction")
        self.assertEqual(element.path, "/robot/transmission/actuator/mechanicalReduction")
        self.assertEqual(element.undefined_element, True)
        self.assertEqual(element.undefined_attributes, {})
        self.assertEqual(element.undefined_text, "50")
        self.assertEqual(element.line_number, 114)

        element = undefined_elements[6]
        self.assertEqual(element.tag, "hardwareInterface")
        self.assertEqual(element.path, "/robot/transmission/actuator/hardwareInterface")
        self.assertEqual(element.undefined_element, True)
        self.assertEqual(element.undefined_attributes, {})
        self.assertEqual(element.undefined_text, "EffortJointInterface")
        self.assertEqual(element.line_number, 115)

        element = undefined_elements[7]
        self.assertEqual(element.tag, "gazebo")
        self.assertEqual(element.path, "/robot/gazebo")
        self.assertEqual(element.undefined_element, True)
        self.assertEqual(element.line_number, 119)

        element = undefined_elements[8]
        self.assertEqual(element.tag, "static")
        self.assertEqual(element.path, "/robot/gazebo/static")
        self.assertEqual(element.undefined_element, True)
        self.assertEqual(element.undefined_text, "true")
        self.assertEqual(element.line_number, 120)

        element = undefined_elements[9]
        self.assertEqual(element.tag, "custom")
        self.assertEqual(element.path, "/robot/link/visual/custom")
        self.assertEqual(element.undefined_element, True)
        self.assertEqual(element.undefined_attributes, {})
        self.assertEqual(element.line_number, 26)

        element = undefined_elements[10]
        self.assertEqual(element.tag, "item1")
        self.assertEqual(element.path, "/robot/link/visual/custom/item1")
        self.assertEqual(element.undefined_element, True)
        self.assertEqual(element.undefined_attributes, {"name": "data1", "value": "1"})
        self.assertEqual(element.line_number, 27)

        element = undefined_elements[11]
        self.assertEqual(element.tag, "item2")
        self.assertEqual(element.path, "/robot/link/visual/custom/item2")
        self.assertEqual(element.undefined_element, True)
        self.assertEqual(element.undefined_attributes, {"name": "data2", "value": "2"})
        self.assertEqual(element.line_number, 28)

        element = undefined_elements[12]
        self.assertEqual(element.tag, "data")
        self.assertEqual(element.path, "/robot/link/visual/custom/data")
        self.assertEqual(element.undefined_element, True)
        self.assertEqual(element.undefined_attributes, {"attr": "attr foo"})
        self.assertEqual(element.undefined_text, "custom text")
        self.assertEqual(element.line_number, 29)

        # When adding custom attributes to an existing element.
        # In this case, element.undefined_element will be False.
        element = undefined_elements[13]
        self.assertEqual(element.tag, "visual")
        self.assertEqual(element.path, "/robot/link/visual")
        self.assertEqual(element.undefined_element, False)
        self.assertEqual(element.undefined_attributes, {"data": "custom_data"})
        self.assertEqual(element.line_number, 39)

    def test_fixed_joint_axis_0(self):
        # In the case of a fixed joint, the axis (0, 0, 0) is skipped without causing an error.
        model_path = pathlib.Path("tests/data/fixed_joint_axis_0.urdf")
        parser = URDFParser(model_path)
        parser.parse()

        root_element = parser.get_root_element()
        self.assertEqual(len(root_element.joints), 1)
        joint = root_element.joints[0]
        self.assertEqual(joint.name, "JointA")
        self.assertEqual(joint.type, "fixed")
        self.assertTrue(joint.parent)
        self.assertEqual(joint.parent.link, "BaseLink")
        self.assertTrue(joint.child)
        self.assertEqual(joint.child.link, "link2")

    def test_load_warning_urdf_11_capsule_quatxyz(self):
        model_path = pathlib.Path("tests/data/warning_urdf_11_capsule_quatxyz.urdf")
        parser = URDFParser(model_path)

        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*capsule and quat_xyzw are unavailable when the URDF version is earlier than 1.1.*"),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            parser.parse()

    def test_load_error_origin_rpy_and_quat_xyzw(self):
        model_path = pathlib.Path("tests/data/error_origin_rpy_and_quat_xyzw.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(
            RuntimeError,
            r".*origin: Both rpy and quat_xyzw orientations are defined. Use either one or the other \(line: 7\).*",
        ):
            parser.parse()

    def test_load_origin_rpy_and_quat_xyzw_urdf_10(self):
        model_path = pathlib.Path("tests/data/warning_urdf_10_rpy_and_quat_xyzw.urdf")
        parser = URDFParser(model_path)

        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*capsule and quat_xyzw are unavailable when the URDF version is earlier than 1.1.*"),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            parser.parse()

    def test_load_warning_urdf_10_inertial_quat_xyzw(self):
        model_path = pathlib.Path("tests/data/warning_urdf_10_inertial_quat_xyzw.urdf")
        parser = URDFParser(model_path)

        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*capsule and quat_xyzw are unavailable when the URDF version is earlier than 1.1.*"),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            parser.parse()

    def test_load_urdf_11_quat_xyzw_normalize(self):
        model_path = pathlib.Path("tests/data/urdf_11_quat_xyzw_normalize.urdf")
        parser = URDFParser(model_path)
        parser.parse()

        root_element = parser.get_root_element()
        visual_origin = root_element.links[0].visuals[0].origin
        self.assertEqual(visual_origin.quat_xyzw, (0.0, 0.0, 0.0, 1.0))
        inertial_origin = root_element.links[0].inertial.origin
        self.assertEqual(inertial_origin.quat_xyzw, (0.0, 0.0, 0.0, 1.0))

    def test_load_error_capsule_no_radius(self):
        model_path = pathlib.Path("tests/data/error_capsule_no_radius.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*capsule: Radius is required \(line: 7\).*"):
            parser.parse()

    def test_load_error_capsule_no_length(self):
        model_path = pathlib.Path("tests/data/error_capsule_no_length.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*capsule: Length is required \(line: 7\).*"):
            parser.parse()

    def test_load_error_capsule_negative_radius(self):
        model_path = pathlib.Path("tests/data/error_capsule_negative_radius.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*capsule: Radius must be a non-negative finite value \(line: 7\).*"):
            parser.parse()

    def test_load_error_capsule_non_finite_length(self):
        model_path = pathlib.Path("tests/data/error_capsule_non_finite_length.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*capsule: Length must be a non-negative finite value \(line: 7\).*"):
            parser.parse()

    def test_load_urdf_12_joint_limits(self):
        model_path = pathlib.Path("tests/data/urdf_12_joint_limits.urdf")
        parser = URDFParser(model_path)
        parser.parse()

        root_element = parser.get_root_element()
        self.assertEqual(root_element.get_with_default("version"), "1.2")
        self.assertEqual(len(root_element.joints), 3)

        revolute = root_element.joints[0]
        self.assertEqual(revolute.name, "revolute_joint")
        self.assertEqual(revolute.limit.lower, -1.57)
        self.assertEqual(revolute.limit.upper, 1.57)
        self.assertEqual(revolute.limit.effort, 100.0)
        self.assertEqual(revolute.limit.velocity, 1.0)
        self.assertEqual(revolute.limit.acceleration, 10.0)
        self.assertEqual(revolute.limit.deceleration, 5.0)
        self.assertEqual(revolute.limit.jerk, 200.0)

        prismatic = root_element.joints[1]
        self.assertEqual(prismatic.name, "prismatic_joint")
        self.assertEqual(prismatic.limit.acceleration, 2.0)
        self.assertEqual(prismatic.limit.deceleration, 1.5)
        self.assertEqual(prismatic.limit.jerk, 20.0)

        continuous = root_element.joints[2]
        self.assertEqual(continuous.name, "continuous_joint")
        self.assertEqual(continuous.limit.acceleration, 3.0)
        self.assertIsNone(continuous.limit.deceleration)
        self.assertIsNone(continuous.limit.effort)
        self.assertIsNone(continuous.limit.velocity)
        self.assertIsNone(continuous.limit.jerk)
        self.assertIsNone(continuous.limit.lower)
        self.assertIsNone(continuous.limit.upper)
        self.assertEqual(parser.get_limit_with_default(continuous.limit, "deceleration"), 3.0)
        self.assertEqual(parser.get_limit_with_default(continuous.limit, "acceleration"), 3.0)
        self.assertEqual(parser.get_limit_with_default(continuous.limit, "effort"), math.inf)
        self.assertEqual(parser.get_limit_with_default(continuous.limit, "velocity"), math.inf)
        self.assertEqual(parser.get_limit_with_default(continuous.limit, "jerk"), math.inf)
        self.assertEqual(parser.get_limit_with_default(continuous.limit, "lower"), -math.inf)
        self.assertEqual(parser.get_limit_with_default(continuous.limit, "upper"), math.inf)

    def test_load_warning_urdf_12_joint_limits(self):
        model_path = pathlib.Path("tests/data/warning_urdf_12_joint_limits.urdf")
        parser = URDFParser(model_path)

        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*acceleration, deceleration, and jerk are unavailable when the URDF version is earlier than 1.2.*"),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            parser.parse()

    def test_load_error_urdf_12_revolute_no_lower(self):
        model_path = pathlib.Path("tests/data/error_urdf_12_revolute_no_lower.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*limit: Lower is required for revolute and prismatic joints \(line: 10\).*"):
            parser.parse()

    def test_load_error_urdf_12_prismatic_no_upper(self):
        model_path = pathlib.Path("tests/data/error_urdf_12_prismatic_no_upper.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*limit: Upper is required for revolute and prismatic joints \(line: 10\).*"):
            parser.parse()

    def test_load_error_urdf_12_negative_effort(self):
        model_path = pathlib.Path("tests/data/error_urdf_12_negative_effort.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*limit: Effort must be non-negative \(line: 10\).*"):
            parser.parse()

    def test_load_error_urdf_12_negative_velocity(self):
        model_path = pathlib.Path("tests/data/error_urdf_12_negative_velocity.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*limit: Velocity must be non-negative \(line: 10\).*"):
            parser.parse()

    def test_load_error_urdf_12_negative_acceleration(self):
        model_path = pathlib.Path("tests/data/error_urdf_12_negative_acceleration.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*limit: Acceleration must be non-negative \(line: 10\).*"):
            parser.parse()

    def test_load_error_urdf_12_negative_deceleration(self):
        model_path = pathlib.Path("tests/data/error_urdf_12_negative_deceleration.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*limit: Deceleration must be non-negative \(line: 10\).*"):
            parser.parse()

    def test_load_error_urdf_12_negative_jerk(self):
        model_path = pathlib.Path("tests/data/error_urdf_12_negative_jerk.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*limit: Jerk must be non-negative \(line: 10\).*"):
            parser.parse()

    def test_load_error_urdf_12_upper_less_than_lower(self):
        model_path = pathlib.Path("tests/data/error_urdf_12_upper_less_than_lower.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r".*limit: Upper must be greater than or equal to lower \(line: 10\).*"):
            parser.parse()

    def test_load_urdf_11_revolute_optional_limits(self):
        model_path = pathlib.Path("tests/data/urdf_11_revolute_optional_limits.urdf")
        parser = URDFParser(model_path)
        parser.parse()

        joint = parser.get_root_element().joints[0]
        self.assertIsNone(joint.limit.lower)
        self.assertEqual(joint.limit.upper, 1.57)
        self.assertEqual(parser.get_limit_with_default(joint.limit, "lower"), 0.0)

    def test_load_error_urdf_version_unsupported(self):
        model_path = pathlib.Path("tests/data/error_urdf_version_unsupported.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r'.*robot: Unsupported version "2.0". Only 1.0, 1.1, and 1.2 are supported \(line: 3\).*'):
            parser.parse()

    def test_load_error_urdf_version_integer(self):
        model_path = pathlib.Path("tests/data/error_urdf_version_integer.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r'.*robot: Unsupported version "1". Only 1.0, 1.1, and 1.2 are supported \(line: 3\).*'):
            parser.parse()

    def test_load_error_urdf_version_extra(self):
        model_path = pathlib.Path("tests/data/error_urdf_version_extra.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r'.*robot: Unsupported version "1.0.0". Only 1.0, 1.1, and 1.2 are supported \(line: 3\).*'):
            parser.parse()

    def test_load_error_urdf_version_suffix(self):
        model_path = pathlib.Path("tests/data/error_urdf_version_suffix.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r'.*robot: Unsupported version "1.0~pre6". Only 1.0, 1.1, and 1.2 are supported \(line: 3\).*'):
            parser.parse()

    def test_load_error_urdf_version_one_ten(self):
        model_path = pathlib.Path("tests/data/error_urdf_version_one_ten.urdf")
        parser = URDFParser(model_path)

        with self.assertRaisesRegex(RuntimeError, r'.*robot: Unsupported version "1.10". Only 1.0, 1.1, and 1.2 are supported \(line: 3\).*'):
            parser.parse()
