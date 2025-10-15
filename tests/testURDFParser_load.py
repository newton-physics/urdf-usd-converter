# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
import pathlib

from pxr import Gf

from tests.util.ConverterTestCase import ConverterTestCase
from urdf_usd_converter._impl.urdf_parser.parser import URDFParser


class TestURDFParser(ConverterTestCase):
    def setUp(self):
        super().setUp()

        # Parse the URDF file.
        # This file contains an XML header (<?xml version="1.0"?>).
        # We can trace the URDF structure from the root_element.
        # The root element will be "robot".
        self.parser = URDFParser(pathlib.Path("tests/data/simple_primitives.urdf"))
        try:
            self.parser.parse()
        except Exception as e:
            self.fail(f"Error parsing URDF file: {e}")

    def test_get_basic_information(self):
        # Get basic information about a URDF.

        # Get the root element.
        root_element = self.parser.get_root_element()

        # Get the model name ("robot" element's "name" attribute).
        root_model_name = self.parser.get_model_name()

        self.assertTrue(root_element)
        self.assertEqual(root_element.tag, "robot")
        self.assertEqual(root_element.name, "simple_primitives")
        self.assertEqual(root_model_name, "simple_primitives")

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
        self.assertEqual(red_material.color.rgba, Gf.Vec4d(1.0, 0.0, 0.0, 1.0))

        blue_material = self.parser.find_material_by_name("blue")
        self.assertTrue(blue_material)
        self.assertEqual(blue_material.name, "blue")
        self.assertEqual(blue_material.color.rgba, Gf.Vec4d(0.0, 0.0, 1.0, 1.0))

        green_material = self.parser.find_material_by_name("green")
        self.assertTrue(green_material)
        self.assertEqual(green_material.name, "green")
        self.assertEqual(green_material.color.rgba, Gf.Vec4d(0.0, 1.0, 0.0, 1.0))

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
        self.assertEqual(link.name, "BaseLink")
        self.assertIsNone(link.type)

        visual = link.visual
        self.assertTrue(visual)
        origin = visual.origin
        self.assertTrue(origin)
        self.assertEqual(origin.xyz, Gf.Vec3d(-2.0, 0.0, 0.5))
        self.assertEqual(origin.rpy, Gf.Vec3d(0.0, 0.0, 0.0))
        geometry = visual.geometry
        self.assertTrue(geometry)
        self.assertEqual(geometry.geometry.tag, "box")
        self.assertEqual(geometry.geometry.size, Gf.Vec3d(1.0, 1.0, 1.0))
        material = visual.material
        self.assertTrue(material)
        self.assertEqual(material.name, "red")

        # links[1]
        link = root_element.links[1]
        self.assertEqual(link.name, "link2")
        self.assertIsNone(link.type)

        visual = link.visual
        self.assertTrue(visual)
        origin = visual.origin
        self.assertTrue(origin)
        self.assertEqual(origin.xyz, Gf.Vec3d(0.0, 0.0, 0.5))
        self.assertEqual(origin.rpy, Gf.Vec3d(0.0, 0.0, 0.0))
        geometry = visual.geometry
        self.assertTrue(geometry)
        self.assertEqual(geometry.geometry.tag, "sphere")
        self.assertEqual(geometry.geometry.radius, 0.5)
        material = visual.material
        self.assertTrue(material)
        self.assertEqual(material.name, "green")

        # links[2]
        link = root_element.links[2]
        self.assertEqual(link.name, "link3")
        self.assertIsNone(link.type)
        visual = link.visual
        self.assertTrue(visual)
        origin = visual.origin
        self.assertTrue(origin)
        self.assertEqual(origin.xyz, Gf.Vec3d(2.0, 0.0, 0.5))
        self.assertEqual(origin.rpy, Gf.Vec3d(0.0, 0.0, 0.0))
        geometry = visual.geometry
        self.assertTrue(geometry)
        self.assertEqual(geometry.geometry.tag, "cylinder")
        self.assertEqual(geometry.geometry.radius, 0.5)
        self.assertEqual(geometry.geometry.length, 1.0)
        material = visual.material
        self.assertTrue(material)
        self.assertEqual(material.name, "blue")

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
