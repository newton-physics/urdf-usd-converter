# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
import pathlib

from tests.util.ConverterTestCase import ConverterTestCase
from urdf_usd_converter._impl.urdf_parser.parser import URDFParser


class TestURDFParser(ConverterTestCase):
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
            self.assertTrue("8:2: mismatched tag" in e.args[0])

    def test_load_error_no_material_name(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_no_material_name.urdf")
        parser = URDFParser(model_path)

        try:
            parser.parse()
            self.fail("No exception occurred while loading the error XML.")
        except Exception as e:
            self.assertTrue(isinstance(e, RuntimeError))
            self.assertTrue("material: name is required (line: 8)" in e.args[0])

    def test_load_error_material_same_name(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_material_same_name.urdf")
        parser = URDFParser(model_path)

        try:
            parser.parse()
            self.fail("No exception occurred while loading the error XML.")
        except Exception as e:
            self.assertTrue(isinstance(e, RuntimeError))
            self.assertTrue("material: Material name 'green' already exists (line: 11)" in e.args[0])

    def test_load_error_link_same_name(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_link_same_name.urdf")
        parser = URDFParser(model_path)

        try:
            parser.parse()
            self.fail("No exception occurred while loading the error XML.")
        except Exception as e:
            self.assertTrue(isinstance(e, RuntimeError))
            self.assertTrue("link: Link name 'link2' already exists (line: 34)" in e.args[0])

    def test_load_error_material_not_exist(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_material_not_exist.urdf")
        parser = URDFParser(model_path)

        try:
            parser.parse()
            self.fail("No exception occurred while loading the error XML.")
        except Exception as e:
            self.assertTrue(isinstance(e, RuntimeError))
            self.assertTrue("Material name 'green2' not found (line: 27)" in e.args[0])

    def test_load_error_geometry_not_exist(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_visual_geometry_not_exist.urdf")
        parser = URDFParser(model_path)

        try:
            parser.parse()
            self.fail("No exception occurred while loading the error XML.")
        except Exception as e:
            self.assertTrue(isinstance(e, RuntimeError))
            self.assertTrue("Geometry must have one of the following: box, sphere, cylinder, or mesh (line: 24)" in e.args[0])

    def test_load_error_geometry_invalid_type(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_invalid_geometry_type.urdf")
        parser = URDFParser(model_path)

        try:
            parser.parse()
            self.fail("No exception occurred while loading the error XML.")
        except Exception as e:
            self.assertTrue(isinstance(e, RuntimeError))
            self.assertTrue("foo: Invalid geometry type (line: 25)" in e.args[0])

    def test_load_error_collision_geometry_not_exist(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_collision_geometry_not_exist.urdf")
        parser = URDFParser(model_path)

        try:
            parser.parse()
            self.fail("No exception occurred while loading the error XML.")
        except Exception as e:
            self.assertTrue(isinstance(e, RuntimeError))
            self.assertTrue("Geometry must have one of the following: box, sphere, cylinder, or mesh (line: 31)" in e.args[0])

    def test_load_error_joint_non_existent_child_link_name(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_joint_non_existent_child_link_name.urdf")
        parser = URDFParser(model_path)

        try:
            parser.parse()
            self.fail("No exception occurred while loading the error XML.")
        except Exception as e:
            self.assertTrue(isinstance(e, RuntimeError))
            self.assertTrue("joint: Child link 'link_dummy' not found (line: 36)" in e.args[0])

    def test_load_error_joint_non_existent_parent_link_name(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_joint_non_existent_parent_link_name.urdf")
        parser = URDFParser(model_path)

        try:
            parser.parse()
            self.fail("No exception occurred while loading the error XML.")
        except Exception as e:
            self.assertTrue(isinstance(e, RuntimeError))
            self.assertTrue("joint: Parent link 'BaseLink_dummy' not found (line: 35)" in e.args[0])

    def test_load_error_joint_incorrect_type(self):
        # Load the specified URDF file.
        model_path = pathlib.Path("tests/data/error_joint_incorrect_type.urdf")
        parser = URDFParser(model_path)

        try:
            parser.parse()
            self.fail("No exception occurred while loading the error XML.")
        except Exception as e:
            self.assertTrue(isinstance(e, RuntimeError))
            self.assertTrue("joint: Invalid joint type: incorrect_type (line: 22)" in e.args[0])
