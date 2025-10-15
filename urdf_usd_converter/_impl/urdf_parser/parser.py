# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import xml.etree.ElementTree as ET
from pathlib import Path

from pxr import Gf

from .convert_to_usd import convert_vec3d, convert_vec4d
from .elements import *
from .line_number_parser import LineNumberTrackingParser
from .reserved_element_attribute_names import check_element_attribute_name, check_element_name
from .undefined_data import UndefinedData

__all__ = ["URDFParser"]

joint_type_list = ["revolute", "continuous", "prismatic", "fixed", "floating", "planar"]

geometry_type_list = ["box", "sphere", "cylinder", "mesh"]


# -----------------------------------------------------------.
class URDFParser:
    def __init__(self, input_file: Path):
        self.input_file: Path = input_file
        self.root_element: ElementBase = None
        self.line_info: dict[ET.Element, int] = {}
        self.line_tracking_parser = LineNumberTrackingParser()

        # A list of mesh file paths and scales.
        self.meshs: list[tuple[str, Gf.Vec3d]] = []

        self.texture_paths: list[str] = []

    def get_error_message(self, message: str, element: ElementBase | ET.Element) -> str:
        if isinstance(element, ET.Element):
            line_number = self.get_element_line_number(element)
        else:
            line_number = element.line_number
        return f"{element.tag}: {message} (line: {line_number})"

    def parse_xml_elements(self, node: ET.Element, prev_element: ElementBase = None) -> ElementBase:
        """
        Parse the XML recursively and store each element.

        Args:
            node: The current XML element.
            prev_element: The previous element.

        Returns:
            The parsed element.
        """

        prev_element_type = type(prev_element) if prev_element else None
        prev_element_tag = prev_element.tag if prev_element else None

        # Path for tag hierarchy.
        current_path = f"{prev_element.path}/{node.tag}" if prev_element else f"/{node.tag}"

        # Check if the geometry type is valid.
        if prev_element_type == ElementGeometry:
            if node.tag not in geometry_type_list:
                raise ValueError(self.get_error_message("Invalid geometry type", node))

        element = None

        # Stores element names that are not defined in the URDF.
        if prev_element_type == ElementUndefined or not check_element_name(node.tag):
            # All children of an undefined element are stored as undefined.
            element = ElementUndefined()
        else:
            # Get the element class that can use the specified tag name.
            element_class = self.get_element_class(node.tag, prev_element_tag)
            if element_class:
                element = element_class()

        # Error if using reserved tags but structure is different.
        if not element:
            raise ValueError(self.get_error_message("Invalid element type. This uses a reserved tag, but in the wrong place", node))

        element.tag = node.tag
        element.path = current_path
        element.line_number = self.get_element_line_number(node)

        if node.attrib.get("name"):
            element.name = node.attrib["name"]
        else:
            # If the name does not exist and a name is required, an error occurs.
            if (
                type(element) == ElementRobot
                or type(element) == ElementMaterialGlobal
                or type(element) == ElementLink
                or type(element) == ElementJoint
                or type(element) == ElementSensor
                or type(element) == ElementActuatorTransmission
                or type(element) == ElementGapJointTransmission
                or type(element) == ElementPassiveJointTransmission
                or type(element) == ElementTransmission
            ):
                raise ValueError(self.get_error_message("name is required", node))

        if type(element) == ElementJoint:
            if node.attrib.get("type"):
                element.type = node.attrib["type"]
                if element.type not in joint_type_list:
                    raise ValueError(self.get_error_message(f"Invalid joint type: {element.type}", node))
            else:
                raise ValueError(self.get_error_message("Type is required", node))

        # Get and store attributes.
        if "size" in node.attrib:
            try:
                element.size = convert_vec3d(node.attrib["size"])
            except Exception as e:
                raise ValueError(self.get_error_message(f"Invalid size: {e}", node))
        if "xyz" in node.attrib:
            try:
                element.xyz = convert_vec3d(node.attrib["xyz"])
            except Exception as e:
                raise ValueError(self.get_error_message(f"Invalid xyz: {e}", node))
        if "rpy" in node.attrib:
            try:
                element.rpy = convert_vec3d(node.attrib["rpy"])
            except Exception as e:
                raise ValueError(self.get_error_message(f"Invalid rpy: {e}", node))
        if "radius" in node.attrib:
            try:
                element.radius = float(node.attrib["radius"])
            except Exception as e:
                raise ValueError(self.get_error_message(f"Invalid radius: {e}", node))
        if "length" in node.attrib:
            try:
                element.length = float(node.attrib["length"])
            except Exception as e:
                raise ValueError(self.get_error_message(f"Invalid length: {e}", node))

        if "version" in node.attrib:
            element.version = node.attrib["version"]

        if type(element) == ElementUndefined:
            for key, value in node.attrib.items():
                element.undefined_attributes[key] = value

        elif type(element) == ElementColor:
            if "rgba" in node.attrib:
                try:
                    element.rgba = convert_vec4d(node.attrib["rgba"])
                except Exception as e:
                    raise ValueError(self.get_error_message(f"Invalid rgba: {e}", node))

        elif type(element) == ElementTexture:
            if "filename" in node.attrib:
                element.filename = node.attrib["filename"]

        elif type(element) == ElementMass:
            if "mass" in node.attrib:
                element.mass = float(node.attrib["mass"])

        elif type(element) == ElementMesh:
            if "scale" in node.attrib:
                try:
                    element.scale = convert_vec3d(node.attrib["scale"])
                except Exception as e:
                    raise ValueError(self.get_error_message(f"Invalid scale: {e}", node))
            if "filename" in node.attrib:
                element.filename = node.attrib["filename"]
            else:
                raise ValueError(self.get_error_message("Filename is required", node))

        elif type(element) == ElementSafetyController:
            if "soft_lower_limit" in node.attrib:
                element.soft_lower_limit = float(node.attrib["soft_lower_limit"])
            if "soft_upper_limit" in node.attrib:
                element.soft_upper_limit = float(node.attrib["soft_upper_limit"])
            if "k_position" in node.attrib:
                element.k_position = float(node.attrib["k_position"])
            if "k_velocity" in node.attrib:
                element.k_velocity = float(node.attrib["k_velocity"])
            else:
                raise ValueError(self.get_error_message("k_velocity is required", node))

        elif type(element) == ElementInertia:
            if "ixx" in node.attrib:
                element.ixx = float(node.attrib["ixx"])
            if "iyy" in node.attrib:
                element.iyy = float(node.attrib["iyy"])
            if "izz" in node.attrib:
                element.izz = float(node.attrib["izz"])
            if "ixy" in node.attrib:
                element.ixy = float(node.attrib["ixy"])
            if "ixz" in node.attrib:
                element.ixz = float(node.attrib["ixz"])
            if "iyz" in node.attrib:
                element.iyz = float(node.attrib["iyz"])

        elif type(element) == ElementMimic:
            if "joint" in node.attrib:
                element.joint = node.attrib["joint"]
            else:
                raise ValueError(self.get_error_message("Joint is required", node))
            if "multiplier" in node.attrib:
                element.multiplier = float(node.attrib["multiplier"])
            if "offset" in node.attrib:
                element.offset = float(node.attrib["offset"])

        elif type(element) == ElementLimit:
            if "lower" in node.attrib:
                element.lower = float(node.attrib["lower"])
            if "upper" in node.attrib:
                element.upper = float(node.attrib["upper"])
            if "effort" in node.attrib:
                element.effort = float(node.attrib["effort"])
            if "velocity" in node.attrib:
                element.velocity = float(node.attrib["velocity"])

        elif type(element) == ElementCalibration:
            if "reference_position" in node.attrib:
                element.reference_position = float(node.attrib["reference_position"])
            if "rising" in node.attrib:
                element.rising = float(node.attrib["rising"])
            if "falling" in node.attrib:
                element.falling = float(node.attrib["falling"])

        elif type(element) == ElementDynamics:
            if "damping" in node.attrib:
                element.damping = float(node.attrib["damping"])
            if "friction" in node.attrib:
                element.friction = float(node.attrib["friction"])

        elif type(element) == ElementParent or type(element) == ElementChild:
            if "link" in node.attrib:
                element.link = node.attrib["link"]
            else:
                raise ValueError(self.get_error_message("Link is required", node))

        elif type(element) == ElementAxis:
            if "xyz" in node.attrib:
                try:
                    element.xyz = convert_vec3d(node.attrib["xyz"])
                except Exception as e:
                    raise ValueError(self.get_error_message(f"Invalid xyz: {e}", node))

        elif type(element) == ElementVerbose:
            if "value" in node.attrib:
                element.value = node.attrib["value"]

        elif type(element) == ElementActuatorTransmission:
            if "mechanicalReduction" in node.attrib:
                element.mechanicalReduction = float(node.attrib["mechanicalReduction"])

        elif type(element) == ElementGapJointTransmission:
            if "L0" in node.attrib:
                element.L0 = float(node.attrib["L0"])
            if "a" in node.attrib:
                element.a = float(node.attrib["a"])
            if "b" in node.attrib:
                element.b = float(node.attrib["b"])
            if "gear_ratio" in node.attrib:
                element.gear_ratio = float(node.attrib["gear_ratio"])
            if "h" in node.attrib:
                element.h = float(node.attrib["h"])
            if "mechanical_reduction" in node.attrib:
                element.mechanical_reduction = float(node.attrib["mechanical_reduction"])
            if "phi0" in node.attrib:
                element.phi0 = float(node.attrib["phi0"])
            if "r" in node.attrib:
                element.r = float(node.attrib["r"])
            if "screw_reduction" in node.attrib:
                element.screw_reduction = float(node.attrib["screw_reduction"])
            if "t0" in node.attrib:
                element.t0 = float(node.attrib["t0"])
            if "theta0" in node.attrib:
                element.theta0 = float(node.attrib["theta0"])

        elif type(element) == ElementTransmission:
            if "type" in node.attrib:
                element.type = node.attrib["type"]
            else:
                raise ValueError(self.get_error_message("Transmission type is required", node))
            if "mechanicalReduction" in node.attrib:
                element.mechanicalReduction = float(node.attrib["mechanicalReduction"])

        elif type(element) == ElementImage:
            if "width" in node.attrib:
                element.width = int(node.attrib["width"])
            if "height" in node.attrib:
                element.height = int(node.attrib["height"])
            if "format" in node.attrib:
                element.format = node.attrib["format"]
            if "hfov" in node.attrib:
                element.hfov = float(node.attrib["hfov"])
            if "near" in node.attrib:
                element.near = float(node.attrib["near"])
            if "far" in node.attrib:
                element.far = float(node.attrib["far"])

        elif type(element) == ElementLaserRay:
            if "samples" in node.attrib:
                element.samples = int(node.attrib["samples"])
            if "resolution" in node.attrib:
                element.resolution = int(node.attrib["resolution"])
            if "min_angle" in node.attrib:
                element.min_angle = float(node.attrib["min_angle"])
            if "max_angle" in node.attrib:
                element.max_angle = float(node.attrib["max_angle"])

        elif type(element) == ElementSensor:
            if "update_rate" in node.attrib:
                element.update_rate = float(node.attrib["update_rate"])
            if "version" in node.attrib:
                element.version = node.attrib["version"]

        # Parse child elements.
        for child in node:
            self.parse_xml_elements(child, element)

        # The elements are associated so that they form a hierarchical structure.
        if prev_element_type == ElementRobot:
            if node.tag == "link":
                if element.name in [link.name for link in prev_element.links]:
                    raise ValueError(self.get_error_message(f"Link name '{element.name}' already exists", node))
                prev_element.links.append(element)
            elif node.tag == "material" and type(element) == ElementMaterialGlobal:
                if element.name in [material.name for material in prev_element.materials]:
                    raise ValueError(self.get_error_message(f"Material name '{element.name}' already exists", node))
                prev_element.materials.append(element)
            elif node.tag == "joint":
                if element.name in [joint.name for joint in prev_element.joints]:
                    raise ValueError(self.get_error_message(f"Joint name '{element.name}' already exists", node))
                prev_element.joints.append(element)

        elif prev_element_type == ElementMaterialGlobal or prev_element_type == ElementMaterial:
            if node.tag == "color":
                prev_element.color = element
            elif node.tag == "texture":
                prev_element.texture = element

        elif prev_element_type == ElementLink:
            if node.tag == "visual":
                prev_element.visual = element
            elif node.tag == "collision":
                prev_element.collision = element
            elif node.tag == "inertial":
                prev_element.inertial = element

        elif prev_element_type == ElementVisual or prev_element_type == ElementCollision:
            if node.tag == "geometry":
                prev_element.geometry = element
            elif node.tag == "origin":
                prev_element.origin = element
            elif prev_element_type == ElementVisual and node.tag == "material":
                prev_element.material = element
            elif prev_element_type == ElementCollision and node.tag == "verbose":
                prev_element.verbose = element

        elif prev_element_type == ElementGeometry:
            if node.tag == "box" or node.tag == "sphere" or node.tag == "cylinder" or node.tag == "mesh":
                prev_element.geometry = element

        elif prev_element_type == ElementInertial:
            if node.tag == "origin":
                prev_element.origin = element
            elif node.tag == "inertia":
                prev_element.inertia = element
            elif node.tag == "mass":
                prev_element.mass = element

        elif prev_element_type == ElementJoint:
            if node.tag == "origin":
                prev_element.origin = element
            elif node.tag == "limit":
                prev_element.limit = element
            elif node.tag == "parent":
                prev_element.parent = element
            elif node.tag == "child":
                prev_element.child = element
            elif node.tag == "axis":
                prev_element.axis = element
            elif node.tag == "dynamics":
                prev_element.dynamics = element
            elif node.tag == "calibration":
                prev_element.calibration = element
            elif node.tag == "safety_controller":
                prev_element.safety_controller = element
            elif node.tag == "mimic":
                prev_element.mimic = element

        elif prev_element_type == ElementTransmission:
            if node.tag == "actuator":
                prev_element.actuator = element
            elif node.tag == "joint":
                prev_element.joint = element
            elif node.tag == "leftActuator":
                prev_element.leftActuator = element
            elif node.tag == "rightActuator":
                prev_element.rightActuator = element
            elif node.tag == "flexJoint":
                prev_element.flexJoint = element
            elif node.tag == "rollJoint":
                prev_element.rollJoint = element
            elif node.tag == "gap_joint":
                prev_element.gap_joint = element
            elif node.tag == "passive_joint":
                prev_element.passive_joint = element

        elif prev_element_type == ElementCamera:
            if node.tag == "image":
                prev_element.image = element

        elif prev_element_type == ElementRay:
            if node.tag == "horizontal":
                prev_element.horizontal = element
            elif node.tag == "vertical":
                prev_element.vertical = element

        elif prev_element_type == ElementSensor:
            if node.tag == "camera" or node.tag == "ray":
                prev_element.camera_ray = element
            elif node.tag == "origin":
                prev_element.origin = element
            elif node.tag == "parent":
                prev_element.parent = element

        # Stores undefined elements.
        if prev_element_type == ElementUndefined or type(element) == ElementUndefined:
            prev_element.undefined_elements.append(element)
        else:
            # Gets and stores the names and values ​​of elements that are not defined in node.attrib.
            for key, value in node.attrib.items():
                if not check_element_attribute_name(element.tag, key):
                    element.undefined_attributes[key] = value

        return element

    def parse(self):
        """
        Parse the XML file.
        """
        # Check if the file exists.
        if not self.input_file.exists():
            raise FileNotFoundError(f"File not found: {self.input_file}")

        # Parse XML with line number tracking.
        try:
            tree_root, self.line_info = self.line_tracking_parser.parse_with_line_numbers(self.input_file)
            self.root_element = self.parse_xml_elements(tree_root)

            # Validate the parsed elements.
            self.validate()

            # Store the meshs data.
            self.store_meshs()

        except Exception as e:
            raise RuntimeError(f"Error parsing XML: {e}")

    def get_root_element(self):
        """
        Get the root element
        """
        return self.root_element

    def get_element_line_number(self, element: ET.Element) -> int:
        """
        Get the line number of an element
        """
        return self.line_info.get(element, -1)

    def validate(self):
        """
        Validate the parsed elements
        """
        if not self.root_element:
            return

        base_file_dir = self.input_file.parent

        # If there is a material name in the link, check if there is a material with that name in self.root_element.materials.
        for link in self.root_element.links:
            if link.visual and link.visual.material:
                material = link.visual.material
                if material.name and not material.color and not material.texture:
                    if material.name not in [material.name for material in self.root_element.materials]:
                        raise ValueError(self.get_error_message(f"link: Material name '{material.name}' not found", material))

        for joint in self.root_element.joints:
            # Checks if parent and child links exist.
            if not joint.parent:
                raise ValueError(self.get_error_message("joint: Parent link is required", joint))
            if not joint.child:
                raise ValueError(self.get_error_message("joint: Child link is required", joint))

        # If the link name does not exist, an error occurs.
        for joint in self.root_element.joints:
            if joint.parent and joint.parent.link not in [link.name for link in self.root_element.links]:
                raise ValueError(self.get_error_message(f"joint: Parent link '{joint.parent.link}' not found", joint.parent))
            if joint.child and joint.child.link not in [link.name for link in self.root_element.links]:
                raise ValueError(self.get_error_message(f"joint: Child link '{joint.child.link}' not found", joint.child))

        # If no elements exist within the geometry tab of the link, an error occurs.
        for link in self.root_element.links:
            if link.visual and link.visual.geometry:
                geometry = link.visual.geometry.geometry
                if not geometry:
                    raise ValueError(
                        self.get_error_message("Geometry must have one of the following: box, sphere, cylinder, or mesh", link.visual.geometry)
                    )
            if link.collision and link.collision.geometry:
                geometry = link.collision.geometry.geometry
                if not geometry:
                    raise ValueError(
                        self.get_error_message("Geometry must have one of the following: box, sphere, cylinder, or mesh", link.collision.geometry)
                    )

    def get_model_name(self) -> str:
        """
        Get the model name.
        """
        return self.root_element.name

    def get_element_class(self, tag_name: str, prev_element_tag: str) -> type[ElementBase]:
        """Get the element class that can use the specified tag name."""
        if tag_name == "robot" and not prev_element_tag:
            return ElementRobot

        for element_class in ElementBase.__subclasses__():
            if tag_name in element_class.available_tag_names and prev_element_tag in element_class.allowed_parent_tags:
                return element_class
        return None

    def find_material_by_name(self, name: str) -> ElementMaterial:
        """
        Find a material by name.
        """
        for material in self.root_element.materials:
            if material.name == name:
                return material
        return None

    def store_meshs(self):
        """
        Store the meshs.
        A mesh has a filename and a scale.
        """
        geometry_list = []
        for link in self.root_element.links:
            if link.visual and link.visual.geometry:
                geometry = link.visual.geometry.geometry
                if geometry and type(geometry) == ElementMesh:
                    geometry_list.append(geometry)
            if link.collision and link.collision.geometry:
                geometry = link.collision.geometry.geometry
                if geometry and type(geometry) == ElementMesh:
                    geometry_list.append(geometry)

        for geometry in geometry_list:
            scale = geometry.get_with_default("scale")
            for mesh in self.meshs:
                if mesh[0] == geometry.filename and mesh[1] == scale:
                    break
            else:
                self.meshs.append((geometry.filename, scale))

    def get_meshs(self) -> list[tuple[str, Gf.Vec3d]]:
        """
        Get the meshs.
        """
        return self.meshs

    def get_undefined_elements(self) -> list[UndefinedData]:
        """
        Get undefined elements.
        """
        # Trace undefined elements in the root "robot" element.
        undefined_elements: list[UndefinedData] = []
        self.get_undefined_elements_nested(self.root_element, undefined_elements)

        return undefined_elements

    def get_undefined_elements_nested(self, element: ElementBase, undefined_elements: list[UndefinedData]):
        # If there are any undefined elements, they are stored.
        for e in element.undefined_elements:
            for undefined_data in undefined_elements:
                if undefined_data.path == e.path and undefined_data.line_number == e.line_number:
                    break
            else:
                undefined_data = UndefinedData(e, True)
                undefined_elements.append(undefined_data)
                self.get_undefined_elements_nested(e, undefined_elements)

        # If there are any undefined attributes, they are stored.
        if len(element.undefined_attributes) > 0:
            for undefined_data in undefined_elements:
                if undefined_data.path == element.path and undefined_data.line_number == element.line_number:
                    break
            else:
                undefined_data = UndefinedData(element, False)
                undefined_elements.append(undefined_data)

        if type(element) == ElementRobot:
            for e in element.materials:
                self.get_undefined_elements_nested(e, undefined_elements)
            for e in element.links:
                self.get_undefined_elements_nested(e, undefined_elements)
            for e in element.joints:
                self.get_undefined_elements_nested(e, undefined_elements)
            for e in element.transmissions:
                self.get_undefined_elements_nested(e, undefined_elements)
        else:
            for e in element.__dict__:
                if isinstance(element.__dict__[e], ElementBase):
                    self.get_undefined_elements_nested(element.__dict__[e], undefined_elements)
