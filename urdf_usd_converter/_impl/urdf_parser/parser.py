# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import re
import xml.etree.ElementTree as ET
from pathlib import Path

from .elements import (
    ElementAxis,
    ElementBase,
    ElementCalibration,
    ElementChild,
    ElementCollision,
    ElementColor,
    ElementDynamics,
    ElementGeometry,
    ElementInertia,
    ElementInertial,
    ElementJoint,
    ElementLimit,
    ElementLink,
    ElementMass,
    ElementMaterial,
    ElementMaterialGlobal,
    ElementMesh,
    ElementMimic,
    ElementParent,
    ElementRobot,
    ElementSafetyController,
    ElementTexture,
    ElementTransmission,
    ElementTransmissionActuator,
    ElementTransmissionHardwareInterface,
    ElementTransmissionJoint,
    ElementTransmissionMechanicalReduction,
    ElementTransmissionType,
    ElementUndefined,
    ElementVerbose,
    ElementVisual,
)
from .line_number_parser import LineNumberTrackingParser
from .reserved_element_attribute_names import check_element_attribute_name, check_element_name
from .undefined_data import UndefinedData

__all__ = ["URDFParser"]


class URDFParser:
    def __init__(self, input_file: Path):
        self.input_file: Path = input_file
        self.root_element: ElementBase = None
        self.line_info: dict[ET.Element, int] = {}
        self.line_tracking_parser = LineNumberTrackingParser()

        # A list of mesh file paths and scales.
        self.meshes: list[tuple[str, tuple[float, float, float]]] = []

        # A list of material colors and file paths.
        # However, this does not include materials used in mesh(dae, obj).
        self.materials: list[tuple[str, tuple[float, float, float, float], str]] = []

        self.texture_paths: list[str] = []

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
            self.root_element = self._parse_xml_elements(tree_root)

            # Validate the parsed elements.
            self._validate()

            # Store the meshes data.
            self._store_meshes()

            # Store the materials data.
            self._store_materials()

        except Exception as e:
            raise RuntimeError(f"Error parsing XML: {e}")

    def get_root_element(self) -> ElementRobot:
        """
        Get the root element (robot).

        Returns:
            The root element.
        """
        return self.root_element

    def find_material_by_name(self, name: str) -> ElementMaterial:
        """
        Find a material by name.

        Args:
            name: The name of the global material to find.

        Returns:
            The material if found, otherwise None.
        """
        for material in self.root_element.materials:
            if material.name == name:
                return material
        return None

    def get_robot_name(self) -> str:
        """
        Get the robot name.

        Returns:
            The robot name.
        """
        return self.root_element.name

    def get_meshes(self) -> list[tuple[str, tuple[float, float, float]]]:
        """
        Get the meshes.

        Returns:
            A list of tuples containing the mesh filename and scale.
        """
        return self.meshes

    def get_materials(self) -> list[tuple[str, tuple[float, float, float, float], str]]:
        """
        Get the materials.

        Returns:
            A list of tuples containing the material name, color, and file path.
        """
        return self.materials

    def get_undefined_elements(self) -> list[UndefinedData]:
        """
        Get undefined elements.

        Returns:
            A list of UndefinedData objects containing undefined elements and attributes.
        """
        # Trace undefined elements in the root "robot" element.
        undefined_elements: list[UndefinedData] = []
        self._get_undefined_elements_nested(self.root_element, undefined_elements)

        return undefined_elements

    def _convert_attribute_float3(self, element: ElementBase, name: str) -> tuple[float, float, float]:
        """
        Convert a string to a tuple of three floats.
        """
        if name not in element.attrib:
            return None

        attr_value = element.attrib[name].strip()
        # Separated by one or more spaces or tabs.
        values = re.split(r"\s+", attr_value)
        if len(values) != 3:
            raise ValueError(self._get_error_message(f"{name}: Invalid value: {attr_value}", element))
        return (float(values[0]), float(values[1]), float(values[2]))

    def _convert_attribute_float4(self, element: ElementBase, name: str) -> tuple[float, float, float, float]:
        """
        Convert a string to a tuple of four floats.
        """
        if name not in element.attrib:
            return None

        attr_value = element.attrib[name].strip()
        # Separated by one or more spaces or tabs.
        values = re.split(r"\s+", attr_value)
        if len(values) != 4:
            raise ValueError(self._get_error_message(f"{name}: Invalid value: {attr_value}", element))
        return (float(values[0]), float(values[1]), float(values[2]), float(values[3]))

    def _get_error_message(self, message: str, element: ElementBase | ET.Element) -> str:
        """
        Get an error message for an element.

        Args:
            message: The error message.
            element: The element to get the error message for.

        Returns:
            The error message.
        """
        line_number = self._get_element_line_number(element) if isinstance(element, ET.Element) else element.line_number
        return f"{element.tag}: {message} (line: {line_number})"

    def _parse_xml_elements(self, node: ET.Element, prev_element: ElementBase = None) -> ElementBase:
        """
        Parse the XML recursively and store each element.

        Args:
            node: The current XML element.
            prev_element: The previous element.

        Returns:
            The parsed element.
        """
        if not prev_element and node.tag != "robot":
            raise ValueError(self._get_error_message("The root element must be 'robot'", node))

        prev_element_type = type(prev_element) if prev_element else None
        prev_element_tag = prev_element.tag if prev_element else None

        # Path for tag hierarchy.
        current_path = f"{prev_element.path}/{node.tag}" if prev_element else f"/{node.tag}"

        # Check if the geometry type is valid.
        if prev_element_type == ElementGeometry and node.tag not in ElementGeometry.available_geometry_types:
            raise ValueError(self._get_error_message("Invalid geometry type", node))

        element = None

        # Stores element names that are not defined in the URDF.
        if prev_element_type == ElementUndefined or not check_element_name(node.tag):
            # All children of an undefined element are stored as undefined.
            element = ElementUndefined()
        else:
            # Get the element class that can use the specified tag name.
            element_class = self._get_element_class(node.tag, prev_element_tag)
            if element_class:
                element = element_class()

        # Error if using reserved tags but structure is different.
        if not element:
            raise ValueError(self._get_error_message("Invalid element type. This uses a reserved tag, but in the wrong place", node))

        element.tag = node.tag
        element.path = current_path
        element.line_number = self._get_element_line_number(node)

        if node.attrib.get("name"):
            element.name = node.attrib["name"]
        else:
            # If the name does not exist and a name is required, an error occurs.
            if isinstance(
                element,
                ElementRobot
                | ElementMaterialGlobal
                | ElementMaterial
                | ElementLink
                | ElementJoint
                | ElementTransmission
                | ElementTransmissionActuator
                | ElementTransmissionJoint,
            ):
                raise ValueError(self._get_error_message("name is required", node))

        if isinstance(element, ElementJoint):
            if node.attrib.get("type"):
                element.type = node.attrib["type"]
                if element.type not in ElementJoint.available_joint_types:
                    raise ValueError(self._get_error_message(f"Invalid joint type: {element.type}", node))
            else:
                raise ValueError(self._get_error_message("Type is required", node))

        # Get and store attributes.
        element.size = self._convert_attribute_float3(node, "size")
        element.xyz = self._convert_attribute_float3(node, "xyz")
        element.rpy = self._convert_attribute_float3(node, "rpy")
        if "radius" in node.attrib:
            element.radius = float(node.attrib["radius"])
        if "length" in node.attrib:
            element.length = float(node.attrib["length"])

        if "version" in node.attrib:
            element.version = node.attrib["version"]

        if isinstance(element, ElementUndefined):
            for key, value in node.attrib.items():
                element.undefined_attributes[key] = value
            element.undefined_text = node.text

        elif isinstance(element, ElementColor):
            element.rgba = self._convert_attribute_float4(node, "rgba")

        elif isinstance(element, ElementTexture):
            if "filename" in node.attrib:
                element.filename = node.attrib["filename"]

        elif isinstance(element, ElementMass):
            if "value" in node.attrib:
                element.value = float(node.attrib["value"])

        elif isinstance(element, ElementMesh):
            element.scale = self._convert_attribute_float3(node, "scale")
            if "filename" in node.attrib:
                element.filename = node.attrib["filename"]
            else:
                raise ValueError(self._get_error_message("Filename is required", node))

        elif isinstance(element, ElementSafetyController):
            if "soft_lower_limit" in node.attrib:
                element.soft_lower_limit = float(node.attrib["soft_lower_limit"])
            if "soft_upper_limit" in node.attrib:
                element.soft_upper_limit = float(node.attrib["soft_upper_limit"])
            if "k_position" in node.attrib:
                element.k_position = float(node.attrib["k_position"])
            if "k_velocity" in node.attrib:
                element.k_velocity = float(node.attrib["k_velocity"])
            else:
                raise ValueError(self._get_error_message("k_velocity is required", node))

        elif isinstance(element, ElementInertia):
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

        elif isinstance(element, ElementMimic):
            if "joint" in node.attrib:
                element.joint = node.attrib["joint"]
            else:
                raise ValueError(self._get_error_message("Joint is required", node))
            if "multiplier" in node.attrib:
                element.multiplier = float(node.attrib["multiplier"])
            if "offset" in node.attrib:
                element.offset = float(node.attrib["offset"])

        elif isinstance(element, ElementLimit):
            if "lower" in node.attrib:
                element.lower = float(node.attrib["lower"])
            if "upper" in node.attrib:
                element.upper = float(node.attrib["upper"])
            if "effort" in node.attrib:
                element.effort = float(node.attrib["effort"])
            if "velocity" in node.attrib:
                element.velocity = float(node.attrib["velocity"])

        elif isinstance(element, ElementCalibration):
            if "reference_position" in node.attrib:
                element.reference_position = float(node.attrib["reference_position"])
            if "rising" in node.attrib:
                element.rising = float(node.attrib["rising"])
            if "falling" in node.attrib:
                element.falling = float(node.attrib["falling"])

        elif isinstance(element, ElementDynamics):
            if "damping" in node.attrib:
                element.damping = float(node.attrib["damping"])
            if "friction" in node.attrib:
                element.friction = float(node.attrib["friction"])

        elif isinstance(element, ElementParent | ElementChild):
            if "link" in node.attrib:
                element.link = node.attrib["link"]
            else:
                raise ValueError(self._get_error_message("Link is required", node))

        elif isinstance(element, ElementAxis):
            element.xyz = self._convert_attribute_float3(node, "xyz")

        elif isinstance(element, ElementVerbose):
            if "value" in node.attrib:
                element.value = node.attrib["value"]

        elif isinstance(element, ElementTransmissionHardwareInterface):
            if node.text:
                element.text = node.text

        elif isinstance(element, ElementTransmissionMechanicalReduction):
            if node.text:
                element.text = float(node.text)

        elif isinstance(element, ElementTransmissionType):
            if node.text:
                element.text = node.text

        # Parse child elements.
        for child in node:
            self._parse_xml_elements(child, element)

        # The elements are associated so that they form a hierarchical structure.
        if prev_element_type == ElementRobot:
            if node.tag == "link":
                if element.name in [link.name for link in prev_element.links]:
                    raise ValueError(self._get_error_message(f"Link name '{element.name}' already exists", node))
                prev_element.links.append(element)
            elif node.tag == "material" and isinstance(element, ElementMaterialGlobal):
                if element.name in [material.name for material in prev_element.materials]:
                    raise ValueError(self._get_error_message(f"Material name '{element.name}' already exists", node))
                prev_element.materials.append(element)
            elif node.tag == "joint":
                if element.name in [joint.name for joint in prev_element.joints]:
                    raise ValueError(self._get_error_message(f"Joint name '{element.name}' already exists", node))
                prev_element.joints.append(element)
            elif node.tag == "transmission":
                if element.name in [transmission.name for transmission in prev_element.transmissions]:
                    raise ValueError(self._get_error_message(f"Transmission name '{element.name}' already exists", node))
                prev_element.transmissions.append(element)

        elif prev_element_type in (ElementMaterialGlobal, ElementMaterial):
            if node.tag == "color":
                prev_element.color = element
            elif node.tag == "texture":
                prev_element.texture = element

        elif prev_element_type == ElementLink:
            if node.tag == "visual":
                prev_element.visuals.append(element)
            elif node.tag == "collision":
                prev_element.collisions.append(element)
            elif node.tag == "inertial":
                prev_element.inertial = element

        elif prev_element_type in (ElementVisual, ElementCollision):
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
                prev_element.shape = element

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
            elif node.tag == "type":
                prev_element.type = element

        elif prev_element_type == ElementTransmissionActuator:
            if node.tag == "mechanicalReduction":
                prev_element.mechanicalReduction = element
            elif node.tag == "hardwareInterface":
                prev_element.hardwareInterface = element

        elif prev_element_type == ElementTransmissionJoint and node.tag == "hardwareInterface":
            prev_element.hardwareInterface = element

        # Stores undefined elements.
        if prev_element_type == ElementUndefined or isinstance(element, ElementUndefined):
            prev_element.undefined_elements.append(element)
        else:
            # Gets and stores the names and values ​​of elements that are not defined in node.attrib.
            for key, value in node.attrib.items():
                if not check_element_attribute_name(element.tag, key):
                    element.undefined_attributes[key] = value

        return element

    def _get_element_line_number(self, element: ET.Element) -> int:
        """
        Get the line number of an element

        Args:
            element: The element to get the line number for.

        Returns:
            The line number of the element.
        """
        return self.line_info.get(element, -1)

    def _get_defined_material_names(self) -> list[str]:
        """
        Get the defined material names.

        Returns:
            A list of defined material names.
        """
        # Create a list of defined material names.
        # This includes both global materials and materials specified within the visual.
        defined_material_names = [material.name for material in self.root_element.materials]
        for link in self.root_element.links:
            for visual in link.visuals:
                material = visual.material
                if (
                    material
                    and material.name is not None
                    and (material.color is not None or material.texture is not None)
                    and material.name not in defined_material_names
                ):
                    defined_material_names.append(material.name)

        return defined_material_names

    def _validate(self):
        """
        Validate the parsed elements.
        """
        defined_material_names = self._get_defined_material_names()

        # If there is a material name in the link, check if there is a material with that name in self.root_element.materials.
        for link in self.root_element.links:
            for visual in link.visuals:
                if visual.material:
                    material = visual.material
                    if material.name and not material.color and not material.texture and material.name not in defined_material_names:
                        raise ValueError(self._get_error_message(f"link: Material name '{material.name}' not found", material))

        for joint in self.root_element.joints:
            # Checks if parent and child links exist.
            if not joint.parent:
                raise ValueError(self._get_error_message("Parent is required", joint))
            if not joint.child:
                raise ValueError(self._get_error_message("Child is required", joint))

        # If the link name does not exist, an error occurs.
        for joint in self.root_element.joints:
            if joint.parent and joint.parent.link not in [link.name for link in self.root_element.links]:
                raise ValueError(self._get_error_message(f"Parent link '{joint.parent.link}' not found", joint.parent))
            if joint.child and joint.child.link not in [link.name for link in self.root_element.links]:
                raise ValueError(self._get_error_message(f"Child link '{joint.child.link}' not found", joint.child))

        # If no elements exist within the geometry tab of the link, an error occurs.
        for link in self.root_element.links:
            for visual in link.visuals:
                if visual and visual.geometry:
                    geometry = visual.geometry.shape
                    if not geometry:
                        raise ValueError(
                            self._get_error_message("Geometry must have one of the following: box, sphere, cylinder, or mesh", visual.geometry)
                        )
            for collision in link.collisions:
                if collision.geometry:
                    geometry = collision.geometry.shape
                    if not geometry:
                        raise ValueError(
                            self._get_error_message("Geometry must have one of the following: box, sphere, cylinder, or mesh", collision.geometry)
                        )

    def _get_element_class(self, tag_name: str, prev_element_tag: str) -> type[ElementBase]:
        """
        Get the element class that can use the specified tag name.

        Args:
            tag_name: The tag name of the element.
            prev_element_tag: The tag name of the previous element.

        Returns:
            The element class that can use the specified tag name.
        """
        if tag_name == "robot" and not prev_element_tag:
            return ElementRobot

        for element_class in ElementBase.__subclasses__():
            if tag_name in element_class.available_tag_names and prev_element_tag in element_class.allowed_parent_tags:
                return element_class
        return None

    def _store_meshes(self):
        """
        Store the meshes.
        A mesh has a filename and a scale.
        """
        geometry_list = []
        for link in self.root_element.links:
            for visual in link.visuals:
                if visual.geometry:
                    geometry = visual.geometry.shape
                    if geometry and isinstance(geometry, ElementMesh):
                        geometry_list.append(geometry)
            for collision in link.collisions:
                if collision.geometry:
                    geometry = collision.geometry.shape
                    if geometry and isinstance(geometry, ElementMesh):
                        geometry_list.append(geometry)

        for geometry in geometry_list:
            scale = geometry.get_with_default("scale")
            for mesh in self.meshes:
                if mesh[0] == geometry.filename and mesh[1] == scale:
                    break
            else:
                self.meshes.append((geometry.filename, scale))

    def _store_materials(self):
        """
        Store the materials.
        A material has a name, color, and file path.
        """
        # Global material names are unique, so they are stored as is.
        for material in self.root_element.materials:
            color = material.color.get_with_default("rgba") if material.color else (0.0, 0.0, 0.0, 0.0)
            texture = material.texture.get_with_default("filename") if material.texture else None
            self.materials.append((material.name, color, texture))

        for link in self.root_element.links:
            for visual in link.visuals:
                visual_material = visual.material
                if visual_material:
                    material_name = visual_material.name if visual_material.name is not None else ""

                    # If the material name is already stored, skip it.
                    if material_name in [material[0] for material in self.materials]:
                        continue

                    color = visual_material.color.get_with_default("rgba") if visual_material.color else (0.0, 0.0, 0.0, 0.0)
                    texture = visual_material.texture.get_with_default("filename") if visual_material.texture else None
                    self.materials.append((material_name, color, texture))

    def _get_undefined_elements_nested(self, element: ElementBase, undefined_elements: list[UndefinedData]):
        """
        Get undefined elements nested in an element.

        Args:
            element: The element to get the undefined elements for.
            undefined_elements: The list to store the undefined elements in.
        """
        # If there are any undefined elements, they are stored.
        for e in element.undefined_elements:
            undefined_data = UndefinedData(e, True)
            undefined_elements.append(undefined_data)
            self._get_undefined_elements_nested(e, undefined_elements)

        # If there are any undefined attributes, they are stored.
        if len(element.undefined_attributes) > 0:
            for undefined_data in undefined_elements:
                if undefined_data.path == element.path and undefined_data.line_number == element.line_number:
                    break
            else:
                undefined_data = UndefinedData(element, False)
                undefined_elements.append(undefined_data)

        if isinstance(element, ElementRobot):
            for e in element.materials:
                self._get_undefined_elements_nested(e, undefined_elements)
            for e in element.links:
                self._get_undefined_elements_nested(e, undefined_elements)
            for e in element.joints:
                self._get_undefined_elements_nested(e, undefined_elements)
            for e in element.transmissions:
                self._get_undefined_elements_nested(e, undefined_elements)
        elif isinstance(element, ElementLink):
            for e in element.visuals:
                self._get_undefined_elements_nested(e, undefined_elements)
            for e in element.collisions:
                self._get_undefined_elements_nested(e, undefined_elements)
        else:
            for e in element.__dict__:
                if isinstance(element.__dict__[e], ElementBase):
                    self._get_undefined_elements_nested(element.__dict__[e], undefined_elements)
