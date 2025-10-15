# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# Reserved attribute names for elements.
#   https://raw.githubusercontent.com/ros/urdfdom/master/xsd/urdf.xsd

__all__ = ["check_element_attribute_name", "check_element_name"]

# Element names defined in the URDF Schema.
reserved_element_names = [
    "actuator",
    "axis",
    "box",
    "calibration",
    "camera",
    "child",
    "collision",
    "color",
    "cylinder",
    "dynamics",
    "flexJoint",
    "gap_joint",
    "gazebo",
    "geometry",
    "horizontal",
    "image",
    "inertia",
    "inertial",
    "joint",
    "leftActuator",
    "limit",
    "link",
    "mass",
    "material",
    "mechanicalReduction",
    "mesh",
    "mimic",
    "origin",
    "parent",
    "passive_joint",
    "ray",
    "rightActuator",
    "robot",
    "rollJoint",
    "safety_controller",
    "sensor",
    "sphere",
    "texture",
    "transmission",
    "use_simulated_gripper_joint",
    "verbose",
    "vertical",
    "visual",
]

reserved_element_attribute_names = [
    {
        "element": "actuator_transmission",
        "attributes": [
            "mechanicalReduction",
            "name",
        ],
    },
    {
        "element": "axis",
        "attributes": [
            "xyz",
        ],
    },
    {
        "element": "box",
        "attributes": [
            "size",
        ],
    },
    {
        "element": "calibration",
        "attributes": [
            "reference_position",
            "rising",
            "falling",
        ],
    },
    {
        "element": "child",
        "attributes": [
            "link",
        ],
    },
    {
        "element": "collision",
        "attributes": [
            "name",
        ],
    },
    {
        "element": "color",
        "attributes": [
            "rgba",
        ],
    },
    {
        "element": "cylinder",
        "attributes": [
            "length",
            "radius",
        ],
    },
    {
        "element": "dynamics",
        "attributes": [
            "damping",
            "friction",
        ],
    },
    {
        "element": "gap_joint_transmission",
        "attributes": [
            "L0",
            "a",
            "b",
            "gear_ratio",
            "h",
            "mechanical_reduction",
            "name",
            "phi0",
            "r",
            "screw_reduction",
            "t0",
            "theta0",
        ],
    },
    {
        "element": "image",
        "attributes": [
            "width",
            "height",
            "format",
            "hfov",
            "near",
            "far",
        ],
    },
    {
        "element": "inertia",
        "attributes": [
            "ixx",
            "ixy",
            "ixz",
            "iyy",
            "iyz",
            "izz",
        ],
    },
    {
        "element": "joint",
        "attributes": [
            "name",
            "type",
        ],
    },
    {
        "element": "LaserRay",
        "attributes": [
            "samples",
            "resolution",
            "min_angle",
            "max_angle",
        ],
    },
    {
        "element": "limit",
        "attributes": [
            "lower",
            "upper",
            "effort",
            "velocity",
        ],
    },
    {
        "element": "link",
        "attributes": [
            "name",
        ],
    },
    {
        "element": "mass",
        "attributes": [
            "value",
        ],
    },
    {
        "element": "material",
        "attributes": [
            "name",
        ],
    },
    {
        "element": "mesh",
        "attributes": [
            "filename",
            "scale",
        ],
    },
    {
        "element": "mimic",
        "attributes": [
            "joint",
            "multiplier",
            "offset",
        ],
    },
    {
        "element": "origin",
        "attributes": [
            "xyz",
            "rpy",
        ],
    },
    {
        "element": "parent",
        "attributes": [
            "link",
        ],
    },
    {
        "element": "passive_joint_transmission",
        "attributes": [
            "name",
        ],
    },
    {
        "element": "robot",
        "attributes": [
            "name",
        ],
    },
    {
        "element": "safety_controller",
        "attributes": [
            "soft_lower_limit",
            "soft_upper_limit",
            "k_position",
            "k_velocity",
        ],
    },
    {
        "element": "sensor",
        "attributes": [
            "name",
            "update_rate",
            "version",
        ],
    },
    {
        "element": "sphere",
        "attributes": [
            "radius",
        ],
    },
    {
        "element": "texture",
        "attributes": [
            "filename",
        ],
    },
    {
        "element": "transmission",
        "attributes": [
            "name",
            "type",
        ],
    },
    {
        "element": "verbose",
        "attributes": [
            "value",
        ],
    },
]


def check_element_name(element_name: str) -> bool:
    """
    Check if the element name is reserved.

    Args:
        element_name: The name of the element to check.

    Returns:
        True if the element name is reserved, False otherwise.
    """
    return element_name in reserved_element_names


def check_element_attribute_name(element_name: str, attribute_name: str) -> bool:
    """
    Check if the attribute name is reserved.

    Args:
        element_name: The name of the element to check.
        attribute_name: The name of the attribute to check.

    Returns:
        True if the attribute name is reserved, False otherwise.
    """
    for element in reserved_element_attribute_names:
        if element["element"] == element_name:
            return attribute_name in element["attributes"]
    return False
