# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# Elements defined in the URDF Schema.
#   https://raw.githubusercontent.com/ros/urdfdom/master/xsd/urdf.xsd

from typing import Any

from pxr import Gf

__all__ = [
    "ElementActuatorTransmission",
    "ElementAxis",
    "ElementBase",
    "ElementBox",
    "ElementCalibration",
    "ElementCamera",
    "ElementChild",
    "ElementCollision",
    "ElementColor",
    "ElementCylinder",
    "ElementDynamics",
    "ElementGapJointTransmission",
    "ElementGeometry",
    "ElementImage",
    "ElementInertia",
    "ElementInertial",
    "ElementJoint",
    "ElementLaserRay",
    "ElementLimit",
    "ElementLink",
    "ElementMass",
    "ElementMaterial",
    "ElementMaterialGlobal",
    "ElementMesh",
    "ElementMimic",
    "ElementName",
    "ElementParent",
    "ElementPassiveJointTransmission",
    "ElementPose",
    "ElementRay",
    "ElementRobot",
    "ElementSafetyController",
    "ElementSensor",
    "ElementSphere",
    "ElementTexture",
    "ElementTransmission",
    "ElementUndefined",
    "ElementVerbose",
    "ElementVisual",
]


class ElementBase:
    # Allowed tags for parent elements.
    allowed_parent_tags: list[str] = []

    # Available tag names.
    available_tag_names: list[str] = []

    # Default values.
    _defaults: dict[str, Any] = {}

    def __init__(self):
        # Tag name.
        self.tag: str = None

        # Expressed based on the XML tag hierarchy, such as "/robot/link/visual"
        self.path: str = None

        # Line numbers in XML.
        self.line_number: int = None

        # Undefined attributes.
        self.undefined_attributes: dict[str, str] = {}

        # Undefined elements.
        self.undefined_elements: list[ElementUndefined] = []

    def get_with_default(self, attr_name: str) -> Any:
        """Get the value of the attribute with the default value."""
        value = getattr(self, attr_name, None)
        if value is None and attr_name in self.__class__._defaults:
            return self.__class__._defaults[attr_name]
        return value


class ElementUndefined(ElementBase):
    # This is an undefined element.
    def __init__(self):
        super().__init__()


class ElementPose(ElementBase):
    allowed_parent_tags = ["inertial", "visual", "collision", "joint", "sensor"]
    available_tag_names = ["origin"]

    _defaults = {
        "xyz": Gf.Vec3d(0.0, 0.0, 0.0),
        "rpy": Gf.Vec3d(0.0, 0.0, 0.0),
    }

    def __init__(self):
        super().__init__()

        # attributes.
        self.xyz: Gf.Vec3d | None = None
        self.rpy: Gf.Vec3d | None = None


class ElementColor(ElementBase):
    allowed_parent_tags = ["material"]
    available_tag_names = ["color"]

    _defaults = {
        "rgba": Gf.Vec4d(0.0, 0.0, 0.0, 0.0),
    }

    def __init__(self):
        super().__init__()

        # attributes.
        self.rgba: Gf.Vec4d | None = None


class ElementVerbose(ElementBase):
    allowed_parent_tags = ["collision"]
    available_tag_names = ["verbose"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.value: str | None = None


class ElementName(ElementBase):
    allowed_parent_tags = ["transmission"]
    available_tag_names = ["actuator", "joint"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.name: str | None = None


class ElementMass(ElementBase):
    allowed_parent_tags = ["inertial"]
    available_tag_names = ["mass"]

    _defaults = {
        "mass": 0.0,
    }

    def __init__(self):
        super().__init__()

        # attributes.
        self.mass: float | None = None


class ElementInertia(ElementBase):
    allowed_parent_tags = ["inertial"]
    available_tag_names = ["inertia"]

    _defaults = {
        "ixx": 0.0,
        "iyy": 0.0,
        "izz": 0.0,
        "ixy": 0.0,
        "ixz": 0.0,
        "iyz": 0.0,
    }

    def __init__(self):
        super().__init__()

        # attributes.
        self.ixx: float | None = None
        self.iyy: float | None = None
        self.izz: float | None = None
        self.ixy: float | None = None
        self.ixz: float | None = None
        self.iyz: float | None = None


class ElementInertial(ElementBase):
    allowed_parent_tags = ["link"]
    available_tag_names = ["inertial"]

    _defaults = {
        "mass": 0.0,
    }

    def __init__(self):
        super().__init__()

        # elements.
        self.origin: ElementPose | None = None
        self.mass: ElementMass | None = None
        self.inertia: ElementInertia | None = None


class ElementBox(ElementBase):
    allowed_parent_tags = ["geometry"]
    available_tag_names = ["box"]

    _defaults = {
        "size": Gf.Vec3d(0.0, 0.0, 0.0),
    }

    def __init__(self):
        super().__init__()

        # attributes.
        self.size: Gf.Vec3d | None = None


class ElementCylinder(ElementBase):
    allowed_parent_tags = ["geometry"]
    available_tag_names = ["cylinder"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.radius: float = None
        self.length: float = None


class ElementSphere(ElementBase):
    allowed_parent_tags = ["geometry"]
    available_tag_names = ["sphere"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.radius: float = None


class ElementMesh(ElementBase):
    allowed_parent_tags = ["geometry"]
    available_tag_names = ["mesh"]

    _defaults = {
        "scale": Gf.Vec3d(1.0, 1.0, 1.0),
    }

    def __init__(self):
        super().__init__()

        # attributes.
        self.filename: str = None
        self.scale: Gf.Vec3d | None = None


class ElementGeometry(ElementBase):
    allowed_parent_tags = ["visual", "collision"]
    available_tag_names = ["geometry"]

    def __init__(self):
        super().__init__()

        # elements.
        self.geometry: ElementBox | ElementSphere | ElementCylinder | ElementMesh = None


class ElementTexture(ElementBase):
    allowed_parent_tags = ["material"]
    available_tag_names = ["texture"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.filename: str = None


class ElementMaterial(ElementBase):
    allowed_parent_tags = ["visual"]
    available_tag_names = ["material"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.name: str | None = None

        # elements.
        self.color: ElementColor | None = None
        self.texture: ElementTexture | None = None


class ElementMaterialGlobal(ElementBase):
    allowed_parent_tags = ["robot"]
    available_tag_names = ["material"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.name: str = None  # Required for global materials.

        # elements.
        self.color: ElementColor | None = None
        self.texture: ElementTexture | None = None


class ElementVisual(ElementBase):
    allowed_parent_tags = ["link"]
    available_tag_names = ["visual"]

    def __init__(self):
        super().__init__()

        # elements.
        self.origin: ElementPose | None = None
        self.geometry: ElementGeometry = None
        self.material: ElementMaterial | None = None


class ElementCollision(ElementBase):
    allowed_parent_tags = ["link"]
    available_tag_names = ["collision"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.name: str | None = None

        # elements.
        self.origin: ElementPose | None = None
        self.geometry: ElementGeometry = None
        self.verbose: ElementVerbose | None = None


class ElementLink(ElementBase):
    allowed_parent_tags = ["robot"]
    available_tag_names = ["link"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.name: str = None
        self.type: str = None

        # elements.
        self.inertial: ElementInertial | None = None
        self.visual: ElementVisual | None = None
        self.collision: ElementCollision | None = None


class ElementParent(ElementBase):
    allowed_parent_tags = ["joint", "sensor"]
    available_tag_names = ["parent"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.link: str = None


class ElementChild(ElementBase):
    allowed_parent_tags = ["joint"]
    available_tag_names = ["child"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.link: str = None


class ElementAxis(ElementBase):
    allowed_parent_tags = ["joint"]
    available_tag_names = ["axis"]

    _defaults = {
        "xyz": Gf.Vec3d(1.0, 0.0, 0.0),
    }

    def __init__(self):
        super().__init__()

        # attributes.
        self.xyz: Gf.Vec3d | None = None


class ElementCalibration(ElementBase):
    allowed_parent_tags = ["joint"]
    available_tag_names = ["calibration"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.reference_position: float | None = None
        self.rising: float | None = None
        self.falling: float | None = None


class ElementDynamics(ElementBase):
    allowed_parent_tags = ["joint"]
    available_tag_names = ["dynamics"]

    _defaults = {
        "damping": 0.0,
        "friction": 0.0,
    }

    def __init__(self):
        super().__init__()

        # attributes.
        self.damping: float | None = None
        self.friction: float | None = None


class ElementLimit(ElementBase):
    allowed_parent_tags = ["joint"]
    available_tag_names = ["limit"]

    _defaults = {
        "lower": 0.0,
        "upper": 0.0,
        "effort": 0.0,
        "velocity": 0.0,
    }

    def __init__(self):
        super().__init__()

        # attributes.
        self.lower: float | None = None
        self.upper: float | None = None
        self.effort: float | None = None
        self.velocity: float | None = None


class ElementSafetyController(ElementBase):
    allowed_parent_tags = ["joint"]
    available_tag_names = ["safety_controller"]

    _defaults = {
        "soft_lower_limit": 0.0,
        "soft_upper_limit": 0.0,
        "k_position": 0.0,
    }

    def __init__(self):
        super().__init__()

        # attributes.
        self.soft_lower_limit: float | None = None
        self.soft_upper_limit: float | None = None
        self.k_position: float | None = None
        self.k_velocity: float = None


class ElementMimic(ElementBase):
    allowed_parent_tags = ["joint"]
    available_tag_names = ["mimic"]

    _defaults = {
        "multiplier": 1.0,
        "offset": 0.0,
    }

    def __init__(self):
        super().__init__()

        # attributes.
        self.joint: str = None
        self.multiplier: float | None = None
        self.offset: float | None = None


class ElementActuatorTransmission(ElementBase):
    allowed_parent_tags = ["transmission"]
    available_tag_names = ["leftActuator", "rightActuator", "flexJoint", "rollJoint"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.mechanicalReduction: float = None
        self.name: str = None


class ElementGapJointTransmission(ElementBase):
    allowed_parent_tags = ["transmission"]
    available_tag_names = ["gap_joint"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.L0: float = None
        self.a: float = None
        self.b: float = None
        self.gear_ratio: float = None
        self.h: float = None
        self.mechanical_reduction: float = None
        self.name: str = None
        self.phi0: float = None
        self.r: float = None
        self.screw_reduction: float = None
        self.t0: float = None
        self.theta0: float = None


class ElementPassiveJointTransmission(ElementBase):
    allowed_parent_tags = ["transmission"]
    available_tag_names = ["passive_joint"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.name: str = None


class ElementTransmission(ElementBase):
    allowed_parent_tags = ["robot"]
    available_tag_names = ["transmission"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.name: str = None
        self.type: str = None

        # elements.
        self.actuator: ElementName | None = None
        self.joint: ElementName | None = None
        self.leftActuator: ElementActuatorTransmission | None = None
        self.rightActuator: ElementActuatorTransmission | None = None
        self.flexJoint: ElementActuatorTransmission | None = None
        self.rollJoint: ElementActuatorTransmission | None = None
        self.gap_joint: ElementGapJointTransmission | None = None
        self.passive_joint: ElementPassiveJointTransmission | None = None
        self.mechanicalReduction: float | None = None


class ElementImage(ElementBase):
    allowed_parent_tags = ["robot"]
    available_tag_names = ["image"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.width: int = None
        self.height: int = None
        self.format: str = None
        self.hfov: float = None
        self.near: float = None
        self.far: float = None


class ElementCamera(ElementBase):
    allowed_parent_tags = ["sensor"]
    available_tag_names = ["camera"]

    def __init__(self):
        super().__init__()

        # elements.
        self.image: ElementImage | None = None


class ElementLaserRay(ElementBase):
    allowed_parent_tags = ["ray"]
    available_tag_names = ["horizontal", "vertical"]

    _defaults = {
        "samples": 1,
        "resolution": 1,
        "min_angle": 0.0,
        "max_angle": 0.0,
    }

    def __init__(self):
        super().__init__()

        # attributes.
        self.samples: int | None = None
        self.resolution: int | None = None
        self.min_angle: float | None = None
        self.max_angle: float | None = None


class ElementRay(ElementBase):
    allowed_parent_tags = ["sensor"]
    available_tag_names = ["ray"]

    def __init__(self):
        super().__init__()

        # elements.
        self.horizontal: ElementLaserRay | None = None
        self.vertical: ElementLaserRay | None = None


class ElementSensor(ElementBase):
    allowed_parent_tags = ["robot"]
    available_tag_names = ["sensor"]

    _defaults = {
        "version": "1.0",
    }

    def __init__(self):
        super().__init__()

        # attributes.
        self.name: str = None
        self.version: str | None = None
        self.update_rate: float | None = None

        # elements.
        self.origin: ElementPose | None = None
        self.parent: ElementParent = None
        self.camera_ray: ElementCamera | ElementRay | None = None


class ElementJoint(ElementBase):
    allowed_parent_tags = ["robot"]
    available_tag_names = ["joint"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.name: str = None
        self.type: str = None

        # elements.
        self.origin: ElementPose | None = None
        self.parent: ElementParent = None
        self.child: ElementChild = None
        self.axis: ElementAxis | None = None
        self.calibration: ElementCalibration | None = None
        self.dynamics: ElementDynamics | None = None
        self.limit: ElementLimit | None = None
        self.safety_controller: ElementSafetyController | None = None
        self.mimic: ElementMimic | None = None


class ElementRobot(ElementBase):
    available_tag_names = ["robot"]

    _defaults = {
        "version": "1.0",
    }

    def __init__(self):
        super().__init__()

        # attributes.
        self.name: str = None
        self.version: str | None = None

        # elements.
        self.links: list[ElementLink] = []
        self.materials: list[ElementMaterialGlobal] = []
        self.joints: list[ElementJoint] = []
        self.transmissions: list[ElementTransmission] = []
