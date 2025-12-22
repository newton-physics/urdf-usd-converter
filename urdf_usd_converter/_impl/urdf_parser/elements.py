# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0

# Elements defined in the URDF Schema.
#   https://raw.githubusercontent.com/ros/urdfdom/master/xsd/urdf.xsd
#
# The Transmission and Sensor refer to the following specifications.
#   https://wiki.ros.org/urdf/XML

from typing import Any, ClassVar

__all__ = [
    "ElementAxis",
    "ElementBase",
    "ElementBox",
    "ElementCalibration",
    "ElementChild",
    "ElementCollision",
    "ElementColor",
    "ElementCylinder",
    "ElementDynamics",
    "ElementGeometry",
    "ElementInertia",
    "ElementInertial",
    "ElementJoint",
    "ElementLimit",
    "ElementLink",
    "ElementMass",
    "ElementMaterial",
    "ElementMaterialGlobal",
    "ElementMesh",
    "ElementMimic",
    "ElementParent",
    "ElementPose",
    "ElementRobot",
    "ElementSafetyController",
    "ElementSphere",
    "ElementTexture",
    "ElementUndefined",
    "ElementVerbose",
    "ElementVisual",
]


class ElementBase:
    # Allowed tags for parent elements.
    allowed_parent_tags: ClassVar[list[str]] = []

    # Available tag names.
    available_tag_names: ClassVar[list[str]] = []

    # Default values.
    _defaults: ClassVar[dict[str, Any]] = {}

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

        # Undefined text.
        self.undefined_text: str = None

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
    allowed_parent_tags: ClassVar[list[str]] = ["inertial", "visual", "collision", "joint", "sensor"]
    available_tag_names: ClassVar[list[str]] = ["origin"]

    _defaults: ClassVar[dict[str, Any]] = {
        "xyz": (0.0, 0.0, 0.0),
        "rpy": (0.0, 0.0, 0.0),
    }

    def __init__(self):
        super().__init__()

        # attributes.
        self.xyz: tuple[float, float, float] | None = None
        self.rpy: tuple[float, float, float] | None = None


class ElementColor(ElementBase):
    allowed_parent_tags: ClassVar[list[str]] = ["material"]
    available_tag_names: ClassVar[list[str]] = ["color"]

    _defaults: ClassVar[dict[str, Any]] = {
        "rgba": (1.0, 1.0, 1.0, 1.0),
    }

    def __init__(self):
        super().__init__()

        # attributes.
        self.rgba: tuple[float, float, float, float] | None = None


class ElementVerbose(ElementBase):
    allowed_parent_tags: ClassVar[list[str]] = ["collision"]
    available_tag_names: ClassVar[list[str]] = ["verbose"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.value: str | None = None


class ElementMass(ElementBase):
    allowed_parent_tags: ClassVar[list[str]] = ["inertial"]
    available_tag_names: ClassVar[list[str]] = ["mass"]

    _defaults: ClassVar[dict[str, Any]] = {
        "value": 0.0,
    }

    def __init__(self):
        super().__init__()

        # attributes.
        self.value: float | None = None


class ElementInertia(ElementBase):
    allowed_parent_tags: ClassVar[list[str]] = ["inertial"]
    available_tag_names: ClassVar[list[str]] = ["inertia"]

    _defaults: ClassVar[dict[str, Any]] = {
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
    allowed_parent_tags: ClassVar[list[str]] = ["link"]
    available_tag_names: ClassVar[list[str]] = ["inertial"]

    _defaults: ClassVar[dict[str, Any]] = {
        "mass": 0.0,
    }

    def __init__(self):
        super().__init__()

        # elements.
        self.origin: ElementPose | None = None
        self.mass: ElementMass | None = None
        self.inertia: ElementInertia | None = None


class ElementBox(ElementBase):
    allowed_parent_tags: ClassVar[list[str]] = ["geometry"]
    available_tag_names: ClassVar[list[str]] = ["box"]

    _defaults: ClassVar[dict[str, Any]] = {
        "size": (0.0, 0.0, 0.0),
    }

    def __init__(self):
        super().__init__()

        # attributes.
        self.size: tuple[float, float, float] | None = None


class ElementCylinder(ElementBase):
    allowed_parent_tags: ClassVar[list[str]] = ["geometry"]
    available_tag_names: ClassVar[list[str]] = ["cylinder"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.radius: float = None
        self.length: float = None


class ElementSphere(ElementBase):
    allowed_parent_tags: ClassVar[list[str]] = ["geometry"]
    available_tag_names: ClassVar[list[str]] = ["sphere"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.radius: float = None


class ElementMesh(ElementBase):
    allowed_parent_tags: ClassVar[list[str]] = ["geometry"]
    available_tag_names: ClassVar[list[str]] = ["mesh"]

    _defaults: ClassVar[dict[str, Any]] = {
        "scale": (1.0, 1.0, 1.0),
    }

    def __init__(self):
        super().__init__()

        # attributes.
        self.filename: str = None
        self.scale: tuple[float, float, float] | None = None


class ElementGeometry(ElementBase):
    allowed_parent_tags: ClassVar[list[str]] = ["visual", "collision"]
    available_tag_names: ClassVar[list[str]] = ["geometry"]
    available_geometry_types: ClassVar[list[str]] = ["box", "sphere", "cylinder", "mesh"]

    def __init__(self):
        super().__init__()

        # elements.
        self.shape: ElementBox | ElementSphere | ElementCylinder | ElementMesh = None


class ElementTexture(ElementBase):
    allowed_parent_tags: ClassVar[list[str]] = ["material"]
    available_tag_names: ClassVar[list[str]] = ["texture"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.filename: str = None


class ElementMaterial(ElementBase):
    allowed_parent_tags: ClassVar[list[str]] = ["visual"]
    available_tag_names: ClassVar[list[str]] = ["material"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.name: str | None = None

        # elements.
        self.color: ElementColor | None = None
        self.texture: ElementTexture | None = None


class ElementMaterialGlobal(ElementBase):
    allowed_parent_tags: ClassVar[list[str]] = ["robot"]
    available_tag_names: ClassVar[list[str]] = ["material"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.name: str = None  # Required for global materials.

        # elements.
        self.color: ElementColor | None = None
        self.texture: ElementTexture | None = None


class ElementVisual(ElementBase):
    allowed_parent_tags: ClassVar[list[str]] = ["link"]
    available_tag_names: ClassVar[list[str]] = ["visual"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.name: str | None = None

        # elements.
        self.origin: ElementPose | None = None
        self.geometry: ElementGeometry = None
        self.material: ElementMaterial | None = None

    def has_mesh_filename(self) -> bool:
        return (
            self.geometry
            and self.geometry.shape
            and isinstance(self.geometry.shape, ElementMesh)
            and hasattr(self.geometry.shape, "filename")
            and self.geometry.shape.filename
        )


class ElementCollision(ElementBase):
    allowed_parent_tags: ClassVar[list[str]] = ["link"]
    available_tag_names: ClassVar[list[str]] = ["collision"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.name: str | None = None

        # elements.
        self.origin: ElementPose | None = None
        self.geometry: ElementGeometry = None
        self.verbose: ElementVerbose | None = None

    def has_mesh_filename(self) -> bool:
        return (
            self.geometry
            and self.geometry.shape
            and isinstance(self.geometry.shape, ElementMesh)
            and hasattr(self.geometry.shape, "filename")
            and self.geometry.shape.filename
        )


class ElementLink(ElementBase):
    allowed_parent_tags: ClassVar[list[str]] = ["robot"]
    available_tag_names: ClassVar[list[str]] = ["link"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.name: str = None
        self.type: str = None

        # elements.
        self.inertial: ElementInertial | None = None
        self.visuals: list[ElementVisual] = []
        self.collisions: list[ElementCollision] = []


class ElementParent(ElementBase):
    allowed_parent_tags: ClassVar[list[str]] = ["joint", "sensor"]
    available_tag_names: ClassVar[list[str]] = ["parent"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.link: str = None


class ElementChild(ElementBase):
    allowed_parent_tags: ClassVar[list[str]] = ["joint"]
    available_tag_names: ClassVar[list[str]] = ["child"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.link: str = None


class ElementAxis(ElementBase):
    allowed_parent_tags: ClassVar[list[str]] = ["joint"]
    available_tag_names: ClassVar[list[str]] = ["axis"]

    _defaults: ClassVar[dict[str, Any]] = {
        "xyz": (1.0, 0.0, 0.0),
    }

    def __init__(self):
        super().__init__()

        # attributes.
        self.xyz: tuple[float, float, float] | None = None


class ElementCalibration(ElementBase):
    allowed_parent_tags: ClassVar[list[str]] = ["joint"]
    available_tag_names: ClassVar[list[str]] = ["calibration"]

    def __init__(self):
        super().__init__()

        # attributes.
        self.reference_position: float | None = None
        self.rising: float | None = None
        self.falling: float | None = None


class ElementDynamics(ElementBase):
    allowed_parent_tags: ClassVar[list[str]] = ["joint"]
    available_tag_names: ClassVar[list[str]] = ["dynamics"]

    _defaults: ClassVar[dict[str, Any]] = {
        "damping": 0.0,
        "friction": 0.0,
    }

    def __init__(self):
        super().__init__()

        # attributes.
        self.damping: float | None = None
        self.friction: float | None = None


class ElementLimit(ElementBase):
    allowed_parent_tags: ClassVar[list[str]] = ["joint"]
    available_tag_names: ClassVar[list[str]] = ["limit"]

    _defaults: ClassVar[dict[str, Any]] = {
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
    allowed_parent_tags: ClassVar[list[str]] = ["joint"]
    available_tag_names: ClassVar[list[str]] = ["safety_controller"]

    _defaults: ClassVar[dict[str, Any]] = {
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
    allowed_parent_tags: ClassVar[list[str]] = ["joint"]
    available_tag_names: ClassVar[list[str]] = ["mimic"]

    _defaults: ClassVar[dict[str, Any]] = {
        "multiplier": 1.0,
        "offset": 0.0,
    }

    def __init__(self):
        super().__init__()

        # attributes.
        self.joint: str = None
        self.multiplier: float | None = None
        self.offset: float | None = None


class ElementJoint(ElementBase):
    allowed_parent_tags: ClassVar[list[str]] = ["robot"]
    available_tag_names: ClassVar[list[str]] = ["joint"]
    available_joint_types: ClassVar[list[str]] = ["revolute", "continuous", "prismatic", "fixed", "floating", "planar"]

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
    available_tag_names: ClassVar[list[str]] = ["robot"]

    _defaults: ClassVar[dict[str, Any]] = {
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
