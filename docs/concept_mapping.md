# URDF to USD Conceptual Data Mapping

## Introduction

### Overview

URDF (Unified Robot Description Format) is an XML-based format for describing the structure and physical properties of a robot. It was initially developed for ROS (Robot Operating System) and is the standard way to define robot models on ROS. It has widespread adoption beyond ROS, and is supported by many simulators, including Newton, MuJoCo, Isaac Lab/Sim, PyBullet, and many more.

The Robot Operating System (ROS) is a set of open-source software libraries and tools for building robot applications. Although ROS is written as an 'Operating System', it is different from the OS of a general computer. ROS is a kind of middleware or framework, and is described as a 'Meta Operating System'. ROS1 was primarily used for research and academic purposes with a single robot. ROS2 supports multiple robots, allowing for more general-purpose support on an embedded scale.

OpenUSD is a system for authoring, composing, and reading hierarchically organized scene description that scalably encode and interchange static and time-sampled 3D data between Digital Content Creation (DCC) applications. OpenUSD provides powerful mechanisms for large-scale collaboration and context-dependent asset refinement within content pipelines.

To facilitate a shared understanding between subject matter experts of these communities, we provide a mapping of data models between URDF and OpenUSD, identify concept gaps, and provide recommendations to developers building URDF/USD interchange solutions.

### ROS2/URDF References

| Version | Reference Documents |
| :---- | :---- |
| ROS2 | [ROS Humble Docs](https://docs.ros.org/en/humble/index.html) |
| URDF 2.13.0 | [ROS URDF Docs](https://wiki.ros.org/urdf/), [ROS2 URDF Packages](https://github.com/ros2/urdf/tree/2.13.0) [URDF XML Schema](https://raw.githubusercontent.com/ros/urdfdom/master/xsd/urdf.xsd), [MathWorks URDF Guide](https://www.mathworks.com/help/sm/ug/urdf-model-import.html) |

### OpenUSD Reference

| Version | Reference Documents |
| :---- | :---- |
| 25.02 | [OpenUSD API Docs](https://openusd.org/release/api/index.html), [USD Terms and Concepts](https://openusd.org/release/glossary.html), [Github](https://github.com/PixarAnimationStudios/OpenUSD/tree/v25.02a), [Principals of Scalable Asset Structure](https://docs.omniverse.nvidia.com/usd/latest/learn-openusd/independent/asset-structure-principles.html) |

### General Assumptions and Constraints

This URDF to OpenUSD data mapping describes the one-way conversion from URDF to USD. The expectation is that the majority of consumers interested in URDF/USD interchange prefer to migrate URDF robots to USD and continue iterating within the USD Ecosystem, rather than roundtrip back to ROS.

### Definitions, Acronyms, Abbreviations

| Term or Abbreviation | Description |
| :---- | :---- |
| ROS | Robot Operating System; A set of software libraries and tools for building robots. Currently ROS2 is used. |
| ROS2 Package | A collection of data for a single ROS2 project. URDF, mesh files and image files are also included in the package. |
| rviz2 | ROS2 3D visualization tool |
| Gazebo | An open source 3D robot simulator |
| URDF | Unified Robot Description Format; An XML-based format for describing the structure and physical properties of a robot. The file extension is 'urdf'. |
| DCC | Digital Content Creation application; for interactive 3D authoring, visualization, animation, simulation, or rendering workflows. |
| Simulator | Software for physical simulation of 3D content. |
| Renderer | Software for visualization of 3D content. |
| Content Pipeline | Automated or semi-automated chain of processes for digital content ingestion, transformation, and/or authoring. |
| USD Ecosystem | The set of DCCs, simulators, renderers, and content pipelines which offer native or plugin-based USD interchange. |
| USD | Shorthand for OpenUSD; both the interchange specification & APIs. |
|  OBJ | A simple data-format that represents 3D geometry as plain text |
|  STL | A simple data-format that describes a raw, unstructured triangulated surface. There is plane text and binary. |
| DAE | dae format; COLLADA XML scene format. |
| Sensor | A device that understands its surroundings and obtains the information it needs to operate appropriately. |
| Actuator | A device that uses energy such as electricity, hydraulics, or air pressure to create mechanical movement (e.g a motor) |
| rpy | Roll(x) / Pitch(y) / Yaw(z) |
| Composition | USD process of resolving layered opinions about the content into a definitive representation called a “stage”. The composed stage is not optimized for any runtime, but rather for navigability of the data. |
| Asset | Data organization concept within content pipelines; a set of data that can be identified and located; e.g. each robot is an asset, each texture file is an asset. |
| Component | An atomic asset/model representing one high-level element (e.g. prop, actor) in a 3D scene. |

## Data and Serialization Concepts

The following table describes the main data structure & serialization concepts from URDF and maps them to USD concepts where possible.

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| [URDF](#urdf--usda--usdc) | USDA / USDC | Serialized source content describing the robot in 3D space & physics default state |
| [OBJ / STL / DAE](#reference-mesh-files) | USDC | Serialized mesh topology |
|  | [SdfLayer / UsdStage](#sdflayer--usdstage) | In-memory representation of the content |
| PNG / JPG | PNG / JPG / EXR (and others) | Image files referenced as textures to be rendered on meshes |

### URDF / USDA / USDC

URDF is an XML specification, and is defined formally as an [XML Schema](https://raw.githubusercontent.com/ros/urdfdom/master/xsd/urdf.xsd). It is designed to be human readable & easily editable. The analogous serialization in USD is a text-encoded `SdfLayer`, which can be serialized to either `.usda` or `.usd` files. Both of these formats offer legibility & modularity, at the cost of performance.

If performance outweighs legibility, the `SdfLayer` can instead be binary-encoded as a "crate" file, which can be serialized to either `.usdc` or `.usd` files.

Note that while `.usd` files can be used for either ascii or binary encoding, the more explicit identifiers `.usda` and `.usdc` should be preferred for legibility.

When converting URDF to USD, crate files should be preferred for large array data (e.g. meshes), which can be referenced into USDA layers which apply 3D transformation & physics.

### Reference Mesh Files

URDFs externally reference other file formats which store triangulated meshes. Any geometry format is allowed, but specific application compatibility is dependent on implementation. Therefore it is recommended to stick to basic formats like OBJ, STL, and DAE (Collada) for the broadest compatibility.

URDF assumes that all geometry in the referenced file represents a single geometric unit. See [mesh](#element-mesh) for more details.

Any visual materials in the referenced file (including MTL sidecar files in the case of OBJ) should be processed & converted to UsdShade equivalents. See [material](#material) for more details.

Any other information in the referenced files should be discarded.

### SdfLayer / UsdStage

There is no canonical runtime representation of a URDF XML. Different simulators typically parse URDF from XML directly and translate it into their own runtime structures. It is recommended to take the same approach with USD. Well supported XML parsers exist in most programming languages & it is trivial to parse the file into an in-memory element tree, which can then be converted to an in-memory USD structure.

`SdfLayer` is the USD runtime representation of a parsed USDA/USDC file, however the `SdfLayer` still needs to be [composed](https://openusd.org/release/glossary.html#composition) into a UsdStage before it can be meaningfully inspected using higher level APIs. This is because the `SdfLayer` only forms "opinions" about the content and the process of composition may alter the results.

Both `SdfLayer` and `UsdStage` can represent time varying data as well as static data. When mapping URDF to USD there should be no need for authoring time sampled data. It is important to author such USD data using the “default time”, to indicate that it is time-independent, as well as to annotate certain attributes as “uniform” to indicate they are not time varying.

See [Specification Concepts](#specification-concepts) for a detailed mapping of each URDF element to an equivalent USD concept.

### Units

URDF is specified in fixed units. In USD some units are configurable. The layer & stage have top level metrics which inform the overall scale of the content. It is important to set these metrics, so that when assembling larger datasets different layers can be correctively scaled accordingly. Similarly, it is important to transform the URDF data to the expected USD units.

| Element | Unit (URDF) | OpenUSD | Configurable? |
| :---- | :---- | :---- | :---- |
| [Up Axis](#up-axis) | Z | Y | Yes |
| [Length](#linear-units) | meters (m) | centimeters (cm) | Yes  |
| [Mass](#mass-units) | kilograms (kg) | kilograms (kg) | Yes  |
| [Angle](#angles) | radians (rad) | degrees | No |
| Time | seconds (s) | seconds (s) | No |
| Force | Newton (N \= kg\*m/s/s) | N \= kg\*[DIST\_UNITS](#linear-units)/s/s | No |
| Torque | Newton metres (Nm \= kg\*m\*m/s/s) | Nm \= kg\*DIST\_UNITS\*DIST\_UNITS/s/s | No |
| Velocity | rad/s m/s | degrees/s DIST\_UNITS/s | No |

#### Up Axis
URDF uses a right-handed coordinate system with the Z up axis.  
USD also uses a right-handed coordinate system, but the Up Axis is configurable. Either Y or Z are supported.

To ease data conversion it is recommended to always use Z, by authoring `upAxis = “Z”` in the layer metadata.

#### Linear Units
The linear metrics in USD are configurable and the default values are centimeters. To ease data conversion it is recommended to use meters instead, by authoring `metersPerUnit = 1.0` in the layer metadata.

In some of the formulas above, DIST\_UNITS is a variable that varies via `metersPerUnit`. When DIST\_UNITS is meters, the units of Force and Torque in the URDF and USD will match and no data transformation is required.

#### Mass Units

The mass metrics in both URDF and USD default to kilograms, but for clarity it is recommended to explicitly specify mass units, by authoring `kilogramsPerUnit = 1.0` in the layer metadata.

#### Angles

In USD all angle attributes are specified in degrees. While this may be inconvenient for simulation contexts, it stems from OpenUSD’s origins as a graphics & animation format.

When converting URDF data to USD it is always required to convert radians to degrees. Note however, in the case of 3D transformation, USD does support quaternions as well as matrices, so using degrees is not strictly required to orient geometry in space.

## Specification Concepts

The following table describes concept mappings between URDF and USD. All URDF concepts are listed, with gaps identified as “***GAP***” in the USD column. Many other USD concepts are excluded as they aren’t relevant for a one-way mapping.

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| [robot](#robot) | `assetInfo.name`,<br>`UsdGeomXform` (defaultPrim) | Root of the asset / dataset |
| [link](#link) | `UsdGeomXform`,<br>`UsdPhysicsRigidBodyAPI` | A rigid body within the robot |
| [link/inertial](#linkinertial) | `UsdPhysicsMassAPI` | Explicit mass and inertial properties |
| [link/visual](#linkvisual) | Various `UsdGeomGPrims`,<br>`UsdReference` (for meshes) | Defines the appearance of the link |
| [link/collision](#linkcollision) | Various `UsdGeomGPrims`,<br>`UsdReference` (for meshes),<br>`UsdPhysicsCollisionAPI`,<br>`UsdPhysicsMeshCollisionAPI` | Defines collision geometry & physical properties of the link |
| [geometry](#geometry) | Various `UsdGeomGPrims`,<br>`UsdReference` (for meshes) | Defines the geometry for visuals and collisions |
| [material](#material)  | `UsdShadeMaterial`,<br>`UsdShadeShaders`,<br>***GAP*** (projection shaders) | Defines the rendered appearance of the link (not the physical properties) |
| [joint](#joint) | Various `UsdPhysicsJoints`,<br>`UsdGeomXformOps` (for the child link),<br>***GAP*** (calibration, friction, damping, soft limits, mimic) | A joint for connecting two links as well as 3D transformation for the child link. |
| [transmission](#transmission) | N/A<br>(could be `UsdPhysicsDriveAPI` if it was fully specified) | Defines the mechanical transmission mechanism between actuators and joints, but is not well specified in URDF and therefore cannot map to USD. See [Custom Elements](#custom-elements). |
| [gazebo](#gazebo) | N/A | URDF extensions specific to the Gazebo simulator. Not a generalizable URDF element. See [Custom Elements](#custom-elements). |
| sensor (deprecated) | N/A  | Implemented in URDF Dom but unsupported & unmaintained. See [urdf/XML/sensor](https://wiki.ros.org/urdf/XML/sensor) for details. See [Custom Elements](#custom-elements). |

### robot

The robot element in URDF is the top level XML element and defines the name of the robot, which is important to capture for legibility & navigability within a content pipeline.

In USD, this name should be authored as the "name" property of the [AssetInfo](https://openusd.org/release/glossary.html#usdglossary-assetinfo) on the "default prim" of an `SdfLayer`.

It is recommended to author the Asset Info dictionary in an “Asset Interface” layer (see [Appendix D: USD Asset Structure](#appendix-d-usd-asset-structure)). In some content pipelines it is also common to use this name for the default prim, as well as the stem of the serialized USDA/C file. See [Appendix E](#appendix-e-common-attributes--elements) for more naming considerations if using this data for the default prim name.

### link

A [link](https://wiki.ros.org/urdf/XML/link) defines an independent rigid part within a robot. Therefore they must be part of a UsdPhysics articulation, i.e. they must be an `Xformable` Prim and have an applied `UsdPhysicsRigidBodyAPI` (see [Appendix B](#appendix-b-usd-schemas) for an explanation of applied API schemas).

However, the link itself does not define any of the properties necessary to describe the prim aside from a name. To fully describe the link in USD we need to consider the optional [inertial](#linkinertial) child element as well as any joints that target the link as a child. For this reason, it is recommended to map both the link and its inertial element as a single Xformable prim, with the XformOps determined as specified in [Link Hierarchy](#link-hierarchy).

Links may additionally define optional [visual](#linkvisual) and [collision](#linkcollision) child elements, both of which are fixed to the local frame & move along with it. In USD, this implies those fixed child elements must be mapped as child (or descendant) Prims of the link Prim to which they are fixed. Since USD forbids nesting `Boundables` inside `Gprims`, this further implies that a Link should be mapped as a `UsdGeomXform` Prim (rather than other types of Xformable prims).

For legibility within the model/asset it is recommended to place all links under a single container `Scope`, which is itself a child of the root/default Prim (e.g. /Robot/Geometry/Link1). See [Appendix D](#appendix-d-usd-asset-structure) for more information.

#### Links as Anchor Points

Additionally, it is common to find “anchor point” links within a robot, which are just coordinate frames that move relative to an actual rigid body link. These “anchor points” have no inertial, collider, or visual elements of their own, but are connected to a parent link via a fixed joint.

In USD, it is recommended to map "anchor points" as an Xformable (without `UsdPhysicsRigidBodyAPI` applied) and author it as a child of the parent link prim (as siblings of the parent’s visual & collider children). The fixed joint can be omitted as non-body prims are inherently fixed to their parents. Avoiding the additional `UsdPhysics` representation improves both stage traversal & parsing, as well as simulation performance and behavior.

Another common practice is to use a "world” link, which is the origin frame for the entire robot. This link is connected to the robot’s actual root link using either a fixed or floating joint, depending on whether the articulation is fixed to the world or moves freely. As with other “anchor points”, this can be omitted in USD and should instead become the defaultPrim of the robot asset. In the case of a fixed articulation, the `UsdPhysicsFixedJoint` is required, and the body0 relationship should target the defaultPrim.

#### Link Attributes

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| [name](#attribute-name) | `SdfPath` | In USD the name is accessed via the Stage or Layer, it is not a Prim property |

#### Link Hierarchy

In URDF, there must be a single kinematic tree hierarchy, with a single root link. The parent/child relationships of links are specified by the joints. The position and orientation of each link (the link frame) are only specified via the [joint/origin](#element-origin-1) after determining the kinematic hierarchy.

In USD, the link (body) frame must be specified on the body prim itself, using XformOps (see [Appendix E](#element-origin-2)). There are two possible approaches, both require analyzing the [joint](#joint) elements first, to determine the correct kinematic hierarchy.

The first link in the URDF file should be considered the root of the robot (after accounting for the “world” link, see Links as Anchor Points above). This link should be marked as the root by applying `UsdPhysicsArticulationRootAPI`.

The next links must be determined by analyzing the joint connections. Upon recursive traversal, the joints that contain the current link as parent will contain the next link as the child. This traversal can be either Breadth-first-search or Depth-first-search. Implementations may choose to support either approach, but for consistency it is recommended to use depth-first-search for the default traversal.

Once this kinematic hierarchy is identified, the links can be authored in one of two ways:

###### *Nested Bodies*

The body prims representing each link can be nested. 

```
/Robot (Xform)
  /Geometry (Scope)
    /Link1 (Xform) (frame relative to Robot)
      /Link2 (Xform) (frame relative to Link1)
  /Physics (Scope)
    /Joint1 (Joint) (parent=Link1, child=Link2)
```

The advantage of this approach is that the kinematic hierarchy is explicitly specified, improving legibility. Additionally, the link frame values are specified as offsets from the parent, matching the input URDF data from the [joint/origin](#element-origin-1) (albeit converted to degrees or quaternions).

The primary disadvantage is that some USD Ecosystem products do not support nesting of rigid bodies, as it was only recently accepted into the standard.

###### *Flat Bodies*

The body prims representing each link can be stored as a flat list.

```
/Robot (Xform)
  /Geometry (Scope)
    /Link1 (Xform) (frame relative to Robot)
    /Link2 (Xform) (frame relative to Robot)
  /Physics (Scope)
    /Joint1 (Joint) (parent=Link1, child=Link2)
```

The advantage of this approach is compatibility throughout the USD Ecosystem, particularly with free-body simulators which are less likely to support body hierarchy.

The disadvantage is that the kinematic hierarchy remains obfuscated (though it is in URDF as well) and that the child body frame needs to be computed into world (robot) space (in addition to the unit transformation).

### link/inertial

The inertial element within a link defines the link’s mass, center of mass, and its central inertia properties. When not defined, it indicates zero mass and zero inertia.

In USD, this maps to the `UsdPhysicsMassAPI` schema applied to the link Prim, with properties set as described in the table below.

While both inertial & MassAPI are considered optional, the semantics of omission are different. Omitting MassAPI does not indicate zero mass in USD, it indicates that mass should be implicitly computed at runtime. Zero mass bodies are also considered invalid in USD. Given these differences, when a URDF link has no inertial child element, it is recommended to consider this an error case when converting to USD.

#### Inertial Elements

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| [origin](#element-origin) | `physics:centerOfMass`,<br>`physics:principalAxes` (see also [inertia](#element-inertia)) | Position and (non-aligned) orientation of the link's center of mass |
| [mass](#element-mass) | `physics:mass` | Explicit mass of the link in [mass units](#mass-units) |
| [inertia](#element-inertia) | `physics:principalAxes`,<br>`physics:diagonalInertia` | The 6 unique values of an Inertia Matrix |

##### Element: origin

The origin of an inertial element is the position and orientation of the link’s center of mass relative to the link itself. Unlike geometry or joint origin, this origin represents physical behavior (as opposed to placement in 3D space). It is broken down into two properties:

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| [xyz](#property-xyz) | `physics:centerOfMass` | Position |
| [rpy](#property-rpy) | `physics:principalAxes` (see also [inertia](#element-inertia)) | Orientation of the principal axes of inertia, represented as roll, pitch, yaw Euler rotations in radians |

###### *Property: xyz*

The `origin.xyz` maps to `physics:centerOfMass` attribute of `UsdPhysicsMassAPI`, which should be applied in the link Prim.

###### *Property: rpy*

The `origin.rpy` of an inertial element is an additional rotation to the principal axes for the moment of inertia. They are stored in URDF as roll, pitch, yaw Euler rotations in radians.

This concept is not directly representable in UsdPhysics as a separate attribute, so it must be baked into the `physics:principalAxes` attribute. See [inertia](#element-inertia) for details on obtaining the aligned `physics:principalAxes`. Once the aligned axes are obtained, the rpy should be used to rotate them as follows:

1. Convert rpy from Euler rotation radians to a Quaternion  
2. `quatf orientedAxes = orientation * principalAxis  `
3. `physics:principalAxes = orientedAxes`

##### Element: mass

The mass element contains a single property called value. This maps to the `physics:mass` attribute of `UsdPhysicsMassAPI`, which should be applied in the link Prim.

##### Element: inertia

The inertia element contains 6 named properties, which can be used to construct a symmetric 3x3 inertia matrix, which describes how the mass is distributed around the center of mass. These are the principal moments of inertia (ixx, iyy, izz) and the products of inertia (ixy, ixz, iyz).

```
[ ixx  ixy  ixz ]
[ ixy  iyy  iyz ]
[ ixz  iyz  izz ]
```

In USD, this maps to `physics:principalAxes` & `physics:diagonalInertia`, so must be computed via eigenvalue decomposition.

### link/visual

The visual elements within a link define its visual appearance via geometry and materials. These have no impact on simulation and are only used for visualization/rendering.

While there can be multiple visuals within a link, the geometry of a visual is defined by the [visual/geometry](#element-geometry) child element, of which there can be only one.

In USD this maps to various subclasses of `UsdGeomGprim`, which are themselves Xformable. No intermediate `UsdGeomXform` Prim is required; the position and orientation can be encoded on the `Gprim` directly. Therefore each visual should be mapped as an appropriate `UsdGeomGprim` (or a `UsdReference` to one in the case of meshes).

#### Visual Attributes

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| [name](#attribute-name) | `SdfPath` | In USD the name is accessed via the Stage or Layer, it is not a Prim property |

#### Visual Elements

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| [origin](#element-origin-2) | `UsdGeomXformOp` (`TypeTranslate`, `TypeRotateXYZ`) | Local space position and orientation |
| [geometry](#element-geometry) | Various `UsdGeomGPrims`,<br>`UsdReference` (for meshes) | Visual geometry which does not affect simulation |
| [material](#element-material) | `UsdShadeMaterial`,<br>`UsdShadeShaders` | Shading information for the visual geometry |

##### Element: geometry

The geometry element within a visual is required and singular. It has no properties of its own and has a single child element.

Since each visual has only one geometry, there is no need to represent this element as a separate prim in USD. *The visual prim itself is the geometry*. The visual should be mapped as a `UsdGeomGprim` or a `UsdReference` to one. See [geometry](#geometry) for details on choosing the correct prim type.

##### Element: material

The visual/material element provides shading information for the visual geometry. It is allowed to be specified directly within the visual, or to refer to a [material](#material) element outside of the link element (within the top level [robot](#robot) element). If using a global robot material, from within a link element you can then reference the material by name, implying the names must be unique.

In USD, this maps to `UsdShadeMaterialBindingAPI`, which binds a predefined `UsdShadeMaterial` Prim to the target Prim. It is important to remember to apply the schema; in older USD runtimes this was not required, but in modern runtimes it is a strict requirement.

As material networks can get quite large, and are often re-used, it is common to store them in a dedicated library layer which can be a binary compressed crate (usdc) file, mark them as instanceable, and reference them into the main robot hierarchy.

Another important detail is that material bindings must not cross payload or instance boundaries in USD. If materials are authored to library layer in this way, it is recommended to author materials as a `UsdReference`, creating a local copy of the material within the boundary of the default prim (robot), under a dedicated Scope prim (e.g. /Robot/Materials/Mat1). The reference identifier for the material should be determined using the data structure associating the original URDF `material.name` with the resulting USD identifier.

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| [name](#property-name) | `UsdReference` | Named reference to a robot level material driving shading of the visual |

###### *Property: name*

The `material.name` property identifies a top level [material](#material) which applies to the visual & determines the visual properties of the rendered geometry.

### link/collision

The collision elements within a link define its simulated collision geometry. These are often simplified geometric models, which approximate the visuals, to reduce computation time.

While there can be multiple collisions within a link, the geometry of a collision is defined by the [geometry](#element-geometry-1) child element, of which there must be exactly one.

In USD this maps to various subclasses of `UsdGeomGprim`, which are themselves Xformable. No intermediate `UsdGeomXform` Prim is required; the position and orientation can be encoded on the Gprim directly. Therefore each collision should be mapped as an appropriate `UsdGeomGprim` (or a `UsdReference` to one in the case of meshes).

Collision geometry in USD should be additionally tagged using the "guide" [purpose](https://openusd.org/release/glossary.html#usdglossary-purpose) to indicate they are not visuals & need to be explicitly marked as physics colliders by applying the `UsdPhysicsCollisionAPI`.

#### Collision Attributes

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| [name](#attribute-name) | `SdfPath` | In USD the name is accessed via the Stage or Layer, it is not a Prim property |

#### Collision Elements

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| [origin](#element-origin-2) | `UsdGeomXformOp` (`TypeTranslate`, `TypeRotateXYZ`) | Local space position and orientation |
| [geometry](#element-geometry-1) | `UsdGeomCube`,<br>`UsdGeomCylinder`,<br>`UsdGeomSphere`,<br>`UsdGeomMesh` | Geometry information |

##### Element: geometry

The geometry element within a collision is required and singular. It has no properties of its own and has a single child element.

Since each collision has only one geometry, there is no need to represent this element as a separate prim in USD. *The collision prim itself is the geometry*. The collision should be mapped as a `UsdGeomGprim` or a `UsdReference` to one. See [geometry](#geometry) for details on choosing the correct prim type.

Note that if the geometry is a mesh, it should additionally have the `UsdPhysicsMeshCollisionAPI` applied with `physics:approximation = "convexHull"`.

### geometry

The geometry element has no attributes of its own. Instead it has a single required child element, which can be one of 4 types. The child element should determine the appropriate USD prim type for this geometry.

#### Geometry Elements

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| [box](#element-box) | `UsdGeomCube` | A rectangular prism |
| [cylinder](#element-cylinder) | `UsdGeomCylinder` | A cylinder |
| [sphere](#element-sphere) | `UsdGeomSphere` | A sphere |
| [mesh](#element-mesh) | `UsdGeomMesh`,<br>`UsdReference` | A triangulated mesh, described in an externally referenced file. |

##### Element: Box

The box element defines a rectangular prism, which can be scaled independently in all 3 axes.

In USD, this maps to a `UsdGeomCube` with an associated scale XformOp.

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| [size](#property-size) | `UsdGeomXformOp` (`TypeScale`) | Box size |

###### *Property: size*

In URDF the `box.size` specifies the edge lengths of each axis of a rectangular prism along the X, Y and Z axes of the geom’s frame.

In USD, the builtin size attribute of `UsdGeomCube` cannot be used, as it requires a scalar size affecting all dimensions. The size should instead map to an `XformOp`. While newer USD runtimes support individual axis scale ops, this is not yet supported throughout the USD Ecosystem. Instead, use `TypeScale` to specify all 3 dimensions.

Note that `UsdGeomCube.size` = 2 by default, so either the URDF sizes need to be halved when scaling, or the Cube size should be explicitly authored to 1\. If authoring size, it is required to also author explicit extents.

##### Element: Cylinder

The cylinder element defines a basic cylinder primitive oriented along the Z axis of the frame.

In USD, this maps to a `UsdGeomCylinder` with `axis = "Z"`. Note that newer USD runtimes also have a `UsdGeomCylinder_1` schema, which allows tapering the caps, but this is not yet widely supported in the USD Ecosytem & also unnecessary for the URDF mapping.

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| radius | radius | Radius of the cylinder (along X & Y) |
| length | height | Height of the cylinder (along Z) |

##### Element: Sphere

The sphere element defines a basic sphere primitive centered at the origin.

In USD this maps to a `UsdGeomSphere`, which by default has poles aligned to the Z axis (regardless of Stage “upAxis”).

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
|  radius | radius | Radius of the sphere |

##### Element: Mesh

The mesh element defines a reference to a trimesh (triangulated mesh) in an external file. Any geometry format is allowed, but specific application compatibility is dependent on implementation. Therefore it is recommended to stick to the most common formats: OBJ, STL, and DAE (Collada).

URDF assumes that all geometry in the referenced file represents a single geometry component. Some formats support multiple shapes within a single file, but for URDF all of these shapes should be considered as one piece of the robot. If the shapes are polygon meshes, then the topology, normals, and texture coordinates (if available) are considered part of the geometry component. Any other geometric types (e.g. curves, point clouds, etc) are not supported.

The analogous concept in USD is a `UsdGeomMesh` prim. USD meshes can be n-gons and have many extra features (e.g. subdivision attributes), but are still the correct choice for basic triangulated meshes.

When converting to USD, in certain cases it is preferable to represent each polygon mesh separately, and in certain cases it is preferable to merge them all into a single polygon mesh.

If an implementation chooses to keep the shapes separate, each mesh within the file should be converted to a `UsdGeomMesh` prim, and all shapes should be encapsulated under a common `UsdGeomScope` parent prim. This Scope can be considered the representation of the entire [visual](#linkvisual) or [collision](#linkcollision) element. If the mesh is provided for a collision element, the `UsdPhysicsCollisionAPI` and `UsdPhysicsMeshCollisionAPI` must be applied to each of the `UsdGeomMeshes` directly, not on the Scope.

If an implementation chooses to merge the shapes, it may be required to use `UsdGeomSubsets` child prims in order to maintain the expected materials. See [Embedded Materials](#embedded-materials-for-meshes) for details.  If the mesh is used as a collision element, the `UsdPhysicsCollisionAPI` and `UsdPhysicsMeshCollisionAPI` must be applied to the `UsdGeomMesh` directly, not on the `UsdGeomSubsets`.

For each mesh, it is important to set the `subdivisionScheme` to "none", as the default scheme “catmullClark” would cause the rendered surface to shrink away from the polygon cage.

It is also important to author the mesh in the expected handed-ness. Most applications prefer right-handed mesh data. If the source file contains left-handed data, the winding order should be reversed in the data & the `UsdGeomMesh.orientation` attribute should be authored as "rightHanded" to make this explicit.

As mesh files may be re-used multiple times within the robot, it is recommended to convert the mesh to USD once, under a dedicated class Scope with a “library layer” (see [Appendix D](#appendix-d-usd-asset-structure)) and to re-use it via [Reference](https://openusd.org/release/glossary.html#usdglossary-references) composition arcs. This allows for the mesh to be referenced into the main robot structure several times, while avoiding costly array duplication.

It is also recommended to store `UsdGeomMesh` prims in binary compressed crate (`.usdc`) files for best performance, reduced storage requirements, and improved legibility in the main layer file. The heavy array data can be compressed, and the lightweight physics data can remain human readable.

Since the mesh will be converted to a specific SdfPath (possibly within an external `SdfLayer`), it is important for converters to maintain a data structure associating the original `mesh.filename` with the resulting USD identifier.

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
|  [filename](#property-filename) | `SdfAssetPath` | URI to an external mesh file |
|  [scale](#property-scale) | `UsdGeomXformOp` (`TypeScale`) | Scale |

###### *Property: filename*

The `mesh.filename` may be an absolute or relative local filesystem path or it may be specified as a [ROS2 package](#appendix-a-ros2-packages) URI (e.g. `package://<package_name>/<relative_path>`). In the case of a ROS2 Package, see [Resolving Package URIs](#resolving-package-uris) for possible solutions.

In either case, the file should be parsed & converted to a `UsdGeomMesh`. Direct concept mappings for STL/OBJ/DAE are beyond the scope of this document. Recall that URDF requires the file to contain only a single triangulated mesh, greatly reducing scope of the mapping process for these formats. It is also important to consider [texture coordinate](#texture-coordinates) conventions between the various formats.

###### *Property: scale*

The `mesh.scale` is used to scale the mesh's axis-aligned-bounding-box. 

In USD this is analogous to `UsdGeomXformOp` with `TypeScale`. As `UsdGeomMesh` inherits from `UsdGeomXformable`, the `XformOp` can be applied to the mesh directly (no intermediate Xform prim is required).

If using the recommended referencing workflow, this scale should be applied on the  `UsdReference` prim rather than the mesh prim in the library layer, as the scale can be applied independently for each visual targeting the same library mesh.

###### *Texture Coordinates*

In USD, (0,0) is in the lower left corner of the UV tile, so the vertical “t” coordinate of the geometry texcoord may need to be flipped for each UV value in the array, depending on the source format:

`st = GfVec2f(texcoord[0], 1.0 - texcoord[1])`

Note however, that if reading texcoords directly from an OBJ source file, this v-flip is unnecessary, as its conventions match USD already.

Note also STL format does not include texture coordinates, and textures are therefore unsupported on STL meshes.

### material

The material element can be referenced from a visual to drive the rendered appearance of the geometry. There can be multiple named materials as direct children of the [robot](#robot) element. This effectively forms a library of materials for use within the robot. Materials can also be defined directly using the [visual/material](#element-material) element. Additionally, while not documented explicitly in the URDF spec, some URDF applications (e.g. rviz2) support materials embedded in the [referenced mesh files](#reference-mesh-files). In particular, DAE materials & MTL side-car to OBJ files should be considered supported (see [Embedded Materials](#embedded-materials-for-meshes)).

The analogous concept in USD is a `UsdShadeMaterial` prim. However, in USD materials are authored as node graphs of connected shaders. For the `UsdShadeMaterial` Prim to actually define an appearance, it must be driven by connections from one or more `UsdShadeShader` prims. USD materials also offer many “render contexts” so that different node graphs of shaders can be used for each renderer (or render purpose).

For the lowest common denominator across the entire USD Ecosystem, [UsdPreviewSurface](https://openusd.org/release/spec_usdpreviewsurface.html) (UPS) node graphs should be used. However, these node graphs offer a very limited set of PBR-like functionality and are generally considered to be for “preview” rather than high fidelity visualization.

For a fully featured PBR material suitable for high-fidelity visualization, and with broad (but not complete) interoperability across the USD Ecosystem, it is recommended to author MaterialX (Mtlx) node graphs, using an OpenPBR surface shader (e.g. [open\_pbr\_surface](https://github.com/AcademySoftwareFoundation/MaterialX/blob/main/libraries/bxdf/open_pbr_surface.mtlx)).

Note that the resulting Material prim will likely be the target of a UsdReference (e.g. via a [visual/material](#element-material)). As materials may be re-used multiple times within the robot, it is recommended to convert the material to USD once, under a dedicated class Scope within a “library layer” (see [Appendix D](#appendix-d-usd-asset-structure)), mark it as instanceable, and re-use it via [Reference](https://openusd.org/release/glossary.html#usdglossary-references) composition arcs. This allows for the materials and shaders to be referenced into the main robot structure several times, while avoiding excessive prim duplication.

Since the material will be converted to a specific `SdfPath` (possibly within an external SdfLayer), it is important for converters to maintain a data structure associating the original URDF name with the resulting USD identifier.

#### Material Attributes

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| [name](#attribute-name) | `SdfPath` | In USD the name is accessed via the Stage or Layer, it is not a Prim property |

#### Material Elements

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
|  [color](#element-color) | `UsdShadeInputs` | Main (diffuse) color and transparency; usually separate inputs in USD |
|  [texture](#element-texture) | `UsdShadeShader`,<br>`UsdShadeInputs`,<br>***GAP (projection shaders)*** | The texture file to use in the material to drive diffuse/albedo color. |

##### Element: color

The color element provides a single rgba property, which is a 4 float array of red, green, blue, and alpha values in `[0,1]` range.

In USD, this maps to `UsdShadeInputs` on a `UsdShadeShader`, which in turn drives the surface output terminal on a `UsdShadeMaterial`.

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
|  [rgba](#property-rgba) | `UsdShadeInputs` | Diffuse/albedo color and opacity |

###### *Property: rgba*

Most USD shading systems separate color from transparency, so `color.rgba` maps to two separate `UsdShadeInputs` on the surface shader. The names of the inputs differ for each render context:

* UPS  
  * “rgb” maps to “diffuseColor”  
  * “a” maps to “opacity”  
* Mtlx  
  * “rgb” maps to “base\_color”  
  * “a” maps to “geometry\_opacity”

###### *Colorspace*

The URDF colors are specified in sRGB colorspace.

In USD, the values should be specified in linear colorspace for either render context, so the sRGB values need to be converted to linear before authoring.

###### *Texture Interaction*

The [material/texture](#element-texture) references an optional file texture that, if specified, overrides the material/color. This implies that the color can be considered the texture shader's fallback value when the file is not found. The `color.rgba` should map to these inputs on the texture shaders to serve as the fallback value:

* UPS  
  * “rgba” maps to “fallback”  
* Mtlx  
  * “rgba” maps to “default”

##### Element: texture

The material/texture element has a filename property which provides a URI to an external image file, which must be PNG or JPG.

In USD, this maps to a `UsdShadeInput`, which typically appears on a dedicated `UsdShadeShader`, and is connected to the diffuse/albedo color of the surface `UsdShadeShader`. There is typically no direct connection between the texture prim and the `UsdShaderMaterial`, only indirect via the surface shader.

The visualization of the texture is dependent on the type of geometry & this affects which `UsdShadeShaders` are appropriate. See [Mesh Textures](#mesh-textures) for mesh geometry and [Projection Textures](#projection-textures) for all other geometry.

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
|  [filename](#property-filename-1) | `SdfAssetPath` | URI to an external mesh file |

###### *Property: filename*

The filename may be an absolute or relative local filesystem path or it may be specified as a [ROS2 package](#appendix-a-ros2-packages) URI (e.g. `package://<package_name>/<relative_path>`). In the case of a ROS2 Package, see [Resolving Package URIs](#resolving-package-uris) for possible solutions.

In USD, these files will be read by various subsystems (e.g. OpenImageIO) which all natively support PNG and JPG files (and many other formats).

In USD, it is important to identify all external content such that the `ArResolver` system is used to locate it. As such, any `UsdShadeInput` representing a texture file must be of type `SdfAssetPath` & set to the full path (or layer-relative path) on the storage system.

When authoring an atomic component asset (e.g. a robot), it is best practice to ensure all external data (`SdfAssetPath`) is within a layer-relative path (for encapsulation purposes). The USDZ interchange format strictly requires this behavior. As such, it is recommended to copy this file into a “Textures” subdirectory of the root `SdfLayer`.

Once the file has been copied into place, to determine how to author it onto a texture prim, see  [Mesh Textures](#mesh-textures) for mesh geometry and [Projection Textures](#projection-textures) for all other geometry.

###### *Mesh Textures*

For mesh geometry, rendering should use a standard texture mapping approach, using a [texcoord primvar](#texture-coordinates) as converted from the source OBJ or DAE file.

In USD this maps to a standard file/image shader with a primvar reader texture driving the texcoords. This is an intricate network of UsdShadeShader prims in each render context:

* UPS  
  * A [TextureReader](https://openusd.org/release/spec_usdpreviewsurface.html#texture-reader) shader should be connected to the appropriate inputs of the node graph that drive the surface shader.  
  * Set the "file" input based on the [`texture.filename`](#property-filename-1)  
  * A [PrimvarReader](https://openusd.org/release/spec_usdpreviewsurface.html#primvar-reader) shader, with the “varname” input set to “st” should be connected to the “st” input of the `UsdUvTextureReader`.  
  * Use “scale” and “bias” on the `UsdUvTextureReader` to perform any necessary image inversion specified in “hflip” and “vflip”  
* Mtlx  
  * An “image” shader e.g. "ND\_image\_color3" should be connected to the appropriate inputs of the node graph that drive the surface shader.  
  * Set the "file" input based on the [`texture.filename`](#property-filename-1)  
  * A "ND\_geompropvalue\_vector2" shader, with the “geomprop” input set to “st” should be connected to the “texcoord” input of the image shader.

###### *Projection Textures*

For basic primitives (box, cylinder, sphere), the texture behavior is undefined in the URDF specification. In practice, some applications (e.g. rviz2) use a basic projection map.

Projection textures are not yet consistently supported throughout the USD Ecosystem:

* UPS  
  * Not possible; there are no projection shaders  
* Mtlx  
  * Use a “triplanarprojection” shader e.g. "ND\_triplanarprojection\_color3"  
  * Set the "filez" input based on the [`texture.filename`](#property-filename-1)

UPS does not have any way to accomplish this & while MaterialX has a triplanar projection shader, its results may not match other applications like rviz2.

As the expectation is not documented in the URDF specification, and the USD support is lacking, it is recommended to warn or error & skip conversion of textures which are assigned to basic geometry prims.

#### Embedded Materials for Meshes

In addition to direct URDF materials, [referenced mesh files](#reference-mesh-files) may contain embedded materials:

* OBJ files can have MTL sidecar files, which can assign a shader to each of the shapes within the OBJ. Since all shapes within one OBJ are merged into a single mesh in URDF, the MTL shader assignments can also be thought of as mesh subset assignments.  
* DAE files can contain an intricate combination of texture & shader effects applied to a particular subset of each mesh’s vertices or faces.

Mapping the material & shader concepts of these reference formats is outside the scope of this document, other than to say that USD uses PBR materials, but OBJ and DAE files often use Phong shading as the default. In this case, appropriate conversion is necessary, but it may be advisable to only support the key material parameters passed from URDF: diffuse color/texture, emissive color/texture, and opacity.

Whether supporting a partial or a full material conversion, they should be converted to appropriate `UsdShadeMaterial` and `UsdShadeShader` prims, stored alongside the native URDF materials in a library, and bound to the UsdGeom prims using an applied `MaterialBindingAPI`.

In USD, the subset concept maps to `UsdGeomSubset` Prims, which should be children of the `UsdGeomMesh` (see [mesh](#element-mesh)). Each `UsdGeomSubset` can have a `MaterialBindingAPI` applied to it, targeting a different `UsdShadeMaterial`. There is some runtime cost associated with Subsets, so if only one material is applied to the entire mesh, it is preferable to avoid Subset prims & recommended to use a material binding to the mesh itself.

If embedded materials are assigned to meshes or a subset of faces in the OBJ/DAE files, they should be converted to `UsdShadeMaterials` and bound to the appropriate `UsdGeomMesh` or `UsdGeomSubset`. If URDF materials also exist for these meshes/subsets, they should be ignored. Native URDF materials should only be bound to meshes or subsets which do not already receive material bindings from the embedded OBJ/DAE files.

See [material](#material) for further guidance.

### joint

In URDF joints define the relative motion of links & describe the kinematic hierarchy of the robot, but defining motion degrees of freedom between a specified parent link and a child link, as well as the initial 3D position & orientation of the child link.

In USD the two concepts are separated. The 3D frame of the child link needs to be specified on the child body prim itself (see [Link Hierarchy](#link-hierarchy)). The remainder of the URDF joint can be represented by various subclasses of UsdPhysicsJoints, which are typed Prims which define constraints that limit degrees of freedom between 2 bodies. The [`joint.type`](#attribute-type) will inform which specific PhysicsJoints should be used for each joint.

It is important to maintain the order of link relationships for consistent conversion of the URDF kinematic tree to USD. It is recommended to author the parent link as the “Body0” relationship, and the child link as the “Body1” relationship. This 0/1 naming will correspond to various other attributes of `UsdPhysicsJoint`.

#### Joint Hierarchy

As joints in USD are not expressed hierarchically, they can be authored anywhere within the USD prim hierarchy. For legibility & navigability, it is recommended to author the joints under a common Scope prim within the default prim (e.g. /Robot/Physics/Joint1) rather than alongside the bodies.

#### Joint Attributes

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| [name](#attribute-name) | `SdfPath` | In USD the name is accessed via the Stage or Layer, it is not a Prim property |
| [type](#attribute-type) | `UsdPhysicsJoint` | Type of joint |

##### Attribute: type

URDF supports 6 types of joints. The type is required and can be only 1 of the 6 choices.

In USD, this choice of type determines the explicit prim type, among the different UsdPhysicsJoints.

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| revolute | `UsdPhysicsRevoluteJoint` | Rotation around a single axis |
| continuous | `UsdPhysicsRevoluteJoint` | Unlimited rotation around a single axis |
| prismatic | `UsdPhysicsPrismaticJoint` | Linear slider which moves along a single axis |
| fixed | `UsdPhysicsFixedJoint` | Fixed joint that rigidly connects two links with no relative motion |
| floating | N/A | Allows motion for all 6 degrees of freedom.<br><br>In USD this is equivalent to an unconstrained body. |
| planar | `UsdPhysicsJoint` | Allows motion in a plane perpendicular to one axis.<br><br>`UsdPhysicsJoint` is a D6 constraint, but `UsdPhysicsLimitAPI` can be used to lock particular DOFs.<br><br>Planar joints should have unlocked rotation in the plane axis, and unlocked translation in the orthogonal axes (e.g. Z rotation \+ X and Y translation for axis \= "0 0 1").<br><br>Note that simulation behavior can become erratic when the constraint deviates too much from 0\. |

#### Joint Elements

URDF joints have several child elements, some of which are required while others are optional. The most important are parent & child, which specify the links constrained by this joint.

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| [parent](#element-parent) | `physics:body0` | Parent link |
| [child](#element-child) | `physics:body1` | Child link |
| [origin](#element-origin-1) | `physics:localPos0`,<br>`physics:localPos1`,<br>`physics:localRot0`,<br>`physics:localRot1`,<br><br>Also affects XformOps of the child link | Position and orientation of the child link relative to the parent link |
| [axis](#element-axis) | `physics:axis` | The joint's axis of motion. |
| [limit](#element-limit) | `physics:lowerLimit`,<br>`physics:upperLimit` | Physical limits of certain joints |
| [calibration](#element-calibration) | ***GAP*** | Calibration information for the joint |
| [dynamics](#element-dynamics) | ***GAP*** | Friction and damping for the joint |
| [safety\_controller](#element-safety_controller) | ***GAP*** | Safety control of the joint |
| [mimic](#element-mimic) | ***GAP*** | Mimicking the behavior of other joints |

##### Element: parent

The joint/parent element has a single property called link, which names a link that is the parent in the kinematic structure.

In USD, this maps to the body0 `UsdRelationship` property on a `UsdPhysicsJoint` prim.

##### Element: child

The joint/child element has a single property called link, which names a link that is the child in the kinematic structure.

In USD, this maps to the body1 `UsdRelationship` property on a `UsdPhysicsJoint` prim.

##### Element: origin

The joint/origin element specifies a transform from the parent link to the child link. The joint itself exists at the origin of the child link.

In USD, transformation of body prims is handled separately to joints & the latter can additionally be transformed independently within the space of each body. As such, the joint/origin data needs to be reflected on multiple prims:

* The Xform prim representing the child link needs to be transformed accordingly, which may use these values or may require calculating robot-space values, depending on the chosen prim hierarchy of the links (see [Link Hierarchy](#link-hierarchy)).  
* The Joint prim's `physics:localPos0` and `physics:localRot0` attributes need to be authored to match these values (the relative offset from parent to child).  
* The Joint prim's `physics:localPos1` and `physics:localRot1` attributes could be authored with identity values for clarity.

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| xyz | *Child Link* `XformOp` (`TypeTranslate`)<br>*Joint* `physics:localPos0` | Child link position relative to Parent link |
| rpy | *Child Link* `XformOp` (`TypeRotateXYZ` or `TypeOrient`)<br>*Joint* `physics:localRot0` | Child link orientation relative to Parent link |

See [Appendix E](#element-origin-2) for additional consideration when authoring XformOps on the child link.

##### Element: axis

The joint/axis element specifies the operating axis for certain joint types, in the joint frame of reference. This is the axis of rotation for revolute joints, the axis of translation for prismatic joints, and the surface normal for planar joints.

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| [xyz](#property-xyz-1) | `physics:axis`,<br>`physics:localRot0`,<br>`physics:localRot1` | Vector representing the axis of operation |

###### *Property: xyz*

In URDF, xyz is specified as a vector which should be normalized. The default is 1,0,0 (rotation around X).

In USD, this maps to `physics:axis`, which is a token for X, Y, or Z (default is rotation around X).

To account for the arbitrary axis, it is necessary to also author `physics:localRot0` and `physics:localRot1` on the joint Prim, such that it is oriented relative to the respective body frame with the axis aligned to one of X, Y, or Z.

##### Element: limit

The joint/limit element specifies hard limits for revolute and prismatic joints, but is not used for the other types of joints. See also [safety\_controller](#element-safety_controller) for soft limits.

In USD, these joints do have lower & upper limits, but do not have effort & velocity limits. See [Appendix C](#appendix-c-filling-concept-gaps) for possible solutions.

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| lower | `physics:lowerLimit` | minimum position/angle |
| upper | `physics:upperLimit` | maximum position/angle |
| velocity | ***GAP*** | maximum velocity (rad/s or m/s) |
| effort | ***GAP*** | maximum torque/force (Nm/N) |

##### Element: calibration

The reference positions of the joint, used to calibrate the absolute position of the joint.

This concept does not exist in USD. See [Appendix C](#appendix-c-filling-concept-gaps) for possible solutions.

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| rising | ***GAP*** | calibration value for rising edge |
| falling | ***GAP*** | calibration value for falling edge |

##### Element: dynamics

The joint/dynamics element defines friction and damping values for the joint, similar to a passive spring on the joint.

In USD, joints do not have any passive spring mechanism. See [Appendix C](#appendix-c-filling-concept-gaps) for possible solutions.

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| damping | ***GAP*** | motion damping of the joint ( N∙m∙s/rad or N∙s/m) |
| friction | ***GAP*** | static friction of the joint (N∙m or N) |

##### Element: safety\_controller

The joint/safety\_controller element defines soft limits for the joint, which need to be within the range allowed by the hard limits from [joint/limits](#element-limit).

In USD, joints do not have soft limits. See [Appendix C](#appendix-c-filling-concept-gaps) for possible solutions.

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| soft\_lower\_limit | ***GAP*** | Soft lower limit for joint position |
| soft\_upper\_limit | ***GAP*** | Soft upper limit for joint position |
| k\_position | ***GAP*** | Position control gain for safety controller |
| k\_velocity | ***GAP*** | Velocity control gain for safety controller |

##### Element: mimic

The joint/mimic element indicates that this joint “mimics” the behavior of another joint. The value of this joint can be computed as `value = multiplier * other_joint_value + offset`.

In USD there is no equivalent concept. See [Appendix C](#appendix-c-filling-concept-gaps) for possible solutions.

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| joint | ***GAP*** | name of the joint to mimic |
| multiplier | ***GAP*** | multiplier (default: 1.0) |
| offset | ***GAP*** | offset (default: 0.0) |

### transmission

The transmission element is used to describe the mechanical relationship between one or more actuators (e.g motors, servos) and a single joint. This element is very rarely used in practice and largely undocumented.

In USD, actuators are not a typed concept of their own. Instead, joint actuation is described by applying the `UsdPhysicsDriveAPI` to the affected `UsdPhysicsJoint`. The DriveAPI can be applied once for each DoF of the Joint.

However, URDF transmission elements are not well specified and largely undocumented. It is recommended to ignore them when converting to USD and to emit a warning or error, or treat them as [custom elements](#custom-elements). The known parts of the specification are mapped to USD below, but do not provide enough information to author the UsdPhysicsDriveAPI.

#### Transmission Attributes

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| [name](#attribute-name) | `SdfPath` | In USD the name is accessed via the Stage or Layer, it is not a Prim property |

#### Transmission Elements

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| [type](#element-type) | ***GAP*** | The type of transmission mechanism (the valid options are not documented). |
| [joint](#element-joint) | `UsdPhysicsDriveAPI`,<br>***GAP*** (hardwareInterface) | Reference to the joint to be controlled |
| [actuator](#element-actuator) | ***GAP*** | Defines the actual drive (motor, etc.)<br><br>It has the following parameters:<br>\- hardwareInterface<br>\- mechanicalReduction |

##### Element: type

The transmission/type element determines the type of transmission. However, the valid values are not documented in the specification.

One could infer types from the [ROS Control documentation](https://wiki.ros.org/ros_control#Transmission_Interfaces) on Transmission Interfaces, but in the absence of formal URDF specification to match, it is unclear what the values should be.

##### Element: joint

The transmission/joint element has a name property, which identifies the joint affected by this transmission.

In USD, this is the joint that the UsdPhysicsDriveAPI should be applied to.

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| name | `UsdPhysicsDriveAPI` | name of the joint to actuate |
| hardwareInterface | ***GAP*** | The valid values are not documented & inconsistent across simulators |

##### Element: actuator

The transmission/actuator element has a name property, but it is unclear what this is used for.

In USD, as the actuator is an applied API on a Joint, there is no mechanism to name it. Simply apply the API to the joint.

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| name | N/A | name of the actuator |
| mechanicalReduction | ***GAP*** | A mechanical reduction at the actuator transmission, which only applies to some transmission types |
| hardwareInterface | N/A | Deprecated. Use joint/hardwareInterface instead |

### gazebo

The gazebo element is an extension to the URDF robot description format, used for simulation purposes in the Gazebo simulator.

As it is simulator specific & outside the scope of the base URDF specification, we recommend ignoring gazebo elements when converting to USD, or treating them as [custom elements](#custom-elements).

# Appendices

## Appendix A: ROS2 Packages

ROS2 defines a URI mechanism called Packages to manage all the data associated with a robot. This is not part of the URDF specification, but it is common to see in open-source URDF datasets.

When using ROS2 Packages, the URDF, the [referenced mesh files](#reference-mesh-files), and the visual [texture files](#element-texture) can be expressed via URI using a \`package\` scheme, allowing the package to be portable across storage systems (see [Package Examples](#ros2-package-examples)).

This kind of storage portability is a key requirement of Atomic Components in USD, and is strictly required for USDZ files.

However, resolving ROS2 Package URIs requires some explicit knowledge about how to locate the package root. For URDF, the ROS2 software stack must be used to [resolve the package](#resolving-package-uris) root locations.

For USD, a custom `ArResolver` could be configured to resolve ROS2 Package URIs, but this requires building & deploying a custom USD plugin as well as the ROS2 software to each runtime application.

### ROS2 Package Examples

For example, for a texture file within a “my\_robot” package, the URI might be as follows:

```
<material name="left_arm_material">
  <texture filename="package://my_robot/textures/left_arm.png"/>
</material>
```

In this case, the package name is "my\_robot", and the specified file exists at the relative location “./textures/left\_arm.png" from the package root.

Similarly, referenced meshes can be identified the same way:

```
<geometry>
  <mesh filename="package://my_robot/meshes/left_arm.obj"/>
  <material name="left_arm_material"/>
</geometry>
```

In this case, the package name is "my\_robot", and the specified file exists at the relative location “./meshes/simple\_cube.obj" from the package root.

### Resolving Package URIs

The ROS2 configuration determines where the package roots reside on the local storage. Aside from package roots, all URIs are effectively relative paths.

In USD, these relative paths should be expressed as such, using `SdfAssetPaths` (e.g.  "./textures/left\_arm.png")

The challenge is to determine a mapping package name to the package root location.

#### Using ROS2

If the ROS2 software stack is available, we can use the rclpy python module to determine whether a given package exists. Similarly, we can use the ros2 CLI:

```
ros2 pkg list | grep my_robot
```

We can get the full path of a given package by running:

```
ros2 pkg prefix my_robot
```

It is important to note that the ROS2 installation assumes that the package itself has been received on the local storage; it does not synchronize files across storage systems.

#### Without ROS2

If the ROS2 software stack is not available, we must locate the package roots on the local storage through other means. The following techniques could be attempted, falling back to the next check if one fails:

- Ask the client to provide the package mapping explicitly  
   - Ask for a list of configured package root locations or ask for an explicit mapping of package\_name to root location  
- Assume the mesh & textures are local to the URDF  
    - Use a filesystem module (e.g. std::filesystem in C++ or pathlib in python) to determine a common anchor between the URDF file & the mesh or texture URI  
    - Strip off the anchor & compose a relative filesystem path to the mesh or texture file & check if it exists  
- If all of the above failed, it would be reasonable to emit an error for the unresolvable packages

## Appendix B: USD Schemas

### Typed Schemas vs Applied Schemas

In USD, prims are typed using schemas. Each Prim must have only one concrete type, or “IsA” schema (e.g. Cube, Sphere, Mesh, Camera), which defines the Prim’s role or purpose. See [IsA Schema](https://openusd.org/release/glossary.html#usdglossary-isaschema) for more details.

However, in addition to a concrete IsA type, each Prim can gain extended functionality via one or more “applied” schemas. See [API Schemas](https://openusd.org/release/glossary.html#api-schema) for more information.

All of the UsdPhysics schemas are applied API Schemas. For example, to author a rigid body with an explicit mass on a Cube prim, one would first author the `UsdGeomCube` and then apply both the `UsdPhysicsRigidBodyAPI` and `UsdPhysicsMassAPI` to it:

```
def Cube "MyCube" (
  prepend apiSchemas = [“PhysicsRigidBodyAPI”, “PhysicsMassAPI”]
)
{
  float physics:mass = 10.0
}
```

### Coded vs Codeless Schemas

Adding a new schema (applied or typed) to USD can be accomplished as a codeful/coded schema or as a [codeless schema](https://openusd.org/release/api/_usd__page__generating_schemas.html#Codeless_Schemas).

The codeless approach is far simpler for overhead of development, deployment, and integration into consumer runtimes. It will “just work” in any USD runtime. However it provides no developer convenience around the schema. Consumers will need to hardcode strings & data types with no API to assist them.

A coded schema offers a better developer experience for consumers, with convenient get/set methods for all schema attributes, but comes with a large deployment cost to ensure runtime compatibility over a (potentially unknown) matrix of USD runtimes.

It is generally recommended to start with a codeless schema and only transition to a coded schema if determined necessary & after reaching a stable milestone.

In either case, a `generatedSchema.usda` and `plugInfo.json` will need to be created & distributed for the new schema. This process can be manual or automated with [usdGenSchema](https://openusd.org/release/api/_usd__page__generating_schemas.html).

## Appendix C: Filling Concept Gaps

### Attribute/Property Gaps

When an URDF attribute has no matching concept in USD, the gap can be filled using 3 general mechanisms:

1\. A custom attribute within an appropriate namespace, e.g.

```
custom uniform double urdf:mimic:multipler = 1.0
```

2\. A constant primvar with an appropriate namespace, e.g.

```
uniform double primvars:urdf:mimic:multipler = 1.0
```

3\. An [applied schema](#appendix-b-usd-schemas), e.g.

```
def UsdGeomXform "Foo" (
  prepend apiSchemas = ["UrdfJointAPI"]
)
{
	uniform double urdf:mimic:multipler = 1.0
}
```

Both the custom attribute and the primvar are natively available without any additional plugins, but require all consumers to be aware & hardcode the equivalent strings when querying the mesh. The difference between the two is primarily about inheritance; primvars are inherited by descendant Prims in the scene hierarchy whereas custom attributes do not inherit. However, the choice may have more to do with the target content pipeline. Some DCCs make working with custom attributes easier than working with primvars (and vice versa).

The applied schema requires more upfront effort, as well as plugin configuration in each USD runtime, but avoids hardcoded strings in content pipelines & provides more explicit documentation about the meaning of the USD attributes. See [Appendix B: USD Schemas](#appendix-b-usd-schemas) before deciding to create a new schema.

### Prim Gaps

When no suitable USD Prim is available to fill a concept gap, it may be necessary to define a new [Typed Schema](https://openusd.org/release/glossary.html#typed-schema). See [Appendix B: USD Schema](#appendix-b-usd-schemas) before deciding to create a new schema.

## Appendix D: USD Asset Structure

### Disclaimer

There are many ways to structure assets in USD, and any Content Pipeline might have its own trade-offs to make in this respect. For example, a Content Pipeline which wishes to iterate on the URDF as the “source of truth” may prefer a single USD layer reflecting the entire URDF, whereas a Content Pipeline which wishes to export to USD & continue iteration in the USD ecosystem would likely prefer a more modular domain based structure.

For the purposes of this document, we make recommendations on Layer structure & some Prim hierarchy (e.g. organizational Scopes) which represent our ideal structure for robotics assets. These recommendations are highly influenced by NVIDIA’s [Principals of Scalable Asset Structure](https://docs.omniverse.nvidia.com/usd/latest/learn-openusd/independent/asset-structure-principles.html), which emphasize that assets should be [legible, modular, performant, and navigable](https://docs.omniverse.nvidia.com/usd/latest/learn-openusd/independent/asset-structure-principles.html#principles-quick-reference).

These recommendations may not be suitable for some Content Pipelines & deviation is expected for any given converter implementation to suit the requirements of its consumers.

### Asset Layer Structure

Keeping workstreams & features in separate layers can help control storage and network traffic. It can also help communicate intent to consumers & make it easier to reason about the overall robot asset.

Encapsulating local dependencies within an asset via relative paths makes it portable. When authoring an atomic component asset (e.g. a robot), it is best practice to ensure all external data is within a layer-relative path.

We recommend the following layer breakdown:

* A "GeometryLibrary.usdc" library layer (or layers) to hold all mesh data  
* A "MaterialsLibrary.usdc" library layer to hold all visual material (appearance) data.  
  * A “Textures” folder alongside the materials layer to encapsulate all texture files  
* "Geometry.usda" layer defining the prim hierarchy of the robot  
  * Uses references to compose prims from the geometry library into the local hierarchy of the model.  
  * Defines any non-referenced Gprims (e.g. basic primitives for colliders).  
  * Does not define materials or material bindings to geometry, so materials can be edited/muted independently of hierarchy & geometry data.  
  * Does not define any UsdPhysics schemas, so this can remain suitable for kinematic & visualization workflows independently of simulation.  
* A "Materials.usda" layer localizing the materials & binding to the geometry  
  * Uses references to compose prims from the materials libraries into the local hierarchy of the model.  
  * Adds material bindings to geometry (bindings must be localized within the model hierarchy to respect instancing & payload boundaries).  
  * Does not define any UsdPhysics schemas (even physical materials).  
* A "Physics.usda" layer to hold all UsdPhysics schemas/properties  
  * Including any URDF specific physics schemas  
* A "Contents.usda" layer which composes all of the above via sublayers  
* An [Asset Interface](#asset-interface) layer named after the robot/name (e.g. "foo.usda")

### Asset Interface

An Asset Interface is a lightweight USDA layer that only provides metadata & basic parameterization of an asset, with the full content sitting behind a [payload](https://openusd.org/release/glossary.html#usdglossary-payload) (for deferred loading).

This layer typically exposes the AssetInfo dictionary, Kind, extentsHints, & possibly selector variants or root primvars used to control the underlying layers. In particular, it is recommended to expose variants that allow for easy selection of the underlying feature set (e.g. enable geometry & materials, but disable physics).

### Library Prims and Library Layers

Library layers are independent of the robot’s main dataset & represent a container of reusable Prims. In URDF, there is no equivalent element for re-usable meshes, but it is an important optimization in USD.

In USD it is common to split libraries based on what type of Prims they hold & name the root prim of the library accordingly (e.g. “Geometry”, “Materials”, “Sensors”). Typically the root prim of a library is a class Scope. The “class” specifier makes the prim an abstract container that isn’t considered part of the main content.

Libraries can be serialized internally to the main SdfLayer, but it is more common to serialize each library to its own SdfLayer for modularity & to improve legibility in the main file. This also enables different serialization based on the content of the library (e.g. USDC for meshes, USDA for interfaces).

It is recommended for these separate library layer files to be stored relative to the main [Asset Interface](#asset-interface) layer, unless they are being shared among several URDF sources.

## Appendix E: Common Attributes & Elements

### Attribute: name

In USD the name is accessed via the Stage or Layer, it is not a property of the Prim itself. Names are mandatory in USD, but are optional on some URDF elements. In the case of unnamed URDF elements, the element type name should be used as a fallback.

Names are required to be unique among siblings of the same parent Prim. In URDF names are required to be unique across the entire robot, so avoiding sibling name collisions in USD should be mostly unnecessary (see below).

Names in USD also have some limitations that can prevent the native URDF names from being used directly. In many runtimes, the characters that are considered legal for UsdPrim names are quite restrictive. In modern releases (USD 24.03+) UTF-8 characters are supported. However, many USD Ecosystem DCCs still apply the older restriction of ASCII only. Additionally, many characters are reserved, with special meaning in the USD lexicon (e.g. “/”, “.”, “:”, “-”).

Therefore there will always be cases where the URDF names need to be encoded in order to be used as the Prim name. In these cases it is recommended that the “displayName” metadata on the Prim be set to the original URDF name, for legibility & navigability, as there is no restriction on the characters used in metadata values.

The encoding process could introduce name collisions that violate the first requirement: unique sibling names. In order to prevent collisions, it is recommended to use an encoding algorithm that can account for all children of a particular parent Prim & encode them all in one pass.

### Element: origin

A common concept within several URDF elements is the origin, which represents the position and orientation of some element relative to another. It is broken down into two properties:

| URDF | OpenUSD | Description |
| :---- | :---- | :---- |
| [xyz](#property-xyz-2) | `UsdGeomXformOp` (`TypeTranslate`) | Position |
| [rpy](#property-rpy-1) | `UsdGeomXformOp` (`TypeRotateXYZ` or `TypeOrient`) | Orientation as roll, pitch, yaw Euler rotations in radians |

###### *Property: xyz*

The `origin.xyz` can be expressed in USD as a `UsdGeomXformOp` of `TypeTranslate`.

Note that `TypeTranslate` `XformOps` should always prefer double precision.

###### *Property: rpy*

The `origin.rpy` rotation is specified as Roll(X) / Pitch(Y) / Yaw(Z) in radians. There are several ways to encode rotation in USD, the most direct mapping is to use a `UsdGeomXformOp` of `TypeRotateXYZ`, after converting from radians to degrees.

However, most simulators prefer to work in quaternions, so it may be preferable to use a `UsdGeomXformOp` of `TypeOrient` instead. Both are valid approaches in USD.

Note that rotational `XformOps` (whether Euler angles or quaternions) should typically prefer float precision.

## Appendix F: Custom Attributes & Elements

Since URDF is an XML based format, it is very easy to inject out-of-spec custom attributes and even custom elements into the file. This has become common practice in the community, and while this data has no formal meaning across companies & industries, it is still meaningful within individual data pipelines or when targeting specific runtimes.

As such, it is desirable to retain this custom data when converting the robot to USD. However, since the data is not well specified, we cannot rely on schemas as suggested in Appendix C.

### Custom Attributes

Consider a custom attribute on an in-spec element:

```
<joint name="hinge" foo="bar"/>
```

In USD, the \`custom\` tag on `UsdAttribute` identifies an out-of-schema attribute, and we can use a generic namespace prefix like “urdf:” when authoring this attribute, to indicate this attribute came from a URDF file originally.

Rather than assume an attribute type by parsing the XML it is recommended to treat all custom attributes as strings (just as they are in the XML document) & leave interpretation up to the target runtime:

```
def PhysicsRevoluteJoint "hinge"
  custom uniform string urdf:foo = "bar"
```

### Custom Elements

For entirely custom elements, the only option is to use generic container `UsdPrim`. Untyped Prims are technically achievable in USD, but some runtimes consider them invalid. Instead, it is recommended to use the Scope type to indicate this is a container with no specific meaning.

```
<foo bar="baz"/>
```

```
def Scope "foo"
  custom uniform string bar = "baz"
```

Similarly, if there are nested custom elements, mirror the nesting in USD without inferring a specific meaning, but take care to de-duplicate & validate sibling prim names:

```
<foo>
  <bar baz="bongo"/>
  <bar baz="qux"/>
</foo>
```

```
def Scope "foo"
  def Scope "bar"
    custom uniform string baz = "bongo"
  def Scope "bar1"
    custom uniform string baz = "qux"
```

It is recommended that parameters defined in the URDF but not supported in USD be stored as custom attributes on the parent `UsdPrim`.

For example, parameters within a joint ([calibration](#element-calibration), [dynamics](#element-dynamics), [safety_controller](#element-safety_controller)) can be stored as custom attributes of `UsdPhysicsJoints`.

```
<joint name="foo" type="fixed">
  <calibration rising="0.3" falling="0.2" reference_position="0.1"/>
</joint>
```

```
def PhysicsFixedJoint "foo"
  custom float urdf:calibration:rising = 0.3
  custom float urdf:calibration:falling = 0.2
  custom float urdf:calibration:reference_position = 0.1
```
