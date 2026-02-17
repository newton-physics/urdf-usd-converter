# 0.1.0b1

## Features

- Calibration, Dynamics, Safety Controller, and Mimic are all converted as custom attributes on the Joint prim
- Convert custom elements as Scopes and custom attributes with `custom` and `urdf:` namespace
- Adapted link handling for URDF's "ghost link" convention, where the first link may be a
  purely used to identifiy fixed vs floating articulations

## Fixes

- Stopped triangulating OBJ meshes to support n-gons
- Fixed handling of URDF materials with no material name

## Documentation

- Added benchmarks reporting for several URDF datasets
- Added documentation on how to specify ROS packages

# 0.1.0a2

## Features

- Added conversion of DAE embedded materials to `UsdPreviewSurface` materials
- Improved performance by optimizing numpy processing when converting STL, OBJ, and DAE meshes

## Fixes

- Fixed several DAE mesh conversion issues
  - Fixed to correctly parse even when the DAE file structure is corrupted
  - Fixed an issue where the UV array could not be acquired correctly in some cases
  - Fixed `familyType` attribute when meshes contain subsets
- Fixed OBJ per-face material assignments via `UsdGeomSubsets`
- Fixed material overrides beween native URDF & embedded OBJ/DAE materials
  - We now match `rviz` & `urdfviewer` where embedded OBJ/DAE materials take priority over URDF materials
- Fixed texture wrapping behaviour using `repeat` mode on all `UsdUvTexture` shaders
  - The `wrapMode` is exposed on the material interface so users can change it as needed
- Fixed color issue related to the specular workflow
  - Specular workflow has been unconditionally disabled. There is no meaningful mapping of URDF specular color (phong based materials) to UsdPreviewSurface (simplistic PBR based materials) so we ignore specular color for now. This gives results more closely matching `rviz` & `urdfviewer`
- Fixed URDF Parser to allow invalid `axis` specification on `fixed` joints
  - Many sample assets have `axis="0 0 0"` on fixed joints, which is meaningless but harmless
- Fixed URDF Parser to handle errors when no links exist in the file

## Documention

- Update Concept Mapping document to reflect new stance on material overrides

# 0.1.0a1

## Features

- **USD Asset Structure**
  - Output Assets are completely standalone with no dependencies on the source URDF, OBJ, DAE, or STL files
  - Atomic Component structure with Asset Interface layer and payloaded contents
  - Separate geometry, material, and physics content layers for easy asset-reuse across domains
  - Library-based asset references for meshes and materials to avoid heavy data duplication
  - Explicit USD stage metadata with units (meters, kilograms) and up-axis (Z)
- **Link Conversion**
  - URDF links are converted as `UsdGeom.Xform` prims with `UsdPhysics.RigidBodyAPI` applied
  - The root link has `UsdPhysics.ArticulationRootAPI` applied to indicate the root of the kinematic tree
  - Links are nested in USD, reflecting the kinematic hierarchy of the source URDF rather than the XML file structure
  - Complete mass properties including explicit inertia & center of mass via `UsdPhysics.MassAPI`
- **Joint Conversion**
  - Revolute joints as `UsdPhysics.RevoluteJoint` with angular limits
  - Continuous joints as `UsdPhysics.RevoluteJoint` without limits
  - Prismatic joints as `UsdPhysics.PrismaticJoint` with linear limits
  - Fixed joints as `UsdPhysics.FixedJoint`
  - Planar joints as `UsdPhysics.Joint` with the appropriate `UsdPhysics.LimitAPI` applied to constrain the locked DOFs
  - Floating joints (bodies are free by default in USD)
  - All joints have automatic joint frame alignment between Body0 and Body1, accounting for URDF joint axis, position, and orientation.
  - Joint limits for velocity & effort have no equivalent in `UsdPhysics`, but are authored as custom attributes `urdf:limit:velocity` and `urdf:limit:effort` respectively.
- **Geometry Conversion**
  - All visual and collision geometry is converted to USD
    - Visuals are set with `default` UsdPurpose and colliders with `guide` UsdPurpose
  - `UsdPhysics.CollisionAPI` is applied to colliders
  - Meshes as `UsdGeom.Mesh`
    - Automatic mesh library generation with reference-based asset structure, to avoid duplicate topology
    - STL files converted to USD using `numpy-stl` and `usd-exchange` with normal processing
    - OBJ files converted using `tinyobjloader` and `usd-exchange` with UV coordinates and normal mapping
    - DAE files converted using `pycollada` and `usd-exchange` with UV coordinates, normal mapping, and `UsdGeom.Subset` support
    - OBJ and DAE files specifying multiple meshes convert as a list of meshes under a common parent prim
    - `UsdPhysics.MeshCollisionAPI` is applied to mesh colliders with convex hull specified as the approximation preference
  - Spheres as `UsdGeom.Sphere`
  - Boxes as `UsdGeom.Cube` with scale transforms
  - Cylinders as `UsdGeom.Cylinder`
- **Visual Material and Texture Conversion**
  - All materials are converted to `UsdShade.Material` graphs using `UsdPreviewSurface` shaders, and encapsulated as instanceable material interfaces
  - PNG texture support with automatic texture copying and path resolution
  - URDF materials convert rgba as diffuse color and opacity, with support for diffuse textures
  - OBJ embedded materials (MTL files) convert diffuse color, specular color, dissolve (opacity), roughness (not shininess), metallic, and ior
    - diffuse, specular, normal/bump, roughness, metallic, and opacity textures are all supported
- **Prim Naming**
  - If URDF/DAE/OBJ names are not valid USD specifiers they are automatically transcoded & made unique & valid
  - Display name metadata preserves the original source names on the USD Prims
- **Command Line Interface**
  - Input is an URDF file and default output is a USD Layer as a structured Atomic Component with an Asset Interface USDA layer
    - All heavy data is compressed binary data (via USDC layers) while lightweight data is plain text for legibility
  - Optional comment string embedded into all authored USD Layers
  - Optional Stage flattening for single-file output
  - Optionally skip the `UsdPhysics.Scene` (this may be desirable for multi-asset setups)
  - Error handling with graceful failures
  - Enable verbose output for debugging (exposes any traceback info)
- **Python API**
  - Full programmatic access via `urdf_usd_converter.Converter` class with configurable parameters for all CLI flags
  - Enables interactive editing of the USD Layers after conversion

# Known Limitations

## USD Data Conversion

- **Joint Conversion**
  - Calibration, Dynamics, Safety Controller, and Mimic have no equivalent in `UsdPhysics`
  - These are all converted as custom attributes on the Joint prim
- **Geometry Conversion**
  - No other file formats beyond OBJ/DAE/STL are supported
  - For DAE files, only "TriangleSet", "Triangles", "Polylist", and "Polygons" are supported
  - For OBJ files, only objects with faces are supported (i.e. no points, lines, or free-form curves/surfaces)
- **Visual Material and Texture Conversion**
  - Projection shaders for basic geometry primitives (box, cylinder, sphere) are not implemented
  - More accurate PBR materials (e.g. OpenPBR via UsdMtlx) are not implemented
- **Other Elements**
  - Transmission, Gazebo, and other out-of-spec URDF extensions have no equivalent in `UsdPhysics`
  - These are all converted as Scope Prims with custom attributes to retain the data

## Using the USD Asset in other USD Ecosystem applications

- The USD Asset contains nested rigid bodies within articulations.
  - Support for nested bodies in UsdPhysics is fairly new (as of USD 25.11), and some existing applications may not support this style of nesting.
