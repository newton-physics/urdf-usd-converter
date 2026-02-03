# urdf-usd-converter

# Overview

A [URDF](https://wiki.ros.org/urdf/XML) to [OpenUSD](https://openusd.org) Data Converter

> Important: This is currently an Alpha product. See the [CHANGELOG](./CHANGELOG.md) for features and known limitations.

Key Features:
- Converts an input URDF file into an OpenUSD Layer
- Supports data conversion of visual geometry & materials, as well as the links, collision geometry, and joints necessary for kinematic simulation.
- Available as a python module or command line interface (CLI).
- Creates a standalone, self-contained artifact with no connection to the source URDF, OBJ, DAE, or STL data.
  - Structured as an [Atomic Component](https://docs.omniverse.nvidia.com/usd/latest/learn-openusd/independent/asset-structure-principles.html#atomic-model-structure-flowerpot)
  - Suitable for visualization & rendering in any OpenUSD Ecosystem application.
  - Suitable for import & simulation in [Newton](https://github.com/newton-physics/newton).

This project is part of [Newton](https://github.com/newton-physics), a [Linux Foundation](https://www.linuxfoundation.org) project which is community-built and maintained.

## Implementation Details & Dependencies

Specific implementation details are based on our [URDF to USD Conceptual Data Mapping](./docs/concept_mapping.md).

The output asset structure is based on NVIDIA's [Principles of Scalable Asset Structure in OpenUSD](https://docs.omniverse.nvidia.com/usd/latest/learn-openusd/independent/asset-structure-principles.html).

The implementation also leverages the following dependencies:
- NVIDIA's [OpenUSD Exchange SDK](https://docs.omniverse.nvidia.com/usd/code-docs/usd-exchange-sdk/latest/index.html) to author consistent & correct USD data.
- Pixar's OpenUSD python modules & native libraries (vendored via the `usd-exchange` wheel).
- [tinyobjloader](https://github.com/tinyobjloader/tinyobjloader), [pycollada](https://github.com/pycollada/pycollada), and [numpy-stl](https://numpy-stl.readthedocs.io) for parsing any mesh data referenced by the input URDF datasets.

# Get Started

To start using the converter, install the python wheel into a virtual environment using your favorite package manager:

```bash
python -m venv .venv
source .venv/bin/activate
pip install urdf-usd-converter
urdf_usd_converter /path/to/robot.urdf /path/to/usd_robot
```

See `urdf_usd_converter --help` for CLI arguments.

Alternatively, the same converter functionality can be accessed from the python module directly, which is useful when further transforming the USD data after conversion.

```python
import urdf_usd_converter
import usdex.core
from pxr import Sdf, Usd

converter = urdf_usd_converter.Converter()
asset: Sdf.AssetPath = converter.convert("/path/to/robot.urdf", "/path/to/usd_robot")
stage: Usd.Stage = Usd.Stage.Open(asset.path)
# modify further using Usd or usdex.core functionality
usdex.core.saveStage(stage, comment="modified after conversion")
```

## Specifying ROS packages

If a filename within a mesh or texture in the URDF file is specified as `package://<package_name>/<path>`, we must separately provide the actual path where the package is located.  
When the converter traces back to the parent directory with the URDF file as the current directory, if a file matching the specified path is found, the combination of the package name and path at that time is automatically used.  

If the ROS package path still cannot be found, we must manually specify the path combination for the package name.  

### CLI

Specify the path to the package name by using the `--package` argument in the CLI.  
We can also specify multiple packages.  

When a filename is specified in an URDF file as shown below, the "robot_package" following `package://` is the package name.  
ROS packages are assigned either a relative path or an absolute path from the URDF relative to the package name.  

```xml
  <material name="body_mat">
    <color rgba="1.0 1.0 1.0 1.0"/>
    <texture filename="package://robot_package/textures/body_image.png"/>
  </material>
```

When specifying the `--package` argument in the CLI as shown below, the actual path will be "/path/to/assets/textures/body_image.png".  

```bash  
urdf_usd_converter /path/to/robot.urdf /path/to/usd_robot --package robot_package=/path/to/assets
```

If multiple packages and paths exist, specify them as follows.  

```bash
urdf_usd_converter /path/to/robot.urdf /path/to/usd_robot --package robot_package=/path/to/assets --package robot_foo=/path/to/foo
```
If the path contains spaces, please enclose it in double quotation marks.  

### Python

When specifying a list of ROS package names and paths in Python, assign the package name to the "name" key and the path to the "path" key in a dictionary.  
Specify the list of packages for this package in the `ros_packages` argument of `urdf_usd_converter.Converter`.  

```python
import urdf_usd_converter
import usdex.core
from pxr import Sdf, Usd

packages = [
    {"name": "robot_package", "path": "/path/to/assets"},
    {"name": "robot_foo", "path": "/path/to/foo"},
]
converter = urdf_usd_converter.Converter(ros_packages=packages)
asset: Sdf.AssetPath = converter.convert("/path/to/robot.urdf", "/path/to/usd_robot")
```

## Loading the USD Asset

Once your asset is saved to storage, it can be loaded into an OpenUSD Ecosystem application.

We recommend starting with [usdview](https://docs.omniverse.nvidia.com/usd/latest/usdview/index.html), a simple graphics application to confirm the visual geometry & materials are working as expected. You can inspect any of the USD properties in this application, including the UsdPhysics properties.

> Tip: [OpenUSD Exchange Samples](https://github.com/NVIDIA-Omniverse/usd-exchange-samples) provides `./usdview.sh` and `.\usdview.bat` commandline tools which bootstrap usdview with the necessary third party dependencies.

However, you cannot start simulating in usdview, as there is no native simulation engine in this application.

To simulate this asset in Newton, call [newton.ModelBuilder.add_usd()](https://newton-physics.github.io/newton/api/_generated/newton.ModelBuilder.html#newton.ModelBuilder.add_usd) to parse the asset and add it to your Newton model.

Simulating in other UsdPhysics enabled products (e.g. NVIDIA Omniverse, Unreal Engine, etc) may provided mixed results. The rigid bodies are structured hierarchically, which maximal coordinate solvers often do not support. In order to see faithful simulation in these applications, the USD asset will need to be modified to suit the expectations of each target runtime.

# Contribution Guidelines

Contributions from the community are welcome. See [CONTRIBUTING.md](./CONTRIBUTING.md) to learn about contributing via GitHub issues, as well as building the project from source and our development workflow.

General contribution guidelines for Newton repositories are available [here](https://github.com/newton-physics/newton-governance/blob/main/CONTRIBUTING.md).

# Community

For questions about this urdf-usd-converter, feel free to join or start a [GitHub Discussions](https://github.com/newton-physics/urdf-usd-converter/discussions).

For questions about OpenUSD Exchange SDK, use the [USD Exchange GitHub Discussions](https://github.com/NVIDIA-Omniverse/usd-exchange/discussions).

For general questions about OpenUSD itself, use the [Alliance for OpenUSD Forum](https://forum.aousd.org).

By participating in this community, you agree to abide by the Linux Foundation [Code of Conduct](https://lfprojects.org/policies/code-of-conduct/).

# References

- [URDF XML Docs](https://wiki.ros.org/urdf/XML)
- [NVIDIA OpenUSD Exchange SDK Docs](https://docs.omniverse.nvidia.com/usd/code-docs/usd-exchange-sdk)
- [OpenUSD API Docs](https://openusd.org/release/api/index.html)
- [OpenUSD User Docs](https://openusd.org/release/index.html)
- [NVIDIA OpenUSD Resources and Learning](https://developer.nvidia.com/usd)

# License

The urdf-usd-converter is provided under the [Apache License, Version 2.0](https://www.apache.org/licenses/LICENSE-2.0), as is the [OpenUSD Exchange SDK](https://docs.omniverse.nvidia.com/usd/code-docs/usd-exchange-sdk/latest/docs/licenses.html).
