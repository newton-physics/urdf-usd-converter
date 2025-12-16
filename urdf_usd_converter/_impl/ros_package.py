# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import os
import pathlib

from pxr import Tf

from .data import ConversionData
from .urdf_parser.parser import URDFParser

__all__ = ["resolve_ros_package_paths", "search_ros_packages"]


def resolve_ros_package_paths(uri: str, data: ConversionData) -> pathlib.Path:
    """
    Resolve the ROS package paths for the given filename.

    Args:
        uri: The path to resolve (Material textures, mesh paths).
        data: The conversion data.

    Returns:
        The resolved path.
    """
    if uri.startswith("package://"):
        package_name, relative_path = _split_package_name_and_path(uri)
        if not package_name or not relative_path:
            Tf.Warn(f"Invalid ROS package URI. No relative path specified: {uri}")
            return pathlib.Path()

        package_path = data.ros_packages.get(package_name, None)
        if package_path:
            base_path = pathlib.Path(package_path)
            return base_path / relative_path
        else:
            return relative_path

    return pathlib.Path(uri)


def search_ros_packages(urdf_parser: URDFParser) -> dict[str]:
    """
    Automatically searches for packages based on the paths to meshes and material textures found in the urdf file.

    Args:
        urdf_parser: The URDF parser.

    Returns:
        A dictionary with the package name and path.
    """
    meshes = urdf_parser.get_meshes()
    materials = urdf_parser.get_materials()

    # Store references beginning with "package://".
    package_filenames = []
    for mesh in meshes:
        filename = mesh[0]
        if filename and filename.startswith("package://"):
            package_filenames.append(filename)

    for material in materials:
        filename = material[2]
        if filename and filename.startswith("package://"):
            package_filenames.append(filename)

    ros_packages = {}

    for filename in package_filenames:
        package_name, relative_path = _split_package_name_and_path(filename)
        if package_name and relative_path and package_name not in ros_packages:
            # URDF file directory
            urdf_dir = urdf_parser.input_file.parent

            # Traverse up from urdf_dir to parent directories.
            # if the file path "urdf_dir / relative_path" exists, store it in ros_packages.
            while True:
                file_path = urdf_dir / relative_path
                if file_path.exists():
                    # Relative path from the path containing the urdf file.
                    ros_packages[package_name] = pathlib.Path(os.path.relpath(urdf_dir, urdf_parser.input_file.parent))
                    break
                if urdf_dir.parent == urdf_dir:
                    # Reached the root directory
                    break
                urdf_dir = urdf_dir.parent

    return ros_packages


def _split_package_name_and_path(uri: str) -> tuple[str, pathlib.Path]:
    """
    Split the package name and path from the URI.

    Args:
        uri: The URI to split.

    Returns:
        The package name and path.
        None if the URI is invalid.
    """
    # Remove "package://" from the URI.
    _, _, path_with_package_name = uri.partition("package://")

    # [package name]/[relative path]
    split_dir = pathlib.Path(path_with_package_name)

    split_dirs = split_dir.parts
    if len(split_dirs) < 2:
        # The result after removing "package://[package name]" is an empty string.
        return None, None

    package_name = split_dirs[0]
    relative_path = pathlib.Path(*split_dirs[1:])
    return package_name, relative_path
