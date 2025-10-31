# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

from pxr import Tf

from .data import ConversionData

__all__ = ["resolve_ros_package_paths"]


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
        # Remove "package://" from the URI.
        _, _, path_with_package_name = uri.partition("package://")

        # [package name]/[relative path]
        split_dir = pathlib.Path(path_with_package_name)

        split_dirs = split_dir.parts
        if len(split_dirs) < 2:
            Tf.Warn(f"Invalid ROS package URI. No relative path specified: {uri}")

            # The result after removing "package://[package name]" is an empty string.
            return pathlib.Path()

        package_name = split_dirs[0]
        relative_path = pathlib.Path(*split_dirs[1:])

        package_path = data.ros_packages.get(package_name, None)
        if package_path:
            base_path = pathlib.Path(package_path)
            return base_path / relative_path
        else:
            return relative_path

    return pathlib.Path(uri)
