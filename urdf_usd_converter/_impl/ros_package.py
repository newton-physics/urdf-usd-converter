# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
from .data import ConversionData

__all__ = ["resolve_ros_package_paths"]


def resolve_ros_package_paths(uri: str, data: ConversionData) -> str:
    """
    Resolve the ROS package paths for the given filename.

    Args:
        uri: The path to resolve (Material textures, mesh paths).
        data: The conversion data.

    Returns:
        The resolved path.
    """
    if uri.startswith("package://"):
        split_uri = uri.replace("\\", "/").split("package://")
        split_dirs = split_uri[1].split("/")
        if len(split_dirs) < 2:
            raise ValueError(f"Invalid ROS package URI. No relative path specified: {uri}")
        package_name = split_dirs[0]
        package_path = data.ros_packages.get(package_name, None)
        if package_path:
            package_path = package_path.replace("\\", "/")
            package_path += "/" if not package_path.endswith("/") else ""
            return package_path + "/".join(split_dirs[1:])
        else:
            relative_path = "/".join(split_dirs[1:])
            return relative_path
    return uri
