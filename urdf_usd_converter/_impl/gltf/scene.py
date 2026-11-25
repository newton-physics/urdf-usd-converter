# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
"""
Scene and node hierarchy.

Only local transforms are stored. World-matrix baking belongs to conversion_gltf.py.
"""

from dataclasses import dataclass

import numpy as np

__all__ = ["Node", "Scene", "local_matrix", "parse_nodes", "parse_scenes"]


@dataclass
class Node:
    """One node. An explicit `matrix` replaces TRS."""

    name: str | None
    children: list[int]
    mesh: int | None
    matrix: np.ndarray | None
    translation: np.ndarray
    rotation: np.ndarray
    scale: np.ndarray


@dataclass
class Scene:
    """One scene. `nodes` holds root-node indices."""

    name: str | None
    nodes: list[int]


def parse_nodes(document: dict) -> list[Node]:
    nodes: list[Node] = []
    for raw in document.get("nodes", []):
        matrix = raw.get("matrix")
        nodes.append(
            Node(
                name=None if raw.get("name") is None else str(raw["name"]),
                children=[int(index) for index in raw.get("children", [])],
                mesh=None if raw.get("mesh") is None else int(raw["mesh"]),
                # glTF stores 16 column-major floats. order="F" makes the array
                # ready for column-vector math (matrix @ point) without another transpose.
                matrix=None if matrix is None else np.array(matrix, dtype=np.float64).reshape((4, 4), order="F"),
                translation=np.array(raw.get("translation", [0.0, 0.0, 0.0]), dtype=np.float64),
                rotation=np.array(raw.get("rotation", [0.0, 0.0, 0.0, 1.0]), dtype=np.float64),
                scale=np.array(raw.get("scale", [1.0, 1.0, 1.0]), dtype=np.float64),
            )
        )
    return nodes


def parse_scenes(document: dict) -> tuple[list[Scene], int | None]:
    scenes = [
        Scene(
            name=None if raw.get("name") is None else str(raw["name"]),
            nodes=[int(index) for index in raw.get("nodes", [])],
        )
        for raw in document.get("scenes", [])
    ]
    scene = document.get("scene")
    if scene is None and scenes:
        scene = 0
    return scenes, None if scene is None else int(scene)


def local_matrix(node: Node) -> np.ndarray:
    """
    Return the local 4x4 matrix for column-vector math (`matrix @ point`).

    Args:
        node: The glTF node.

    Returns:
        The explicit `matrix` of the node, or the matrix composed from its translation, rotation, and scale.
    """
    if node.matrix is not None:
        return node.matrix
    translation = np.eye(4, dtype=np.float64)
    translation[:3, 3] = node.translation
    scale = np.diag([node.scale[0], node.scale[1], node.scale[2], 1.0])
    return translation @ _rotation_matrix(node.rotation) @ scale


def _rotation_matrix(quaternion: np.ndarray) -> np.ndarray:
    """Build a rotation matrix from a quaternion stored as (x, y, z, w)."""
    x, y, z, w = (float(value) for value in quaternion)
    matrix = np.eye(4, dtype=np.float64)
    matrix[0, 0] = 1.0 - 2.0 * (y * y + z * z)
    matrix[0, 1] = 2.0 * (x * y - z * w)
    matrix[0, 2] = 2.0 * (x * z + y * w)
    matrix[1, 0] = 2.0 * (x * y + z * w)
    matrix[1, 1] = 1.0 - 2.0 * (x * x + z * z)
    matrix[1, 2] = 2.0 * (y * z - x * w)
    matrix[2, 0] = 2.0 * (x * z - y * w)
    matrix[2, 1] = 2.0 * (y * z + x * w)
    matrix[2, 2] = 1.0 - 2.0 * (x * x + y * y)
    return matrix
