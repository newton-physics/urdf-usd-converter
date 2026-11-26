# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
"""
Meshes and primitives.

Attributes are NumPy arrays taken from accessors. Node transforms are not applied.
Non-normalized integer attributes become float32 without scaling (an unsigned
short value of 2 becomes 2.0). `JOINTS_*` stays integer because those values
are joint indices.
"""

from dataclasses import dataclass

import numpy as np

from .binary import read_accessor
from .errors import warn_once

__all__ = [
    "TRIANGLES",
    "TRIANGLE_FAN",
    "TRIANGLE_MODES",
    "TRIANGLE_STRIP",
    "Mesh",
    "Primitive",
    "parse_meshes",
]

# glTF triangle primitive modes. TRIANGLES is the default.
TRIANGLES = 4
TRIANGLE_STRIP = 5
TRIANGLE_FAN = 6
TRIANGLE_MODES = {TRIANGLES, TRIANGLE_STRIP, TRIANGLE_FAN}

_DRACO = "KHR_draco_mesh_compression"
_MESHOPT = ("KHR_meshopt_compression", "EXT_meshopt_compression")
# Compressed primitives cannot be read. They are skipped with a warning (the loader fails earlier when they are required).
_COMPRESSION_EXTENSIONS = {_DRACO, *_MESHOPT}


@dataclass
class Primitive:
    """
    One primitive. The default `mode` is TRIANGLES (4).

    - `mode`: 0 POINTS, 1 LINES, 2 LINE_LOOP, 3 LINE_STRIP, 4 TRIANGLES,
      5 TRIANGLE_STRIP, 6 TRIANGLE_FAN. Modes 0-3 are kept and warned about.
    - `attributes`: glTF attribute name to a per-vertex NumPy array.
      Node transforms are not applied. `TANGENT`, `JOINTS_n`, and
      `WEIGHTS_n` are preserved for the converter to ignore.
    - `indices`: vertex indices, or None when the vertex order is the topology.
    - `material`: index into `Gltf.materials`, or None when unassigned.
    """

    mode: int
    attributes: dict[str, np.ndarray]
    indices: np.ndarray | None
    material: int | None


@dataclass
class Mesh:
    """One mesh. Primitive order is preserved."""

    name: str | None
    primitives: list[Primitive]


def parse_meshes(document: dict, buffers: list[bytes], warnings: list[str]) -> list[Mesh]:
    # Reuse an accessor array when several primitives reference it.
    cache: dict[tuple[int, str], np.ndarray] = {}

    def cached_attribute(name: str, accessor_index: int) -> np.ndarray:
        kind = "joints" if name.startswith("JOINTS_") else "vertex"
        key = (accessor_index, kind)
        found = cache.get(key)
        if found is None:
            found = _attribute_array(document, buffers, accessor_index, name)
            cache[key] = found
        return found

    def cached_indices(accessor_index: int) -> np.ndarray:
        key = (accessor_index, "indices")
        found = cache.get(key)
        if found is None:
            found = read_accessor(document, buffers, accessor_index, vertex_attribute=False)
            cache[key] = found
        return found

    seen_compression: set[str] = set()
    meshes: list[Mesh] = []
    for mesh_index, raw in enumerate(document.get("meshes", [])):
        primitives: list[Primitive] = []
        for primitive_index, primitive in enumerate(raw.get("primitives", [])):
            compression = _compression_name(document, primitive)
            if compression is not None:
                seen_compression.add(compression)
                warn_once(
                    warnings,
                    f"{compression} is not supported. meshes[{mesh_index}].primitives[{primitive_index}] was skipped because it cannot be read.",
                )
                continue
            extensions = primitive.get("extensions", {})
            if "KHR_gaussian_splatting" in extensions:
                # The primitive is skipped whatever its mode, as the concept mapping says.
                warn_once(warnings, "KHR_gaussian_splatting is skipped.")
                continue
            mode = int(primitive.get("mode", TRIANGLES))
            if mode not in TRIANGLE_MODES:
                warn_once(warnings, f"Contains non-triangle primitives (mode={mode}). The converter skips them.")
            attributes = {name: cached_attribute(name, int(accessor_index)) for name, accessor_index in primitive.get("attributes", {}).items()}
            indices = None
            if "indices" in primitive:
                indices = cached_indices(int(primitive["indices"]))
            material = None if primitive.get("material") is None else int(primitive["material"])
            primitives.append(Primitive(mode=mode, attributes=attributes, indices=indices, material=material))
        meshes.append(Mesh(name=None if raw.get("name") is None else str(raw["name"]), primitives=primitives))
    for name in document.get("extensionsUsed", []):
        if name in _COMPRESSION_EXTENSIONS and name not in seen_compression:
            warn_once(warnings, f"{name} is not supported.")
    return meshes


def _compression_name(document: dict, primitive: dict) -> str | None:
    """Return the compression extension on this primitive, if it cannot be read."""
    if _DRACO in primitive.get("extensions", {}):
        return _DRACO
    accessor_indices = [int(value) for value in primitive.get("attributes", {}).values()]
    if "indices" in primitive:
        accessor_indices.append(int(primitive["indices"]))
    for accessor_index in accessor_indices:
        meshopt = _meshopt_name(document, accessor_index)
        if meshopt is not None:
            return meshopt
    return None


def _meshopt_name(document: dict, accessor_index: int) -> str | None:
    """Return the meshopt extension on the bufferView of this accessor, if any."""
    accessor = document["accessors"][accessor_index]
    if "bufferView" not in accessor:
        return None
    extensions = document["bufferViews"][int(accessor["bufferView"])].get("extensions") or {}
    for name in _MESHOPT:
        if name in extensions:
            return name
    return None


def _attribute_array(document: dict, buffers: list[bytes], accessor_index: int, name: str) -> np.ndarray:
    values = read_accessor(document, buffers, accessor_index, vertex_attribute=True)
    if name.startswith("JOINTS_"):
        return values
    if values.dtype.kind == "f" and values.dtype.itemsize == 4:
        return values
    return values.astype(np.float32, copy=False)
