# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
"""
Assemble a glTF document into scene, geometry, material, and texture objects.

This module does not author USD. It returns the scene hierarchy, local
transforms, geometry, materials, and textures.
"""

import pathlib
from dataclasses import dataclass, field

from .asset import AssetInfo, parse_asset
from .errors import GltfError, warn_once
from .geometry import Mesh, parse_meshes
from .io import GltfSource, read_source
from .material import Material, parse_materials
from .scene import Node, Scene, parse_nodes, parse_scenes
from .texture import Image, Sampler, Texture, parse_images, parse_samplers, parse_textures

__all__ = ["Gltf", "load"]

# Extensions that may appear in extensionsRequired without failing the load.
_SUPPORTED_REQUIRED = {
    "KHR_mesh_quantization",
    "KHR_texture_transform",
}

# Extensions that are warned about and skipped even when they appear in extensionsRequired.
_WARNED_REQUIRED = {
    "KHR_gaussian_splatting",
}


@dataclass
class Gltf:
    """Loaded asset. Arrays are NumPy arrays. Names are Python strings."""

    asset: AssetInfo
    scenes: list[Scene]
    scene: int | None
    nodes: list[Node]
    meshes: list[Mesh]
    materials: list[Material]
    textures: list[Texture]
    samplers: list[Sampler]
    images: list[Image]
    extensions_used: list[str] = field(default_factory=list)
    extensions_required: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


def load(path: str | pathlib.Path) -> Gltf:
    """
    Read a `.gltf` or `.glb` file.

    This is the only entry point of the loader. One call reads the file and returns the document.
    Non-fatal issues are collected in `Gltf.warnings`.

    Args:
        path: The path to the glTF or GLB file.

    Returns:
        The loaded glTF document.

    Raises:
        GltfError: If the file cannot be read, or if it is malformed (e.g. a required property is missing or an index is out of range).
    """
    try:
        return _load(pathlib.Path(path))
    except GltfError:
        raise
    except (KeyError, IndexError, TypeError, ValueError) as exc:
        # A malformed document surfaces as one of these when a property is missing, has the wrong type, or refers to a missing item.
        raise GltfError(f"The glTF file is malformed: {type(exc).__name__}: {exc}") from exc


def _load(path: pathlib.Path) -> Gltf:
    source = read_source(path)
    document = source.document
    warnings = source.warnings
    used, required = _check_extensions(document, warnings)
    if document.get("animations"):
        warn_once(warnings, "Animations are skipped.")
    if document.get("skins"):
        warn_once(warnings, "Skins are skipped.")
    scenes, scene = parse_scenes(document)
    try:
        return Gltf(
            asset=parse_asset(document),
            scenes=scenes,
            scene=scene,
            nodes=parse_nodes(document),
            meshes=parse_meshes(document, source.buffers, warnings),
            materials=parse_materials(document, warnings),
            textures=parse_textures(document),
            samplers=parse_samplers(document),
            images=parse_images(document, source.buffers, source.base_dir, warnings),
            extensions_used=used,
            extensions_required=required,
            warnings=warnings,
        )
    except GltfError as exc:
        raise _with_skipped_buffers(exc, source) from exc


def _with_skipped_buffers(error: GltfError, source: GltfSource) -> GltfError:
    """
    Name the skipped buffers in the error. A mesh or an image may read from one of them, and the cause is not otherwise reported.
    """
    if not source.skipped_buffers:
        return error
    reasons = " ".join(f"buffers[{index}]: {reason}." for index, reason in source.skipped_buffers.items())
    return GltfError(f"{error} Skipped buffers: {reasons}")


def _check_extensions(document: dict, warnings: list[str]) -> tuple[list[str], list[str]]:
    used = [str(name) for name in document.get("extensionsUsed", [])]
    required = [str(name) for name in document.get("extensionsRequired", [])]
    missing = sorted(set(required) - set(used))
    if missing:
        raise GltfError(f"extensionsRequired is not listed in extensionsUsed: {', '.join(missing)}")
    # Draco and meshopt fail here when required. When only used, parse_meshes skips the primitives that cannot be read.
    unsupported = sorted(set(required) - _SUPPORTED_REQUIRED - _WARNED_REQUIRED)
    if unsupported:
        raise GltfError(f"Unsupported required extension: {', '.join(unsupported)}")
    if "KHR_gaussian_splatting" in used:
        warn_once(warnings, "KHR_gaussian_splatting is skipped.")
    for name in used:
        if name.startswith("KHR_materials_"):
            warn_once(warnings, f"{name} is ignored with a warning. Only core PBR is used.")
    return used, required
