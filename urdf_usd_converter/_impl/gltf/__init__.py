# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
"""
Read glTF 2.0 assets into Python objects.

This package does not import USD. `conversion_gltf.py` turns the returned objects into USD.

`load` is the entry point. One call reads the file and returns a `Gltf` document.

.. code-block:: python

    from . import gltf

    model = gltf.load(path)
    scene = model.scenes[model.scene]
    for node_index in scene.nodes:
        node = model.nodes[node_index]
        local = gltf.local_matrix(node)

Callers walk attributes on the returned objects. An unreadable file raises `GltfError`.
Non-fatal issues found by the loader are collected in `Gltf.warnings`.
"""

from .asset import AssetInfo
from .errors import GltfError
from .geometry import TRIANGLE_FAN, TRIANGLE_MODES, TRIANGLE_STRIP, TRIANGLES, Mesh, Primitive
from .loader import Gltf, load
from .material import BLEND, MASK, OPAQUE, Material, TextureRef, TextureTransform
from .scene import Node, Scene, local_matrix
from .texture import CLAMP_TO_EDGE, MIRRORED_REPEAT, REPEAT, Image, Sampler, Texture

__all__ = [
    "BLEND",
    "CLAMP_TO_EDGE",
    "MASK",
    "MIRRORED_REPEAT",
    "OPAQUE",
    "REPEAT",
    "TRIANGLES",
    "TRIANGLE_FAN",
    "TRIANGLE_MODES",
    "TRIANGLE_STRIP",
    "AssetInfo",
    "Gltf",
    "GltfError",
    "Image",
    "Material",
    "Mesh",
    "Node",
    "Primitive",
    "Sampler",
    "Scene",
    "Texture",
    "TextureRef",
    "TextureTransform",
    "load",
    "local_matrix",
]
