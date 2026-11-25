# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
"""
Materials.

Reads core pbrMetallicRoughness plus normal, occlusion, emissive, and alpha.
`KHR_materials_*` payloads are discarded after a warning. `KHR_texture_transform`
is kept on the texture reference and is not baked into UV arrays.
"""

from dataclasses import dataclass

import numpy as np

from .errors import warn_once

__all__ = ["BLEND", "MASK", "OPAQUE", "Material", "TextureRef", "TextureTransform", "parse_materials"]

# material.alphaMode values. OPAQUE is the default.
BLEND = "BLEND"
MASK = "MASK"
OPAQUE = "OPAQUE"


@dataclass
class TextureTransform:
    """KHR_texture_transform parameters. They are not applied to UV arrays here."""

    offset: np.ndarray
    rotation: float
    scale: np.ndarray
    tex_coord: int | None


@dataclass
class TextureRef:
    """A material reference to one texture."""

    index: int
    tex_coord: int
    transform: TextureTransform | None = None
    scale: float | None = None
    strength: float | None = None


@dataclass
class Material:
    """One material. Missing factors are filled with the glTF defaults."""

    name: str | None
    base_color_factor: np.ndarray
    base_color_texture: TextureRef | None
    metallic_factor: float
    roughness_factor: float
    metallic_roughness_texture: TextureRef | None
    normal_texture: TextureRef | None
    occlusion_texture: TextureRef | None
    emissive_factor: np.ndarray
    emissive_texture: TextureRef | None
    alpha_mode: str
    alpha_cutoff: float
    double_sided: bool


def parse_materials(document: dict, warnings: list[str]) -> list[Material]:
    materials: list[Material] = []
    for raw in document.get("materials", []):
        _warn_material_extensions(raw.get("extensions", {}), warnings)
        pbr = raw.get("pbrMetallicRoughness", {})
        materials.append(
            Material(
                name=None if raw.get("name") is None else str(raw["name"]),
                base_color_factor=np.array(pbr.get("baseColorFactor", [1.0, 1.0, 1.0, 1.0]), dtype=np.float64),
                base_color_texture=_texture_ref(pbr.get("baseColorTexture")),
                metallic_factor=float(pbr.get("metallicFactor", 1.0)),
                roughness_factor=float(pbr.get("roughnessFactor", 1.0)),
                metallic_roughness_texture=_texture_ref(pbr.get("metallicRoughnessTexture")),
                normal_texture=_texture_ref(raw.get("normalTexture"), scale_key="scale"),
                occlusion_texture=_texture_ref(raw.get("occlusionTexture"), strength_key="strength"),
                emissive_factor=np.array(raw.get("emissiveFactor", [0.0, 0.0, 0.0]), dtype=np.float64),
                emissive_texture=_texture_ref(raw.get("emissiveTexture")),
                alpha_mode=str(raw.get("alphaMode", OPAQUE)),
                alpha_cutoff=float(raw.get("alphaCutoff", 0.5)),
                double_sided=bool(raw.get("doubleSided", False)),
            )
        )
    return materials


def _warn_material_extensions(extensions: dict, warnings: list[str]) -> None:
    for name in extensions:
        if name.startswith("KHR_materials_"):
            warn_once(warnings, f"{name} is ignored with a warning. Only core PBR is used.")


def _texture_ref(raw: dict | None, *, scale_key: str | None = None, strength_key: str | None = None) -> TextureRef | None:
    if not raw:
        return None
    transform = None
    tex_coord = int(raw.get("texCoord", 0))
    extension = raw.get("extensions", {}).get("KHR_texture_transform")
    if extension:
        if "texCoord" in extension:
            tex_coord = int(extension["texCoord"])
        transform = TextureTransform(
            offset=np.array(extension.get("offset", [0.0, 0.0]), dtype=np.float64),
            rotation=float(extension.get("rotation", 0.0)),
            scale=np.array(extension.get("scale", [1.0, 1.0]), dtype=np.float64),
            tex_coord=None if "texCoord" not in extension else int(extension["texCoord"]),
        )
    return TextureRef(
        index=int(raw["index"]),
        tex_coord=tex_coord,
        transform=transform,
        scale=None if scale_key is None or scale_key not in raw else float(raw[scale_key]),
        strength=None if strength_key is None or strength_key not in raw else float(raw[strength_key]),
    )
