# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

from pxr import Gf

__all__ = ["MaterialData"]


class MaterialData:
    """
    Temporary data when storing material.
    """

    def __init__(self):
        # The path to the mesh file. For global materials used in URDF, None is entered.
        self.mesh_file_path: pathlib.Path | None = None

        # The name of the material.
        # For dae files, the material name or material ID.
        self.name: str | None = None

        # In the case of dae, the material ID is stored in self.name.
        # If there are no duplicate material names within each dae file, the identifier used at this time will be the "material name".
        # If material names are duplicated, the "material ID" will be used as the distinguishing identifier.
        self.use_material_id: bool = False

        # Material name for the dae file.
        # Duplicate names may exist within the dae file.
        # This is the name used for the displayName of the material prim in USD.
        self.material_name: str | None = None

        # The safe name of the material.
        # This is a unique name that does not overlap with other material names.
        self.safe_name: str | None = None

        # The material properties.
        self.diffuse_color: Gf.Vec3f = Gf.Vec3f(1.0, 1.0, 1.0)
        self.specular_color: Gf.Vec3f = Gf.Vec3f(0.0, 0.0, 0.0)
        self.emissive_color: Gf.Vec3f = Gf.Vec3f(0.0, 0.0, 0.0)
        self.opacity: float = 1.0
        self.roughness: float = 0.5
        self.metallic: float = 0.0
        self.ior: float = 0.0

        # True when diffuse_color and emissive_color are already linear (glTF).
        # Otherwise they are sRGB values and are converted to linear when the material is authored.
        self.linear_color: bool = False

        # The opacity threshold (UsdPreviewSurface opacityThreshold). None leaves the threshold unset.
        self.opacity_threshold: float | None = None

        self.diffuse_texture_path: pathlib.Path | None = None
        self.specular_texture_path: pathlib.Path | None = None
        self.emissive_texture_path: pathlib.Path | None = None
        self.normal_texture_path: pathlib.Path | None = None
        self.roughness_texture_path: pathlib.Path | None = None
        self.metallic_texture_path: pathlib.Path | None = None
        self.opacity_texture_path: pathlib.Path | None = None

        # When True, the alpha channel of the diffuse texture drives the opacity (glTF baseColorTexture).
        self.opacity_from_diffuse_texture: bool = False

        # A packed texture where G is roughness and B is metallic (glTF metallicRoughnessTexture).
        # R is the occlusion only when occlusion_in_orm_texture is True. Otherwise R is unused.
        self.orm_texture_path: pathlib.Path | None = None
        self.occlusion_in_orm_texture: bool = False

    def get_display_name(self) -> str:
        """
        Get the display name of the material.
        """
        return self.material_name if self.material_name else self.name
