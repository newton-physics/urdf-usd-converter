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
        self.name: str | None = None

        # The safe name of the material.
        # This is a unique name that does not overlap with other material names.
        self.safe_name: str | None = None

        # The material properties.
        self.diffuse_color: Gf.Vec3f = Gf.Vec3f(1.0, 1.0, 1.0)
        self.specular_color: Gf.Vec3f = Gf.Vec3f(0.0, 0.0, 0.0)
        self.opacity: float = 1.0
        self.roughness: float = 0.5
        self.metallic: float = 0.0

        self.diffuse_texture_path: pathlib.Path | None = None
        self.specular_texture_path: pathlib.Path | None = None
        self.normal_texture_path: pathlib.Path | None = None
        self.roughness_texture_path: pathlib.Path | None = None
        self.metallic_texture_path: pathlib.Path | None = None
        self.opacity_texture_path: pathlib.Path | None = None
