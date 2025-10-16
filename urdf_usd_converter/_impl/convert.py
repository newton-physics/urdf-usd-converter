# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
import pathlib
from dataclasses import dataclass

import usdex.core
from pxr import Sdf, Tf, UsdGeom

from .utils import get_authoring_metadata

__all__ = ["Converter"]


class Converter:
    @dataclass
    class Params:
        layer_structure: bool = True
        scene: bool = True
        comment: str = ""

    def __init__(self, layer_structure: bool = True, scene: bool = True, comment: str = ""):
        self.params = self.Params(layer_structure=layer_structure, scene=scene, comment=comment)

    def convert(self, input_file: str, output_dir: str) -> Sdf.AssetPath:
        """
        Convert a URDF to a USD stage.

        Args:
            input_file: Path to the input URDF file.
            output_dir: Path to the output USD directory.

        Returns:
            The path to the created USD asset.

        Raises:
            ValueError: If input_file does not exist or is not a readable file.
            ValueError: If input_file cannot be parsed as a valid URDF.
            ValueError: If output_dir exists but is not a directory.
        """
        input_path = pathlib.Path(input_file)
        if not input_path.exists() or not input_path.is_file():
            raise ValueError(f"Input file {input_file} is not a readable file")

        output_path = pathlib.Path(output_dir)
        if output_path.exists() and not output_path.is_dir():
            raise ValueError(f"Output directory {output_dir} is not a directory")

        if not output_path.exists():
            output_path.mkdir(parents=True)

        file_name = f"{input_file.stem}.usda"
        asset_identifier = str(output_dir / file_name)
        Tf.Status(f"Converting {input_path} into {output_path}")
        asset_stage = usdex.core.createStage(
            asset_identifier,
            defaultPrimName="Robot",  # TODO: use parsed name
            upAxis=UsdGeom.Tokens.z,
            linearUnits=UsdGeom.LinearUnits.meters,
            authoringMetadata=get_authoring_metadata(),
        )
        # TODO: implement core logic
        usdex.core.saveStage(asset_stage, comment=self.params.comment)
        return Sdf.AssetPath(asset_identifier)
