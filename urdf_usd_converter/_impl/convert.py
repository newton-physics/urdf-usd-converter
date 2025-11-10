# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib
import tempfile
from dataclasses import dataclass

import usdex.core
from pxr import Sdf, Tf, Usd, UsdGeom, UsdPhysics

from ._flatten import export_flattened
from .data import ConversionData, Tokens
from .link import convert_links
from .material import convert_materials
from .mesh import convert_meshes
from .scene import convert_scene
from .urdf_parser.elements import ElementRobot
from .urdf_parser.parser import URDFParser
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

        Tf.Status(f"Converting {input_path} into {output_path}")

        # Parsing XML.
        parser = URDFParser(input_path)
        parser.parse()

        # Create the conversion data object
        data = ConversionData(
            urdf_parser=parser,
            content={},
            libraries={},
            references={},
            name_cache=usdex.core.NameCache(),
            scene=self.params.scene,
            comment=self.params.comment,
        )

        # setup the main output layer (which will become an asset interface later)
        robot_name = parser.get_robot_name()

        if not self.params.layer_structure:
            asset_dir = tempfile.mkdtemp()
            asset_format = "usdc"
        else:
            asset_dir = output_path.absolute().as_posix()
            asset_format = "usda"
        asset_stem = f"{robot_name}"
        asset_identifier = str(pathlib.Path(asset_dir) / f"{asset_stem}.{asset_format}")
        asset_name = usdex.core.getValidPrimName(robot_name)
        asset_stage = usdex.core.createStage(
            asset_identifier,
            defaultPrimName=asset_name,
            upAxis=UsdGeom.Tokens.z,
            linearUnits=UsdGeom.LinearUnits.meters,
            authoringMetadata=get_authoring_metadata(),
        )

        data.content[Tokens.Asset] = asset_stage
        data.content[Tokens.Asset].SetMetadata(UsdPhysics.Tokens.kilogramsPerUnit, 1)
        root: Usd.Prim = usdex.core.defineXform(asset_stage, asset_stage.GetDefaultPrim().GetPath()).GetPrim()
        if asset_name != robot_name:
            usdex.core.setDisplayName(root, robot_name)

        # setup the root layer of the payload
        data.content[Tokens.Contents] = usdex.core.createAssetPayload(asset_stage)

        # author the mesh library
        convert_meshes(data)
        # setup a content layer for referenced meshes
        data.content[Tokens.Geometry] = usdex.core.addAssetContent(data.content[Tokens.Contents], Tokens.Geometry, format="usda")

        # author the material library and setup the content layer for materials only if there are materials
        convert_materials(data)

        # setup a content layer for physics
        data.content[Tokens.Physics] = usdex.core.addAssetContent(data.content[Tokens.Contents], Tokens.Physics, format="usda")
        data.content[Tokens.Physics].SetMetadata(UsdPhysics.Tokens.kilogramsPerUnit, 1)
        data.references[Tokens.Physics] = {}

        # author the physics scene
        if self.params.scene:
            convert_scene(data)

        # Joints and links are converted into a hierarchical structure
        convert_links(data)

        # create the asset interface
        usdex.core.addAssetInterface(asset_stage, source=data.content[Tokens.Contents])

        # optionally flatten the asset
        if not self.params.layer_structure:
            export_flattened(asset_stage, output_dir, asset_dir, asset_stem, asset_format, self.params.comment)
        else:
            usdex.core.saveStage(asset_stage, comment=self.params.comment)

        # warn about known limitations
        self.warn(parser)

        return Sdf.AssetPath(asset_identifier)

    def warn(self, parser: URDFParser):
        element_root: ElementRobot = parser.get_root_element()

        if element_root.transmissions:
            Tf.Warn("Transmissions are not supported")

        if "gazebo" in [element.tag for element in element_root.undefined_elements]:
            Tf.Warn("Gazebo is not supported")

        for joint in element_root.joints:
            if joint.calibration:
                Tf.Warn("Calibration is not supported")
                break

        for joint in element_root.joints:
            if joint.dynamics:
                Tf.Warn("Dynamics is not supported")
                break

        for joint in element_root.joints:
            if joint.mimic:
                Tf.Warn("Mimic is not supported")
                break

        for joint in element_root.joints:
            if joint.safety_controller:
                Tf.Warn("Safety controller is not supported")
                break
