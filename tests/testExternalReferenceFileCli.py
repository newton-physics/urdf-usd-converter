# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import os
import shutil
from pathlib import Path
from unittest.mock import patch

from pxr import Usd, UsdGeom, UsdShade

from tests.util.ConverterTestCase import ConverterTestCase
from urdf_usd_converter._impl.cli import run


class TestExternalReferenceFileCli(ConverterTestCase):
    def test_external_reference_file(self):
        """
        If the mesh or texture URI specifies "file:///path/to/file.png" or "file://path/to/file.png"
        """
        # URDF files, texture files, and mesh files are copied to the temporary directory.
        # External references within the urdf file will have "file:///" replaced with the path to the working directory.
        source_path = "tests/data/external_reference_file.urdf"
        source_texture_path = "tests/data/assets/grid.png"
        source_mesh_path = "tests/data/assets/box.obj"
        dist_dir = self.tmpDir()

        # Create an "urdf" directory in data_dir and copy the urdf and related files into it.
        data_dir = Path(dist_dir) / "urdf"
        data_asset_dir = data_dir / "assets"
        data_dir.mkdir(parents=True, exist_ok=True)
        data_asset_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy(source_path, data_dir)
        shutil.copy(source_texture_path, data_asset_dir)
        shutil.copy(source_mesh_path, data_asset_dir)

        input_path = f"{data_dir.as_posix()}/external_reference_file.urdf"
        output_dir = self.tmpDir()

        # Find <texture filename="file:///home/user/urdf/assets/grid.png"/> in the URDF at input_path,
        # and, filename will be replaced with the absolute path specific to each OS.
        source_texture_path = "file:///home/user/urdf/assets/grid.png"
        target_texture_path = f"file:///{data_dir.as_posix()}/assets/grid.png" if os.name == "nt" else f"file://{data_dir.as_posix()}/assets/grid.png"

        with Path.open(input_path) as file:
            content = file.read()
            content = content.replace(source_texture_path, target_texture_path)
            with Path.open(input_path, "w") as file:
                file.write(content)

        # Convert the URDF file to USD.
        with patch("sys.argv", ["urdf_usd_converter", input_path, output_dir]):
            self.assertEqual(run(), 0, f"Failed to convert {input_path}")

        # Check the USD file after converting external_reference_file.urdf.
        usd_path = Path(output_dir) / "external_reference_file.usda"
        self.assertTrue(usd_path.exists())

        self.stage: Usd.Stage = Usd.Stage.Open(str(usd_path))
        self.assertIsValidUsd(self.stage)

        default_prim = self.stage.GetDefaultPrim()
        geometry_scope_prim = self.stage.GetPrimAtPath(default_prim.GetPath().AppendChild("Geometry"))
        self.assertTrue(geometry_scope_prim.IsValid())

        # Check the mesh reference.
        base_link_prim = geometry_scope_prim.GetChild("BaseLink")
        self.assertTrue(base_link_prim.IsValid())
        base_link_box_prim = base_link_prim.GetChild("box")
        self.assertTrue(base_link_box_prim.IsValid())
        self.assertTrue(base_link_box_prim.IsA(UsdGeom.Mesh))
        self.assertTrue(base_link_box_prim.HasAuthoredReferences())

        # Check material.
        material_scope_prim = default_prim.GetChild("Materials")
        self.assertTrue(material_scope_prim.IsValid())
        texture_material_prim = material_scope_prim.GetChild("texture_material")
        self.assertTrue(texture_material_prim.IsValid())
        self.assertTrue(texture_material_prim.IsA(UsdShade.Material))
        texture_material = UsdShade.Material(texture_material_prim)
        self.assertTrue(texture_material.GetPrim().HasAuthoredReferences())
        texture_path = self.get_material_texture_path(texture_material, "diffuseColor")
        self.assertEqual(texture_path, Path("./Textures/grid.png"))
