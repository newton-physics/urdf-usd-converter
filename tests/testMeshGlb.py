# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import numpy as np
import trimesh
import usdex.core
import usdex.test
from PIL import Image
from pxr import Gf, Tf, Usd, UsdGeom, UsdShade

import urdf_usd_converter
from tests.util.ConverterTestCase import ConverterTestCase


def _write_glb(path: pathlib.Path):
    base_color = Image.fromarray(np.full((2, 2, 4), [255, 128, 64, 128], dtype=np.uint8))
    metallic_roughness = Image.fromarray(np.full((2, 2, 3), [0, 128, 64], dtype=np.uint8))
    normal = Image.fromarray(np.full((2, 2, 3), [128, 128, 255], dtype=np.uint8))
    emissive = Image.fromarray(np.full((2, 2, 3), [32, 64, 128], dtype=np.uint8))
    material = trimesh.visual.material.PBRMaterial(
        name="test_material",
        baseColorFactor=[64, 128, 191, 128],
        baseColorTexture=base_color,
        metallicFactor=0.2,
        metallicRoughnessTexture=metallic_roughness,
        roughnessFactor=0.4,
        emissiveFactor=[0.1, 0.2, 0.3],
        emissiveTexture=emissive,
        normalTexture=normal,
        alphaMode="MASK",
        alphaCutoff=0.25,
    )
    visual = trimesh.visual.texture.TextureVisuals(
        uv=np.asarray([[0, 0], [1, 0], [0, 1]], dtype=np.float64),
        material=material,
    )
    mesh = trimesh.Trimesh(
        vertices=[[0, 0, 0], [1, 0, 0], [0, 1, 0]],
        faces=[[0, 1, 2]],
        vertex_normals=[[0, 0, 1]] * 3,
        visual=visual,
        process=False,
    )
    scene = trimesh.Scene()
    for name, translation in [("first", [1, 2, 3]), ("second", [-2, 0, 0])]:
        scene.add_geometry(
            mesh,
            node_name=name,
            geom_name="triangle",
            transform=trimesh.transformations.translation_matrix(translation),
        )
    path.write_bytes(scene.export(file_type="glb"))


def _write_urdf(path: pathlib.Path):
    path.write_text(
        """<?xml version="1.0"?>
<robot name="glb_import">
  <link name="link_glb">
    <visual>
      <geometry><mesh filename="static_mesh.glb"/></geometry>
    </visual>
  </link>
</robot>
""",
        encoding="utf-8",
    )


class TestMeshGlb(ConverterTestCase):
    def test_static_glb_scene(self):
        root = pathlib.Path(self.tmpDir())
        glb_path = root / "static_mesh.glb"
        urdf_path = root / "glb_import.urdf"
        _write_glb(glb_path)
        _write_urdf(urdf_path)

        asset_path = urdf_usd_converter.Converter().convert(urdf_path, root / "output")

        stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)
        root_prim = stage.GetPrimAtPath("/glb_import/Geometry/link_glb/static_mesh")
        meshes = [UsdGeom.Mesh(child) for child in root_prim.GetChildren() if child.IsA(UsdGeom.Mesh)]
        self.assertEqual(len(meshes), 2)

        first_points = sorted(tuple(mesh.GetPointsAttr().Get()[0]) for mesh in meshes)
        self.assertEqual(first_points, [(-2.0, 0.0, 0.0), (1.0, 2.0, 3.0)])
        for mesh in meshes:
            self.assertEqual(list(mesh.GetFaceVertexCountsAttr().Get()), [3])
            self.assertEqual(list(mesh.GetFaceVertexIndicesAttr().Get()), [0, 1, 2])

            primvars = UsdGeom.PrimvarsAPI(mesh)
            normals = primvars.GetPrimvar("normals")
            uvs = primvars.GetPrimvar("st")
            self.assertEqual(normals.GetInterpolation(), UsdGeom.Tokens.vertex)
            self.assertEqual(uvs.GetInterpolation(), UsdGeom.Tokens.vertex)
            self.assertEqual(len(normals.Get()), 1)
            self.assertEqual(len(uvs.Get()), 3)

        material = UsdShade.Material(stage.GetPrimAtPath("/glb_import/Materials/test_material"))
        self.assertTrue(material)
        self.assertTrue(
            Gf.IsClose(
                self._texture_scale(material, "DiffuseTexture"),
                Gf.Vec4f(64 / 255, 128 / 255, 191 / 255, 128 / 255),
                1e-6,
            )
        )
        self.assertEqual(self._texture_scale(material, "RoughnessTexture"), Gf.Vec4f(0.4))
        self.assertEqual(self._texture_scale(material, "MetallicTexture"), Gf.Vec4f(0.2))
        self.assertEqual(self._texture_scale(material, "OpacityTexture"), Gf.Vec4f(128 / 255))
        self.assertEqual(self._texture_scale(material, "EmissiveTexture"), Gf.Vec4f(0.1, 0.2, 0.3, 1.0))
        self.assertEqual(material.GetInput("opacityThreshold").Get(), 0.25)
        for mesh in meshes:
            self.check_material_binding(mesh.GetPrim(), material)

        texture_dir = root / "output" / "Payload" / "Textures"
        self.assertEqual(len(list(texture_dir.iterdir())), 4)
        for input_name in ["diffuseColor", "emissiveColor", "normal", "roughness", "metallic", "opacity"]:
            texture_path = self.get_material_texture_path(material, input_name)
            self.assertTrue((root / "output" / "Payload" / texture_path).is_file())
        self.assertEqual(self._connected_output(material, "roughness"), "g")
        self.assertEqual(self._connected_output(material, "metallic"), "b")
        self.assertEqual(self._connected_output(material, "opacity"), "a")

    def test_invalid_glb_uses_existing_mesh_warning(self):
        root = pathlib.Path(self.tmpDir())
        glb_path = root / "static_mesh.glb"
        urdf_path = root / "glb_import.urdf"
        _write_glb(glb_path)
        glb_path.write_bytes(b"BAD!" + glb_path.read_bytes()[4:])
        _write_urdf(urdf_path)

        with usdex.test.ScopedDiagnosticChecker(
            self,
            [(Tf.TF_DIAGNOSTIC_WARNING_TYPE, ".*Failed to convert mesh:.*incorrect header on GLB file.*")],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            asset_path = urdf_usd_converter.Converter().convert(urdf_path, root / "output")

        self.assertIsValidUsd(Usd.Stage.Open(asset_path.path))

    @staticmethod
    def _connected_output(material: UsdShade.Material, input_name: str) -> str:
        surface = usdex.core.computeEffectivePreviewSurfaceShader(material)
        return str(surface.GetInput(input_name).GetConnectedSource()[1])

    @staticmethod
    def _texture_scale(material: UsdShade.Material, shader_name: str) -> Gf.Vec4f:
        return UsdShade.Shader(material.GetPrim().GetChild(shader_name)).GetInput("scale").Get()
