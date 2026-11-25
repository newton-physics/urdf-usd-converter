# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import base64
import json
import pathlib
import struct
import zlib

import numpy as np
import usdex.core
import usdex.test
from pxr import Gf, Tf, Usd, UsdGeom, UsdShade

import urdf_usd_converter
from tests.util.ConverterTestCase import ConverterTestCase


def _png() -> bytes:
    """A 1x1 RGBA PNG."""

    def chunk(tag: bytes, body: bytes) -> bytes:
        return struct.pack(">I", len(body)) + tag + body + struct.pack(">I", zlib.crc32(tag + body) & 0xFFFFFFFF)

    header = struct.pack(">IIBBBBB", 1, 1, 8, 6, 0, 0, 0)
    pixel = b"\x00\xff\x80\x40\xff"  # filter type 0, then RGBA
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", zlib.compress(pixel)) + chunk(b"IEND", b"")


def _data_uri(data: bytes, mime_type: str) -> str:
    return f"data:{mime_type};base64,{base64.b64encode(data).decode()}"


def _warning(pattern: str) -> tuple:
    return (Tf.TF_DIAGNOSTIC_WARNING_TYPE, pattern)


class TestMaterialGltf(ConverterTestCase):
    """
    Materials of glTF meshes. The glTF files are written by the tests: one triangle per material.
    The tests check the UsdPreviewSurface network, and the OpenPBR network where it differs.
    """

    def _convert(
        self,
        materials: list[dict],
        textures: list[dict] | None = None,
        images: list[dict] | None = None,
        samplers: list[dict] | None = None,
        diagnostics: list[tuple] | None = None,
        openpbr: bool = True,
        extensions: list[str] | None = None,
        extra_files: dict[str, bytes] | None = None,
    ) -> tuple[Usd.Stage, pathlib.Path]:
        """
        Write model.gltf (one triangle per material) and a URDF for it, then convert.
        Returns the stage and the directory of the converted asset.
        """
        # The i-th triangle is placed at x = 2 * i, so the triangles do not touch each other.
        buffer = b""
        accessors = []
        buffer_views = []
        primitives = []
        for i in range(len(materials)):
            arrays = {
                "POSITION": (np.array([[2 * i, 0, 0], [2 * i + 1, 0, 0], [2 * i, 1, 0]], dtype=np.float32), "VEC3"),
                "NORMAL": (np.array([[0, 0, 1]] * 3, dtype=np.float32), "VEC3"),
                "TEXCOORD_0": (np.array([[0, 0], [1, 0], [0, 1]], dtype=np.float32), "VEC2"),
            }
            attributes = {}
            for name, (array, accessor_type) in arrays.items():
                attributes[name] = len(accessors)
                accessors.append({"bufferView": len(buffer_views), "componentType": 5126, "count": 3, "type": accessor_type})
                if name == "POSITION":
                    accessors[-1].update({"min": array.min(axis=0).tolist(), "max": array.max(axis=0).tolist()})
                buffer_views.append({"buffer": 0, "byteOffset": len(buffer), "byteLength": array.nbytes})
                buffer += array.tobytes()
            primitives.append({"attributes": attributes, "material": i})

        document = {
            "asset": {"version": "2.0"},
            "scene": 0,
            "scenes": [{"nodes": [0]}],
            "nodes": [{"mesh": 0}],
            "meshes": [{"primitives": primitives}],
            "accessors": accessors,
            "bufferViews": buffer_views,
            "buffers": [{"uri": _data_uri(buffer, "application/octet-stream"), "byteLength": len(buffer)}],
            "materials": materials,
        }
        if textures:
            document["textures"] = textures
        if images:
            document["images"] = images
        if samplers:
            document["samplers"] = samplers
        if extensions:
            document["extensionsUsed"] = extensions

        source_dir = pathlib.Path(self.tmpDir())
        (source_dir / "model.gltf").write_text(json.dumps(document), encoding="utf-8")
        for relative_path, content in (extra_files or {}).items():
            (source_dir / relative_path).parent.mkdir(parents=True, exist_ok=True)
            (source_dir / relative_path).write_bytes(content)
        (source_dir / "robot.urdf").write_text(
            """<?xml version="1.0"?>
<robot name="robot">
  <link name="base">
    <visual><geometry><mesh filename="model.gltf"/></geometry></visual>
    <collision><geometry><box size="1 1 1"/></geometry></collision>
  </link>
</robot>
""",
            encoding="utf-8",
        )

        output_dir = pathlib.Path(self.tmpDir())
        with usdex.test.ScopedDiagnosticChecker(self, diagnostics or [], level=usdex.core.DiagnosticsLevel.eWarning):
            asset_path = urdf_usd_converter.Converter(openpbr=openpbr).convert(str(source_dir / "robot.urdf"), str(output_dir))
        self.assertIsNotNone(asset_path)

        stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)
        return stage, output_dir

    def _material(self, stage: Usd.Stage, name: str) -> UsdShade.Material:
        material = UsdShade.Material(stage.GetDefaultPrim().GetChild("Materials").GetChild(name))
        self.assertTrue(material, msg=name)
        return material

    def _connected(self, shader: UsdShade.Shader, input_name: str) -> tuple[UsdShade.Shader, str] | None:
        """
        The shader and the output name that an input is connected to.
        None when it is not connected, or when it is connected to an input of the material (the interface), which is a plain value.
        """
        shader_input = shader.GetInput(input_name)
        if not shader_input or not shader_input.HasConnectedSource():
            return None
        source = shader_input.GetConnectedSource()
        if not source[0].GetPrim().IsA(UsdShade.Shader):
            return None
        return UsdShade.Shader(source[0].GetPrim()), source[1]

    def _preview(self, material: UsdShade.Material) -> UsdShade.Shader:
        return usdex.core.computeEffectivePreviewSurfaceShader(material)

    def _texture_file(self, shader: UsdShade.Shader) -> pathlib.Path:
        return pathlib.Path(self._get_input_value(shader, "file").path)

    def test_embedded_image_is_written_to_textures(self):
        # A data URI image has no file, so the bytes are written to the Textures directory under the name of the glTF file and the image.
        png = _png()
        stage, output_dir = self._convert(
            [{"name": "textured", "pbrMetallicRoughness": {"baseColorTexture": {"index": 0}}}],
            textures=[{"source": 0}],
            images=[{"name": "tex", "mimeType": "image/png", "uri": _data_uri(png, "image/png")}],
        )

        material = self._material(stage, "textured")
        self.assertEqual(self.get_material_texture_path(material, "diffuseColor"), pathlib.Path("./Textures/model_tex.png"))
        self.assertEqual((output_dir / "Payload" / "Textures" / "model_tex.png").read_bytes(), png)

    def test_external_image_is_copied(self):
        # An external image is a file relative to the glTF. A percent-encoded path is decoded.
        png = _png()
        stage, output_dir = self._convert(
            [{"name": "textured", "pbrMetallicRoughness": {"baseColorTexture": {"index": 0}}}],
            textures=[{"source": 0}],
            images=[{"uri": "images/tex%2D1.png"}],
            extra_files={"images/tex-1.png": png},
        )

        material = self._material(stage, "textured")
        self.assertEqual(self.get_material_texture_path(material, "diffuseColor"), pathlib.Path("./Textures/tex-1.png"))
        self.assertEqual((output_dir / "Payload" / "Textures" / "tex-1.png").read_bytes(), png)

    def test_colors_are_linear(self):
        # glTF colors are already linear, so they are authored as they are.
        stage, _ = self._convert(
            [
                {
                    "name": "colored",
                    "pbrMetallicRoughness": {"baseColorFactor": [0.5, 0.25, 0.1, 1.0], "metallicFactor": 0.25, "roughnessFactor": 0.75},
                    "emissiveFactor": [0.2, 0.1, 0.0],
                }
            ]
        )

        material = self._material(stage, "colored")
        self.assertTrue(Gf.IsClose(self.get_material_diffuse_color(material), Gf.Vec3f(0.5, 0.25, 0.1), 1e-6))
        self.assertAlmostEqual(self.get_material_metallic(material), 0.25, places=6)
        self.assertAlmostEqual(self.get_material_roughness(material), 0.75, places=6)
        self.assertTrue(Gf.IsClose(self.get_material_emissive_color(material), Gf.Vec3f(0.2, 0.1, 0.0), 1e-6))
        self.assertTrue(Gf.IsClose(self.get_openpbr_input_value(material, "base_color"), Gf.Vec3f(0.5, 0.25, 0.1), 1e-6))
        self.assertTrue(Gf.IsClose(self.get_openpbr_input_value(material, "emission_color"), Gf.Vec3f(0.2, 0.1, 0.0), 1e-6))

    def test_defaults(self):
        # The glTF defaults are white, metallic 1.0 and roughness 1.0. There is no emission.
        stage, _ = self._convert([{"name": "default"}])

        material = self._material(stage, "default")
        self.assertTrue(Gf.IsClose(self.get_material_diffuse_color(material), Gf.Vec3f(1.0, 1.0, 1.0), 1e-6))
        self.assertAlmostEqual(self.get_material_metallic(material), 1.0, places=6)
        self.assertAlmostEqual(self.get_material_roughness(material), 1.0, places=6)
        self.assertAlmostEqual(self.get_material_opacity(material), 1.0, places=6)
        self.assertFalse(self._preview(material).GetInput("emissiveColor"))

    def test_alpha_modes(self):
        stage, _ = self._convert(
            [
                {"name": "opaque", "pbrMetallicRoughness": {"baseColorFactor": [1, 1, 1, 0.3]}},
                {"name": "blend", "alphaMode": "BLEND", "pbrMetallicRoughness": {"baseColorFactor": [1, 1, 1, 0.3]}},
                {"name": "mask", "alphaMode": "MASK", "alphaCutoff": 0.4, "pbrMetallicRoughness": {"baseColorFactor": [1, 1, 1, 0.8]}},
            ]
        )

        # OPAQUE ignores the alpha.
        opaque = self._material(stage, "opaque")
        self.assertAlmostEqual(self.get_material_opacity(opaque), 1.0, places=6)
        self.assertFalse(self._preview(opaque).GetInput("opacityThreshold"))

        # BLEND uses the alpha and has no threshold.
        blend = self._material(stage, "blend")
        self.assertAlmostEqual(self.get_material_opacity(blend), 0.3, places=6)
        self.assertAlmostEqual(self.get_openpbr_input_value(blend, "geometry_opacity"), 0.3, places=6)
        self.assertFalse(self._preview(blend).GetInput("opacityThreshold"))

        # MASK uses the alpha and sets the threshold to alphaCutoff.
        mask = self._material(stage, "mask")
        self.assertAlmostEqual(self.get_material_opacity(mask), 0.8, places=6)
        self.assertAlmostEqual(self._get_input_value(self._preview(mask), "opacityThreshold"), 0.4, places=6)

    def test_unknown_alpha_mode(self):
        stage, _ = self._convert(
            [{"name": "unknown", "alphaMode": "TRANSLUCENT", "pbrMetallicRoughness": {"baseColorFactor": [1, 1, 1, 0.3]}}],
            diagnostics=[_warning('.*Material "unknown" has an unknown alphaMode "TRANSLUCENT". OPAQUE is used.*')],
        )

        self.assertAlmostEqual(self.get_material_opacity(self._material(stage, "unknown")), 1.0, places=6)

    def test_alpha_of_base_color_texture(self):
        # With BLEND or MASK the alpha channel of the base color texture is the opacity. The helpers only read a single channel
        # (the first one), so the alpha is connected here.
        png = _png()
        materials = [
            {"name": "blend", "alphaMode": "BLEND", "pbrMetallicRoughness": {"baseColorTexture": {"index": 0}}},
            {"name": "opaque", "pbrMetallicRoughness": {"baseColorTexture": {"index": 0}}},
        ]
        textures = [{"source": 0}]
        images = [{"name": "tex", "mimeType": "image/png", "uri": _data_uri(png, "image/png")}]

        stage, _ = self._convert(materials, textures=textures, images=images)

        blend = self._material(stage, "blend")
        texture, output = self._connected(self._preview(blend), "opacity")
        self.assertEqual(output, "a")
        self.assertEqual(texture.GetShaderId(), "UsdUVTexture")
        # The same texture is the diffuse color.
        self.assertEqual(self._connected(self._preview(blend), "diffuseColor")[0].GetPath(), texture.GetPath())

        # OpenPBR reads the alpha of the same file through a color4 reader.
        mtlx = usdex.core.computeEffectiveMtlxSurfaceShader(blend)
        separate, output = self._connected(mtlx, "geometry_opacity")
        self.assertEqual(separate.GetShaderId(), "ND_separate4_color4")
        self.assertEqual(output, "outa")
        reader, _ = self._connected(separate, "in")
        self.assertEqual(reader.GetShaderId(), "ND_tiledimage_color4")
        color_reader, _ = self._connected(mtlx, "base_color")
        for input_name in ("file", "texcoord"):
            self.assertEqual(reader.GetInput(input_name).GetAttr().GetConnections(), color_reader.GetInput(input_name).GetAttr().GetConnections())

        # OPAQUE ignores the alpha.
        opaque = self._material(stage, "opaque")
        self.assertIsNone(self._connected(self._preview(opaque), "opacity"))

    def test_alpha_factor_with_base_color_texture(self):
        # An alpha of the baseColorFactor below 1 is the scalar opacity. The texture is used as it is, and there is no warning
        # because the color of the factor is white.
        png = _png()
        stage, _ = self._convert(
            [
                {
                    "name": "half",
                    "alphaMode": "BLEND",
                    "pbrMetallicRoughness": {"baseColorFactor": [1, 1, 1, 0.5], "baseColorTexture": {"index": 0}},
                },
                {
                    "name": "masked",
                    "alphaMode": "MASK",
                    "alphaCutoff": 0.25,
                    "pbrMetallicRoughness": {"baseColorFactor": [1, 1, 1, 0.5], "baseColorTexture": {"index": 0}},
                },
            ],
            textures=[{"source": 0}],
            images=[{"mimeType": "image/png", "uri": _data_uri(png, "image/png")}],
            diagnostics=[],
        )

        for name in ("half", "masked"):
            material = self._material(stage, name)
            preview = self._preview(material)
            self.assertAlmostEqual(self.get_material_opacity(material), 0.5, places=6)
            self.assertIsNone(self._connected(preview, "opacity"))
            self.assertIsNotNone(self._connected(preview, "diffuseColor"))
            self.assertAlmostEqual(self.get_openpbr_input_value(material, "geometry_opacity"), 0.5, places=6)
        self.assertAlmostEqual(self._get_input_value(self._preview(self._material(stage, "masked")), "opacityThreshold"), 0.25, places=6)

    def test_alpha_of_base_color_texture_preview_material_only(self):
        png = _png()
        stage, _ = self._convert(
            [{"name": "blend", "alphaMode": "BLEND", "pbrMetallicRoughness": {"baseColorTexture": {"index": 0}}}],
            textures=[{"source": 0}],
            images=[{"mimeType": "image/png", "uri": _data_uri(png, "image/png")}],
            openpbr=False,
        )

        material = self._material(stage, "blend")
        self.assertFalse(material.GetPrim().HasAPI("MaterialXConfigAPI"))
        _, output = self._connected(self._preview(material), "opacity")
        self.assertEqual(output, "a")

    def test_base_color_factor_is_not_applied_to_texture(self):
        # The texture is used as it is, so the factor is not applied. Black would hide the texture, so the texture is not used then.
        png = _png()
        images = [{"mimeType": "image/png", "uri": _data_uri(png, "image/png")}]
        stage, _ = self._convert(
            [
                {"name": "tinted", "pbrMetallicRoughness": {"baseColorFactor": [0.5, 1, 1, 1], "baseColorTexture": {"index": 0}}},
                {"name": "black", "pbrMetallicRoughness": {"baseColorFactor": [0, 0, 0, 1], "baseColorTexture": {"index": 0}}},
            ],
            textures=[{"source": 0}],
            images=images,
            diagnostics=[_warning('.*Material "tinted" has a baseColorFactor color that is not applied to its baseColorTexture.*')],
        )

        self.assertIsNotNone(self._connected(self._preview(self._material(stage, "tinted")), "diffuseColor"))
        self.assertIsNone(self._connected(self._preview(self._material(stage, "black")), "diffuseColor"))

    def test_metallic_roughness_texture(self):
        # G is the roughness and B is the metallic. R is not an occlusion unless the occlusionTexture shares the image.
        png = _png()
        images = [
            {"name": "mr", "mimeType": "image/png", "uri": _data_uri(png, "image/png")},
            {"name": "other", "mimeType": "image/png", "uri": _data_uri(png, "image/png")},
        ]
        textures = [{"source": 0}, {"source": 0}, {"source": 1}]
        stage, output_dir = self._convert(
            [
                {"name": "mr_only", "pbrMetallicRoughness": {"metallicRoughnessTexture": {"index": 0}}},
                {"name": "orm", "pbrMetallicRoughness": {"metallicRoughnessTexture": {"index": 0}}, "occlusionTexture": {"index": 1}},
                {"name": "other_occlusion", "pbrMetallicRoughness": {"metallicRoughnessTexture": {"index": 0}}, "occlusionTexture": {"index": 2}},
            ],
            textures=textures,
            images=images,
            diagnostics=[_warning('.*Material "other_occlusion" has an occlusionTexture that is not shared with its metallicRoughnessTexture.*')],
        )

        for name, has_occlusion in (("mr_only", False), ("orm", True), ("other_occlusion", False)):
            preview = self._preview(self._material(stage, name))
            roughness_texture, roughness_output = self._connected(preview, "roughness")
            metallic_texture, metallic_output = self._connected(preview, "metallic")
            self.assertEqual(roughness_output, "g", msg=name)
            self.assertEqual(metallic_output, "b", msg=name)
            self.assertEqual(roughness_texture.GetPath(), metallic_texture.GetPath(), msg=name)
            self.assertEqual(self._texture_file(roughness_texture), pathlib.Path("./Textures/model_mr.png"), msg=name)
            occlusion = self._connected(preview, "occlusion")
            if has_occlusion:
                self.assertEqual(occlusion[1], "r", msg=name)
            else:
                self.assertIsNone(occlusion, msg=name)

            # OpenPBR has no occlusion. Roughness and metallic are separated from the same texture.
            mtlx = usdex.core.computeEffectiveMtlxSurfaceShader(self._material(stage, name))
            self.assertIsNotNone(self._connected(mtlx, "specular_roughness"), msg=name)
            self.assertIsNotNone(self._connected(mtlx, "base_metalness"), msg=name)

        self.assertTrue((output_dir / "Payload" / "Textures" / "model_mr.png").exists())

    def test_metallic_roughness_factors_are_not_applied_to_texture(self):
        png = _png()
        stage, _ = self._convert(
            [
                {"name": "factors", "pbrMetallicRoughness": {"metallicFactor": 0.5, "metallicRoughnessTexture": {"index": 0}}},
                {"name": "zero", "pbrMetallicRoughness": {"metallicFactor": 0, "roughnessFactor": 0, "metallicRoughnessTexture": {"index": 0}}},
            ],
            textures=[{"source": 0}],
            images=[{"mimeType": "image/png", "uri": _data_uri(png, "image/png")}],
            diagnostics=[
                _warning('.*Material "factors" has a metallicFactor or roughnessFactor that is not applied to its metallicRoughnessTexture.*')
            ],
        )

        self.assertIsNotNone(self._connected(self._preview(self._material(stage, "factors")), "roughness"))
        # Both factors are 0, so the texture would not have any effect.
        zero = self._preview(self._material(stage, "zero"))
        self.assertIsNone(self._connected(zero, "roughness"))
        self.assertIsNone(self._connected(zero, "metallic"))

    def test_normal_texture(self):
        png = _png()
        stage, _ = self._convert(
            [
                {"name": "normal", "normalTexture": {"index": 0}},
                {"name": "scaled", "normalTexture": {"index": 0, "scale": 2.0}},
            ],
            textures=[{"source": 0}],
            images=[{"name": "n", "mimeType": "image/png", "uri": _data_uri(png, "image/png")}],
            diagnostics=[_warning('.*Material "scaled" has a normalTexture scale that is not applied.*')],
        )

        material = self._material(stage, "normal")
        self.assertEqual(self.get_material_texture_path(material, "normal"), pathlib.Path("./Textures/model_n.png"))

    def test_emissive(self):
        # The default emissiveFactor is 0, which hides the texture.
        png = _png()
        stage, _ = self._convert(
            [
                {"name": "emissive", "emissiveFactor": [1, 1, 1], "emissiveTexture": {"index": 0}},
                {"name": "no_factor", "emissiveTexture": {"index": 0}},
                {"name": "dim", "emissiveFactor": [0.5, 0.5, 0.5], "emissiveTexture": {"index": 0}},
            ],
            textures=[{"source": 0}],
            images=[{"name": "e", "mimeType": "image/png", "uri": _data_uri(png, "image/png")}],
            diagnostics=[_warning('.*Material "dim" has an emissiveFactor that is not applied to its emissiveTexture.*')],
        )

        emissive = self._material(stage, "emissive")
        self.assertEqual(self.get_material_texture_path(emissive, "emissiveColor"), pathlib.Path("./Textures/model_e.png"))
        self.assertAlmostEqual(self.get_openpbr_input_value(emissive, "emission_luminance"), 1000.0)

        self.assertIsNone(self._connected(self._preview(self._material(stage, "no_factor")), "emissiveColor"))

    def test_unsupported_image_format(self):
        stage, _ = self._convert(
            [{"name": "webp", "pbrMetallicRoughness": {"baseColorTexture": {"index": 0}}}],
            textures=[{"source": 0}],
            images=[{"mimeType": "image/webp", "uri": _data_uri(b"RIFF", "image/webp")}],
            diagnostics=[_warning(r".*Material \"webp\" has an image of an unsupported format \(image/webp\)\. Only PNG and JPEG are supported.*")],
        )

        self.assertIsNone(self._connected(self._preview(self._material(stage, "webp")), "diffuseColor"))

    def test_sampler_wrap_mode(self):
        png = _png()
        stage, _ = self._convert(
            [{"name": "clamped", "pbrMetallicRoughness": {"baseColorTexture": {"index": 0}}}],
            textures=[{"source": 0, "sampler": 0}],
            images=[{"mimeType": "image/png", "uri": _data_uri(png, "image/png")}],
            samplers=[{"wrapS": 33071, "wrapT": 33071}],
            diagnostics=[_warning('.*Material "clamped" has a texture with a wrap mode other than REPEAT. REPEAT is used.*')],
        )

        self.assertEqual(self.get_material_wrap_mode(self._material(stage, "clamped")), "repeat")

    def test_material_names(self):
        # A name that is empty or shared gets its index, so the materials stay distinguishable. The display name is the original one.
        stage, _ = self._convert([{"name": "shared"}, {"name": "shared"}, {}, {"name": "unique"}])

        scope = stage.GetDefaultPrim().GetChild("Materials")
        self.assertEqual([child.GetName() for child in scope.GetChildren()], ["shared_0", "shared_1", "Material_2", "unique"])
        self.assertEqual(usdex.core.getDisplayName(scope.GetChild("shared_0")), "shared")
        self.assertEqual(usdex.core.getDisplayName(scope.GetChild("shared_1")), "shared")

    def test_single_material_binds_the_mesh(self):
        # With one material there is no subset, and the mesh is bound directly.
        # (The subsets of several materials are checked with boxes.gltf in testMeshGltf.)
        stage, _ = self._convert([{"name": "only"}])

        meshes = [prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Mesh)]
        self.assertEqual(len(meshes), 1)
        self.check_material_binding(meshes[0], self._material(stage, "only"))

    def test_texture_transform_must_be_shared(self):
        # Only one st primvar exists, so one transform is baked. Textures with different transforms warn.
        png = _png()
        transform = {"extensions": {"KHR_texture_transform": {"offset": [0.5, 0.0]}}}
        stage, _ = self._convert(
            [
                {
                    "name": "different",
                    "pbrMetallicRoughness": {"baseColorTexture": {"index": 0, **transform}, "metallicRoughnessTexture": {"index": 0}},
                },
            ],
            textures=[{"source": 0}],
            images=[{"mimeType": "image/png", "uri": _data_uri(png, "image/png")}],
            extensions=["KHR_texture_transform"],
            diagnostics=[_warning('.*Material "different" has textures with different KHR_texture_transform.*')],
        )

        # The transform of the base color texture is baked: u' = u + 0.5.
        mesh = next(prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Mesh))
        st = np.array(UsdGeom.PrimvarsAPI(mesh).GetPrimvar("st").ComputeFlattened())
        np.testing.assert_allclose(sorted(map(tuple, st)), sorted([(0.5, 1.0), (1.5, 1.0), (0.5, 0.0)]), atol=1e-6)

    def test_texcoord_other_than_zero(self):
        png = _png()
        stage, _ = self._convert(
            [{"name": "uv1", "pbrMetallicRoughness": {"baseColorTexture": {"index": 0, "texCoord": 1}}}],
            textures=[{"source": 0}],
            images=[{"mimeType": "image/png", "uri": _data_uri(png, "image/png")}],
            diagnostics=[_warning('.*Material "uv1" has a texture that uses TEXCOORD_1. Only TEXCOORD_0 is supported, so TEXCOORD_0 is used.*')],
        )

        self.assertIsNotNone(self._connected(self._preview(self._material(stage, "uv1")), "diffuseColor"))
