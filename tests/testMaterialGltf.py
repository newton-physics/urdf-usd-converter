# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import json
import pathlib

import numpy as np
import usdex.core
import usdex.test
from pxr import Gf, Tf, Usd, UsdGeom, UsdShade

import urdf_usd_converter
from tests.testMeshGltf import _POSITIONS, _gltf_document
from tests.util.ConverterTestCase import ConverterTestCase
from urdf_usd_converter._impl import gltf

_ASSETS = pathlib.Path("tests/data/assets")


def _warning(pattern: str) -> tuple:
    return (Tf.TF_DIAGNOSTIC_WARNING_TYPE, pattern)


# The warnings of materials.gltf in the order they are emitted: the UV transforms and sets (while the primitives are merged), then the materials
# (in the order of the materials). The file itself is a valid glTF, so the loader does not warn.
_WARNINGS = [
    _warning('.*Material "different_texture_transform" has textures with different KHR_texture_transform.*'),
    _warning(".*Only TEXCOORD_0 and COLOR_0 are supported. The other sets are ignored.*"),
    _warning('.*Material "uv1" has a texture that uses TEXCOORD_1. Only TEXCOORD_0 is supported, so TEXCOORD_0 is used.*'),
    _warning('.*Material "tinted" has a baseColorFactor color that is not applied to its baseColorTexture.*'),
    _warning('.*Material "other_occlusion" has an occlusionTexture that is not shared with its metallicRoughnessTexture.*'),
    _warning('.*Material "mr_factors" has a metallicFactor or roughnessFactor that is not applied to its metallicRoughnessTexture.*'),
    _warning('.*Material "normal_scaled" has a normalTexture scale that is not applied.*'),
    _warning('.*Material "emissive_dim" has an emissiveFactor that is not applied to its emissiveTexture.*'),
    _warning('.*Material "clamped" has a texture with a wrap mode other than REPEAT. REPEAT is used.*'),
]

# The warnings of the document of _convert_invalid in the order they are emitted: the loader (in the order of the images), then the materials.
_INVALID_WARNINGS = [
    _warning(".*External URLs are not supported: http://example.com/tex.png.*"),
    _warning(".*Absolute paths are not supported: /absolute/tex.png.*"),
    _warning('.*Material "unknown_alpha_mode" has an unknown alphaMode "TRANSLUCENT". OPAQUE is used.*'),
    _warning(r'.*Material "unsupported_embedded_format" has an image of an unsupported format \(image/webp\)\. Only PNG and JPEG are supported.*'),
    _warning(r'.*Material "unsupported_external_format" has an image of an unsupported format \(\.webp\)\. Only PNG and JPEG are supported.*'),
    _warning('.*Material "no_texture" references texture 99 that does not exist. The texture is ignored.*'),
    _warning('.*Material "no_source" references a texture without a valid image. The texture is ignored.*'),
    _warning('.*Material "no_image" references a texture without a valid image. The texture is ignored.*'),
]


class TestMaterialGltf(ConverterTestCase):
    """
    Materials of glTF meshes. tests/data/materials_gltf.urdf references tests/data/assets/materials.gltf.

    The glTF file has one triangle per material, and each test checks its own materials by name.
    The file is a valid glTF (it opens in glTF viewers). The textures are the images of tests/data/assets. Only the embedded images are
    in materials.bin. Some materials are made to be warned about, so every conversion has the same warnings.
    The tests check the UsdPreviewSurface network, and the OpenPBR network where it differs.

    A glTF that is not valid (an image that cannot be read, a reference that does not exist, ...) is written by the tests, see _convert_invalid.
    """

    def _convert(self, openpbr: bool = True) -> tuple[Usd.Stage, pathlib.Path]:
        """
        Convert the URDF of materials.gltf. Returns the stage and the directory of the converted asset.
        """
        output_dir = pathlib.Path(self.tmpDir())
        with usdex.test.ScopedDiagnosticChecker(self, _WARNINGS, level=usdex.core.DiagnosticsLevel.eWarning):
            asset_path = urdf_usd_converter.Converter(openpbr=openpbr).convert("tests/data/materials_gltf.urdf", str(output_dir))
        self.assertIsNotNone(asset_path)

        stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)
        return stage, output_dir

    def _convert_invalid(self) -> Usd.Stage:
        """
        Convert a glTF written here, with images and references that a valid glTF does not have. One triangle per material.
        """
        source_dir = pathlib.Path(self.tmpDir())
        # The texture that each material uses. Texture 99 does not exist, texture 4 has no image, and the image of texture 5 does not exist.
        textures = {
            "unsupported_embedded_format": 0,
            "unsupported_external_format": 1,
            "url_image": 2,
            "absolute_image": 3,
            "no_texture": 99,
            "no_source": 4,
            "no_image": 5,
        }
        materials = [{"name": "unknown_alpha_mode", "alphaMode": "TRANSLUCENT", "pbrMetallicRoughness": {"baseColorFactor": [1, 1, 1, 0.3]}}]
        materials += [{"name": name, "pbrMetallicRoughness": {"baseColorTexture": {"index": index}}} for name, index in textures.items()]
        document = _gltf_document(
            [{"POSITION": _POSITIONS, "material": index} for index in range(len(materials))],
            materials=materials,
            images=[
                {"uri": "unsupported.webp"},
                {"uri": "http://example.com/tex.png"},
                {"uri": "/absolute/tex.png"},
                {"uri": "data:image/webp;base64,UklGRg=="},
            ],
            textures=[{"source": 3}, {"source": 0}, {"source": 1}, {"source": 2}, {}, {"source": 99}],
        )
        (source_dir / "invalid.gltf").write_text(json.dumps(document), encoding="utf-8")
        (source_dir / "unsupported.webp").write_bytes(b"RIFF")
        (source_dir / "robot.urdf").write_text(
            """<?xml version="1.0"?>
<robot name="robot">
  <link name="base">
    <visual><geometry><mesh filename="invalid.gltf"/></geometry></visual>
    <collision><geometry><box size="1 1 1"/></geometry></collision>
  </link>
</robot>
""",
            encoding="utf-8",
        )

        with usdex.test.ScopedDiagnosticChecker(self, _INVALID_WARNINGS, level=usdex.core.DiagnosticsLevel.eWarning):
            asset_path = urdf_usd_converter.Converter().convert(str(source_dir / "robot.urdf"), str(self.tmpDir()))
        self.assertIsNotNone(asset_path)
        return Usd.Stage.Open(asset_path.path)

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

    def _embedded_images(self, name: str) -> list[tuple[int, bytes]]:
        """
        The index and the bytes of the embedded images of materials.gltf with the name.
        """
        model = gltf.load(_ASSETS / "materials.gltf")
        return [(index, image.data) for index, image in enumerate(model.images) if image.name == name]

    def test_embedded_image_is_written_to_textures(self):
        # An embedded image has no file, so the bytes are written to the Textures directory under the name of the glTF file and the image.
        stage, output_dir = self._convert()

        material = self._material(stage, "embedded_texture")
        self.assertEqual(self.get_material_texture_path(material, "diffuseColor"), pathlib.Path("./Textures/materials_embedded.png"))
        ((_, data),) = self._embedded_images("embedded")
        self.assertEqual((output_dir / "Payload" / "Textures" / "materials_embedded.png").read_bytes(), data)

    def test_external_image_is_copied(self):
        # An external image is a file relative to the glTF. A percent-encoded path (grid%2Dopacity.png) is decoded.
        stage, output_dir = self._convert()

        material = self._material(stage, "external_texture")
        self.assertEqual(self.get_material_texture_path(material, "diffuseColor"), pathlib.Path("./Textures/grid-opacity.png"))
        self.assertEqual((output_dir / "Payload" / "Textures" / "grid-opacity.png").read_bytes(), (_ASSETS / "grid-opacity.png").read_bytes())

    def test_colors_are_linear(self):
        # glTF colors are already linear, so they are authored as they are.
        stage, _ = self._convert()

        material = self._material(stage, "colored")
        self.assertTrue(Gf.IsClose(self.get_material_diffuse_color(material), Gf.Vec3f(0.5, 0.25, 0.1), 1e-6))
        self.assertAlmostEqual(self.get_material_metallic(material), 0.25, places=6)
        self.assertAlmostEqual(self.get_material_roughness(material), 0.75, places=6)
        self.assertTrue(Gf.IsClose(self.get_material_emissive_color(material), Gf.Vec3f(0.2, 0.1, 0.0), 1e-6))
        self.assertTrue(Gf.IsClose(self.get_openpbr_input_value(material, "base_color"), Gf.Vec3f(0.5, 0.25, 0.1), 1e-6))
        self.assertTrue(Gf.IsClose(self.get_openpbr_input_value(material, "emission_color"), Gf.Vec3f(0.2, 0.1, 0.0), 1e-6))

    def test_defaults(self):
        # The glTF defaults are white, metallic 1.0 and roughness 1.0. There is no emission.
        stage, _ = self._convert()

        material = self._material(stage, "default")
        self.assertTrue(Gf.IsClose(self.get_material_diffuse_color(material), Gf.Vec3f(1.0, 1.0, 1.0), 1e-6))
        self.assertAlmostEqual(self.get_material_metallic(material), 1.0, places=6)
        self.assertAlmostEqual(self.get_material_roughness(material), 1.0, places=6)
        self.assertAlmostEqual(self.get_material_opacity(material), 1.0, places=6)
        self.assertFalse(self._preview(material).GetInput("emissiveColor"))

    def test_alpha_modes(self):
        stage, _ = self._convert()

        # OPAQUE ignores the alpha.
        opaque = self._material(stage, "alpha_opaque")
        self.assertAlmostEqual(self.get_material_opacity(opaque), 1.0, places=6)
        self.assertFalse(self._preview(opaque).GetInput("opacityThreshold"))

        # BLEND uses the alpha and has no threshold.
        blend = self._material(stage, "alpha_blend")
        self.assertAlmostEqual(self.get_material_opacity(blend), 0.3, places=6)
        self.assertAlmostEqual(self.get_openpbr_input_value(blend, "geometry_opacity"), 0.3, places=6)
        self.assertFalse(self._preview(blend).GetInput("opacityThreshold"))

        # MASK uses the alpha and sets the threshold to alphaCutoff.
        mask = self._material(stage, "alpha_mask")
        self.assertAlmostEqual(self.get_material_opacity(mask), 0.8, places=6)
        self.assertAlmostEqual(self._get_input_value(self._preview(mask), "opacityThreshold"), 0.4, places=6)

    def test_unknown_alpha_mode(self):
        stage = self._convert_invalid()

        self.assertAlmostEqual(self.get_material_opacity(self._material(stage, "unknown_alpha_mode")), 1.0, places=6)

    def test_alpha_of_base_color_texture(self):
        # With BLEND or MASK the alpha channel of the base color texture is the opacity. The helpers only read a single channel
        # (the first one), so the alpha is connected here.
        stage, _ = self._convert()

        blend = self._material(stage, "texture_alpha_blend")
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
        opaque = self._material(stage, "texture_alpha_opaque")
        self.assertIsNone(self._connected(self._preview(opaque), "opacity"))

    def test_alpha_factor_with_base_color_texture(self):
        # An alpha of the baseColorFactor below 1 is the scalar opacity. The texture is used as it is, and there is no warning
        # because the color of the factor is white.
        stage, _ = self._convert()

        for name in ("texture_alpha_factor_blend", "texture_alpha_factor_mask"):
            material = self._material(stage, name)
            preview = self._preview(material)
            self.assertAlmostEqual(self.get_material_opacity(material), 0.5, places=6, msg=name)
            self.assertIsNone(self._connected(preview, "opacity"), msg=name)
            self.assertIsNotNone(self._connected(preview, "diffuseColor"), msg=name)
            self.assertAlmostEqual(self.get_openpbr_input_value(material, "geometry_opacity"), 0.5, places=6, msg=name)
        masked = self._preview(self._material(stage, "texture_alpha_factor_mask"))
        self.assertAlmostEqual(self._get_input_value(masked, "opacityThreshold"), 0.25, places=6)

    def test_alpha_of_base_color_texture_preview_material_only(self):
        stage, _ = self._convert(openpbr=False)

        material = self._material(stage, "texture_alpha_blend")
        self.assertFalse(material.GetPrim().HasAPI("MaterialXConfigAPI"))
        _, output = self._connected(self._preview(material), "opacity")
        self.assertEqual(output, "a")

    def test_base_color_factor_is_not_applied_to_texture(self):
        # The texture is used as it is, so the factor is not applied. Black would hide the texture, so the texture is not used then.
        stage, _ = self._convert()

        self.assertIsNotNone(self._connected(self._preview(self._material(stage, "tinted")), "diffuseColor"))
        self.assertIsNone(self._connected(self._preview(self._material(stage, "black")), "diffuseColor"))

    def test_metallic_roughness_texture(self):
        # G is the roughness and B is the metallic. R is not an occlusion unless the occlusionTexture shares the image.
        # With openpbr=False only UsdPreviewSurface is authored.
        for openpbr in (True, False):
            with self.subTest(openpbr=openpbr):
                stage, output_dir = self._convert(openpbr=openpbr)

                for name, has_occlusion in (("mr_only", False), ("orm", True), ("other_occlusion", False)):
                    preview = self._preview(self._material(stage, name))
                    roughness_texture, roughness_output = self._connected(preview, "roughness")
                    metallic_texture, metallic_output = self._connected(preview, "metallic")
                    self.assertEqual(roughness_output, "g", msg=name)
                    self.assertEqual(metallic_output, "b", msg=name)
                    self.assertEqual(roughness_texture.GetPath(), metallic_texture.GetPath(), msg=name)
                    self.assertEqual(self._texture_file(roughness_texture), pathlib.Path("./Textures/metallic_roughness.png"), msg=name)
                    occlusion = self._connected(preview, "occlusion")
                    if has_occlusion:
                        self.assertEqual(occlusion[1], "r", msg=name)
                        self.assertEqual(occlusion[0].GetPath(), roughness_texture.GetPath(), msg=name)
                    else:
                        # The R channel is not an occlusion, so the connection made by the ORM helper is removed.
                        self.assertIsNone(occlusion, msg=name)

                    if openpbr:
                        # OpenPBR has no occlusion. Roughness and metallic are separated from the same texture.
                        mtlx = usdex.core.computeEffectiveMtlxSurfaceShader(self._material(stage, name))
                        self.assertIsNotNone(self._connected(mtlx, "specular_roughness"), msg=name)
                        self.assertIsNotNone(self._connected(mtlx, "base_metalness"), msg=name)

                self.assertTrue((output_dir / "Payload" / "Textures" / "metallic_roughness.png").exists())

    def test_metallic_roughness_factors_are_not_applied_to_texture(self):
        stage, _ = self._convert()

        self.assertIsNotNone(self._connected(self._preview(self._material(stage, "mr_factors")), "roughness"))
        # Both factors are 0, so the texture would not have any effect.
        zero = self._preview(self._material(stage, "mr_zero"))
        self.assertIsNone(self._connected(zero, "roughness"))
        self.assertIsNone(self._connected(zero, "metallic"))

    def test_normal_texture(self):
        stage, _ = self._convert()

        for name in ("normal", "normal_scaled"):
            material = self._material(stage, name)
            self.assertEqual(self.get_material_texture_path(material, "normal"), pathlib.Path("./Textures/normal.png"), msg=name)

    def test_emissive(self):
        # The default emissiveFactor is 0, which hides the texture.
        stage, _ = self._convert()

        emissive = self._material(stage, "emissive")
        self.assertEqual(self.get_material_texture_path(emissive, "emissiveColor"), pathlib.Path("./Textures/emissive.png"))
        self.assertAlmostEqual(self.get_openpbr_input_value(emissive, "emission_luminance"), 1000.0)

        self.assertIsNone(self._connected(self._preview(self._material(stage, "emissive_no_factor")), "emissiveColor"))

    def test_unsupported_image_format(self):
        # The format of an embedded image is decided by its MIME type, and the format of an external image by its suffix.
        stage = self._convert_invalid()

        for name in ("unsupported_embedded_format", "unsupported_external_format"):
            self.assertIsNone(self._connected(self._preview(self._material(stage, name)), "diffuseColor"), msg=name)

    def test_image_outside_of_the_file_is_ignored(self):
        # An external URL and an absolute path cannot be resolved. The loader warns, and the texture is ignored.
        stage = self._convert_invalid()

        self.assertIsNone(self._connected(self._preview(self._material(stage, "url_image")), "diffuseColor"))
        self.assertIsNone(self._connected(self._preview(self._material(stage, "absolute_image")), "diffuseColor"))

    def test_texture_references_that_do_not_exist(self):
        # A texture that does not exist, a texture without an image, and a texture of an image that does not exist are ignored.
        stage = self._convert_invalid()

        for name in ("no_texture", "no_source", "no_image"):
            self.assertIsNone(self._connected(self._preview(self._material(stage, name)), "diffuseColor"), msg=name)

    def test_embedded_images_with_the_same_name(self):
        # The file name of an embedded image comes from its name, so images with the same name get their index to stay distinct.
        stage, output_dir = self._convert()

        (_, first), (second_index, second) = self._embedded_images("same")
        self.assertNotEqual(first, second)
        second_name = f"materials_same_{second_index}.png"
        self.assertEqual(
            self.get_material_texture_path(self._material(stage, "same_name_first"), "diffuseColor"), pathlib.Path("./Textures/materials_same.png")
        )
        self.assertEqual(
            self.get_material_texture_path(self._material(stage, "same_name_second"), "diffuseColor"), pathlib.Path(f"./Textures/{second_name}")
        )
        self.assertEqual((output_dir / "Payload" / "Textures" / "materials_same.png").read_bytes(), first)
        self.assertEqual((output_dir / "Payload" / "Textures" / second_name).read_bytes(), second)

    def test_sampler_wrap_mode(self):
        stage, _ = self._convert()

        self.assertEqual(self.get_material_wrap_mode(self._material(stage, "clamped")), "repeat")

    def test_material_names(self):
        # A name that is empty or shared gets its index, so the materials stay distinguishable. The display name is the original one.
        # The first materials of materials.gltf are for the names, so their indices are 0 to 6.
        stage, _ = self._convert()

        scope = stage.GetDefaultPrim().GetChild("Materials")
        self.assertEqual([child.GetName() for child in scope.GetChildren()][:4], ["shared_0", "shared_1", "Material_2", "unique"])
        self.assertEqual(usdex.core.getDisplayName(scope.GetChild("shared_0")), "shared")
        self.assertEqual(usdex.core.getDisplayName(scope.GetChild("shared_1")), "shared")

    def test_material_names_do_not_collide(self):
        # An index added to a shared name must not equal the name of another material.
        # The materials 4 and 5 are named "a", and the material 6 is named "a_4", which is what the material 4 would get.
        stage, _ = self._convert()

        scope = stage.GetDefaultPrim().GetChild("Materials")
        self.assertEqual([child.GetName() for child in scope.GetChildren()][4:7], ["a_4_", "a_5", "a_4"])

    def test_texture_transform_must_be_shared(self):
        # Only one st primvar exists, so one transform is baked. Textures with different transforms warn.
        stage, _ = self._convert()

        # The transform of the base color texture is baked: u' = u + 0.5. Only the triangle of this material has it.
        mesh = UsdGeom.Mesh(next(prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Mesh)))
        names = [child.GetName() for child in stage.GetDefaultPrim().GetChild("Materials").GetChildren()]
        subset = UsdGeom.Subset.GetAllGeomSubsets(mesh)[names.index("different_texture_transform")]
        face = subset.GetIndicesAttr().Get()[0]
        vertices = list(mesh.GetFaceVertexIndicesAttr().Get())[3 * face : 3 * face + 3]
        st = np.array(UsdGeom.PrimvarsAPI(mesh).GetPrimvar("st").ComputeFlattened())
        np.testing.assert_allclose(sorted(map(tuple, st[vertices])), sorted([(0.5, 1.0), (1.5, 1.0), (0.5, 0.0)]), atol=1e-6)

    def test_texcoord_other_than_zero(self):
        stage, _ = self._convert()

        self.assertIsNotNone(self._connected(self._preview(self._material(stage, "uv1")), "diffuseColor"))
