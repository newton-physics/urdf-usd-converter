# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import pathlib

import omni.asset_validator
import usdex.test
from pxr import Gf, Usd, UsdGeom, UsdShade


class ConverterTestCase(usdex.test.TestCase):

    defaultUpAxis = UsdGeom.Tokens.z  # noqa: N815

    def setUp(self):
        super().setUp()
        # All conversion results should be valid atomic assets
        self.validationEngine.enable_rule(omni.asset_validator.AnchoredAssetPathsChecker)
        self.validationEngine.enable_rule(omni.asset_validator.SupportedFileTypesChecker)

    def check_material_binding(self, prim: Usd.Prim, material: UsdShade.Material):
        material_binding = UsdShade.MaterialBindingAPI(prim)
        self.assertTrue(material_binding)
        self.assertTrue(material_binding.GetDirectBindingRel())
        self.assertEqual(len(material_binding.GetDirectBindingRel().GetTargets()), 1)
        bound_material = material_binding.GetDirectBindingRel().GetTargets()[0]
        self.assertEqual(bound_material, material.GetPrim().GetPath())

    def _get_input_value(self, shader: UsdShade.Shader, input_name: str):
        value_attrs = UsdShade.Utils.GetValueProducingAttributes(shader.GetInput(input_name))

        # If no value is set, returns None.
        if not value_attrs or len(value_attrs) == 0:
            return None

        return value_attrs[0].Get()

    def _get_material_input_value(self, material: UsdShade.Material, input_name: str):
        shader: UsdShade.Shader = usdex.core.computeEffectivePreviewSurfaceShader(material)
        return self._get_input_value(shader, input_name)

    def get_material_diffuse_color(self, material: UsdShade.Material) -> Gf.Vec3f | None:
        return self._get_material_input_value(material, "diffuseColor")

    def get_material_emissive_color(self, material: UsdShade.Material) -> Gf.Vec3f | None:
        return self._get_material_input_value(material, "emissiveColor")

    def get_material_specular_color(self, material: UsdShade.Material) -> Gf.Vec3f | None:
        return self._get_material_input_value(material, "specularColor")

    def get_material_specular_workflow(self, material: UsdShade.Material) -> bool:
        return self._get_material_input_value(material, "useSpecularWorkflow") == 1

    def get_material_opacity(self, material: UsdShade.Material) -> float:
        return self._get_material_input_value(material, "opacity")

    def get_material_roughness(self, material: UsdShade.Material) -> float:
        return self._get_material_input_value(material, "roughness")

    def get_material_metallic(self, material: UsdShade.Material) -> float:
        return self._get_material_input_value(material, "metallic")

    def get_material_ior(self, material: UsdShade.Material) -> float:
        return self._get_material_input_value(material, "ior")

    def get_material_wrap_mode(self, material: UsdShade.Material) -> str:
        wrap_mode_input = material.GetInput("wrapMode")
        wrap_mode = wrap_mode_input.Get() if wrap_mode_input else None

        for child in material.GetPrim().GetAllChildren():
            shader = UsdShade.Shader(child)
            if shader.GetShaderId() == "UsdUVTexture":
                wrap_s = shader.GetInput("wrapS").Get()
                wrap_t = shader.GetInput("wrapT").Get()
                self.assertEqual(wrap_mode, wrap_s)
                self.assertEqual(wrap_mode, wrap_t)
        return wrap_mode

    def get_material_texture_path(self, material: UsdShade.Material, texture_type: str = "diffuseColor") -> pathlib.Path:
        """
        Get the texture path for the given texture type.

        Args:
            material: The material.
            texture_type: The texture type. Valid values are "diffuseColor", "normal", "roughness" and "metallic".

        Returns:
            The texture path.
        """
        shader: UsdShade.Shader = usdex.core.computeEffectivePreviewSurfaceShader(material)
        texture_input: UsdShade.Input = shader.GetInput(texture_type)
        self.assertTrue(texture_input.HasConnectedSource())

        connected_source = texture_input.GetConnectedSource()
        texture_shader = UsdShade.Shader(connected_source[0].GetPrim())
        texture_file_value = self._get_input_value(texture_shader, "file")
        return pathlib.Path(texture_file_value.path)

    def get_material_diffuse_color_texture_fallback(self, material: UsdShade.Material) -> Gf.Vec4f | None:
        shader: UsdShade.Shader = usdex.core.computeEffectivePreviewSurfaceShader(material)
        diffuse_color_input = shader.GetInput("diffuseColor")
        if diffuse_color_input.HasConnectedSource():
            source = diffuse_color_input.GetConnectedSource()
            if len(source) > 0 and isinstance(source[0], UsdShade.ConnectableAPI) and source[0].GetPrim().IsA(UsdShade.Shader):
                diffuse_texture_shader = UsdShade.Shader(source[0].GetPrim())
                return self._get_input_value(diffuse_texture_shader, "fallback")
        return None
