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

    def get_material_diffuse_color(self, material: UsdShade.Material) -> Gf.Vec3f | None:
        shader: UsdShade.Shader = usdex.core.computeEffectivePreviewSurfaceShader(material)
        return shader.GetInput("diffuseColor").Get()

    def get_material_opacity(self, material: UsdShade.Material) -> float:
        shader: UsdShade.Shader = usdex.core.computeEffectivePreviewSurfaceShader(material)
        return shader.GetInput("opacity").Get()

    def get_material_roughness(self, material: UsdShade.Material) -> float:
        shader: UsdShade.Shader = usdex.core.computeEffectivePreviewSurfaceShader(material)
        return shader.GetInput("roughness").Get()

    def get_material_metallic(self, material: UsdShade.Material) -> float:
        shader: UsdShade.Shader = usdex.core.computeEffectivePreviewSurfaceShader(material)
        return shader.GetInput("metallic").Get()

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
        texture_prim = connected_source[0].GetPrim()
        texture_file_attr = texture_prim.GetAttribute("inputs:file")
        return pathlib.Path(texture_file_attr.Get().path)

    def get_material_diffuse_color_texture_fallback(self, material: UsdShade.Material) -> Gf.Vec4f | None:
        shader: UsdShade.Shader = usdex.core.computeEffectivePreviewSurfaceShader(material)
        diffuse_color_input = shader.GetInput("diffuseColor")
        if diffuse_color_input.HasConnectedSource():
            source = diffuse_color_input.GetConnectedSource()
            if len(source) > 0 and isinstance(source[0], UsdShade.ConnectableAPI) and source[0].GetPrim().IsA(UsdShade.Shader):
                diffuse_texture_shader = UsdShade.Shader(source[0].GetPrim())
                return diffuse_texture_shader.GetInput("fallback").Get()
        return None
