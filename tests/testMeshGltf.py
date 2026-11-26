# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import base64
import json
import pathlib

import numpy as np
import usdex.core
import usdex.test
from pxr import Gf, Tf, Usd, UsdGeom, UsdShade

import urdf_usd_converter
from tests.util.ConverterTestCase import ConverterTestCase
from urdf_usd_converter._impl import gltf
from urdf_usd_converter._impl.conversion_gltf import _define_mesh, _merge_geometry, _MergedGeometry, _triangle_list, _Warner

_POSITIONS = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=np.float32)


def _warning(pattern: str) -> tuple:
    return (Tf.TF_DIAGNOSTIC_WARNING_TYPE, pattern)


def _node(mesh: int | None = 0, children: tuple = (), scale: tuple = (1.0, 1.0, 1.0)) -> gltf.Node:
    return gltf.Node(
        name=None,
        children=list(children),
        mesh=mesh,
        matrix=None,
        translation=np.zeros(3),
        rotation=np.array([0.0, 0.0, 0.0, 1.0]),
        scale=np.array(scale, dtype=np.float64),
    )


def _primitive(*, mode: int = 4, indices: tuple | None = (0, 1, 2), material: int | None = None, **attributes: np.ndarray) -> gltf.Primitive:
    """A triangle by default. The attributes are added to POSITION."""
    attributes.setdefault("POSITION", _POSITIONS)
    return gltf.Primitive(
        mode=mode, attributes=attributes, indices=None if indices is None else np.array(indices, dtype=np.uint16), material=material
    )


def _model(nodes: list[gltf.Node], primitives: list[gltf.Primitive], scene: int | None = 0) -> gltf.Gltf:
    """A model with one mesh of the primitives. The root of the scene is the first node."""
    return gltf.Gltf(
        asset=gltf.AssetInfo(version="2.0"),
        scenes=[gltf.Scene(name=None, nodes=[0])],
        scene=scene,
        nodes=nodes,
        meshes=[gltf.Mesh(name=None, primitives=primitives)],
        materials=[],
        textures=[],
        samplers=[],
        images=[],
    )


def _gltf_document(*, plain: bool = True, colored: bool = False, **sections) -> dict:
    """A glTF with a triangle (plain) and a triangle with COLOR_0 (colored) in one mesh. The sections replace the default ones."""
    colors = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float32)
    buffer = _POSITIONS.tobytes() + colors.tobytes()
    primitives = []
    if plain:
        primitives.append({"attributes": {"POSITION": 0}})
    if colored:
        primitives.append({"attributes": {"POSITION": 0, "COLOR_0": 1}})
    document = {
        "asset": {"version": "2.0"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"mesh": 0}],
        "meshes": [{"primitives": primitives}],
        "accessors": [
            {"bufferView": 0, "componentType": 5126, "count": 3, "type": "VEC3", "min": [0, 0, 0], "max": [1, 1, 0]},
            {"bufferView": 1, "componentType": 5126, "count": 3, "type": "VEC3"},
        ],
        "bufferViews": [{"buffer": 0, "byteOffset": 0, "byteLength": 36}, {"buffer": 0, "byteOffset": 36, "byteLength": 36}],
        "buffers": [{"uri": "data:application/octet-stream;base64," + base64.b64encode(buffer).decode(), "byteLength": len(buffer)}],
    }
    document.update(sections)
    return document


class TestMeshGltf(ConverterTestCase):
    """
    tests/data/mesh_gltf.urdf references tests/data/assets/boxes.gltf (authored in Blender).

    The glTF file has 6 cubes (one material each) under a node rotated about Y, and a scaled plane.
    The 6th cube has vertex colors (COLOR_0). Every material is double sided.

    The primitives with COLOR_0 do not have a material, and are merged into a separate mesh ("boxes_vertexColor").
    The other 5 cubes and the plane are merged into one mesh ("boxes"). Both are children of the "boxes" Xform.
    """

    def _convert(self) -> Usd.Stage:
        input_path = "tests/data/mesh_gltf.urdf"
        output_dir = self.tmpDir()

        converter = urdf_usd_converter.Converter()
        # vertexColor_mat is for the cube with COLOR_0. The vertex colors are used as the display color instead of the material.
        # texture_mat has a roughnessFactor (0.5) together with a metallicRoughnessTexture. The factor is not applied.
        # (base_color_texture_opacity_mat has a baseColorFactor of (1, 1, 1, 0.5) with a baseColorTexture,
        # but the alpha is the scalar opacity, so there is no warning for it.)
        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (
                    Tf.TF_DIAGNOSTIC_WARNING_TYPE,
                    '.*Material "vertexColor_mat" is ignored for a primitive with COLOR_0. The vertex colors are used as the display color instead.*',
                ),
                (
                    Tf.TF_DIAGNOSTIC_WARNING_TYPE,
                    '.*Material "texture_mat" has a metallicFactor or roughnessFactor that is not applied to its metallicRoughnessTexture.*',
                ),
            ],
            level=usdex.core.DiagnosticsLevel.eWarning,
        ):
            asset_path = converter.convert(input_path, output_dir)
        self.assertIsNotNone(asset_path)
        self.assertTrue(pathlib.Path(asset_path.path).exists())

        return Usd.Stage.Open(asset_path.path)

    def _get_mesh(self, stage: Usd.Stage, name: str = "boxes") -> UsdGeom.Mesh:
        geometry_scope_prim = stage.GetPrimAtPath(stage.GetDefaultPrim().GetPath().AppendChild("Geometry"))
        self.assertTrue(geometry_scope_prim.IsValid())

        # link_mesh_gltf has only a visual, so it is nested under its parent link.
        xform_path = geometry_scope_prim.GetPath().AppendChild("BaseLink").AppendChild("link_mesh_gltf").AppendChild("boxes")
        xform_prim = stage.GetPrimAtPath(xform_path)
        self.assertTrue(xform_prim.IsValid())
        self.assertTrue(xform_prim.IsA(UsdGeom.Xform))

        # The file has primitives with and without COLOR_0, so there are two meshes below the Xform.
        mesh_prim = stage.GetPrimAtPath(xform_path.AppendChild(name))
        self.assertTrue(mesh_prim.IsValid())
        self.assertTrue(mesh_prim.IsA(UsdGeom.Mesh))
        return UsdGeom.Mesh(mesh_prim)

    def _get_vertex_color_mesh(self, stage: Usd.Stage) -> UsdGeom.Mesh:
        return self._get_mesh(stage, "boxes_vertexColor")

    def test_boxes_gltf(self):
        stage = self._convert()
        self.assertIsValidUsd(stage)

        # The Xform has the two meshes only.
        xform_prim = self._get_mesh(stage).GetPrim().GetParent()
        self.assertEqual([child.GetName() for child in xform_prim.GetChildren()], ["boxes", "boxes_vertexColor"])

        mesh = self._get_mesh(stage)

        # 5 cubes (24 vertices, 12 triangles each) and a plane (4 vertices, 2 triangles) are merged into one mesh.
        # The cube with COLOR_0 is in the other mesh.
        self.assertEqual(len(mesh.GetPointsAttr().Get()), 5 * 24 + 4)
        face_vertex_counts = list(mesh.GetFaceVertexCountsAttr().Get())
        self.assertEqual(face_vertex_counts, [3] * (5 * 12 + 2))
        self.assertEqual(mesh.GetOrientationAttr().Get(), UsdGeom.Tokens.rightHanded)

        # The node hierarchy is baked into the points. No Xform or other child prim is authored,
        # except for the subsets.
        self.assertTrue(all(child.IsA(UsdGeom.Subset) for child in mesh.GetPrim().GetChildren()))

        # Every material is double sided.
        self.assertTrue(mesh.GetDoubleSidedAttr().Get())

        primvars_api = UsdGeom.PrimvarsAPI(mesh.GetPrim())
        self.assertTrue(primvars_api.GetPrimvar("normals").IsDefined())
        self.assertTrue(primvars_api.GetPrimvar("st").IsDefined())

    def test_boxes_gltf_subsets(self):
        stage = self._convert()
        mesh = self._get_mesh(stage)

        # One subset per material, in the order the nodes are visited: the 5 cubes (without COLOR_0), then the plane.
        subsets = UsdGeom.Subset.GetAllGeomSubsets(mesh)
        expected = [list(range(12 * i, 12 * (i + 1))) for i in range(5)] + [[60, 61]]
        self.assertEqual([list(subset.GetIndicesAttr().Get()) for subset in subsets], expected)

    def test_boxes_gltf_axes_are_kept(self):
        # The glTF coordinates are used as they are (glTF +Y up is not rotated to +Z up).
        # The plane (glTF y = 0, +-2 scaled by 1.5) lies on y = 0 and the cubes sit above it in +Y.
        # The points are in the mesh frame, so the URDF visual origin does not affect them.
        stage = self._convert()
        mesh = self._get_mesh(stage)

        points = np.array(mesh.GetPointsAttr().Get())
        np.testing.assert_allclose(points.min(axis=0), [-3.0, 0.0, -3.0], atol=1e-4)
        np.testing.assert_allclose(points.max(axis=0), [3.0, 1.5, 3.0], atol=1e-4)

        # The plane is the last 4 vertices.
        np.testing.assert_allclose(points[-4:, 1], 0.0, atol=1e-6)

        # mesh_gltf.urdf rotates the Y-up glTF to Z-up with the visual origin (rpy="1.5707963 0 0").
        xformable = UsdGeom.Xformable(mesh.GetPrim())
        world = np.array(xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
        up = np.array([0.0, 1.0, 0.0, 0.0]) @ world  # USD matrices are row-vector
        np.testing.assert_allclose(up[:3], [0.0, 0.0, 1.0], atol=1e-4)

    def test_boxes_gltf_uv_is_flipped(self):
        # v_usd = 1 - v_gltf. The 5 cubes without COLOR_0 (the first 120 vertices) have no texture transform.
        # The vertex order is the order the nodes are visited, so compare as sets of (u, v) values.
        model = gltf.load("tests/data/assets/boxes.gltf")
        cube_uvs = np.concatenate([model.meshes[i].primitives[0].attributes["TEXCOORD_0"] for i in range(5)])
        expected = np.stack([cube_uvs[:, 0], 1.0 - cube_uvs[:, 1]], axis=1)

        stage = self._convert()
        mesh = self._get_mesh(stage)
        st = np.array(UsdGeom.PrimvarsAPI(mesh.GetPrim()).GetPrimvar("st").ComputeFlattened())

        self.assertEqual(len(st), 5 * 24 + 4)
        self.assertEqual(sorted(map(tuple, np.round(st[:-4], 5))), sorted(map(tuple, np.round(expected, 5))))

    def test_boxes_gltf_texture_transform(self):
        # The plane (the last 4 vertices) has KHR_texture_transform (offset, rotation -45 degrees, scale 0.5).
        # It was authored with a Blender Mapping node (Point, location 0, rotation -45 degrees, scale 0.5).
        # Blender's UV has the origin at the bottom left, the same as USD's st, so the result is the Blender mapping of the flipped UV:
        # st' = scale * R(rotation) * st.
        model = gltf.load("tests/data/assets/boxes.gltf")
        plane_uvs = model.meshes[6].primitives[0].attributes["TEXCOORD_0"]
        st_before = np.stack([plane_uvs[:, 0], 1.0 - plane_uvs[:, 1]], axis=1)
        cos, sin = np.cos(-np.pi / 4), np.sin(-np.pi / 4)
        expected = 0.5 * st_before @ np.array([[cos, -sin], [sin, cos]]).T

        stage = self._convert()
        mesh = self._get_mesh(stage)
        st = np.array(UsdGeom.PrimvarsAPI(mesh.GetPrim()).GetPrimvar("st").ComputeFlattened())
        np.testing.assert_allclose(st[-4:], expected, atol=1e-6)

    def test_boxes_gltf_materials(self):
        stage = self._convert()
        materials_scope = stage.GetDefaultPrim().GetChild("Materials")
        # vertexColor_mat is not authored, because the only primitive that uses it has COLOR_0.
        names = ["base_color_mat", "base_color_texture_opacity_mat", "metallic_mat", "opacity_texture_mat", "texture_mat", "texture_transform_mat"]
        self.assertEqual([child.GetName() for child in materials_scope.GetChildren()], names)
        materials = {name: UsdShade.Material(materials_scope.GetChild(name)) for name in names}

        # Each subset is bound to the material of its primitive. The order is the order the nodes are visited.
        subsets = UsdGeom.Subset.GetAllGeomSubsets(self._get_mesh(stage))
        self.assertEqual(len(subsets), len(names))
        for subset, name in zip(subsets, names):
            self.check_material_binding(subset.GetPrim(), materials[name])

        # The glTF color is linear, so it is authored as it is.
        self.assertTrue(Gf.IsClose(self.get_material_diffuse_color(materials["base_color_mat"]), Gf.Vec3f(0.80002296, 0.11174544, 0.06623038), 1e-6))
        self.assertAlmostEqual(self.get_material_roughness(materials["base_color_mat"]), 0.5, places=6)
        self.assertAlmostEqual(self.get_material_metallic(materials["base_color_mat"]), 0.0, places=6)

        # metallic_mat has no metallicFactor, so the glTF default (1.0) is used. Its roughnessFactor is 0.
        self.assertAlmostEqual(self.get_material_metallic(materials["metallic_mat"]), 1.0, places=6)
        self.assertAlmostEqual(self.get_material_roughness(materials["metallic_mat"]), 0.0, places=6)

        # Textures are copied to the Textures directory.
        self.assertEqual(self.get_material_texture_path(materials["texture_mat"], "diffuseColor"), pathlib.Path("./Textures/grid.png"))
        self.assertEqual(self.get_material_texture_path(materials["texture_transform_mat"], "diffuseColor"), pathlib.Path("./Textures/grid.png"))
        self.assertEqual(
            self.get_material_texture_path(materials["opacity_texture_mat"], "diffuseColor"), pathlib.Path("./Textures/grid-opacity.png")
        )

        # BLEND uses the alpha of the base color texture as the opacity. OPAQUE (texture_mat) ignores it.
        texture_input = usdex.core.computeEffectivePreviewSurfaceShader(materials["opacity_texture_mat"]).GetInput("opacity")
        self.assertEqual(texture_input.GetConnectedSource()[1], "a")
        # An alpha of the baseColorFactor is the scalar opacity, and the texture alpha is not connected.
        self.assertAlmostEqual(self.get_material_opacity(materials["base_color_texture_opacity_mat"]), 0.5, places=6)
        scalar_input = usdex.core.computeEffectivePreviewSurfaceShader(materials["base_color_texture_opacity_mat"]).GetInput("opacity")
        self.assertNotEqual(scalar_input.GetConnectedSource()[1], "a")
        self.assertAlmostEqual(self.get_openpbr_input_value(materials["base_color_texture_opacity_mat"], "geometry_opacity"), 0.5, places=6)
        opaque_input = usdex.core.computeEffectivePreviewSurfaceShader(materials["texture_mat"]).GetInput("opacity")
        self.assertNotEqual(opaque_input.GetConnectedSource()[1], "a")

        # texture_mat has a normal texture and a metallic roughness texture (the roughnessFactor is not applied, with a warning).
        self.assertEqual(self.get_material_texture_path(materials["texture_mat"], "normal"), pathlib.Path("./Textures/normal.png"))
        self.assertEqual(self.get_material_texture_path(materials["texture_mat"], "roughness"), pathlib.Path("./Textures/metallic_roughness.png"))
        self.assertEqual(self.get_material_texture_path(materials["texture_mat"], "metallic"), pathlib.Path("./Textures/metallic_roughness.png"))

    def test_boxes_gltf_vertex_colors(self):
        # COLOR_0 (RGB) of the 6th cube is the display color of the separate mesh "boxes_vertexColor".
        # COLOR_0 is a linear color, the same as the display color, so the values are used as they are.
        model = gltf.load("tests/data/assets/boxes.gltf")
        cube_colors = model.meshes[5].primitives[0].attributes["COLOR_0"]
        self.assertEqual(cube_colors.shape, (24, 3))

        stage = self._convert()
        mesh = self._get_vertex_color_mesh(stage)
        self.assertEqual(len(mesh.GetPointsAttr().Get()), 24)
        self.assertEqual(len(mesh.GetFaceVertexCountsAttr().Get()), 12)
        primvars_api = UsdGeom.PrimvarsAPI(mesh.GetPrim())

        display_color = primvars_api.GetPrimvar("displayColor")
        self.assertEqual(display_color.GetInterpolation(), UsdGeom.Tokens.vertex)
        np.testing.assert_allclose(np.array(display_color.ComputeFlattened()), cube_colors, atol=1e-6)

        # An RGB COLOR_0 has no alpha, so no display opacity value is authored.
        self.assertIsNone(primvars_api.GetPrimvar("displayOpacity").Get())

        # The mesh has no material, so the display color is not hidden. It has no subsets either.
        # It is still double sided, because the (ignored) material vertexColor_mat is.
        self.assertEqual(len(mesh.GetPrim().GetChildren()), 0)
        self.assertFalse(UsdShade.MaterialBindingAPI(mesh.GetPrim()).ComputeBoundMaterial()[0])
        self.assertTrue(mesh.GetDoubleSidedAttr().Get())

        # The mesh with the materials has no vertex colors.
        other_primvars_api = UsdGeom.PrimvarsAPI(self._get_mesh(stage).GetPrim())
        self.assertIsNone(other_primvars_api.GetPrimvar("displayColor").Get())
        self.assertIsNone(other_primvars_api.GetPrimvar("displayOpacity").Get())

    def test_vertex_colors_rgba(self):
        # An RGBA COLOR_0 gives the display opacity. A normalized integer COLOR_0 is decoded to [0, 1].
        # This is not covered by boxes.gltf, so a glTF file is written.
        positions = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=np.float32)
        normals = np.array([[0, 0, 1]] * 3, dtype=np.float32)
        colors = np.array([[255, 0, 0, 255], [0, 255, 0, 51], [0, 0, 255, 0]], dtype=np.uint8)
        buffer = positions.tobytes() + normals.tobytes() + colors.tobytes()
        document = {
            "asset": {"version": "2.0"},
            "scene": 0,
            "scenes": [{"nodes": [0]}],
            "nodes": [{"mesh": 0}],
            "meshes": [{"primitives": [{"attributes": {"POSITION": 0, "NORMAL": 1, "COLOR_0": 2}}]}],
            "accessors": [
                {"bufferView": 0, "componentType": 5126, "count": 3, "type": "VEC3", "min": [0, 0, 0], "max": [1, 1, 0]},
                {"bufferView": 1, "componentType": 5126, "count": 3, "type": "VEC3"},
                {"bufferView": 2, "componentType": 5121, "normalized": True, "count": 3, "type": "VEC4"},
            ],
            "bufferViews": [
                {"buffer": 0, "byteOffset": 0, "byteLength": positions.nbytes},
                {"buffer": 0, "byteOffset": positions.nbytes, "byteLength": normals.nbytes},
                {"buffer": 0, "byteOffset": positions.nbytes + normals.nbytes, "byteLength": colors.nbytes},
            ],
            "buffers": [{"uri": "data:application/octet-stream;base64," + base64.b64encode(buffer).decode(), "byteLength": len(buffer)}],
        }
        source_dir = pathlib.Path(self.tmpDir())
        (source_dir / "colors.gltf").write_text(json.dumps(document), encoding="utf-8")
        (source_dir / "robot.urdf").write_text(
            '<?xml version="1.0"?>\n'
            '<robot name="robot"><link name="base">'
            '<visual><geometry><mesh filename="colors.gltf"/></geometry></visual>'
            '<collision><geometry><box size="1 1 1"/></geometry></collision>'
            "</link></robot>\n",
            encoding="utf-8",
        )

        asset_path = urdf_usd_converter.Converter().convert(str(source_dir / "robot.urdf"), str(self.tmpDir()))
        self.assertIsNotNone(asset_path)
        stage = Usd.Stage.Open(asset_path.path)
        self.assertIsValidUsd(stage)

        # Every primitive has COLOR_0, so there is only one mesh. It is the mesh itself, not a child of an Xform.
        mesh_prims = [prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Mesh)]
        self.assertEqual(len(mesh_prims), 1)
        mesh_prim = mesh_prims[0]
        self.assertEqual(len(mesh_prim.GetChildren()), 0)
        self.assertFalse(UsdShade.MaterialBindingAPI(mesh_prim).ComputeBoundMaterial()[0])
        primvars_api = UsdGeom.PrimvarsAPI(mesh_prim)
        np.testing.assert_allclose(np.array(primvars_api.GetPrimvar("displayColor").ComputeFlattened()), [[1, 0, 0], [0, 1, 0], [0, 0, 1]], atol=1e-6)
        display_opacity = primvars_api.GetPrimvar("displayOpacity")
        self.assertEqual(display_opacity.GetInterpolation(), UsdGeom.Tokens.vertex)
        np.testing.assert_allclose(np.array(display_opacity.ComputeFlattened()), [1.0, 0.2, 0.0], atol=1e-6)

    def test_vertex_colors_merge(self):
        # The primitives are split by COLOR_0. With vertex_colors=True only the primitives with COLOR_0 are merged.
        # The display opacity exists only when a primitive has an RGBA COLOR_0, and the opacity of an RGB COLOR_0 is 1.
        positions = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=np.float32)

        def make_model(*attribute_sets: dict) -> gltf.Gltf:
            return gltf.Gltf(
                asset=gltf.AssetInfo(version="2.0"),
                scenes=[gltf.Scene(name=None, nodes=[0])],
                scene=0,
                nodes=[
                    gltf.Node(
                        name=None,
                        children=[],
                        mesh=0,
                        matrix=None,
                        translation=np.zeros(3),
                        rotation=np.array([0.0, 0.0, 0.0, 1.0]),
                        scale=np.ones(3),
                    )
                ],
                meshes=[
                    gltf.Mesh(
                        name=None,
                        primitives=[
                            gltf.Primitive(mode=4, attributes={"POSITION": positions, **attributes}, indices=np.array([0, 1, 2]), material=None)
                            for attributes in attribute_sets
                        ],
                    )
                ],
                materials=[],
                textures=[],
                samplers=[],
                images=[],
            )

        path = pathlib.Path("memory.gltf")
        rgb = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float32)
        rgba = np.array([[1, 0, 0, 0.5], [0, 1, 0, 0.5], [0, 0, 1, 0.5]], dtype=np.float32)

        # Without vertex_colors, the primitive with COLOR_0 is left out. The merged vertices have no color.
        model = make_model({}, {"COLOR_0": rgb})
        geometry = _merge_geometry(model, path)
        self.assertEqual(len(geometry.points), 3)
        self.assertIsNone(geometry.display_colors)
        self.assertIsNone(geometry.display_opacities)

        # With vertex_colors, only the primitive with COLOR_0 is merged. Nothing is filled with white.
        geometry = _merge_geometry(model, path, vertex_colors=True)
        self.assertEqual(len(geometry.points), 3)
        np.testing.assert_allclose(geometry.display_colors, rgb)
        self.assertIsNone(geometry.display_opacities)

        # RGB and RGBA are merged together. The opacity of the RGB is 1.
        geometry = _merge_geometry(make_model({"COLOR_0": rgb}, {"COLOR_0": rgba}), path, vertex_colors=True)
        np.testing.assert_allclose(geometry.display_colors, np.concatenate([rgb, rgb]))
        np.testing.assert_allclose(geometry.display_opacities, [1, 1, 1, 0.5, 0.5, 0.5])

        # There is nothing to merge for the other kind.
        self.assertIsNone(_merge_geometry(make_model({}), path, vertex_colors=True))
        self.assertIsNone(_merge_geometry(make_model({"COLOR_0": rgb}), path))

        # A COLOR_0 with the wrong shape is ignored, so the primitive has no vertex colors.
        model = make_model({"COLOR_0": np.zeros((2, 3), dtype=np.float32)})
        self.assertIsNone(_merge_geometry(model, path, vertex_colors=True))
        self.assertIsNone(_merge_geometry(model, path).display_colors)

    def test_triangle_list(self):
        indices = np.array([0, 1, 2, 3], dtype=np.uint16)

        # A trailing partial triangle is ignored.
        triangles = _triangle_list(4, np.array([0, 1, 2, 3, 2], dtype=np.uint16), 4)
        self.assertEqual(triangles.tolist(), [[0, 1, 2]])

        # Odd triangles of a strip swap their last two vertices, so the winding stays consistent.
        self.assertEqual(_triangle_list(5, indices, 4).tolist(), [[0, 1, 2], [1, 3, 2]])

        # A fan shares the first vertex.
        self.assertEqual(_triangle_list(6, indices, 4).tolist(), [[1, 2, 0], [2, 3, 0]])

        # Without indices, the vertex order is the topology.
        self.assertEqual(_triangle_list(4, None, 3).tolist(), [[0, 1, 2]])

        # Too few vertices for a strip or a fan.
        self.assertEqual(len(_triangle_list(5, np.array([0, 1], dtype=np.uint16), 2)), 0)

    def test_negative_scale_reverses_winding(self):
        # A mirrored node flips the winding order, so it is flipped back to keep the surface facing the same way.
        model = gltf.Gltf(
            asset=gltf.AssetInfo(version="2.0"),
            scenes=[gltf.Scene(name=None, nodes=[0])],
            scene=0,
            nodes=[
                gltf.Node(
                    name=None,
                    children=[],
                    mesh=0,
                    matrix=None,
                    translation=np.zeros(3),
                    rotation=np.array([0.0, 0.0, 0.0, 1.0]),
                    scale=np.array([-1.0, 1.0, 1.0]),
                )
            ],
            meshes=[
                gltf.Mesh(
                    name=None,
                    primitives=[
                        gltf.Primitive(
                            mode=4,
                            attributes={"POSITION": np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=np.float32)},
                            indices=np.array([0, 1, 2], dtype=np.uint16),
                            material=None,
                        )
                    ],
                )
            ],
            materials=[],
            textures=[],
            samplers=[],
            images=[],
        )

        geometry = _merge_geometry(model, pathlib.Path("memory.gltf"))

        # x is mirrored.
        np.testing.assert_allclose(geometry.points, [[0, 0, 0], [-1, 0, 0], [0, 1, 0]], atol=1e-6)
        self.assertEqual(geometry.face_vertex_indices.tolist(), [0, 2, 1])

        # The face normal still points the same way as the unmirrored triangle (+Z).
        a, b, c = geometry.points[geometry.face_vertex_indices]
        normal = np.cross(b - a, c - a)
        self.assertGreater(normal[2], 0.0)

    # Geometry that cannot be merged is skipped with a warning (the file still converts).

    def _merge(self, model: gltf.Gltf, warnings: list[str] | None = None) -> _MergedGeometry | None:
        with usdex.test.ScopedDiagnosticChecker(
            self, [_warning(f".*{message}.*") for message in warnings or []], level=usdex.core.DiagnosticsLevel.eWarning
        ):
            return _merge_geometry(model, pathlib.Path("memory.gltf"))

    def test_scene_that_does_not_exist(self):
        for scene in (None, 5):
            with self.subTest(scene=scene):
                self.assertIsNone(self._merge(_model([_node()], [_primitive()], scene=scene), ["The file has no scene"]))

    def test_node_that_is_referenced_twice_or_does_not_exist(self):
        # The first node is its own child, and the child 7 does not exist. Each node is visited once, so there is no endless loop.
        model = _model([_node(children=(0, 7))], [_primitive()])

        geometry = self._merge(
            model,
            [
                "Node 0 is referenced more than once or does not exist. It is skipped",
                "Node 7 is referenced more than once or does not exist. It is skipped",
            ],
        )

        self.assertEqual(len(geometry.points), 3)

    def test_mesh_that_does_not_exist(self):
        self.assertIsNone(self._merge(_model([_node(mesh=3)], [_primitive()]), ["Node 0 references a mesh that does not exist. It is skipped"]))

    def test_singular_transform(self):
        # A node scaled to zero in one axis cannot be inverted for the normals, so its meshes are skipped.
        self.assertIsNone(
            self._merge(_model([_node(scale=(0.0, 1.0, 1.0))], [_primitive()]), ["A node has a singular transform. Its meshes are skipped"])
        )

    def test_primitives_that_are_skipped(self):
        primitives = [
            _primitive(mode=0),  # POINTS is not geometry. The loader warns about it, so there is no warning here.
            gltf.Primitive(mode=4, attributes={}, indices=np.array([0, 1, 2], dtype=np.uint16), material=None),  # No POSITION.
            _primitive(indices=(0, 1)),  # Not even one triangle. Nothing to skip silently.
            _primitive(indices=(0, 1, 9)),  # The vertex 9 does not exist.
            _primitive(),
        ]

        geometry = self._merge(
            _model([_node()], primitives),
            ["A primitive has no POSITION. It is skipped", "A primitive has a vertex index out of range. It is skipped"],
        )

        # Only the valid triangle is merged.
        self.assertEqual(len(geometry.points), 3)
        self.assertEqual(geometry.face_vertex_indices.tolist(), [0, 1, 2])

    def test_attribute_sets_other_than_zero(self):
        primitive = _primitive(TEXCOORD_1=np.zeros((3, 2), dtype=np.float32), COLOR_1=np.zeros((3, 3), dtype=np.float32))

        geometry = self._merge(_model([_node()], [primitive]), ["Only TEXCOORD_0 and COLOR_0 are supported. The other sets are ignored"])

        self.assertIsNone(geometry.uvs)
        self.assertIsNone(geometry.display_colors)

    def test_material_that_does_not_exist(self):
        geometry = self._merge(
            _model([_node()], [_primitive(material=3)]),
            ["A primitive references material 3 that does not exist. It is treated as having no material"],
        )

        self.assertEqual(list(geometry.subsets), [None])

    def test_unused_vertices_are_removed(self):
        # The vertex 2 is not used by any face. It is removed, and the indices, the normals and the UVs follow.
        positions = np.array([[0, 0, 0], [1, 0, 0], [5, 5, 5], [0, 1, 0]], dtype=np.float32)
        normals = np.array([[0, 0, 1]] * 4, dtype=np.float32)
        uvs = np.array([[0, 0], [1, 0], [0.5, 0.5], [0, 1]], dtype=np.float32)

        geometry = self._merge(_model([_node()], [_primitive(indices=(0, 1, 3), POSITION=positions, NORMAL=normals, TEXCOORD_0=uvs)]))

        np.testing.assert_allclose(geometry.points, [[0, 0, 0], [1, 0, 0], [0, 1, 0]])
        self.assertEqual(geometry.face_vertex_indices.tolist(), [0, 1, 2])
        self.assertEqual(geometry.normals.shape, (3, 3))
        # V is flipped for USD.
        np.testing.assert_allclose(geometry.uvs, [[0, 1], [1, 1], [0, 0]])

    def test_normals_on_only_some_primitives(self):
        primitives = [_primitive(NORMAL=np.array([[0, 0, 1]] * 3, dtype=np.float32)), _primitive()]

        geometry = self._merge(_model([_node()], primitives), ["Only some primitives have NORMAL. The normals are ignored"])

        self.assertIsNone(geometry.normals)

    # The conversion of a whole file. A file that cannot be converted warns, and the rest of the URDF is still converted.

    def _convert_file(self, content: bytes | dict, file_name: str = "model.gltf", diagnostics: list[tuple] | None = None) -> Usd.Stage:
        source_dir = pathlib.Path(self.tmpDir())
        (source_dir / file_name).write_bytes(content if isinstance(content, bytes) else json.dumps(content).encode())
        (source_dir / "robot.urdf").write_text(
            f"""<?xml version="1.0"?>
<robot name="robot">
  <link name="base">
    <visual><geometry><mesh filename="{file_name}"/></geometry></visual>
    <collision><geometry><box size="1 1 1"/></geometry></collision>
  </link>
</robot>
""",
            encoding="utf-8",
        )

        with usdex.test.ScopedDiagnosticChecker(self, diagnostics or [], level=usdex.core.DiagnosticsLevel.eWarning):
            asset_path = urdf_usd_converter.Converter().convert(str(source_dir / "robot.urdf"), str(self.tmpDir()))
        self.assertIsNotNone(asset_path)
        return Usd.Stage.Open(asset_path.path)

    @staticmethod
    def _meshes(stage: Usd.Stage) -> list[Usd.Prim]:
        return [prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Mesh)]

    def test_file_that_cannot_be_parsed(self):
        stage = self._convert_file(b"{", diagnostics=[_warning(".*could not be parsed. Cannot parse glTF JSON.*")])

        self.assertEqual(self._meshes(stage), [])

    def test_malformed_file(self):
        # A property that refers to an item that does not exist is reported as a parse error, not as an exception.
        document = _gltf_document(meshes=[{"primitives": [{"attributes": {"POSITION": 9}}]}])

        stage = self._convert_file(document, diagnostics=[_warning(".*could not be parsed. The glTF file is malformed.*")])

        self.assertEqual(self._meshes(stage), [])

    def test_loader_warnings_are_reported_with_the_file(self):
        document = _gltf_document(animations=[{"channels": [], "samplers": []}])

        stage = self._convert_file(document, diagnostics=[_warning('.*Animations are skipped. in file: ".*model.gltf".*')])

        self.assertEqual(len(self._meshes(stage)), 1)

    def test_file_without_meshes(self):
        stage = self._convert_file(_gltf_document(nodes=[{}]), diagnostics=[_warning(".*contains no meshes.*")])

        self.assertEqual(self._meshes(stage), [])

    def test_display_name_keeps_the_unsafe_mesh_name(self):
        # A hyphen cannot be used in a prim name. The prim name is made safe, and the display name keeps the file name.
        stage = self._convert_file(_gltf_document(), file_name="my-model.gltf")

        (mesh,) = self._meshes(stage)
        self.assertNotEqual(mesh.GetName(), "my-model")
        self.assertTrue(Tf.IsValidIdentifier(mesh.GetName()))
        self.assertEqual(usdex.core.getDisplayName(mesh), "my-model")

    def test_mesh_that_usd_exchange_rejects(self):
        # A point that no face uses is invalid topology for definePolyMesh. _merge_geometry always removes such points,
        # so no glTF file reaches this failure. The test calls _define_mesh directly with such a geometry.
        # usd-exchange rejects it for real (no mock), which is reported as a runtime error, and _define_mesh warns and returns None.
        source = pathlib.Path("memory.gltf")
        geometry = _MergedGeometry(
            points=np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [5, 5, 5]], dtype=np.float32),
            normals=None,
            uvs=None,
            display_colors=None,
            display_opacities=None,
            face_vertex_indices=np.array([0, 1, 2], dtype=np.int32),
            subsets={None: np.array([0], dtype=np.int32)},
            double_sided=False,
        )
        stage = Usd.Stage.CreateInMemory()
        parent = usdex.core.defineXform(stage, "/Root").GetPrim()

        with usdex.test.ScopedDiagnosticChecker(
            self,
            [
                (Tf.TF_DIAGNOSTIC_RUNTIME_ERROR_TYPE, ".*invalid topology: Some points are not referenced by the faces"),
                _warning('.*Failed to convert mesh "/Root/unused" from memory.gltf.*'),
            ],
        ):
            mesh = _define_mesh(parent, "unused", "unused", geometry, _model([_node()], []), source, None, _Warner(source))

        self.assertIsNone(mesh)
        self.assertFalse(stage.GetPrimAtPath("/Root/unused").IsValid())
