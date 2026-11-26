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
from urdf_usd_converter._impl.conversion_gltf import _define_mesh, _MergedGeometry, _Warner

_POSITIONS = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=np.float32)
_NORMALS = np.array([[0, 0, 1]] * 3, dtype=np.float32)

# accessor.componentType of the arrays that _gltf_document writes, and accessor.type by the number of components.
_COMPONENT_TYPES = {np.dtype(np.float32): 5126, np.dtype(np.uint8): 5121, np.dtype(np.uint16): 5123}
_ACCESSOR_TYPES = {1: "SCALAR", 2: "VEC2", 3: "VEC3", 4: "VEC4"}


def _warning(pattern: str) -> tuple:
    return (Tf.TF_DIAGNOSTIC_WARNING_TYPE, pattern)


def _empty_model() -> gltf.Gltf:
    return gltf.Gltf(
        asset=gltf.AssetInfo(version="2.0"),
        scenes=[],
        scene=None,
        nodes=[],
        meshes=[],
        materials=[],
        textures=[],
        samplers=[],
        images=[],
    )


def _gltf_document(primitives: list[dict] | None = None, **sections) -> dict:
    """
    A glTF with one node and one mesh of the primitives. The sections replace the default ones. The default primitive is a triangle.

    A primitive is a dict of numpy arrays: its attributes ("POSITION", "NORMAL", "TEXCOORD_0", "COLOR_0", ...) and optionally "indices".
    The integers "mode" and "material" are written as they are. An attribute of an integer type is normalized.
    """
    buffer = bytearray()
    buffer_views = []
    accessors = []

    def add_accessor(array: np.ndarray, normalized: bool = False) -> int:
        buffer.extend(b"\x00" * (-len(buffer) % 4))
        buffer_views.append({"buffer": 0, "byteOffset": len(buffer), "byteLength": array.nbytes})
        buffer.extend(array.tobytes())
        accessor = {
            "bufferView": len(buffer_views) - 1,
            "componentType": _COMPONENT_TYPES[array.dtype],
            "count": len(array),
            "type": _ACCESSOR_TYPES[array.shape[1] if array.ndim == 2 else 1],
        }
        if normalized:
            accessor["normalized"] = True
        accessors.append(accessor)
        return len(accessors) - 1

    mesh_primitives = []
    for spec in [{"POSITION": _POSITIONS}] if primitives is None else primitives:
        attributes = dict(spec)
        primitive = {key: attributes.pop(key) for key in ("mode", "material") if key in attributes}
        if "indices" in attributes:
            primitive["indices"] = add_accessor(attributes.pop("indices"))
        primitive["attributes"] = {name: add_accessor(array, normalized=array.dtype.kind == "u") for name, array in attributes.items()}
        mesh_primitives.append(primitive)

    document = {
        "asset": {"version": "2.0"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"mesh": 0}],
        "meshes": [{"primitives": mesh_primitives}],
        "accessors": accessors,
        "bufferViews": buffer_views,
        "buffers": [{"uri": "data:application/octet-stream;base64," + base64.b64encode(bytes(buffer)).decode(), "byteLength": len(buffer)}],
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
        colors = np.array([[255, 0, 0, 255], [0, 255, 0, 51], [0, 0, 255, 0]], dtype=np.uint8)
        stage = self._convert_file(_gltf_document([{"POSITION": _POSITIONS, "NORMAL": _NORMALS, "COLOR_0": colors}]))
        self.assertIsValidUsd(stage)

        # Every primitive has COLOR_0, so there is only one mesh. It is the mesh itself, not a child of an Xform.
        mesh_prim = self._only_mesh(stage).GetPrim()
        self.assertEqual(len(mesh_prim.GetChildren()), 0)
        self.assertFalse(UsdShade.MaterialBindingAPI(mesh_prim).ComputeBoundMaterial()[0])
        primvars_api = UsdGeom.PrimvarsAPI(mesh_prim)
        np.testing.assert_allclose(np.array(primvars_api.GetPrimvar("displayColor").ComputeFlattened()), [[1, 0, 0], [0, 1, 0], [0, 0, 1]], atol=1e-6)
        display_opacity = primvars_api.GetPrimvar("displayOpacity")
        self.assertEqual(display_opacity.GetInterpolation(), UsdGeom.Tokens.vertex)
        np.testing.assert_allclose(np.array(display_opacity.ComputeFlattened()), [1.0, 0.2, 0.0], atol=1e-6)

    def test_vertex_colors_rgb_and_rgba_are_merged(self):
        # RGB and RGBA COLOR_0 are merged into one mesh. The opacity of an RGB COLOR_0 is 1.
        rgb = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float32)
        rgba = np.array([[1, 0, 0, 0.5], [0, 1, 0, 0.5], [0, 0, 1, 0.5]], dtype=np.float32)
        stage = self._convert_file(_gltf_document([{"POSITION": _POSITIONS, "COLOR_0": rgb}, {"POSITION": _POSITIONS, "COLOR_0": rgba}]))

        primvars_api = UsdGeom.PrimvarsAPI(self._only_mesh(stage).GetPrim())
        np.testing.assert_allclose(np.array(primvars_api.GetPrimvar("displayColor").ComputeFlattened()), np.concatenate([rgb, rgb]))
        np.testing.assert_allclose(np.array(primvars_api.GetPrimvar("displayOpacity").ComputeFlattened()), [1, 1, 1, 0.5, 0.5, 0.5])

    def test_vertex_colors_with_the_wrong_shape_are_ignored(self):
        # A COLOR_0 without a color for every vertex is ignored, so the primitive is an ordinary one without vertex colors.
        stage = self._convert_file(_gltf_document([{"POSITION": _POSITIONS, "COLOR_0": np.zeros((2, 3), dtype=np.float32)}]))

        self.assertIsNone(UsdGeom.PrimvarsAPI(self._only_mesh(stage).GetPrim()).GetPrimvar("displayColor").Get())

    def test_triangle_modes(self):
        # Every mode becomes a triangle list. The vertices that no triangle uses are removed.
        quad = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]], dtype=np.float32)
        cases = {
            # A trailing partial triangle is ignored, so the vertex 3 is removed.
            "triangles": (4, [0, 1, 2, 3, 2], [0, 1, 2]),
            # Odd triangles of a strip swap their last two vertices, so the winding stays consistent.
            "strip": (5, [0, 1, 2, 3], [0, 1, 2, 1, 3, 2]),
            # A fan shares the first vertex.
            "fan": (6, [0, 1, 2, 3], [1, 2, 0, 2, 3, 0]),
        }
        for name, (mode, indices, expected) in cases.items():
            with self.subTest(name):
                stage = self._convert_file(_gltf_document([{"POSITION": quad, "indices": np.array(indices, dtype=np.uint16), "mode": mode}]))

                mesh = self._only_mesh(stage)
                self.assertEqual(list(mesh.GetFaceVertexIndicesAttr().Get()), expected)
                self.assertEqual(list(mesh.GetFaceVertexCountsAttr().Get()), [3] * (len(expected) // 3))

        # Without indices, the vertex order is the topology.
        stage = self._convert_file(_gltf_document([{"POSITION": quad}]))
        mesh = self._only_mesh(stage)
        self.assertEqual(list(mesh.GetFaceVertexIndicesAttr().Get()), [0, 1, 2])

    def test_strip_and_fan_without_a_triangle(self):
        # Too few vertices for a strip or a fan, so there is nothing to convert.
        for mode in (5, 6):
            with self.subTest(mode=mode):
                document = _gltf_document([{"POSITION": _POSITIONS, "indices": np.array([0, 1], dtype=np.uint16), "mode": mode}])

                stage = self._convert_file(document, diagnostics=[_warning(".*contains no meshes.*")])

                self.assertEqual(self._meshes(stage), [])

    def test_negative_scale_reverses_winding(self):
        # A mirrored node flips the winding order, so it is flipped back to keep the surface facing the same way.
        stage = self._convert_file(_gltf_document(nodes=[{"mesh": 0, "scale": [-1, 1, 1]}]))

        mesh = self._only_mesh(stage)
        points = np.array(mesh.GetPointsAttr().Get())
        face_vertex_indices = list(mesh.GetFaceVertexIndicesAttr().Get())

        # x is mirrored.
        np.testing.assert_allclose(points, [[0, 0, 0], [-1, 0, 0], [0, 1, 0]], atol=1e-6)
        self.assertEqual(face_vertex_indices, [0, 2, 1])

        # The face normal still points the same way as the unmirrored triangle (+Z).
        a, b, c = points[face_vertex_indices]
        normal = np.cross(b - a, c - a)
        self.assertGreater(normal[2], 0.0)

    # Geometry that cannot be merged is skipped with a warning (the file still converts).

    def test_scene_that_does_not_exist(self):
        for name, sections in {"no scene": {"scenes": [], "scene": None}, "scene out of range": {"scene": 5}}.items():
            with self.subTest(name):
                stage = self._convert_file(
                    _gltf_document(**sections),
                    diagnostics=[_warning(".*The file has no scene.*"), _warning(".*contains no meshes.*")],
                )

                self.assertEqual(self._meshes(stage), [])

    def test_node_that_is_referenced_twice_or_does_not_exist(self):
        # The first node is its own child, and the child 7 does not exist. Each node is visited once, so there is no endless loop.
        stage = self._convert_file(
            _gltf_document(nodes=[{"mesh": 0, "children": [0, 7]}]),
            diagnostics=[
                _warning(".*Node 0 is referenced more than once or does not exist. It is skipped.*"),
                _warning(".*Node 7 is referenced more than once or does not exist. It is skipped.*"),
            ],
        )

        self.assertEqual(len(self._only_mesh(stage).GetPointsAttr().Get()), 3)

    def test_mesh_that_does_not_exist(self):
        stage = self._convert_file(
            _gltf_document(nodes=[{"mesh": 3}]),
            diagnostics=[_warning(".*Node 0 references a mesh that does not exist. It is skipped.*"), _warning(".*contains no meshes.*")],
        )

        self.assertEqual(self._meshes(stage), [])

    def test_singular_transform(self):
        # A node scaled to zero in one axis cannot be inverted for the normals, so its meshes are skipped.
        stage = self._convert_file(
            _gltf_document(nodes=[{"mesh": 0, "scale": [0, 1, 1]}]),
            diagnostics=[_warning(".*A node has a singular transform. Its meshes are skipped.*"), _warning(".*contains no meshes.*")],
        )

        self.assertEqual(self._meshes(stage), [])

    def test_primitives_that_are_skipped(self):
        primitives = [
            {"POSITION": _POSITIONS, "mode": 0},  # POINTS is not geometry. The loader warns about it, so the converter does not.
            {"indices": np.array([0, 1, 2], dtype=np.uint16)},  # No POSITION.
            {"POSITION": _POSITIONS, "indices": np.array([0, 1], dtype=np.uint16)},  # Not even one triangle. Nothing to skip silently.
            {"POSITION": _POSITIONS, "indices": np.array([0, 1, 9], dtype=np.uint16)},  # The vertex 9 does not exist.
            {"POSITION": _POSITIONS},
        ]

        stage = self._convert_file(
            _gltf_document(primitives),
            diagnostics=[
                _warning(r".*Contains non-triangle primitives \(mode=0\).*"),
                _warning(".*A primitive has no POSITION. It is skipped.*"),
                _warning(".*A primitive has a vertex index out of range. It is skipped.*"),
            ],
        )

        # Only the valid triangle is merged.
        mesh = self._only_mesh(stage)
        self.assertEqual(len(mesh.GetPointsAttr().Get()), 3)
        self.assertEqual(list(mesh.GetFaceVertexIndicesAttr().Get()), [0, 1, 2])

    def test_attribute_sets_other_than_zero(self):
        primitive = {"POSITION": _POSITIONS, "TEXCOORD_1": np.zeros((3, 2), dtype=np.float32), "COLOR_1": np.zeros((3, 3), dtype=np.float32)}

        stage = self._convert_file(
            _gltf_document([primitive]),
            diagnostics=[_warning(".*Only TEXCOORD_0 and COLOR_0 are supported. The other sets are ignored.*")],
        )

        primvars_api = UsdGeom.PrimvarsAPI(self._only_mesh(stage).GetPrim())
        self.assertFalse(primvars_api.GetPrimvar("st").IsDefined())
        self.assertIsNone(primvars_api.GetPrimvar("displayColor").Get())

    def test_material_that_does_not_exist(self):
        stage = self._convert_file(
            _gltf_document([{"POSITION": _POSITIONS, "material": 3}]),
            diagnostics=[_warning(".*A primitive references material 3 that does not exist. It is treated as having no material.*")],
        )

        # With no material there is no subset, and nothing is bound.
        mesh_prim = self._only_mesh(stage).GetPrim()
        self.assertEqual(len(mesh_prim.GetChildren()), 0)
        self.assertFalse(UsdShade.MaterialBindingAPI(mesh_prim).ComputeBoundMaterial()[0])

    def test_single_material_binds_the_mesh(self):
        # With one material there is no subset, and the mesh is bound directly.
        # (The subsets of several materials are checked with boxes.gltf and materials.gltf.)
        stage = self._convert_file(_gltf_document([{"POSITION": _POSITIONS, "NORMAL": _NORMALS, "material": 0}], materials=[{"name": "only"}]))

        mesh_prim = self._only_mesh(stage).GetPrim()
        self.assertEqual(len(mesh_prim.GetChildren()), 0)
        self.check_material_binding(mesh_prim, UsdShade.Material(stage.GetDefaultPrim().GetChild("Materials").GetChild("only")))

    def test_unused_vertices_are_removed(self):
        # The vertex 2 is not used by any face. It is removed, and the indices, the normals and the UVs follow.
        positions = np.array([[0, 0, 0], [1, 0, 0], [5, 5, 5], [0, 1, 0]], dtype=np.float32)
        normals = np.array([[0, 0, 1]] * 4, dtype=np.float32)
        uvs = np.array([[0, 0], [1, 0], [0.5, 0.5], [0, 1]], dtype=np.float32)
        primitive = {"POSITION": positions, "NORMAL": normals, "TEXCOORD_0": uvs, "indices": np.array([0, 1, 3], dtype=np.uint16)}

        stage = self._convert_file(_gltf_document([primitive]))
        mesh = self._only_mesh(stage)

        np.testing.assert_allclose(np.array(mesh.GetPointsAttr().Get()), [[0, 0, 0], [1, 0, 0], [0, 1, 0]])
        self.assertEqual(list(mesh.GetFaceVertexIndicesAttr().Get()), [0, 1, 2])
        primvars_api = UsdGeom.PrimvarsAPI(mesh.GetPrim())
        self.assertEqual(np.array(primvars_api.GetPrimvar("normals").ComputeFlattened()).shape, (3, 3))
        # V is flipped for USD.
        np.testing.assert_allclose(np.array(primvars_api.GetPrimvar("st").ComputeFlattened()), [[0, 1], [1, 1], [0, 0]])

    def test_normals_on_only_some_primitives(self):
        primitives = [{"POSITION": _POSITIONS, "NORMAL": _NORMALS}, {"POSITION": _POSITIONS}]

        stage = self._convert_file(
            _gltf_document(primitives),
            diagnostics=[_warning(".*Only some primitives have NORMAL. The normals are ignored.*")],
        )

        self.assertFalse(UsdGeom.PrimvarsAPI(self._only_mesh(stage).GetPrim()).GetPrimvar("normals").IsDefined())

    def test_uvs_on_only_some_primitives(self):
        # The vertices of a primitive without TEXCOORD_0 get (0, 0). The primitives without UVs differ in size.
        uvs = np.array([[0, 0], [1, 0], [0, 1]], dtype=np.float32)
        quad = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]], dtype=np.float32)
        primitives = [
            {"POSITION": _POSITIONS, "TEXCOORD_0": uvs},
            {"POSITION": _POSITIONS},
            {"POSITION": quad, "indices": np.array([0, 1, 2, 0, 2, 3], dtype=np.uint16)},
        ]

        stage = self._convert_file(_gltf_document(primitives))

        st = UsdGeom.PrimvarsAPI(self._only_mesh(stage).GetPrim()).GetPrimvar("st")
        # V is flipped for USD. The vertices without UVs are (0, 0) as they are, which equals the flipped (0, 1).
        np.testing.assert_allclose(np.array(st.ComputeFlattened()), [[0, 1], [1, 1], [0, 0]] + [[0, 0]] * 7)
        # Equal values are stored once.
        self.assertEqual(len(st.Get()), 3)

    def test_mesh_under_several_nodes(self):
        # One mesh under three nodes: as it is, rotated 90 degrees about X, and moved. The points and the normals follow the node,
        # the UVs do not. The values that come out equal are stored once, even though they come from different transforms.
        half = float(np.sqrt(0.5))
        nodes = [
            {"mesh": 0},
            {"mesh": 0, "rotation": [half, 0, 0, half]},
            {"mesh": 0, "translation": [5, 0, 0]},
        ]
        uvs = np.array([[0, 0], [1, 0], [0, 1]], dtype=np.float32)
        primitive = {"POSITION": _POSITIONS, "NORMAL": _NORMALS, "TEXCOORD_0": uvs}

        stage = self._convert_file(_gltf_document([primitive], scenes=[{"nodes": [0, 1, 2]}], nodes=nodes))

        mesh = self._only_mesh(stage)
        np.testing.assert_allclose(
            np.array(mesh.GetPointsAttr().Get()),
            [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 0], [1, 0, 0], [0, 0, 1], [5, 0, 0], [6, 0, 0], [5, 1, 0]],
            atol=1e-6,
        )
        self.assertEqual(list(mesh.GetFaceVertexIndicesAttr().Get()), list(range(9)))

        primvars_api = UsdGeom.PrimvarsAPI(mesh.GetPrim())
        normals = primvars_api.GetPrimvar("normals")
        np.testing.assert_allclose(
            np.array(normals.ComputeFlattened()),
            [[0, 0, 1]] * 3 + [[0, -1, 0]] * 3 + [[0, 0, 1]] * 3,
            atol=1e-6,
        )
        self.assertEqual(len(normals.Get()), 2)

        st = primvars_api.GetPrimvar("st")
        np.testing.assert_allclose(np.array(st.ComputeFlattened()), [[0, 1], [1, 1], [0, 0]] * 3)
        self.assertEqual(len(st.Get()), 3)

    # COLOR_0 and the material. glTF multiplies the baseColorFactor, the baseColorTexture and COLOR_0, but only one of the
    # material and the vertex colors can be kept. The boxes.gltf test above covers a white baseColorFactor without a texture.

    def _display_color(self, mesh: UsdGeom.Mesh):
        return UsdGeom.PrimvarsAPI(mesh.GetPrim()).GetPrimvar("displayColor").Get()

    def test_vertex_colors_that_are_all_one_are_ignored(self):
        # COLOR_0 of all 1 changes nothing in glTF, so the material is kept and nothing is lost. There is no warning.
        # One primitive has RGB and the other RGBA. Both stay in one mesh with a subset for each material.
        primitives = [
            {"POSITION": _POSITIONS, "NORMAL": _NORMALS, "COLOR_0": np.ones((3, 3), dtype=np.float32), "material": 0},
            {"POSITION": _POSITIONS, "NORMAL": _NORMALS, "COLOR_0": np.ones((3, 4), dtype=np.float32), "material": 1},
        ]
        materials = [
            {"name": "dark", "pbrMetallicRoughness": {"baseColorFactor": [0.05, 0.05, 0.05, 1]}},
            {"name": "red", "pbrMetallicRoughness": {"baseColorFactor": [0.8, 0.1, 0.1, 1]}},
        ]

        stage = self._convert_file(_gltf_document(primitives, materials=materials))

        # There is no separate mesh for vertex colors.
        mesh = self._only_mesh(stage)
        self.assertIsNone(self._display_color(mesh))
        self.assertEqual(len(UsdGeom.Subset.GetAllGeomSubsets(mesh)), 2)
        self.assertEqual({child.GetName() for child in stage.GetDefaultPrim().GetChild("Materials").GetChildren()}, {"dark", "red"})

    def test_vertex_colors_are_ignored_when_the_base_color_is_set(self):
        # With a baseColorFactor other than white, or with a baseColorTexture, the material is kept and COLOR_0 is ignored.
        colors = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float32)
        primitives = [
            {"POSITION": _POSITIONS, "NORMAL": _NORMALS, "COLOR_0": colors, "material": 0},
            {"POSITION": _POSITIONS, "NORMAL": _NORMALS, "COLOR_0": colors, "material": 1},
        ]
        materials = [
            {"name": "tinted", "pbrMetallicRoughness": {"baseColorFactor": [0.5, 0.5, 0.5, 1]}},
            {"name": "textured", "pbrMetallicRoughness": {"baseColorTexture": {"index": 0}}},
        ]
        document = _gltf_document(primitives, materials=materials, textures=[{"source": 0}], images=[{"uri": "data:image/png,%89PNG"}])

        stage = self._convert_file(
            document,
            diagnostics=[
                _warning('.*Material "tinted" has a baseColorFactor or baseColorTexture that cannot be combined with COLOR_0. The vertex colors.*'),
                _warning('.*Material "textured" has a baseColorFactor or baseColorTexture that cannot be combined with COLOR_0. The vertex colors.*'),
            ],
        )

        mesh = self._only_mesh(stage)
        self.assertIsNone(self._display_color(mesh))
        self.assertEqual(len(UsdGeom.Subset.GetAllGeomSubsets(mesh)), 2)
        self.assertEqual({child.GetName() for child in stage.GetDefaultPrim().GetChild("Materials").GetChildren()}, {"tinted", "textured"})

    def test_vertex_colors_without_material(self):
        # Without a material, COLOR_0 is the display color, even when it is all 1 (the default material of glTF is white).
        varied = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float32)
        ones = np.ones((3, 3), dtype=np.float32)
        primitives = [
            {"POSITION": _POSITIONS, "COLOR_0": varied},
            {"POSITION": _POSITIONS + 2, "COLOR_0": ones},
        ]

        stage = self._convert_file(_gltf_document(primitives))

        mesh = self._only_mesh(stage)
        np.testing.assert_allclose(
            np.array(UsdGeom.PrimvarsAPI(mesh.GetPrim()).GetPrimvar("displayColor").ComputeFlattened()), np.concatenate([varied, ones])
        )
        self.assertEqual(len(mesh.GetPrim().GetChildren()), 0)
        self.assertFalse(UsdShade.MaterialBindingAPI(mesh.GetPrim()).ComputeBoundMaterial()[0])

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

    def _only_mesh(self, stage: Usd.Stage) -> UsdGeom.Mesh:
        (prim,) = self._meshes(stage)
        return UsdGeom.Mesh(prim)

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
            mesh = _define_mesh(parent, "unused", "unused", geometry, _empty_model(), source, None, _Warner(source))

        self.assertIsNone(mesh)
        self.assertFalse(stage.GetPrimAtPath("/Root/unused").IsValid())
