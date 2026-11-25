# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
"""
Tests for the glTF loader only (urdf_usd_converter/_impl/gltf).

Like tests/test_parser.py for the URDF parser, these tests do not convert anything to USD.
The glTF and GLB files are written by the tests, and the objects returned by ``gltf.load`` are checked.
"""

import ast
import base64
import json
import math
import pathlib
import struct
import tempfile
import unittest

import numpy as np

from urdf_usd_converter._impl import gltf
from urdf_usd_converter._impl.gltf.binary import read_accessor
from urdf_usd_converter._impl.gltf.errors import GltfError, warn_once

# accessor.componentType values.
BYTE = 5120
UNSIGNED_BYTE = 5121
SHORT = 5122
UNSIGNED_SHORT = 5123
UNSIGNED_INT = 5125
FLOAT = 5126

_GLB_MAGIC = b"glTF"
_JSON_CHUNK = 0x4E4F534A
_BIN_CHUNK = 0x004E4942

_PNG = b"\x89PNG\r\n\x1a\nX"  # Not a valid image. The loader does not decode pixels.


def _document(**sections) -> dict:
    document = {"asset": {"version": "2.0"}}
    document.update(sections)
    return document


def _data_uri(data: bytes, mime_type: str = "application/octet-stream") -> str:
    return f"data:{mime_type};base64,{base64.b64encode(data).decode()}"


def _glb(document: dict, binary: bytes | None = None) -> bytes:
    """Build a GLB. Chunks are padded to 4 bytes like the glTF specification asks."""
    json_bytes = json.dumps(document).encode("utf-8")
    json_bytes += b" " * (-len(json_bytes) % 4)
    chunks = struct.pack("<II", len(json_bytes), _JSON_CHUNK) + json_bytes
    if binary is not None:
        binary += b"\x00" * (-len(binary) % 4)
        chunks += struct.pack("<II", len(binary), _BIN_CHUNK) + binary
    return struct.pack("<4sII", _GLB_MAGIC, 2, 12 + len(chunks)) + chunks


class _Builder:
    """Collect arrays into one buffer. Each array gets a bufferView and an accessor."""

    def __init__(self):
        self.buffer = bytearray()
        self.views: list[dict] = []
        self.accessors: list[dict] = []

    def add(self, array: np.ndarray, type_name: str, component_type: int, normalized: bool = False) -> int:
        """Add an array and return the index of its accessor."""
        while len(self.buffer) % 4:
            self.buffer.append(0)
        self.views.append({"buffer": 0, "byteOffset": len(self.buffer), "byteLength": array.nbytes})
        self.buffer += array.tobytes()
        accessor = {"bufferView": len(self.views) - 1, "componentType": component_type, "count": len(array), "type": type_name}
        if normalized:
            accessor["normalized"] = True
        self.accessors.append(accessor)
        return len(self.accessors) - 1

    def add_triangle(self) -> tuple[int, int]:
        """Add a triangle. Return the accessor of the positions and the accessor of the indices."""
        positions = self.add(np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=np.float32), "VEC3", FLOAT)
        indices = self.add(np.array([0, 1, 2], dtype=np.uint16), "SCALAR", UNSIGNED_SHORT)
        return positions, indices

    def document(self, uri: str | None = "data", **sections) -> dict:
        """
        Return the glTF document. ``uri`` is the uri of the buffer: "data" embeds the buffer as a data URI,
        and None leaves it out (the BIN chunk of a GLB).
        """
        buffer = {"byteLength": len(self.buffer)}
        if uri == "data":
            buffer["uri"] = _data_uri(bytes(self.buffer))
        elif uri is not None:
            buffer["uri"] = uri
        return _document(buffers=[buffer], bufferViews=self.views, accessors=self.accessors, **sections)


def _triangle_mesh_document(**sections) -> tuple[_Builder, dict]:
    builder = _Builder()
    positions, indices = builder.add_triangle()
    document = builder.document(
        meshes=[{"name": "tri", "primitives": [{"attributes": {"POSITION": positions}, "indices": indices}]}],
        **sections,
    )
    return builder, document


class TestGltfLoader(unittest.TestCase):
    def setUp(self):
        self.directory = pathlib.Path(self.enterContext(tempfile.TemporaryDirectory()))

    def _write(self, document: dict, name: str = "model.gltf") -> pathlib.Path:
        path = self.directory / name
        path.write_text(json.dumps(document, ensure_ascii=False), encoding="utf-8")
        return path

    def _write_bytes(self, data: bytes, name: str) -> pathlib.Path:
        path = self.directory / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        return path

    def _load(self, document: dict) -> gltf.Gltf:
        return gltf.load(self._write(document))

    # Files and formats.

    def test_does_not_import_usd(self):
        # The loader is independent of USD, so it can be tested and used without USD.
        package_dir = pathlib.Path(gltf.__file__).parent
        imported = set()
        for path in package_dir.glob("*.py"):
            for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
                if isinstance(node, ast.Import):
                    imported.update(alias.name.split(".")[0] for alias in node.names)
                elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                    imported.add(node.module.split(".")[0])
        self.assertIn("numpy", imported)  # The imports are collected.
        self.assertFalse(imported & {"pxr", "usdex"}, msg=sorted(imported))

    def test_load_non_existent_file(self):
        with self.assertRaises(FileNotFoundError):
            gltf.load(self.directory / "non_existent.gltf")

    def test_load_accepts_str_path(self):
        model = gltf.load(str(self._write(_document())))
        self.assertEqual(model.asset.version, "2.0")

    def test_load_error_json_syntax(self):
        path = self._write_bytes(b"{", "model.gltf")
        with self.assertRaisesRegex(GltfError, "Cannot parse glTF JSON"):
            gltf.load(path)

    def test_load_error_json_is_not_object(self):
        path = self._write_bytes(b"[]", "model.gltf")
        with self.assertRaisesRegex(GltfError, "glTF JSON must be an object"):
            gltf.load(path)

    def test_load_error_not_utf8(self):
        path = self._write_bytes(b'{"asset": {"version": "2.0"}, "x": "\xff"}', "model.gltf")
        with self.assertRaisesRegex(GltfError, "glTF JSON is not UTF-8"):
            gltf.load(path)

    def test_load_non_ascii_names(self):
        # The JSON is UTF-8, so non-ASCII names are kept.
        model = self._load(
            _document(
                nodes=[{"name": "ノード"}],
                scenes=[{"name": "シーン", "nodes": [0]}],
                meshes=[{"name": "メッシュ", "primitives": []}],
                materials=[{"name": "マテリアル"}],
            )
        )
        self.assertEqual(model.nodes[0].name, "ノード")
        self.assertEqual(model.scenes[0].name, "シーン")
        self.assertEqual(model.meshes[0].name, "メッシュ")
        self.assertEqual(model.materials[0].name, "マテリアル")

    def test_load_glb(self):
        builder = _Builder()
        positions, indices = builder.add_triangle()
        document = builder.document(uri=None, meshes=[{"primitives": [{"attributes": {"POSITION": positions}, "indices": indices}]}])
        path = self._write_bytes(_glb(document, bytes(builder.buffer)), "model.glb")

        model = gltf.load(path)

        primitive = model.meshes[0].primitives[0]
        np.testing.assert_array_equal(primitive.attributes["POSITION"], [[0, 0, 0], [1, 0, 0], [0, 1, 0]])
        np.testing.assert_array_equal(primitive.indices, [0, 1, 2])
        self.assertEqual(model.warnings, [])

    def test_load_glb_is_detected_by_magic(self):
        # A GLB is read as a GLB even when the suffix is not .glb.
        builder = _Builder()
        positions, _ = builder.add_triangle()
        document = builder.document(uri=None, meshes=[{"primitives": [{"attributes": {"POSITION": positions}}]}])
        path = self._write_bytes(_glb(document, bytes(builder.buffer)), "model.dat")

        model = gltf.load(path)

        self.assertEqual(len(model.meshes[0].primitives[0].attributes["POSITION"]), 3)

    def test_load_error_glb(self):
        header = struct.Struct("<4sII")
        cases = {
            "GLB header is too short": _GLB_MAGIC + b"\x02\x00",
            "Unsupported GLB version: 1": header.pack(_GLB_MAGIC, 1, 12),
            "GLB header length does not match the file size": header.pack(_GLB_MAGIC, 2, 99),
            "GLB chunk extends past the end of the file": header.pack(_GLB_MAGIC, 2, 20) + struct.pack("<II", 100, _JSON_CHUNK),
            "The first GLB chunk is not JSON": header.pack(_GLB_MAGIC, 2, 24) + struct.pack("<II", 4, _BIN_CHUNK) + b"\x00" * 4,
            "GLB has no JSON chunk": header.pack(_GLB_MAGIC, 2, 12),
        }
        for message, data in cases.items():
            with self.subTest(message):
                path = self._write_bytes(data, "model.glb")
                with self.assertRaisesRegex(GltfError, message):
                    gltf.load(path)

    def test_load_error_glb_magic(self):
        # A .glb suffix means GLB, whatever the content is.
        path = self._write_bytes(json.dumps(_document()).encode(), "model.glb")
        with self.assertRaisesRegex(GltfError, "GLB magic is not 'glTF'"):
            gltf.load(path)

    # Buffers.

    def test_load_external_buffer(self):
        # The path is relative to the glTF. A percent-encoded path is decoded.
        builder, document = _triangle_mesh_document()
        document["buffers"][0]["uri"] = "bin/my%20buffer.bin"
        self._write_bytes(bytes(builder.buffer), "bin/my buffer.bin")

        model = self._load(document)

        np.testing.assert_array_equal(model.meshes[0].primitives[0].indices, [0, 1, 2])
        self.assertEqual(model.warnings, [])

    def test_load_error_missing_buffer_file(self):
        _, document = _triangle_mesh_document()
        document["buffers"][0]["uri"] = "missing.bin"
        with self.assertRaisesRegex(GltfError, "Referenced file does not exist: missing.bin"):
            self._load(document)

    def test_load_error_buffer_is_shorter_than_byte_length(self):
        _, document = _triangle_mesh_document()
        document["buffers"][0]["byteLength"] += 4
        with self.assertRaisesRegex(GltfError, r"buffers\[0\] is shorter than byteLength"):
            self._load(document)

    def test_load_error_buffer_without_data(self):
        # Only buffers[0] of a GLB can use the BIN chunk. A glTF has no BIN chunk.
        with self.assertRaisesRegex(GltfError, r"buffers\[0\] has no data"):
            self._load(_document(buffers=[{"byteLength": 4}]))

    def test_buffer_url_is_skipped_with_warning(self):
        model = self._load(_document(buffers=[{"uri": "http://example.com/model.bin", "byteLength": 4}]))
        self.assertIn("External URLs are not supported: http://example.com/model.bin", model.warnings)

    def test_load_error_data_uri(self):
        with self.assertRaisesRegex(GltfError, "Malformed data URI"):
            self._load(_document(buffers=[{"uri": "data:application/octet-stream;base64", "byteLength": 4}]))
        with self.assertRaisesRegex(GltfError, "Cannot decode base64 in data URI"):
            self._load(_document(buffers=[{"uri": "data:application/octet-stream;base64,abc", "byteLength": 4}]))

    # The asset.

    def test_asset(self):
        model = self._load({"asset": {"version": "2.1", "minVersion": "2.0", "generator": "Test Generator", "copyright": "(c) Test"}})

        self.assertEqual(model.asset.version, "2.1")
        self.assertEqual(model.asset.min_version, "2.0")
        self.assertEqual(model.asset.generator, "Test Generator")
        self.assertEqual(model.asset.copyright, "(c) Test")

    def test_asset_optional_fields(self):
        asset = self._load(_document()).asset

        self.assertIsNone(asset.min_version)
        self.assertIsNone(asset.generator)
        self.assertIsNone(asset.copyright)

    def test_load_error_asset(self):
        cases = {
            "asset.version is missing": {},
            "asset.version is missing (no version)": {"asset": {"generator": "x"}},
            "Only glTF 2.x is supported: 1.0": {"asset": {"version": "1.0"}},
            "Unsupported minVersion: 3.0": {"asset": {"version": "2.0", "minVersion": "3.0"}},
            "Malformed version string: two": {"asset": {"version": "two"}},
            "Malformed version string: (empty)": {"asset": {"version": ""}},
            "Malformed version string: .0": {"asset": {"version": ".0"}},
            "Malformed version string: -2.0": {"asset": {"version": "-2.0"}},
            "Malformed version string: \u00b20.0": {"asset": {"version": "\u00b20.0"}},
        }
        for name, document in cases.items():
            with self.subTest(name), self.assertRaisesRegex(GltfError, name.split(" (")[0]):
                self._load(document)

    # Extensions and unsupported features.

    def test_extensions(self):
        model = self._load(_document(extensionsUsed=["KHR_texture_transform", "KHR_mesh_quantization"], extensionsRequired=["KHR_texture_transform"]))

        self.assertEqual(model.extensions_used, ["KHR_texture_transform", "KHR_mesh_quantization"])
        self.assertEqual(model.extensions_required, ["KHR_texture_transform"])
        self.assertEqual(model.warnings, [])

    def test_load_error_extensions_required(self):
        with self.assertRaisesRegex(GltfError, "extensionsRequired is not listed in extensionsUsed: KHR_texture_transform"):
            self._load(_document(extensionsRequired=["KHR_texture_transform"]))
        with self.assertRaisesRegex(GltfError, "Unsupported required extension: EXT_unknown"):
            self._load(_document(extensionsUsed=["EXT_unknown"], extensionsRequired=["EXT_unknown"]))

    def test_compression_required_fails(self):
        # Draco and meshopt cannot be read, so they fail when the asset requires them.
        for name in ("KHR_draco_mesh_compression", "KHR_meshopt_compression", "EXT_meshopt_compression"):
            with self.subTest(name), self.assertRaisesRegex(GltfError, f"Unsupported required extension: {name}"):
                self._load(_document(extensionsUsed=[name], extensionsRequired=[name]))

    def test_gaussian_splatting_required_warns_without_failing(self):
        model = self._load(
            _document(
                extensionsUsed=["KHR_gaussian_splatting"],
                extensionsRequired=["KHR_gaussian_splatting"],
            )
        )

        self.assertEqual(model.extensions_required, ["KHR_gaussian_splatting"])
        self.assertEqual(model.warnings, ["KHR_gaussian_splatting is skipped."])

    def test_unsupported_features_warn(self):
        model = self._load(
            _document(
                animations=[{}],
                skins=[{}],
                extensionsUsed=["KHR_materials_clearcoat", "KHR_gaussian_splatting"],
            )
        )

        self.assertIn("Animations are skipped.", model.warnings)
        self.assertIn("Skins are skipped.", model.warnings)
        self.assertIn("KHR_materials_clearcoat is ignored with a warning. Only core PBR is used.", model.warnings)
        self.assertIn("KHR_gaussian_splatting is skipped.", model.warnings)

    def test_warn_once(self):
        warnings = []
        warn_once(warnings, "a")
        warn_once(warnings, "b")
        warn_once(warnings, "a")
        self.assertEqual(warnings, ["a", "b"])

    # Scenes and nodes.

    def test_scenes(self):
        model = self._load(_document(scenes=[{"name": "first", "nodes": [0]}, {"nodes": [1, 2]}], scene=1))

        self.assertEqual(model.scene, 1)
        self.assertEqual([scene.name for scene in model.scenes], ["first", None])
        self.assertEqual(model.scenes[1].nodes, [1, 2])

    def test_default_scene(self):
        # Without "scene" the first scene is used. Without scenes there is none.
        self.assertEqual(self._load(_document(scenes=[{"nodes": []}])).scene, 0)
        model = self._load(_document())
        self.assertIsNone(model.scene)
        self.assertEqual(model.scenes, [])

    def test_node_defaults(self):
        node = self._load(_document(nodes=[{}])).nodes[0]

        self.assertIsNone(node.name)
        self.assertEqual(node.children, [])
        self.assertIsNone(node.mesh)
        self.assertIsNone(node.matrix)
        np.testing.assert_array_equal(node.translation, [0, 0, 0])
        np.testing.assert_array_equal(node.rotation, [0, 0, 0, 1])
        np.testing.assert_array_equal(node.scale, [1, 1, 1])
        np.testing.assert_array_equal(gltf.local_matrix(node), np.eye(4))

    def test_node_hierarchy(self):
        nodes = self._load(_document(nodes=[{"name": "parent", "children": [1], "mesh": 0}, {"name": "child"}])).nodes

        self.assertEqual(nodes[0].name, "parent")
        self.assertEqual(nodes[0].children, [1])
        self.assertEqual(nodes[0].mesh, 0)

    def test_local_matrix_from_trs(self):
        # The matrix is T * R * S for column vectors. A quaternion is (x, y, z, w).
        half = math.sqrt(0.5)
        node = self._load(_document(nodes=[{"translation": [1, 2, 3], "rotation": [0, 0, half, half], "scale": [2, 2, 2]}])).nodes[0]

        point = gltf.local_matrix(node) @ np.array([1.0, 0.0, 0.0, 1.0])

        # Scaled to (2, 0, 0), rotated 90 degrees about Z to (0, 2, 0), then translated.
        np.testing.assert_allclose(point, [1, 4, 3, 1], atol=1e-12)

    def test_local_matrix_from_matrix(self):
        # A glTF matrix is 16 column-major floats, so the translation is the last column.
        values = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 5, 6, 7, 1]
        node = self._load(_document(nodes=[{"matrix": values}])).nodes[0]

        np.testing.assert_array_equal(node.matrix[:3, 3], [5, 6, 7])
        self.assertIs(gltf.local_matrix(node), node.matrix)

    # Accessors.

    def test_accessor_float_vec3(self):
        builder = _Builder()
        values = np.array([[1, 2, 3], [4, 5, 6]], dtype=np.float32)
        index = builder.add(values, "VEC3", FLOAT)

        result = read_accessor(builder.document(), [bytes(builder.buffer)], index)

        self.assertEqual(result.dtype, np.float32)
        self.assertEqual(result.shape, (2, 3))
        np.testing.assert_array_equal(result, values)

    def test_accessor_scalar_and_matrix_shapes(self):
        builder = _Builder()
        scalar = builder.add(np.array([1, 2, 3], dtype=np.uint16), "SCALAR", UNSIGNED_SHORT)
        matrix = builder.add(np.zeros((2, 16), dtype=np.float32), "MAT4", FLOAT)
        document = builder.document()
        buffers = [bytes(builder.buffer)]

        self.assertEqual(read_accessor(document, buffers, scalar).shape, (3,))
        self.assertEqual(read_accessor(document, buffers, matrix).shape, (2, 16))

    def test_accessor_component_types(self):
        builder = _Builder()
        arrays = {
            BYTE: np.array([-128, 127], dtype=np.int8),
            UNSIGNED_BYTE: np.array([0, 255], dtype=np.uint8),
            SHORT: np.array([-32768, 32767], dtype=np.int16),
            UNSIGNED_SHORT: np.array([0, 65535], dtype=np.uint16),
            UNSIGNED_INT: np.array([0, 4294967295], dtype=np.uint32),
        }
        indices = {component_type: builder.add(array, "SCALAR", component_type) for component_type, array in arrays.items()}
        document = builder.document()

        # The stored dtype is kept for values that are not normalized.
        for component_type, array in arrays.items():
            result = read_accessor(document, [bytes(builder.buffer)], indices[component_type])
            self.assertEqual(result.dtype, array.dtype)
            np.testing.assert_array_equal(result, array)

    def test_accessor_normalized(self):
        # Normalized integers become float32. A signed value clamps at -1 (-128 would be a little less than -1).
        builder = _Builder()
        cases = {
            "ubyte": (builder.add(np.array([0, 255], dtype=np.uint8), "SCALAR", UNSIGNED_BYTE, normalized=True), [0.0, 1.0]),
            "ushort": (builder.add(np.array([0, 65535], dtype=np.uint16), "SCALAR", UNSIGNED_SHORT, normalized=True), [0.0, 1.0]),
            "byte": (builder.add(np.array([-128, 0, 127], dtype=np.int8), "SCALAR", BYTE, normalized=True), [-1.0, 0.0, 1.0]),
            "short": (builder.add(np.array([-32768, 0, 32767], dtype=np.int16), "SCALAR", SHORT, normalized=True), [-1.0, 0.0, 1.0]),
        }
        document = builder.document()

        for name, (index, expected) in cases.items():
            with self.subTest(name):
                result = read_accessor(document, [bytes(builder.buffer)], index)
                self.assertEqual(result.dtype, np.float32)
                np.testing.assert_allclose(result, expected, atol=1e-7)

    def test_accessor_byte_stride(self):
        # Interleaved vertices: a position (12 bytes) and a texture coordinate (8 bytes) per vertex.
        positions = np.array([[1, 2, 3], [4, 5, 6]], dtype=np.float32)
        uvs = np.array([[0.5, 0.25], [0.75, 0.125]], dtype=np.float32)
        buffer = b"".join(positions[i].tobytes() + uvs[i].tobytes() for i in range(2))
        document = {
            "accessors": [
                {"bufferView": 0, "componentType": FLOAT, "count": 2, "type": "VEC3"},
                {"bufferView": 0, "byteOffset": 12, "componentType": FLOAT, "count": 2, "type": "VEC2"},
            ],
            "bufferViews": [{"buffer": 0, "byteLength": len(buffer), "byteStride": 20}],
        }

        np.testing.assert_array_equal(read_accessor(document, [buffer], 0, vertex_attribute=True), positions)
        np.testing.assert_array_equal(read_accessor(document, [buffer], 1, vertex_attribute=True), uvs)

    def test_accessor_byte_stride_is_ignored_for_indices(self):
        buffer = np.array([0, 1, 2], dtype=np.uint16).tobytes()
        document = {
            "accessors": [{"bufferView": 0, "componentType": UNSIGNED_SHORT, "count": 3, "type": "SCALAR"}],
            "bufferViews": [{"buffer": 0, "byteLength": len(buffer), "byteStride": 4}],
        }

        np.testing.assert_array_equal(read_accessor(document, [buffer], 0), [0, 1, 2])

    def test_accessor_unaligned_offset(self):
        # A view that does not start at a multiple of the element size cannot be a view of the buffer, so it is copied.
        values = np.array([1.5, 2.5, 3.5], dtype=np.float32)
        buffer = b"\x00\x00" + values.tobytes()
        document = {
            "accessors": [{"bufferView": 0, "componentType": FLOAT, "count": 3, "type": "SCALAR"}],
            "bufferViews": [{"buffer": 0, "byteOffset": 2, "byteLength": 12}],
        }

        np.testing.assert_array_equal(read_accessor(document, [buffer], 0), values)

    def test_accessor_without_buffer_view_is_zero(self):
        document = {
            "accessors": [
                {"componentType": FLOAT, "count": 2, "type": "VEC3"},
                {"componentType": FLOAT, "count": 0, "type": "VEC3"},
                {"componentType": FLOAT, "count": 0, "type": "SCALAR"},
            ]
        }

        np.testing.assert_array_equal(read_accessor(document, [], 0), np.zeros((2, 3)))
        self.assertEqual(read_accessor(document, [], 1).shape, (0, 3))
        self.assertEqual(read_accessor(document, [], 2).shape, (0,))

    def test_accessor_sparse(self):
        # Sparse values replace the base values. The buffer is not modified.
        base = np.array([1, 2, 3, 4], dtype=np.float32).tobytes()
        indices = np.array([1, 3], dtype=np.uint8).tobytes() + b"\x00\x00"
        values = np.array([50, 70], dtype=np.float32).tobytes()
        buffer = base + indices + values
        document = {
            "accessors": [
                {
                    "bufferView": 0,
                    "componentType": FLOAT,
                    "count": 4,
                    "type": "SCALAR",
                    "sparse": {
                        "count": 2,
                        "indices": {"bufferView": 1, "componentType": UNSIGNED_BYTE},
                        "values": {"bufferView": 2},
                    },
                },
                {
                    "componentType": FLOAT,
                    "count": 4,
                    "type": "SCALAR",
                    "sparse": {
                        "count": 2,
                        "indices": {"bufferView": 1, "componentType": UNSIGNED_BYTE},
                        "values": {"bufferView": 2},
                    },
                },
            ],
            "bufferViews": [
                {"buffer": 0, "byteOffset": 0, "byteLength": 16},
                {"buffer": 0, "byteOffset": 16, "byteLength": 2},
                {"buffer": 0, "byteOffset": 20, "byteLength": 8},
            ],
        }

        np.testing.assert_array_equal(read_accessor(document, [buffer], 0), [1, 50, 3, 70])
        # Without a bufferView the base values are zero.
        np.testing.assert_array_equal(read_accessor(document, [buffer], 1), [0, 50, 0, 70])
        self.assertEqual(buffer[:16], base)

    def test_accessor_errors(self):
        buffer = np.zeros(2, dtype=np.float32).tobytes()
        views = [{"buffer": 0, "byteLength": len(buffer)}]
        cases = {
            "Unsupported accessor.type: VEC5": {"componentType": FLOAT, "count": 1, "type": "VEC5"},
            "Unsupported componentType: 5124": {"bufferView": 0, "componentType": 5124, "count": 1, "type": "SCALAR"},
            "FLOAT accessors cannot be normalized": {"bufferView": 0, "componentType": FLOAT, "count": 1, "type": "SCALAR", "normalized": True},
            "Accessor exceeds the bufferView range": {"bufferView": 0, "componentType": FLOAT, "count": 3, "type": "SCALAR"},
        }
        for message, accessor in cases.items():
            with self.subTest(message), self.assertRaisesRegex(GltfError, message):
                read_accessor({"accessors": [accessor], "bufferViews": views}, [buffer], 0)

    def test_accessor_error_byte_stride_is_too_small(self):
        buffer = np.zeros(3, dtype=np.float32).tobytes()
        document = {
            "accessors": [{"bufferView": 0, "componentType": FLOAT, "count": 1, "type": "VEC3"}],
            "bufferViews": [{"buffer": 0, "byteLength": len(buffer), "byteStride": 4}],
        }
        with self.assertRaisesRegex(GltfError, r"byteStride \(4\) is smaller than the element size \(12\)"):
            read_accessor(document, [buffer], 0, vertex_attribute=True)

    # Meshes and primitives.

    def test_primitive(self):
        _, document = _triangle_mesh_document()

        model = self._load(document)

        self.assertEqual(model.warnings, [])
        mesh = model.meshes[0]
        self.assertEqual(mesh.name, "tri")
        primitive = mesh.primitives[0]
        self.assertEqual(primitive.mode, 4)
        self.assertIsNone(primitive.material)
        self.assertEqual(primitive.attributes["POSITION"].dtype, np.float32)
        np.testing.assert_array_equal(primitive.attributes["POSITION"], [[0, 0, 0], [1, 0, 0], [0, 1, 0]])
        self.assertEqual(primitive.indices.dtype, np.uint16)
        np.testing.assert_array_equal(primitive.indices, [0, 1, 2])

    def test_primitive_without_indices_and_with_material(self):
        _, document = _triangle_mesh_document()
        primitive = document["meshes"][0]["primitives"][0]
        del primitive["indices"]
        primitive["material"] = 2

        loaded = self._load(document).meshes[0].primitives[0]

        self.assertIsNone(loaded.indices)
        self.assertEqual(loaded.material, 2)

    def test_primitive_modes(self):
        _, document = _triangle_mesh_document()
        primitive = document["meshes"][0]["primitives"][0]
        # TRIANGLE_STRIP and TRIANGLE_FAN are triangles, so there is no warning.
        for mode in (4, 5, 6):
            primitive["mode"] = mode
            model = self._load(document)
            self.assertEqual(model.meshes[0].primitives[0].mode, mode)
            self.assertEqual(model.warnings, [])

        # Others are kept with a warning, and the converter skips them.
        primitive["mode"] = 1
        model = self._load(document)
        self.assertEqual(model.meshes[0].primitives[0].mode, 1)
        self.assertIn("Contains non-triangle primitives (mode=1). The converter skips them.", model.warnings)

    def test_attribute_dtypes(self):
        # Integers become float32 without scaling (a quantized position), unless they are normalized.
        # JOINTS_n is joint indices, so it stays integer.
        builder = _Builder()
        positions = builder.add(np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]], dtype=np.int16), "VEC3", SHORT)
        uvs = builder.add(np.array([[0, 0], [65535, 0], [0, 65535]], dtype=np.uint16), "VEC2", UNSIGNED_SHORT, normalized=True)
        joints = builder.add(np.array([[0, 1, 2, 3]] * 3, dtype=np.uint8), "VEC4", UNSIGNED_BYTE)
        document = builder.document(meshes=[{"primitives": [{"attributes": {"POSITION": positions, "TEXCOORD_0": uvs, "JOINTS_0": joints}}]}])

        attributes = self._load(document).meshes[0].primitives[0].attributes

        self.assertEqual(attributes["POSITION"].dtype, np.float32)
        np.testing.assert_array_equal(attributes["POSITION"], [[1, 2, 3], [4, 5, 6], [7, 8, 9]])
        self.assertEqual(attributes["TEXCOORD_0"].dtype, np.float32)
        np.testing.assert_allclose(attributes["TEXCOORD_0"], [[0, 0], [1, 0], [0, 1]])
        self.assertEqual(attributes["JOINTS_0"].dtype, np.uint8)

    def test_accessor_shared_by_primitives_is_read_once(self):
        builder = _Builder()
        positions, indices = builder.add_triangle()
        primitive = {"attributes": {"POSITION": positions}, "indices": indices}
        model = self._load(builder.document(meshes=[{"primitives": [primitive, dict(primitive)]}]))

        first, second = model.meshes[0].primitives
        self.assertIs(first.attributes["POSITION"], second.attributes["POSITION"])
        self.assertIs(first.indices, second.indices)

    def test_compressed_primitive_is_skipped(self):
        # The primitive that cannot be read is skipped with a warning, and the others are kept.
        builder = _Builder()
        positions, indices = builder.add_triangle()
        compressed = {"attributes": {"POSITION": positions}, "extensions": {"KHR_draco_mesh_compression": {}}}
        plain = {"attributes": {"POSITION": positions}, "indices": indices}
        document = builder.document(
            meshes=[{"primitives": [compressed, plain]}],
            extensionsUsed=["KHR_draco_mesh_compression"],
        )

        model = self._load(document)

        self.assertEqual(len(model.meshes[0].primitives), 1)
        self.assertIsNotNone(model.meshes[0].primitives[0].indices)
        self.assertEqual(
            model.warnings,
            ["KHR_draco_mesh_compression is not supported. meshes[0].primitives[0] was skipped because it cannot be read."],
        )

    def test_meshopt_primitive_is_skipped(self):
        _, document = _triangle_mesh_document()
        document["bufferViews"][0]["extensions"] = {"EXT_meshopt_compression": {}}

        model = self._load(document)

        self.assertEqual(model.meshes[0].primitives, [])
        self.assertEqual(
            model.warnings,
            ["EXT_meshopt_compression is not supported. meshes[0].primitives[0] was skipped because it cannot be read."],
        )

    def test_compression_extension_without_primitive_warns(self):
        _, document = _triangle_mesh_document(extensionsUsed=["KHR_draco_mesh_compression"])

        model = self._load(document)

        self.assertEqual(len(model.meshes[0].primitives), 1)
        self.assertEqual(model.warnings, ["KHR_draco_mesh_compression is not supported."])

    def test_gaussian_splatting_primitive_is_kept_with_warning(self):
        _, document = _triangle_mesh_document(extensionsUsed=["KHR_gaussian_splatting"])
        document["meshes"][0]["primitives"][0]["extensions"] = {"KHR_gaussian_splatting": {}}

        model = self._load(document)

        self.assertEqual(len(model.meshes[0].primitives), 1)
        self.assertEqual(model.warnings, ["KHR_gaussian_splatting is skipped."])

    # Materials.

    def test_material_defaults(self):
        material = self._load(_document(materials=[{}])).materials[0]

        self.assertIsNone(material.name)
        np.testing.assert_array_equal(material.base_color_factor, [1, 1, 1, 1])
        self.assertEqual(material.metallic_factor, 1.0)
        self.assertEqual(material.roughness_factor, 1.0)
        np.testing.assert_array_equal(material.emissive_factor, [0, 0, 0])
        self.assertEqual(material.alpha_mode, "OPAQUE")
        self.assertEqual(material.alpha_cutoff, 0.5)
        self.assertFalse(material.double_sided)
        for texture in (
            material.base_color_texture,
            material.metallic_roughness_texture,
            material.normal_texture,
            material.occlusion_texture,
            material.emissive_texture,
        ):
            self.assertIsNone(texture)

    def test_material(self):
        material = self._load(
            _document(
                materials=[
                    {
                        "name": "m",
                        "pbrMetallicRoughness": {
                            "baseColorFactor": [0.1, 0.2, 0.3, 0.4],
                            "baseColorTexture": {"index": 0, "texCoord": 1},
                            "metallicFactor": 0.25,
                            "roughnessFactor": 0.75,
                            "metallicRoughnessTexture": {"index": 1},
                        },
                        "normalTexture": {"index": 2, "scale": 2.0},
                        "occlusionTexture": {"index": 3, "strength": 0.5},
                        "emissiveFactor": [1.0, 0.5, 0.0],
                        "emissiveTexture": {"index": 4},
                        "alphaMode": "MASK",
                        "alphaCutoff": 0.25,
                        "doubleSided": True,
                    }
                ]
            )
        ).materials[0]

        self.assertEqual(material.name, "m")
        np.testing.assert_allclose(material.base_color_factor, [0.1, 0.2, 0.3, 0.4])
        self.assertEqual((material.base_color_texture.index, material.base_color_texture.tex_coord), (0, 1))
        self.assertEqual(material.metallic_factor, 0.25)
        self.assertEqual(material.roughness_factor, 0.75)
        self.assertEqual((material.metallic_roughness_texture.index, material.metallic_roughness_texture.tex_coord), (1, 0))
        self.assertEqual((material.normal_texture.index, material.normal_texture.scale), (2, 2.0))
        self.assertEqual((material.occlusion_texture.index, material.occlusion_texture.strength), (3, 0.5))
        np.testing.assert_array_equal(material.emissive_factor, [1.0, 0.5, 0.0])
        self.assertEqual(material.emissive_texture.index, 4)
        self.assertEqual(material.alpha_mode, "MASK")
        self.assertEqual(material.alpha_cutoff, 0.25)
        self.assertTrue(material.double_sided)

    def test_material_texture_ref_without_scale_and_strength(self):
        material = self._load(_document(materials=[{"normalTexture": {"index": 0}, "occlusionTexture": {"index": 1}}])).materials[0]

        self.assertIsNone(material.normal_texture.scale)
        self.assertIsNone(material.occlusion_texture.strength)

    def test_material_texture_transform(self):
        # The transform is kept as it is. It is not baked into UV arrays by the loader.
        transform = {"offset": [0.5, 0.25], "rotation": 0.5, "scale": [2, 3], "texCoord": 2}
        partial = {"offset": [1, 0]}
        document = _document(
            materials=[
                {
                    "pbrMetallicRoughness": {
                        "baseColorTexture": {"index": 0, "texCoord": 1, "extensions": {"KHR_texture_transform": transform}},
                        "metallicRoughnessTexture": {"index": 0, "extensions": {"KHR_texture_transform": partial}},
                    },
                    "normalTexture": {"index": 0},
                }
            ]
        )

        material = self._load(document).materials[0]

        base_color = material.base_color_texture
        # The texCoord of the extension replaces the texCoord of the texture.
        self.assertEqual(base_color.tex_coord, 2)
        np.testing.assert_array_equal(base_color.transform.offset, [0.5, 0.25])
        self.assertEqual(base_color.transform.rotation, 0.5)
        np.testing.assert_array_equal(base_color.transform.scale, [2, 3])
        self.assertEqual(base_color.transform.tex_coord, 2)

        metallic_roughness = material.metallic_roughness_texture
        self.assertEqual(metallic_roughness.tex_coord, 0)
        np.testing.assert_array_equal(metallic_roughness.transform.offset, [1, 0])
        self.assertEqual(metallic_roughness.transform.rotation, 0.0)
        np.testing.assert_array_equal(metallic_roughness.transform.scale, [1, 1])
        self.assertIsNone(metallic_roughness.transform.tex_coord)

        self.assertIsNone(material.normal_texture.transform)

    def test_material_extensions_warn_once(self):
        extensions = {"KHR_materials_emissive_strength": {"emissiveStrength": 2.0}}
        model = self._load(_document(materials=[{"extensions": extensions}, {"extensions": extensions}]))

        self.assertEqual(model.warnings, ["KHR_materials_emissive_strength is ignored with a warning. Only core PBR is used."])

    # Textures, samplers, and images.

    def test_textures(self):
        textures = self._load(_document(textures=[{"name": "t", "source": 1, "sampler": 0}, {}])).textures

        self.assertEqual((textures[0].name, textures[0].source, textures[0].sampler), ("t", 1, 0))
        self.assertEqual((textures[1].name, textures[1].source, textures[1].sampler), (None, None, None))

    def test_samplers(self):
        samplers = self._load(_document(samplers=[{}, {"magFilter": 9729, "minFilter": 9987, "wrapS": 33071, "wrapT": 33648}])).samplers

        # The wrap modes default to REPEAT (10497).
        self.assertEqual((samplers[0].mag_filter, samplers[0].min_filter, samplers[0].wrap_s, samplers[0].wrap_t), (None, None, 10497, 10497))
        self.assertEqual((samplers[1].mag_filter, samplers[1].min_filter, samplers[1].wrap_s, samplers[1].wrap_t), (9729, 9987, 33071, 33648))

    def test_image_data_uri(self):
        model = self._load(
            _document(
                images=[
                    {"name": "base64", "uri": _data_uri(_PNG, "image/png")},
                    {"uri": _data_uri(_PNG, "image/png"), "mimeType": "image/jpeg"},
                    {"uri": "data:image/png,%89PNG"},
                ]
            )
        )

        base64_image, explicit_mime, percent_encoded = model.images
        self.assertEqual(base64_image.name, "base64")
        self.assertEqual(base64_image.data, _PNG)
        self.assertIsNone(base64_image.uri)
        # The MIME type of the data URI is used when mimeType is missing.
        self.assertEqual(base64_image.mime_type, "image/png")
        self.assertEqual(explicit_mime.mime_type, "image/jpeg")
        self.assertEqual(percent_encoded.data, b"\x89PNG")
        self.assertEqual(model.warnings, [])

    def test_image_file_is_not_read(self):
        # An external image is not read. The converter copies the file from its uri.
        self._write_bytes(_PNG, "images/tex.png")
        self._write_bytes(_PNG, "images/tex-1.JPG")
        self._write_bytes(_PNG, "images/tex.webp")
        model = self._load(
            _document(
                images=[
                    {"uri": "images/tex.png"},
                    {"uri": "images/tex%2D1.JPG"},
                    {"uri": "images/tex.png", "mimeType": "image/jpeg"},
                    {"uri": "images/tex.webp"},
                ]
            )
        )

        png, jpeg, explicit, webp = model.images
        self.assertIsNone(png.data)
        self.assertEqual(png.uri, "images/tex.png")
        # The MIME type comes from the suffix when mimeType is missing.
        self.assertEqual(png.mime_type, "image/png")
        self.assertEqual(jpeg.uri, "images/tex%2D1.JPG")
        self.assertEqual(jpeg.mime_type, "image/jpeg")
        self.assertEqual(explicit.mime_type, "image/jpeg")
        self.assertIsNone(webp.mime_type)
        self.assertEqual(model.warnings, [])

    def test_image_file_warnings(self):
        model = self._load(
            _document(
                images=[
                    {"uri": "missing.png"},
                    {"uri": "http://example.com/a.png"},
                    {"uri": "/absolute/a.png"},
                ]
            )
        )

        # The images are still returned, so the indices of the textures stay valid.
        self.assertEqual([image.uri for image in model.images], ["missing.png", "http://example.com/a.png", "/absolute/a.png"])
        self.assertIn("Referenced file does not exist: missing.png", model.warnings)
        self.assertIn("External URLs are not supported: http://example.com/a.png", model.warnings)
        self.assertIn("Absolute paths are not supported: /absolute/a.png", model.warnings)

    def test_image_buffer_view(self):
        # The bytes of an embedded image are owned bytes, not a view of the GLB.
        document = _document(
            buffers=[{"byteLength": len(_PNG)}],
            bufferViews=[{"buffer": 0, "byteLength": len(_PNG)}],
            images=[{"bufferView": 0, "mimeType": "image/png"}],
        )
        path = self._write_bytes(_glb(document, _PNG), "model.glb")

        image = gltf.load(path).images[0]

        self.assertIsInstance(image.data, bytes)
        self.assertEqual(image.data, _PNG)
        self.assertIsNone(image.uri)
        self.assertEqual(image.mime_type, "image/png")

    def test_load_error_image(self):
        with self.assertRaisesRegex(GltfError, "An image cannot have both uri and bufferView"):
            self._load(_document(images=[{"uri": "a.png", "bufferView": 0}]))
        with self.assertRaisesRegex(GltfError, "An image must have either uri or bufferView"):
            self._load(_document(images=[{"name": "empty"}]))


if __name__ == "__main__":
    unittest.main()
