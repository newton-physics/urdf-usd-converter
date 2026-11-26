# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
"""
Read a glTF or GLB file and resolve buffer bytes.

JSON is decoded as UTF-8. External references currently accepted here are
relative paths and data URIs. Absolute URIs are not read. They produce a
warning and the rest of the asset is still returned.
"""

import base64
import json
import pathlib
import struct
from dataclasses import dataclass, field
from urllib.parse import unquote, unquote_to_bytes

from .errors import GltfError, warn_once

__all__ = ["Buffer", "GltfSource", "note_image_reference", "read_bytes", "read_source"]

_GLB_MAGIC = b"glTF"
_JSON_CHUNK = 0x4E4F534A
_BIN_CHUNK = 0x004E4942


# A BIN payload is bytes, or a memoryview into the file. A memoryview is not copied.
Buffer = bytes | memoryview


@dataclass
class GltfSource:
    """Parsed JSON and buffers in index order, before accessor decoding."""

    document: dict
    buffers: list[Buffer]
    base_dir: pathlib.Path
    warnings: list[str] = field(default_factory=list)
    # Buffer index to the reason it was skipped. The buffer is empty in `buffers`.
    skipped_buffers: dict[int, str] = field(default_factory=dict)


def read_source(path: pathlib.Path) -> GltfSource:
    """
    Return the JSON document and buffer list. A glTF magic header is read as GLB even without a .glb suffix.

    Args:
        path: The path to the glTF or GLB file.

    Returns:
        The parsed JSON, the buffers in index order, and the warnings found while reading them.

    Raises:
        GltfError: If the file cannot be parsed.
    """
    data = path.read_bytes()
    if data[:4] == _GLB_MAGIC or path.suffix.lower() == ".glb":
        document, binary = _read_glb(data)
    else:
        document = _loads_json(data)
        binary = None
    warnings: list[str] = []
    skipped_buffers: dict[int, str] = {}
    buffers = _resolve_buffers(document, binary, path.parent, warnings, skipped_buffers)
    return GltfSource(document=document, buffers=buffers, base_dir=path.parent, warnings=warnings, skipped_buffers=skipped_buffers)


def note_image_reference(uri: str, base_dir: pathlib.Path, warnings: list[str]) -> None:
    """
    Warn when a file image cannot be used. The file contents are not read.

    An external URL, an absolute path, and a missing relative file all warn.
    The caller keeps the URI and leaves the image bytes empty.

    Args:
        uri: The image URI.
        base_dir: The directory that relative URIs are resolved against.
        warnings: The list that receives the warning.
    """
    message = _non_portable_message(uri)
    if message is not None:
        warn_once(warnings, message)
        return
    target = (base_dir / unquote(uri)).resolve()
    if not target.is_file():
        warn_once(warnings, f"Referenced file does not exist: {uri}")


def read_bytes(uri: str, base_dir: pathlib.Path, warnings: list[str]) -> tuple[bytes | None, str | None]:
    """
    Return the bytes referenced by a URI and the MIME type written in a data URI.

    An absolute URI returns None after a warning. A missing relative file still
    raises `GltfError` because buffer geometry cannot be recovered without it.

    Args:
        uri: A data URI or a relative path.
        base_dir: The directory that relative paths are resolved against.
        warnings: The list that receives the warning for an external or absolute URI.

    Returns:
        The bytes and the MIME type of a data URI. Both are None when the URI is skipped.

    Raises:
        GltfError: If a data URI is malformed or a relative file does not exist.
    """
    if uri.startswith("data:"):
        return _decode_data_uri(uri)
    message = _non_portable_message(uri)
    if message is not None:
        warn_once(warnings, message)
        return None, None
    target = (base_dir / unquote(uri)).resolve()
    if not target.is_file():
        raise GltfError(f"Referenced file does not exist: {uri}")
    return target.read_bytes(), None


def _loads_json(data: bytes) -> dict:
    """Decode UTF-8 JSON. Non-ASCII names are preserved."""
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise GltfError("glTF JSON is not UTF-8.") from exc
    try:
        document = json.loads(text)
    except json.JSONDecodeError as exc:
        raise GltfError(f"Cannot parse glTF JSON: {exc}") from exc
    if not isinstance(document, dict):
        raise GltfError("glTF JSON must be an object.")
    return document


def _read_glb(data: bytes) -> tuple[dict, Buffer | None]:
    if len(data) < 12:
        raise GltfError("GLB header is too short.")
    magic, version, length = struct.unpack_from("<4sII", data, 0)
    if magic != _GLB_MAGIC:
        raise GltfError("GLB magic is not 'glTF'.")
    if version != 2:
        raise GltfError(f"Unsupported GLB version: {version}")
    if length != len(data):
        raise GltfError("GLB header length does not match the file size.")

    offset = 12
    document: dict | None = None
    binary: Buffer | None = None
    file_bytes = memoryview(data)
    chunk_index = 0
    while offset + 8 <= len(data):
        chunk_length, chunk_type = struct.unpack_from("<II", data, offset)
        start = offset + 8
        end = start + chunk_length
        if end > len(data):
            raise GltfError("GLB chunk extends past the end of the file.")
        chunk = file_bytes[start:end]
        if chunk_index == 0 and chunk_type != _JSON_CHUNK:
            raise GltfError("The first GLB chunk is not JSON.")
        if chunk_type == _JSON_CHUNK:
            # The JSON chunk is small. Copy it to bytes and drop trailing padding.
            document = _loads_json(chunk.tobytes().rstrip(b" "))
        elif chunk_type == _BIN_CHUNK and binary is None:
            binary = chunk
        offset = end
        chunk_index += 1
    if document is None:
        raise GltfError("GLB has no JSON chunk.")
    return document, binary


def _resolve_buffers(
    document: dict,
    glb_binary: Buffer | None,
    base_dir: pathlib.Path,
    warnings: list[str],
    skipped_buffers: dict[int, str],
) -> list[Buffer]:
    buffers: list[Buffer] = []
    for index, entry in enumerate(document.get("buffers", [])):
        byte_length = int(entry["byteLength"])
        uri = entry.get("uri")
        if uri is None:
            if index != 0 or glb_binary is None:
                raise GltfError(f"buffers[{index}] has no data. In GLB, only buffers[0] can use the BIN chunk.")
            payload: Buffer | None = glb_binary
        else:
            payload, _mime = read_bytes(uri, base_dir, warnings)
        if payload is None:
            # The URI was skipped after a warning. Keep the buffer index with no bytes, and remember why.
            skipped_buffers[index] = _non_portable_message(uri) or f"{uri} could not be read"
            buffers.append(b"")
            continue
        if len(payload) < byte_length:
            raise GltfError(f"buffers[{index}] is shorter than byteLength.")
        buffers.append(_clip_buffer(payload, byte_length))
    return buffers


def _clip_buffer(payload: Buffer, byte_length: int) -> Buffer:
    """Return the byteLength prefix. Padding is sliced off without copying."""
    if len(payload) == byte_length:
        return payload
    return memoryview(payload)[:byte_length]


def _non_portable_message(uri: str) -> str | None:
    """Return a warning for an external URL or absolute path. Relative paths return None."""
    scheme = uri.split(":", 1)[0] if ":" in uri else ""
    if "://" in uri or scheme.lower() in {"http", "https", "file"}:
        return f"External URLs are not supported: {uri}"
    path = pathlib.Path(uri)
    if path.is_absolute() or uri.startswith(("/", "\\")):
        return f"Absolute paths are not supported: {uri}"
    return None


def _decode_data_uri(uri: str) -> tuple[bytes, str | None]:
    header, separator, payload = uri.partition(",")
    if separator != "," or not header.startswith("data:"):
        raise GltfError("Malformed data URI.")
    meta = header[5:]
    parts = [part for part in meta.split(";") if part]
    mime = parts[0] if parts and "/" in parts[0] else None
    if "base64" in parts:
        try:
            return base64.b64decode(payload, validate=False), mime
        except ValueError as exc:
            raise GltfError("Cannot decode base64 in data URI.") from exc
    return unquote_to_bytes(payload), mime
