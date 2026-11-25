# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
"""
Images and textures.

Embedded images keep their PNG or JPEG bytes. A file URI is not read: `data`
stays None and `uri` is the path. An external URL, an absolute path, and a
missing file warn and leave the image empty. Pixels are not decoded. Core glTF
images are PNG or JPEG. `_SUFFIX_MIME` fills in a missing mimeType from those
extensions only.
"""

import pathlib
from dataclasses import dataclass

from .errors import GltfError
from .io import note_image_reference, read_bytes

__all__ = [
    "CLAMP_TO_EDGE",
    "MIRRORED_REPEAT",
    "REPEAT",
    "Image",
    "Sampler",
    "Texture",
    "parse_images",
    "parse_samplers",
    "parse_textures",
]

# sampler.wrapS and sampler.wrapT values. These are the OpenGL type constants. REPEAT is the default.
CLAMP_TO_EDGE = 33071
MIRRORED_REPEAT = 33648
REPEAT = 10497

_SUFFIX_MIME = {
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
}


@dataclass
class Image:
    """
    One image.

    `data` holds the PNG or JPEG bytes of an embedded image (a data URI or a
    bufferView). A file reference leaves `data` as None. `uri` is that file
    path, and is None for embedded images.
    """

    name: str | None
    mime_type: str | None
    data: bytes | None
    uri: str | None = None


@dataclass
class Sampler:
    """A sampler. Omitted filters stay None. Wrap defaults to REPEAT."""

    mag_filter: int | None = None
    min_filter: int | None = None
    wrap_s: int = REPEAT
    wrap_t: int = REPEAT


@dataclass
class Texture:
    """One texture. `source` is an image index."""

    name: str | None
    source: int | None
    sampler: int | None


def parse_images(document: dict, buffers: list[bytes], base_dir: pathlib.Path, warnings: list[str]) -> list[Image]:
    images: list[Image] = []
    for raw in document.get("images", []):
        name = None if raw.get("name") is None else str(raw["name"])
        mime_type = None if raw.get("mimeType") is None else str(raw["mimeType"])
        if "uri" in raw and "bufferView" in raw:
            raise GltfError("An image cannot have both uri and bufferView.")
        uri = None if "uri" not in raw else str(raw["uri"])
        if uri is not None and uri.startswith("data:"):
            data, uri_mime = read_bytes(uri, base_dir, warnings)
            file_uri = None
            if mime_type is None:
                mime_type = uri_mime
        elif uri is not None:
            # A file reference is not read. Callers copy the file from `uri`.
            note_image_reference(uri, base_dir, warnings)
            data = None
            file_uri = uri
            if mime_type is None:
                mime_type = _SUFFIX_MIME.get(pathlib.Path(uri.split("?", 1)[0]).suffix.lower())
        elif "bufferView" in raw:
            data = _view_bytes(document, buffers, int(raw["bufferView"]))
            file_uri = None
        else:
            raise GltfError("An image must have either uri or bufferView.")
        images.append(Image(name=name, mime_type=mime_type, data=data, uri=file_uri))
    return images


def parse_samplers(document: dict) -> list[Sampler]:
    return [
        Sampler(
            mag_filter=None if raw.get("magFilter") is None else int(raw["magFilter"]),
            min_filter=None if raw.get("minFilter") is None else int(raw["minFilter"]),
            wrap_s=int(raw.get("wrapS", REPEAT)),
            wrap_t=int(raw.get("wrapT", REPEAT)),
        )
        for raw in document.get("samplers", [])
    ]


def parse_textures(document: dict) -> list[Texture]:
    return [
        Texture(
            name=None if raw.get("name") is None else str(raw["name"]),
            source=None if raw.get("source") is None else int(raw["source"]),
            sampler=None if raw.get("sampler") is None else int(raw["sampler"]),
        )
        for raw in document.get("textures", [])
    ]


def _view_bytes(document: dict, buffers: list[bytes], view_index: int) -> bytes:
    view = document["bufferViews"][view_index]
    start = int(view.get("byteOffset", 0))
    end = start + int(view["byteLength"])
    piece = buffers[int(view["buffer"])][start:end]
    # Image payloads are owned bytes. A memoryview of the GLB buffer is copied here.
    return piece.tobytes() if isinstance(piece, memoryview) else piece
