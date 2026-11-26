# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
"""glTF asset metadata. This is the `asset` object."""

from dataclasses import dataclass

from .errors import GltfError

__all__ = ["AssetInfo", "parse_asset"]


@dataclass
class AssetInfo:
    """Asset version and generator strings. Names stay as Python strings."""

    version: str
    min_version: str | None = None
    generator: str | None = None
    copyright: str | None = None


def parse_asset(document: dict) -> AssetInfo:
    raw = document.get("asset")
    if not isinstance(raw, dict) or "version" not in raw:
        raise GltfError("asset.version is missing.")
    version = str(raw["version"])
    if _major(version) != 2:
        raise GltfError(f"Only glTF 2.x is supported: {version}")
    min_version = raw.get("minVersion")
    if min_version is not None and _major(str(min_version)) > 2:
        raise GltfError(f"Unsupported minVersion: {min_version}")
    return AssetInfo(
        version=version,
        min_version=None if min_version is None else str(min_version),
        generator=None if raw.get("generator") is None else str(raw["generator"]),
        copyright=None if raw.get("copyright") is None else str(raw["copyright"]),
    )


def _major(version: str) -> int:
    """Get the major version from a version string."""
    head = version.split(".", 1)[0]
    # isdigit() accepts characters such as "²", which int() then rejects. Only ASCII digits are allowed.
    if not (head.isascii() and head.isdigit()):
        raise GltfError(f"Malformed version string: {version}")
    return int(head)
