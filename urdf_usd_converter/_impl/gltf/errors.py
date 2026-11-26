# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
"""Errors and warnings raised while reading a glTF asset."""

__all__ = ["GltfError", "warn_once"]


class GltfError(ValueError):
    """Raised when a glTF asset cannot be read."""


def warn_once(warnings: list[str], message: str) -> None:
    """Append a warning unless the same message is already recorded."""
    if message not in warnings:
        warnings.append(message)
