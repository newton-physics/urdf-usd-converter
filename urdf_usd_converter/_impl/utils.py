# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
from .._version import __version__

__all__ = ["get_authoring_metadata"]


def get_authoring_metadata() -> str:
    return f"URDF USD Converter v{__version__}"
