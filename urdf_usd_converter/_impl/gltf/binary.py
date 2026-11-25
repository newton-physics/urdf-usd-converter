# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
"""
Read an accessor from a buffer as a NumPy array.

Tightly packed arrays are `np.frombuffer` views. Strided arrays are copied.
Normalized integers are converted to float32 with the glTF formulas.
"""

import numpy as np

from .errors import GltfError

__all__ = ["BYTE", "FLOAT", "SHORT", "UNSIGNED_BYTE", "UNSIGNED_INT", "UNSIGNED_SHORT", "read_accessor"]

# accessor.componentType values. These are the OpenGL type constants.
BYTE = 5120
UNSIGNED_BYTE = 5121
SHORT = 5122
UNSIGNED_SHORT = 5123
UNSIGNED_INT = 5125
FLOAT = 5126

# Little-endian NumPy dtype for each componentType.
_DTYPE = {
    BYTE: np.int8,
    UNSIGNED_BYTE: np.uint8,
    SHORT: np.int16,
    UNSIGNED_SHORT: np.uint16,
    UNSIGNED_INT: np.uint32,
    FLOAT: np.float32,
}

# Component count for each accessor.type. MAT2 is 2x2, so 4 components.
_COMPONENTS = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT2": 4, "MAT3": 9, "MAT4": 16}


def read_accessor(
    document: dict,
    buffers: list[bytes],
    accessor_index: int,
    *,
    vertex_attribute: bool = False,
) -> np.ndarray:
    """
    Return one accessor as an array.

    SCALAR has shape `(count,)`. Other types have shape `(count, components)`.
    Normalized integers are already float32. Other values keep their stored dtype.
    `bufferView.byteStride` is applied only for vertex attributes. Indices and
    sparse values are tightly packed.

    Args:
        document: The glTF JSON document.
        buffers: The buffer bytes in index order.
        accessor_index: The index of the accessor to read.
        vertex_attribute: Whether the accessor is a vertex attribute. Only then is `byteStride` applied.

    Returns:
        The accessor values as a NumPy array.

    Raises:
        GltfError: If the accessor is malformed or exceeds its bufferView.
    """
    accessor = document["accessors"][accessor_index]
    count = int(accessor["count"])
    type_name = accessor["type"]
    if type_name not in _COMPONENTS:
        raise GltfError(f"Unsupported accessor.type: {type_name}")
    components = _COMPONENTS[type_name]
    component_type = int(accessor["componentType"])
    if component_type not in _DTYPE:
        raise GltfError(f"Unsupported componentType: {component_type}")
    dtype = np.dtype(_DTYPE[component_type]).newbyteorder("<")
    normalized = bool(accessor.get("normalized", False))
    if normalized and component_type == FLOAT:
        raise GltfError("FLOAT accessors cannot be normalized.")

    if count == 0:
        values = np.zeros((0, components), dtype=dtype)
    elif "bufferView" in accessor:
        values = _read_view(
            document,
            buffers,
            int(accessor["bufferView"]),
            int(accessor.get("byteOffset", 0)),
            count,
            components,
            dtype,
            use_stride=vertex_attribute,
        )
    else:
        values = np.zeros((count, components), dtype=dtype)

    sparse = accessor.get("sparse")
    if sparse:
        values = _apply_sparse(document, buffers, sparse, values, components, dtype)

    if normalized:
        values = _normalize(values, component_type)

    if components == 1:
        return values.reshape(count)
    return values


def _read_view(
    document: dict,
    buffers: list[bytes],
    view_index: int,
    accessor_byte_offset: int,
    count: int,
    components: int,
    dtype: np.dtype,
    *,
    use_stride: bool,
) -> np.ndarray:
    view = document["bufferViews"][view_index]
    buffer = buffers[int(view["buffer"])]
    view_offset = int(view.get("byteOffset", 0))
    view_length = int(view["byteLength"])
    element_size = int(dtype.itemsize) * components
    stride = int(view["byteStride"]) if use_stride and "byteStride" in view else element_size
    if stride < element_size:
        raise GltfError(f"byteStride ({stride}) is smaller than the element size ({element_size}).")

    start = view_offset + accessor_byte_offset
    nbytes = stride * (count - 1) + element_size
    view_end = view_offset + view_length
    if start < 0 or start + nbytes > view_end or start + nbytes > len(buffer):
        raise GltfError("Accessor exceeds the bufferView range.")

    if stride == element_size and start % dtype.alignment == 0:
        array = np.frombuffer(buffer, dtype=dtype, count=count * components, offset=start)
        return array.reshape(count, components)

    raw = np.frombuffer(buffer, dtype=np.uint8, count=nbytes, offset=start)
    rows = np.lib.stride_tricks.as_strided(raw, shape=(count, element_size), strides=(stride, 1))
    packed = np.ascontiguousarray(rows).view(dtype)
    return packed.reshape(count, components)


def _apply_sparse(
    document: dict,
    buffers: list[bytes],
    sparse: dict,
    values: np.ndarray,
    components: int,
    dtype: np.dtype,
) -> np.ndarray:
    """Apply sparse replacements. The source view is copied because it is read-only."""
    replaced = np.array(values, copy=True)
    sparse_count = int(sparse["count"])
    indices_def = sparse["indices"]
    index_type = int(indices_def["componentType"])
    if index_type not in _DTYPE:
        raise GltfError(f"Unsupported componentType for sparse indices: {index_type}")
    indices = _read_view(
        document,
        buffers,
        int(indices_def["bufferView"]),
        int(indices_def.get("byteOffset", 0)),
        sparse_count,
        1,
        np.dtype(_DTYPE[index_type]).newbyteorder("<"),
        use_stride=False,
    ).reshape(sparse_count)
    sparse_values = _read_view(
        document,
        buffers,
        int(sparse["values"]["bufferView"]),
        int(sparse["values"].get("byteOffset", 0)),
        sparse_count,
        components,
        dtype,
        use_stride=False,
    )
    replaced[indices.astype(np.int64, copy=False)] = sparse_values
    return replaced


def _normalize(values: np.ndarray, component_type: int) -> np.ndarray:
    """Convert a normalized integer to float32. Signed values clamp at -1."""
    source = values.astype(np.float32, copy=False)
    if component_type == BYTE:
        return np.maximum(source / np.float32(127.0), np.float32(-1.0))
    if component_type == UNSIGNED_BYTE:
        return source / np.float32(255.0)
    if component_type == SHORT:
        return np.maximum(source / np.float32(32767.0), np.float32(-1.0))
    if component_type == UNSIGNED_SHORT:
        return source / np.float32(65535.0)
    raise GltfError(f"componentType cannot be normalized: {component_type}")
