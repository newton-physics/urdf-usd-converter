# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import numpy as np
from pxr import Gf, Vt

__all__ = ["convert_face_indices_array", "convert_matrix4d", "convert_vec2f_array", "convert_vec3f_array"]


def convert_vec2f_array(source: np.ndarray) -> Vt.Vec2fArray:
    """
    Convert a numpy array of 2D vectors to a USD Vec2fArray.

    Args:
        source: A numpy array of shape (N, M) where M is divisible by 2,
                 representing N elements each with M/2 2D vectors.

    Returns:
        Vt.Vec2fArray: A USD array of 2D vectors.

    Raises:
        AssertionError: If the second dimension of the input array is not divisible by 2.
    """
    num_elements, element_size = source.shape
    assert element_size % 2 == 0

    # Reshape to (total_vectors, 2) and create Vec2f objects in batch
    reshaped = source.reshape(-1, 2).astype(np.float32) if element_size != 2 else source
    result = [Gf.Vec2f(float(v[0]), float(v[1])) for v in reshaped]
    return Vt.Vec2fArray(result)


def convert_vec3f_array(source: np.ndarray) -> Vt.Vec3fArray:
    """
    Convert a numpy array of 3D vectors to a USD Vec3fArray.

    Args:
        source: A numpy array of shape (N, M) where M is divisible by 3,
                 representing N elements each with M/3 3D vectors.

    Returns:
        Vt.Vec3fArray: A USD array of 3D vectors.

    Raises:
        AssertionError: If the second dimension of the input array is not divisible by 3.
    """
    num_elements, element_size = source.shape
    assert element_size % 3 == 0

    # In the case of STL, element_size=9, and it holds the three vertices of a triangle as a one-dimensional array.
    # Therefore, we need to reshape the array to (total_vectors, 3) before creating the Vec3f objects.

    # Reshape to (total_vectors, 3) and create Vec3f objects in batch
    reshaped = source.reshape(-1, 3).astype(np.float32) if element_size != 3 else source
    result = [Gf.Vec3f(float(v[0]), float(v[1]), float(v[2])) for v in reshaped]
    return Vt.Vec3fArray(result)


def convert_face_indices_array(source: np.ndarray) -> tuple[list[int], list[int]]:
    """
    Converts an array of face indices into a list of face counts and vertex indices.

    Args:
        source: A numpy array of shape (N, M) where M is the number of face vertex indices,
                 representing N elements each with M face vertex indices.

    Returns:
        tuple[list[int], list[int]]: A tuple of lists of integers.
        The first list is the face vertex counts, the second list is the face vertex indices.
    """
    # Use numpy operations for better performance
    face_vertex_counts = []
    face_vertex_indices = []
    if isinstance(source, np.ndarray) and source.ndim == 2:
        # All rows have the same length in a 2D array
        face_vertex_counts = [source.shape[1]] * len(source)
        # Flatten using numpy's ravel() which is faster than nested loops
        face_vertex_indices = source.ravel().astype(int).tolist()

    return face_vertex_counts, face_vertex_indices


def convert_matrix4d(source: np.ndarray) -> Gf.Matrix4d:
    """
    Convert a numpy array to a USD Gf.Matrix4d.

    Args:
        source: A numpy array of shape (4, 4) representing a 4x4 transformation matrix.

    Returns:
        Gf.Matrix4d: A USD 4x4 matrix.

    Raises:
        AssertionError: If the input array is not a 4x4 matrix.
    """
    assert source.shape == (4, 4), f"Expected shape (4, 4), got {source.shape}"

    # Convert the numpy array to a list of floats using numpy operations
    # Transpose to get column-major order, then flatten
    matrix_list = source.T.ravel().astype(float).tolist()

    return Gf.Matrix4d(*matrix_list)
