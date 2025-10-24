# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
import numpy as np
from pxr import Gf, Vt

__all__ = ["convert_vec3f_array"]


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
    result = []
    for i in range(num_elements):
        result.extend([Gf.Vec3f(float(source[i][j]), float(source[i][j + 1]), float(source[i][j + 2])) for j in range(0, element_size, 3)])
    return Vt.Vec3fArray(result)
