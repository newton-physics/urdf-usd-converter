# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
import usdex.core
from pxr import Gf, Tf, Usd, UsdGeom

from .data import ConversionData, Tokens
from .urdf_parser.elements import (
    ElementBox,
    ElementCylinder,
    ElementGeometry,
    ElementMesh,
    ElementSphere,
)
from .utils import float3_to_vec3d

__all__ = ["convert_geometry"]


def convert_geometry(parent: Usd.Prim, name: str, geometry: ElementGeometry, data: ConversionData) -> UsdGeom.Gprim:
    if isinstance(geometry.geometry, ElementBox):
        return convert_box(parent, name, geometry.geometry, data)
    elif isinstance(geometry.geometry, ElementSphere):
        return convert_sphere(parent, name, geometry.geometry, data)
    elif isinstance(geometry.geometry, ElementCylinder):
        return convert_cylinder(parent, name, geometry.geometry, data)
    elif isinstance(geometry.geometry, ElementMesh):
        return convert_mesh(parent, name, geometry.geometry, data)


def convert_box(parent: Usd.Prim, name: str, box: ElementBox, data: ConversionData) -> UsdGeom.Gprim:
    cube_prim = usdex.core.defineCube(parent, name, 1.0)
    size = float3_to_vec3d(box.get_with_default("size"))
    usdex.core.setLocalTransform(cube_prim.GetPrim(), Gf.Vec3d(0), Gf.Quatf.GetIdentity(), Gf.Vec3f(size))
    return cube_prim


def convert_sphere(parent: Usd.Prim, name: str, sphere: ElementSphere, data: ConversionData) -> UsdGeom.Gprim:
    radius = sphere.get_with_default("radius")
    sphere_prim = usdex.core.defineSphere(parent, name, radius)
    return sphere_prim


def convert_cylinder(parent: Usd.Prim, name: str, cylinder: ElementCylinder, data: ConversionData) -> UsdGeom.Gprim:
    radius = cylinder.get_with_default("radius")
    length = cylinder.get_with_default("length")
    cylinder_prim = usdex.core.defineCylinder(parent, name, radius, length, UsdGeom.Tokens.z)
    return cylinder_prim


def convert_mesh(parent: Usd.Prim, name: str, mesh: ElementMesh, data: ConversionData) -> UsdGeom.Gprim:
    filename = mesh.get_with_default("filename")
    scale = mesh.get_with_default("scale")
    mesh_safe_name = data.mesh_data.get_safe_name(filename, scale)

    ref_mesh: Usd.Prim = data.references[Tokens.Geometry].get(mesh_safe_name)
    if not ref_mesh:  # pragma: no cover
        # The process never gets here.
        Tf.RaiseRuntimeError(f"Mesh '{mesh_safe_name}' not found in Geometry Library {data.libraries[Tokens.Geometry].GetRootLayer().identifier}")

    prim = usdex.core.defineReference(parent, ref_mesh, name)
    # the reference mesh may have an invalid source name, and thus a display name
    # however, the prim name may already be valid and override this, in which case
    # we need to block the referenced display name
    if prim.GetPrim().GetName() != ref_mesh.GetPrim().GetName():
        usdex.core.blockDisplayName(prim.GetPrim())

    scale_vec3d = float3_to_vec3d(scale)
    usdex.core.setLocalTransform(prim.GetPrim(), Gf.Vec3d(0), Gf.Quatf.GetIdentity(), Gf.Vec3f(scale_vec3d))

    return UsdGeom.Mesh(prim)
