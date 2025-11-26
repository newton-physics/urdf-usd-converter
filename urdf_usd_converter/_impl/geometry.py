# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
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

__all__ = ["convert_geometry"]


def convert_geometry(parent: Usd.Prim, name: str, geometry: ElementGeometry, data: ConversionData) -> UsdGeom.Gprim:
    if isinstance(geometry.shape, ElementBox):
        return convert_box(parent, name, geometry.shape, data)
    elif isinstance(geometry.shape, ElementSphere):
        return convert_sphere(parent, name, geometry.shape, data)
    elif isinstance(geometry.shape, ElementCylinder):
        return convert_cylinder(parent, name, geometry.shape, data)
    elif isinstance(geometry.shape, ElementMesh):
        return convert_mesh(parent, name, geometry.shape, data)


def convert_box(parent: Usd.Prim, name: str, box: ElementBox, data: ConversionData) -> UsdGeom.Gprim:
    # Define a cube with a size of 1.0 meter.
    cube_prim = usdex.core.defineCube(parent, name, size=1.0)
    size = Gf.Vec3f(box.get_with_default("size"))
    scale_op = cube_prim.AddScaleOp()
    scale_op.Set(size)
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
    mesh_safe_name = data.mesh_cache.get_safe_name(filename)

    ref_mesh: Usd.Prim = data.references[Tokens.Geometry].get(mesh_safe_name)
    if not ref_mesh:  # pragma: no cover
        # The process never gets here.
        Tf.RaiseRuntimeError(f"Mesh '{mesh_safe_name}' not found in Geometry Library {data.libraries[Tokens.Geometry].GetRootLayer().identifier}")

    prim = usdex.core.defineReference(parent, ref_mesh, name)

    if scale != Gf.Vec3d(1):
        scale_op = UsdGeom.Xformable(prim).AddScaleOp()
        scale_op.Set(Gf.Vec3f(scale))

    return UsdGeom.Mesh(prim)
