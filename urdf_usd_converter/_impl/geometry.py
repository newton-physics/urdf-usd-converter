# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
import usdex.core
from pxr import Gf, Tf, Usd, UsdGeom, UsdPhysics

from .data import ConversionData, Tokens
from .urdf_parser.elements import (
    ElementBox,
    ElementCollision,
    ElementCylinder,
    ElementMesh,
    ElementSphere,
    ElementVisual,
)
from .utils import set_transform

__all__ = ["convert_geometry"]


def convert_geometry(parent: Usd.Prim, name: str, safe_name: str, geometry: ElementVisual | ElementCollision, data: ConversionData) -> Usd.Prim:
    prim = None
    if isinstance(geometry.geometry.shape, ElementBox):
        prim = convert_box(parent, safe_name, geometry.geometry.shape, data)
    elif isinstance(geometry.geometry.shape, ElementSphere):
        prim = convert_sphere(parent, safe_name, geometry.geometry.shape, data)
    elif isinstance(geometry.geometry.shape, ElementCylinder):
        prim = convert_cylinder(parent, safe_name, geometry.geometry.shape, data)
    elif isinstance(geometry.geometry.shape, ElementMesh):
        prim = convert_mesh(parent, safe_name, geometry.geometry.shape, data)

    if not prim:  # pragma: no cover
        # The process never gets here.
        Tf.RaiseRuntimeError(f"Invalid geometry: {geometry.geometry.shape.tag} for geometry '{name}'")

    # When meshes are stored during preprocessing, there are cases where the displayName is already set for a mesh.
    # In this case, we need to check if the displayName is different from the name and safe_name.
    display_name = usdex.core.computeEffectiveDisplayName(prim.GetPrim())
    if name != display_name:
        usdex.core.setDisplayName(prim.GetPrim(), name)
    set_transform(prim, geometry)
    if isinstance(geometry, ElementCollision):
        UsdGeom.Imageable(prim).GetPurposeAttr().Set(UsdGeom.Tokens.guide)

        # Apply CollisionAPI to collision geometry
        apply_physics_collision(prim.GetPrim(), data)

    return prim


def apply_physics_collision(prim: Usd.Prim, data: ConversionData):
    """
    Apply the physics collision to a collision geometry.
    This applies CollisionAPI and MeshCollisionAPI (for mesh types).
    """
    # Get all Gprim children of a prim.
    gprims = [_prim for _prim in Usd.PrimRange(prim) if _prim.IsA(UsdGeom.Gprim)]

    for gprim in gprims:
        geom_over = data.content[Tokens.Physics].OverridePrim(gprim.GetPath())

        # Apply CollisionAPI
        UsdPhysics.CollisionAPI.Apply(geom_over)

        # If it's a mesh, apply MeshCollisionAPI with appropriate approximation
        if gprim.IsA(UsdGeom.Mesh):
            mesh_collider: UsdPhysics.MeshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(geom_over)
            mesh_collider.GetApproximationAttr().Set(UsdPhysics.Tokens.convexHull)


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


def convert_mesh(parent: Usd.Prim, name: str, mesh: ElementMesh, data: ConversionData) -> Usd.Prim:
    filename = mesh.get_with_default("filename")
    scale = mesh.get_with_default("scale")
    mesh_safe_name = data.mesh_cache.get_safe_name(filename)

    ref_mesh: Usd.Prim = data.references[Tokens.Geometry].get(mesh_safe_name)
    if not ref_mesh:  # pragma: no cover
        # The process never gets here.
        Tf.RaiseRuntimeError(f"Mesh '{mesh_safe_name}' not found in Geometry Library {data.libraries[Tokens.Geometry].GetRootLayer().identifier}")

    prim = usdex.core.defineReference(parent, ref_mesh, name)

    if scale != Gf.Vec3d(1):
        prim_scale = usdex.core.getLocalTransform(prim).GetScale()
        scale_op = UsdGeom.Xformable(prim).GetScaleOp()
        if not scale_op:
            scale_op = UsdGeom.Xformable(prim).AddScaleOp()
        scale_op.Set(Gf.Vec3f(Gf.CompMult(prim_scale, scale)))

    return prim
