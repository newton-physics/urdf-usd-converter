# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0

import usdex.core
from pxr import Sdf, Usd

from .data import ConversionData, Tokens
from .urdf_parser.elements import ElementBase

__all__ = ["convert_undefined", "convert_undefined_elements"]

# Namespaces for storing custom attributes in USD.
URDF_CUSTOM_ATTRIBUTE_NAMESPACE = "urdf"


def convert_undefined_elements(element: ElementBase, prim: Usd.Prim, data: ConversionData) -> list[tuple[str, int]]:
    """
    Store custom attributes and elements for the specified element.
    When "force_store" is True, all elements and attributes are recursively stored.

    Args:
        element: Elements in URDF
        prim: USD prim
        data: The conversion data.

    Returns:
        The list of checked paths and line numbers.
    """
    if not element.undefined_attributes and not element.undefined_elements and not element.undefined_text:
        return []

    checked_paths = []
    if element.undefined_elements:
        # Create a scope for each undefined element.
        # This recursively creates a hierarchy of elements.
        names = [elem.tag for elem in element.undefined_elements]
        safe_names = data.name_cache.getPrimNames(prim, names)
        for elem, safe_name in zip(element.undefined_elements, safe_names):
            _prim = usdex.core.defineScope(prim, safe_name).GetPrim()
            if safe_name != elem.tag:
                usdex.core.setDisplayName(_prim, elem.tag)

            checked_paths.append((elem.path, elem.line_number))
            for undefined_data in data.undefined_elements:
                if undefined_data.path == elem.path and undefined_data.line_number == elem.line_number:
                    _checked_paths = convert_undefined_elements(elem, _prim, data)
                    checked_paths.extend(_checked_paths)
                    break

    if element.undefined_attributes:
        for key, value in element.undefined_attributes.items():
            prim.CreateAttribute(f"{URDF_CUSTOM_ATTRIBUTE_NAMESPACE}:{key}", Sdf.ValueTypeNames.String, custom=True).Set(value)

    if element.undefined_text:
        _text = element.undefined_text.strip()
        if _text:
            prim.CreateAttribute(f"{URDF_CUSTOM_ATTRIBUTE_NAMESPACE}:text", Sdf.ValueTypeNames.String, custom=True).Set(_text)

    return checked_paths


def _convert_undefined_materials(data: ConversionData):
    """
    Convert undefined elements for materials.
    """
    for material_data in data.material_data_list:
        # Skip if the material is not a URDF global material.
        if material_data.mesh_file_path:
            continue

        # Find the URDF global material element by name.
        material_element = data.urdf_parser.find_material_by_name(material_data.name)
        if material_element:
            if not material_element.undefined_attributes and not material_element.undefined_elements and not material_element.undefined_text:
                continue

            material_prim = data.references[Tokens.Materials][material_data.safe_name]
            convert_undefined_elements(material_element, material_prim.GetPrim(), data)


def convert_undefined(data: ConversionData):
    """
    Convert undefined elements.
    """
    if not len(data.undefined_elements):
        return

    # Convert undefined elements for materials.
    _convert_undefined_materials(data)

    geo_scope = data.content[Tokens.Geometry].GetDefaultPrim().GetChild(Tokens.Geometry).GetPrim()

    undefined_data_list = []
    for undefined_data in data.undefined_elements:
        # Undefined elements or attributes in link, joint, or material are already stored, so skip them.
        if (
            undefined_data.path.startswith("/robot/link")
            or undefined_data.path.startswith("/robot/joint")
            or undefined_data.path.startswith("/robot/material")
        ):
            continue
        undefined_data_list.append(undefined_data)

    if not len(undefined_data_list):
        return

    # Custom elements are stored in "Geometry/custom".
    prim_name = "custom"
    safe_name = data.name_cache.getPrimName(geo_scope, prim_name)
    custom_prim = usdex.core.defineScope(geo_scope, safe_name).GetPrim()
    if safe_name != prim_name:
        usdex.core.setDisplayName(custom_prim, prim_name)

    names = [undefined_data.tag for undefined_data in undefined_data_list]
    safe_names = data.name_cache.getPrimNames(custom_prim, names)

    checked_paths = []
    for undefined_data, safe_name in zip(undefined_data_list, safe_names):
        # If the element has already been checked, skip.
        if any(
            undefined_data.path == checked_path and undefined_data.line_number == checked_line_number
            for checked_path, checked_line_number in checked_paths
        ):
            continue

        # Create a prim to store the custom element "undefined_data.tag".
        _prim = usdex.core.defineScope(custom_prim, safe_name).GetPrim()
        if safe_name != undefined_data.tag:
            usdex.core.setDisplayName(_prim, undefined_data.tag)

        # Store custom attributes and elements for the specified element.
        _checked_paths = convert_undefined_elements(undefined_data.element, _prim, data)
        checked_paths.extend(_checked_paths)
