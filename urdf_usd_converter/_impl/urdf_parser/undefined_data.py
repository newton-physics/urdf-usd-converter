# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from .elements import ElementBase

__all__ = ["UndefinedData"]


class UndefinedData:
    def __init__(self, element: ElementBase = None, undefined_element: bool = False):
        # Tag name.
        self.tag = ""

        # Expressed based on the XML tag hierarchy, such as "robot/link/visual"
        self.path = ""

        # If the element is undefined.
        # If False, the tag itself is defined, but undefined attributes are listed in undefined_attributes.
        self.undefined_element: bool = False

        # Undefined attributes.The keys and values ​​are stored in a dict.
        # e.g. {"data1": "1", "data2": "2"}
        self.undefined_attributes: dict[str, str] = {}

        # Undefined text.
        self.undefined_text: str = None

        # Line numbers in XML.
        self.line_number = -1

        if element:
            self.tag = element.tag
            self.path = element.path
            self.undefined_element = undefined_element
            self.undefined_attributes = element.undefined_attributes
            self.undefined_text = element.undefined_text
            self.line_number = element.line_number
