# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import xml.etree.ElementTree as ET
import xml.sax
from io import StringIO
from pathlib import Path
from typing import Any
from xml.sax.handler import ContentHandler
from xml.sax.xmlreader import InputSource


class LineNumberContentHandler(ContentHandler):
    """Handler for tracking line numbers using SAX parser"""

    def __init__(self):
        super().__init__()
        self.element_stack = []
        self.line_offset = 0  # Number of lines skipped (e.g., XML declarations)

    def startElement(self, name, attrs):  # noqa: N802
        # Record current line number
        line_number = self._locator.getLineNumber() if hasattr(self, "_locator") else None
        element_info = {"tag": name, "line": line_number, "attrs": dict(attrs), "children": []}

        if self.element_stack:
            # Add as child of parent element
            self.element_stack[-1]["children"].append(element_info)
        else:
            # Root element
            self.root_element = element_info

        self.element_stack.append(element_info)

    def endElement(self, name):  # noqa: N802
        if self.element_stack:
            self.element_stack.pop()

    def setDocumentLocator(self, locator):  # noqa: N802
        self._locator = locator


class LineNumberTrackingParser:
    """XML parser that tracks line numbers"""

    def __init__(self):
        self.line_info = {}

    def parse_with_line_numbers(self, file_path: Path) -> tuple:
        """
        Parse XML file and return ElementTree with line number information

        Returns:
            tuple: (ElementTree.Element, line_info_dict)
        """
        # Preprocess XML file content (handle XML declarations properly)
        processed_content, line_offset = self._preprocess_xml_content(file_path)

        # Get line numbers using SAX parser
        handler = LineNumberContentHandler()
        handler.line_offset = line_offset  # Set line number offset
        parser = xml.sax.make_parser()
        parser.setContentHandler(handler)

        # Parse preprocessed content
        input_source = InputSource()
        input_source.setCharacterStream(StringIO(processed_content))
        input_source.setSystemId(str(file_path))
        parser.parse(input_source)

        # Parse same preprocessed content with ElementTree
        root = ET.fromstring(processed_content)

        # Map line number information (considering offset)
        line_info = self._create_line_mapping(handler.root_element, root, line_offset)

        return root, line_info

    def _preprocess_xml_content(self, file_path: Path) -> tuple:
        """
        Preprocess XML file content and handle XML declarations properly

        Returns:
            tuple: (processed_content, line_offset)
        """
        lines = []
        line_offset = 0

        with Path.open(file_path, encoding="utf-8") as f:
            for line_num, line in enumerate(f, 1):
                line_strip = line.strip()
                if line_strip.startswith("<?xml") and line_strip.endswith("?>"):
                    line_offset = line_num
                    continue
                lines.append(line)

        content = "".join(lines)
        return content, line_offset

    def _create_line_mapping(self, sax_element: dict[str, Any], et_element: ET.Element, line_offset: int = 0) -> dict[ET.Element, int]:
        """Create line number dictionary by mapping SAX results to ElementTree elements"""
        mapping = {}

        def map_elements(sax_elem, et_elem):
            # Map current element (considering line number offset)
            if sax_elem["line"] is not None:
                mapping[et_elem] = sax_elem["line"] + line_offset

            # Recursively map child elements
            et_children = list(et_elem)
            sax_children = sax_elem.get("children", [])

            # Match by tag name (considering order)
            sax_child_index = 0
            for et_child in et_children:
                if sax_child_index < len(sax_children):
                    map_elements(sax_children[sax_child_index], et_child)
                    sax_child_index += 1

        map_elements(sax_element, et_element)
        return mapping
