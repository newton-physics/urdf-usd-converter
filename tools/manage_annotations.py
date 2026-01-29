# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0

"""
Outputs a list of URDF files contained in the specified directory to a file.

target repository: https://github.com/Daniella1/urdf_files_dataset

Navigate to this repository's directory in the command line and execute it.
If the ROS package reference path cannot be found, we must manually assign the target path.

Activate the virtual environment:
    # Windows
    .venv/Scripts/Activate

    # Linux
    source .venv/bin/activate

Usage:
    # Acquire URDF information for new or additional datasets and create annotation files.
    # Specifying "--urdf-files-dataset-path" retrieves URDF file information from the local repository.
    # Omitting this will "git clone" the urdf_files_dataset repository from GitHub into your working directory to retrieve the URDF information.
    # Also infer ROS package paths.
    # Output destination is "tools/urdf_files_dataset_annotations.yaml".
    python tools/manage_annotations.py --urdf-files-dataset-path /home/foo/urdf_files_dataset --update

    # If the ROS package path cannot be detected, "ros_package_paths" will contain null in the YAML file.
    # In this case, you must manually assign the target path.

    # This does not output a YAML file.
    # Lists information on whether URDF files have been manually checked from the "tools/urdf_files_dataset_annotations.yaml" file.
    python tools/manage_annotations.py --validate
"""

import argparse
import logging
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[logging.StreamHandler(), logging.FileHandler("benchmarks/benchmarks.log", mode="a")],
)
logger = logging.getLogger(__name__)


class AnnotationManager:
    URDF_FILES_DATASET_REPO_URL = "https://github.com/Daniella1/urdf_files_dataset.git"
    DEFAULT_ANNOTATION_FILE = "urdf_files_dataset_annotations.yaml"

    def __init__(self, urdf_files_dataset_path: Path, annotation_file: Path):
        self.urdf_files_dataset_path = urdf_files_dataset_path
        self.annotation_file = annotation_file
        self.annotations: dict[str, dict] = {}
        self.temp_urdf_files_dataset = False

    def get_resolved_ros_package_path(self, urdf_dir: Path, package_path: str) -> Path | None:
        """
        Get the resolved ROS package path from the URDF file directory.

        Args:
            urdf_dir: The URDF file directory.
            package_path: The path of the package.

        Returns:
            The resolved ROS package path.
            None if the package path is not found.
        """

        resolved_package_path = None
        current_dir = urdf_dir
        while True:
            file_path = current_dir / package_path
            if file_path.exists():
                # Relative path from the path containing the urdf file.
                resolved_package_path = Path(os.path.relpath(current_dir, urdf_dir)).as_posix()
                break
            if current_dir.parent == current_dir:
                # Reached the root directory
                break
            current_dir = current_dir.parent
        return resolved_package_path

    def get_ros_package_path(self, urdf_file_path: Path) -> dict[str, Path]:
        """
        Get the ROS package path from the URDF file path.

        Args:
            urdf_file_path: The path to the URDF file.

        Returns:
            A dictionary with the package name and path.
            None if the package path is not found.
        """
        urdf_file_dir = urdf_file_path.parent

        # Retrieve the line containing elements that reference texture or mesh within the urdf_file_path file.
        # If the line contains 'filename="package://...', get the package name and path.
        ros_package_paths: dict[str, Path] = {}
        with Path.open(urdf_file_path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line.startswith(("<texture", "<mesh")):
                    filename_match = re.search(r'filename="package://([^"]+)', line)
                    if filename_match:
                        package_name, package_path = filename_match.group(1).split("/", 1)
                        if package_name not in ros_package_paths or ros_package_paths[package_name] is None:
                            ros_package_paths[package_name] = self.get_resolved_ros_package_path(urdf_file_dir, package_path)

        return ros_package_paths

    def setup_urdf_files_dataset_from_repository(self) -> Path:
        """Setup urdf_files_dataset repository (clone if needed)."""
        if self.urdf_files_dataset_path and self.urdf_files_dataset_path.exists():
            logger.info("Using existing urdf_files_dataset at: %s", self.urdf_files_dataset_path)
            return self.urdf_files_dataset_path

        # Clone to temporary directory
        temp_dir = tempfile.mkdtemp(prefix="urdf_files_dataset_benchmark_")
        urdf_files_dataset_path = Path(temp_dir) / "urdf_files_dataset"

        logger.info("Cloning urdf_files_dataset to: %s", urdf_files_dataset_path)
        try:
            subprocess.run(
                ["git", "clone", "--depth", "1", self.URDF_FILES_DATASET_REPO_URL, str(urdf_files_dataset_path)],
                check=True,
                capture_output=True,
                text=True,
            )
        except subprocess.CalledProcessError as e:
            logger.error("Failed to clone urdf_files_dataset: %s", e)
            raise

        self.urdf_files_dataset_path = urdf_files_dataset_path
        self.temp_urdf_files_dataset = True
        return urdf_files_dataset_path

    def load_annotations(self):
        """Load existing annotations from file."""
        if not self.annotation_file.exists():
            logger.info("Annotation file does not exist: %s", self.annotation_file)
            return {}

        try:
            with Path.open(self.annotation_file, encoding="utf-8") as f:
                self.annotations = yaml.safe_load(f) or {}
            logger.info("Loaded %d existing annotations", len(self.annotations))
            return self.annotations
        except Exception as e:
            logger.error("Failed to load annotations: %s", e)
            return {}

    def discover_urdf_files_dataset(self) -> set[str]:
        """
        Retrieve the URDF file path from the repository.
        """
        # Setup urdf_files_dataset repository.
        # If self.urdf_files_dataset_path already specifies a local path, this will be skipped.
        self.setup_urdf_files_dataset_from_repository()

        if not self.urdf_files_dataset_path.exists():
            logger.error("Local file path does not exist: %s", self.urdf_files_dataset_path)
            return {}

        search_dir_path = self.urdf_files_dataset_path / "urdf_files"
        if not search_dir_path.exists():
            logger.error("Search directory path does not exist: %s", search_dir_path)
            return {}

        # Store the URDF file paths in a set.
        # Skip the "drake" directory and search for URDF files in all other directories.
        urdf_file_paths = set()
        for dir in search_dir_path.iterdir():
            group_name = dir.name

            # "drake" is skipped because the ROS package structure is different.
            if group_name == "drake":
                continue

            for subdir in dir.iterdir():
                for file in subdir.glob("**/*.urdf"):
                    urdf_file_paths.add(Path(os.path.relpath(file, self.urdf_files_dataset_path)).as_posix())

        return urdf_file_paths

    def generate_template_for_urdf_file(self, urdf_file_path: str) -> dict:
        """Generate a template annotation for a new URDF file."""

        # Retrieve the group_name and subgroup_name from the urdf_file_path.
        files = urdf_file_path.split("/")
        group_name = files[1]
        subgroup_name = files[2]

        # Get the ROS package paths from the URDF file.
        ros_package_paths = self.get_ros_package_path(self.urdf_files_dataset_path / urdf_file_path)

        return {
            "group_name": group_name,
            "subgroup_name": subgroup_name,
            "ros_package_paths": ros_package_paths,
            "description": "",
            "evaluation_date": "",
            "evaluator": "",
            "notes": "",
            "verified": "No",
        }

    def update_annotation_file(self, new_urdf_files: set[str], dry_run: bool = False):
        """Update the annotation file with new assets."""
        if not new_urdf_files:
            logger.info("No new assets to add")
            return

        logger.info("Adding templates for %d new URDF files: %s", len(new_urdf_files), ", ".join(sorted(new_urdf_files)))

        # Add templates for new URDF files
        for urdf_file_path in new_urdf_files:
            self.annotations[urdf_file_path] = self.generate_template_for_urdf_file(urdf_file_path)

        if dry_run:
            logger.info("Dry run: would update annotation file")
            return

        # Store the URDF file data in a YAML file.
        with Path.open(self.annotation_file, "w", encoding="utf-8") as f:
            yaml.dump(self.annotations, f, default_flow_style=False, sort_keys=True, allow_unicode=True, width=100)

    def validate_annotations(self) -> list[str]:
        """Validate existing annotations and return any issues."""
        issues = []
        valid_success_values = {"Yes", "No", "Unknown"}

        for urdf_file_path, annotation in self.annotations.items():
            if not isinstance(annotation, dict):
                issues.append(f"{urdf_file_path}: annotation must be a dictionary")
                continue

            # Check verified value
            success = annotation.get("verified", "No")
            if success not in valid_success_values:
                issues.append(f"{urdf_file_path}: verified must be one of {valid_success_values}, got '{success}'")

            # Check required fields exist
            required_fields = ["verified", "notes"]
            issues.extend(f"{urdf_file_path}: missing required field '{field}'" for field in required_fields if field not in annotation)

        return issues

    def cleanup(self):
        """Clean up temporary resources."""
        if self.temp_urdf_files_dataset and self.urdf_files_dataset_path:
            try:
                shutil.rmtree(self.urdf_files_dataset_path.parent)
                logger.info("Cleaned up temporary urdf_files_dataset directory")
            except Exception as e:
                logger.warning("Failed to clean up temporary directory: %s", e)

    def print_summary(self):
        """Print a summary of annotations."""
        if not self.annotations:
            logger.info("No annotations loaded")
            return

        success_counts = {"Yes": 0, "No": 0, "Unknown": 0}
        evaluated_count = 0

        for urdf_file_path, annotation in self.annotations.items():
            success = annotation.get("verified", "No")
            success_counts[success] = success_counts.get(success, 0) + 1

            if annotation.get("evaluation_date") or annotation.get("evaluator"):
                evaluated_count += 1

        print("\nAnnotation Summary:")
        print(f"  Total assets: {len(self.annotations)}")
        print(f"  Evaluated: {evaluated_count}")
        print("  Success breakdown:")
        for status, count in success_counts.items():
            print(f"    {status}: {count}")

        if evaluated_count > 0:
            print("\nRecently evaluated assets:")
            for urdf_file_path, annotation in self.annotations.items():
                if annotation.get("evaluation_date"):
                    date = annotation["evaluation_date"]
                    evaluator = annotation.get("evaluator", "Unknown")
                    success = annotation.get("verified", "No")
                    print(f"  {urdf_file_path}: {success} ({date}, {evaluator})")


def main():
    """
    Outputs a list of URDF files contained in the specified directory to a file.
    """
    parser = argparse.ArgumentParser(
        description="Outputs a list of URDF files contained in the specified directory to a file.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument("--urdf-files-dataset-path", type=Path, default=None, help="Path to the urdf_files_dataset directory")

    parser.add_argument(
        "--annotation-file", type=Path, default=Path(f"tools/{AnnotationManager.DEFAULT_ANNOTATION_FILE}"), help="Path to the annotation YAML file"
    )

    parser.add_argument("--update", action="store_true", help="Update the annotation file with new assets found in urdf_files_dataset")

    parser.add_argument("--dry-run", action="store_true", help="Show what would be done without making changes")

    parser.add_argument("--validate", action="store_true", help="Validate existing annotations")

    args = parser.parse_args()

    # Create annotation manager
    manager = AnnotationManager(args.urdf_files_dataset_path, args.annotation_file)

    # Load existing annotations
    manager.load_annotations()

    # Update the annotations file.
    if args.update:
        # Retrieve elements not present in the existing annotations file.
        new_urdf_files = manager.discover_urdf_files_dataset()
        existing_urdf_files = set(manager.annotations.keys())
        new_urdf_files = new_urdf_files - existing_urdf_files

        if new_urdf_files:
            manager.update_annotation_file(new_urdf_files, dry_run=args.dry_run)
        else:
            logger.info("No new URDF files found")

    # Verify the ROS package paths.
    if args.validate:
        issues = manager.validate_annotations()
        if issues:
            logger.error("Validation issues found:")
            for issue in issues:
                logger.error("  %s", issue)
            return 1
        else:
            logger.info("All annotations are valid")

    # Print summary
    manager.print_summary()

    # Clean up temporary resources.
    manager.cleanup()


if __name__ == "__main__":
    sys.exit(main())
