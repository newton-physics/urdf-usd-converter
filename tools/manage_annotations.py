# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0

"""
Outputs a list of URDF files contained in the specified directory or GitHub repository to a file.

target repository: https://github.com/Daniella1/urdf_files_dataset

Navigate to this repository's directory in the command line and execute it.
Specify either a directory corresponding to a local repository or a repository url on GitHub.

Activate the virtual environment:
    # Windows
    .venv/Scripts/Activate

    # Linux
    source .venv/bin/activate

Usage:
    # Acquire URDF information for new or additional datasets and create annotation files.
    # Specifying "--urdf-repository-path" retrieves URDF file information from the local repository or GitHub repository url.
    # This cannot be omitted.
    # If the repository path is a GitHub repository url, it will "git clone" the repository
    # from GitHub into your working directory to retrieve the URDF information.
    # Also infer ROS package paths.
    # Output destination is "tools/<repository name>_annotations.yaml".

    python tools/manage_annotations.py --urdf-repository-path https://github.com/Daniella1/urdf_files_dataset --update

    # If the ROS package path cannot be detected, "ros_package_paths" will contain null in the YAML file.
    # In this case, you must manually assign the target path.

    # This does not output a YAML file.
    # Lists information on whether URDF files have been manually checked from the "tools/<repository_name>_annotations.yaml" file.

    python tools/manage_annotations.py --urdf-repository-path https://github.com/Daniella1/urdf_files_dataset --validate
"""

import argparse
import logging
import os
import re
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
    def __init__(self, urdf_repository_path: str, annotation_file: Path):
        if urdf_repository_path.startswith("https://") and not urdf_repository_path.startswith("https://github.com/"):
            logger.error("URDF repository path must start with 'https://github.com/'")
            raise

        # Repository URL on GitHub.
        if urdf_repository_path.startswith("https://github.com/"):
            self.urdf_repository_url = urdf_repository_path
        else:
            self.urdf_repository_url = None

        # Local directory path.
        self.local_urdf_directory = Path(urdf_repository_path) if not self.urdf_repository_url else None

        self.repository_name = self.urdf_repository_url.split("/")[-1] if self.urdf_repository_url else self.local_urdf_directory.name
        self.annotation_file = annotation_file
        self.annotations: dict[str, dict] = {}
        self.temp_dir_context = None  # TemporaryDirectory context manager

        if not self.annotation_file:
            self.annotation_file = Path(f"tools/{self.repository_name}_annotations.yaml")

    def _get_resolved_ros_package_path(self, urdf_dir: Path, package_path: str) -> Path | None:
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

    def _get_ros_package_paths(self, urdf_file_path: Path) -> dict[str, Path]:
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
                            ros_package_paths[package_name] = self._get_resolved_ros_package_path(urdf_file_dir, package_path)

        return ros_package_paths

    def _setup_urdf_files_from_repository(self) -> Path:
        """Setup the URDF repository in the temporary directory. (clone if needed)."""
        if self.local_urdf_directory and self.local_urdf_directory.exists():
            logger.info("Using existing URDF repository at: %s", self.local_urdf_directory)
            return self.local_urdf_directory

        # Clone to temporary directory using TemporaryDirectory context manager
        self.temp_dir_context = tempfile.TemporaryDirectory(prefix=f"urdf_{self.repository_name}_benchmark_")
        urdf_files_path = Path(self.temp_dir_context.name) / self.repository_name

        logger.info("Cloning URDF repository to: %s", urdf_files_path)
        try:
            subprocess.run(
                ["git", "clone", "--depth", "1", f"{self.urdf_repository_url}.git", str(urdf_files_path)],
                check=True,
                capture_output=True,
                text=True,
            )
        except subprocess.CalledProcessError as e:
            logger.error("Failed to clone %s: %s", self.repository_name, e)
            # Clean up temporary directory on error
            self.temp_dir_context.cleanup()
            self.temp_dir_context = None
            raise

        self.local_urdf_directory = urdf_files_path
        return urdf_files_path

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

    def _discover_urdf_files(self) -> tuple[set[str], int, int]:
        """
        Retrieve the URDF file path from the repository.
        Additionally, it determines the indices corresponding to the group name and subgroup name from the file path.

        Returns:
            A tuple containing the set of URDF file paths, the group depth index, and the subgroup depth index.
        """
        # Setup the URDF repository in the temporary directory. (clone if needed).
        # If 'self.local_urdf_directory' already specifies a local path, this will be skipped.
        self._setup_urdf_files_from_repository()

        if not self.local_urdf_directory.exists():
            logger.error("Local file path does not exist: %s", self.local_urdf_directory)
            return {}, -1, -1

        # Store the file path from self.local_urdf_directory.
        _urdf_file_paths: list[str] = [
            Path(os.path.relpath(file, self.local_urdf_directory)).as_posix() for file in self.local_urdf_directory.glob("**/*.urdf")
        ]

        if len(_urdf_file_paths) == 0:
            logger.error("No URDF files found in the repository: %s", self.local_urdf_directory)
            return {}, -1, -1

        # From the stored paths, obtain the common portion starting from the beginning of the string.
        # The directory name in this section will not be used as a group name or subgroup name.
        common_prefix = os.path.commonprefix(_urdf_file_paths)
        common_prefix_sep_count = common_prefix.count("/")
        if common_prefix.endswith("/"):
            common_prefix = common_prefix[:-1]

        # Get the minimum and maximum of the directory hierarchy.
        # From here, we determine the directory number to be used as the group name and subgroup name.
        #
        #   example (self.local_urdf_directory = "/path/to/target_repository"):
        #     - /path/to/target_repository/urdf_files/matlab/foo/urdf/urdf_file1.urdf
        #     - /path/to/target_repository/urdf_files/matlab/bar/xxx/urdf/urdf_file2.urdf
        #     - /path/to/target_repository/urdf_files/robotics/foo/urdf/urdf_file3.urdf
        #
        #     common_prefix = "urdf_files"
        #     common_prefix_sep_count = 1
        #     min_depth = 3, max_depth = 4
        #     group_depth_index = 1
        #     subgroup_depth_index = 2
        #
        #  example (self.local_urdf_directory = "/path/to/target_repository"):
        #     - /path/to/target_repository/urdf_files/matlab/urdf_file1.urdf
        #     - /path/to/target_repository/urdf_files/matlab/urdf_file2.urdf
        #     - /path/to/target_repository/urdf_files/robotics/urdf_file3.urdf
        #
        #     common_prefix = "urdf_files"
        #     common_prefix_sep_count = 1
        #     min_depth = 1, max_depth = 1
        #     group_depth_index = 1
        #     subgroup_depth_index = -1
        min_depth = -1
        max_depth = -1
        for path in _urdf_file_paths:
            depth = path.count("/") - common_prefix_sep_count
            min_depth = depth if min_depth < 0 else min(min_depth, depth)
            max_depth = depth if max_depth < 0 else max(max_depth, depth)

        group_depth_index = common_prefix_sep_count + 0 if min_depth > 0 else -1
        subgroup_depth_index = common_prefix_sep_count + 1 if min_depth > 1 and max_depth > 2 else -1

        # Store the URDF file paths in a set.
        # Skip the "drake" directory and search for URDF files in all other directories.
        urdf_file_paths = set()
        for urdf_path in _urdf_file_paths:
            path_parts = urdf_path.split("/")
            group_name = path_parts[group_depth_index] if group_depth_index >= 0 else ""

            # urdf_files_dataset: "drake" is skipped because the ROS package structure is different.
            if self.repository_name == "urdf_files_dataset" and group_name == "drake":
                continue

            urdf_file_paths.add(urdf_path)

        return urdf_file_paths, group_depth_index, subgroup_depth_index

    def _generate_template_for_urdf_file(self, urdf_file_path: str, group_depth_index: int, subgroup_depth_index: int) -> dict:
        """
        Generate a template annotation for a new URDF file.

        Args:
            urdf_file_path: The path to the URDF file.
            group_depth_index: The index of the group name.
            subgroup_depth_index: The index of the subgroup name.

        Returns:
            A dictionary containing the annotation for the URDF file.
        """
        # Retrieve the group_name and subgroup_name from the urdf_file_path.
        path_parts = urdf_file_path.split("/")
        group_name = path_parts[group_depth_index] if group_depth_index >= 0 else ""
        subgroup_name = path_parts[subgroup_depth_index] if subgroup_depth_index >= 0 else ""

        # Get the ROS package paths from the URDF file.
        ros_package_paths = self._get_ros_package_paths(self.local_urdf_directory / urdf_file_path)

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

    def update_annotation_file(self, new_urdf_files: set[str], group_depth_index: int, subgroup_depth_index: int, dry_run: bool = False):
        """
        Update the annotation file with new assets.

        Args:
            new_urdf_files: The set of new URDF file paths.
            group_depth_index: The index of the group name.
            subgroup_depth_index: The index of the subgroup name.
            dry_run: Whether to run in dry run mode.
        """
        if not new_urdf_files:
            logger.info("No new assets to add")
            return

        logger.info("Adding templates for %d new URDF files: %s", len(new_urdf_files), ", ".join(sorted(new_urdf_files)))

        # Add templates for new URDF files
        for urdf_file_path in new_urdf_files:
            self.annotations[urdf_file_path] = self._generate_template_for_urdf_file(urdf_file_path, group_depth_index, subgroup_depth_index)

        if dry_run:
            logger.info("Dry run: would update annotation file")
            return

        # Store the URDF file data in a YAML file.
        with Path.open(self.annotation_file, "w", encoding="utf-8") as f:
            f.write(self._get_annotation_header())
            yaml.dump(self.annotations, f, default_flow_style=False, sort_keys=True, allow_unicode=True, width=100)

    def _get_annotation_header(self) -> str:
        """Get the annotation header."""
        return f"""# '{self.repository_name}' Manual Annotations
# This file contains manual evaluation results for the '{self.repository_name}' benchmark
#
# Format:
#   urdf_file_path:
#     description: "Description of this URDF variant"
#     evaluation_date: "YYYY/MM/DD" (asset-level evaluation date)
#     evaluator: "Name or identifier of person who evaluated"
#     group_name: "Directory names for classification"
#     notes: "Additional notes about the evaluation"
#     ros_package_paths: Array of ROS package names and paths
#       package_name: package_path
#     subgroup_name: "Directory names for sub classification"
#     verified: "Yes" | "No" | "Unknown"
#
# Asset names correspond to directory names in the '{self.repository_name}' repository:
# {self.urdf_repository_url if self.urdf_repository_url else ""}
"""

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
        if self.temp_dir_context:
            try:
                self.temp_dir_context.cleanup()
                logger.info("Cleaned up temporary URDF directory")
            except Exception as e:
                logger.warning("Failed to clean up temporary URDF directory: %s", str(e))
            finally:
                self.temp_dir_context = None

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
    Outputs a list of URDF files contained in the specified directory or GitHub URL to a file.
    """
    parser = argparse.ArgumentParser(
        description="Outputs a list of URDF files contained in the specified directory or GitHub URL to a file.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument(
        "--urdf-repository-path",
        type=str,
        required=True,
        help="URL or local path of the repository containing the URDF.",
    )

    parser.add_argument("--annotation-file", type=Path, default=None, help="Path to the annotation YAML file")

    parser.add_argument("--update", action="store_true", help="Update the annotation file with new models found in urdf repository")

    parser.add_argument("--dry-run", action="store_true", help="Show what would be done without making changes")

    parser.add_argument("--validate", action="store_true", help="Validate existing annotations")

    args = parser.parse_args()

    # Create annotation manager
    manager = AnnotationManager(args.urdf_repository_path, args.annotation_file)

    # Load existing annotations
    manager.load_annotations()

    # Update the annotations file.
    if args.update:
        # Retrieve elements not present in the existing annotations file.
        new_urdf_files, group_depth_index, subgroup_depth_index = manager._discover_urdf_files()
        existing_urdf_files = set(manager.annotations.keys())
        new_urdf_files = new_urdf_files - existing_urdf_files

        if new_urdf_files:
            manager.update_annotation_file(new_urdf_files, group_depth_index, subgroup_depth_index, dry_run=args.dry_run)
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
