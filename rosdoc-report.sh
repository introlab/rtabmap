#!/usr/bin/env bash
# Generate ROS Sphinx documentation at docs_output/rtabmap/index.html
# Run from anywhere; requires a sourced ROS 2 install (see /opt/ros/<distro>).
# Override: ROS_DISTRO=humble ROSDOC_OUTPUT_DIR=/path ./rosdoc-report.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
PACKAGE_NAME="${ROSDOC_PACKAGE_NAME:-rtabmap}"
OUTPUT_DIR="${ROSDOC_OUTPUT_DIR:-$ROOT/docs_output/$PACKAGE_NAME}"
DOC_BUILD_DIR="${ROSDOC_BUILD_DIR:-$ROOT/docs_build}"

need_cmd() {
	command -v "$1" >/dev/null 2>&1 || {
		echo "Error: required command not found: $1" >&2
		exit 1
	}
}

ros_setup="/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ ! -f "$ros_setup" ]]; then
	echo "Error: ROS setup not found: $ros_setup" >&2
	echo "Install ROS 2 ($ROS_DISTRO) or set ROS_DISTRO to an installed distro." >&2
	exit 1
fi

need_cmd rosdoc2
need_cmd doxygen

# Do not source a colcon workspace (rosdoc2 known issue #66).
# shellcheck source=/dev/null
source "$ros_setup"

echo "Building ROS documentation with rosdoc2 (ROS $ROS_DISTRO)..."
rosdoc2 build \
	--package-path "$ROOT" \
	--output-directory "$ROOT/docs_output" \
	--doc-build-directory "$DOC_BUILD_DIR"

if [[ ! -f "$OUTPUT_DIR/index.html" ]]; then
	echo "Error: expected $OUTPUT_DIR/index.html after rosdoc2 build" >&2
	exit 1
fi

echo ""
echo "Done: $OUTPUT_DIR/index.html"
