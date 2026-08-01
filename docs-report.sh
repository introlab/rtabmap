#!/usr/bin/env bash
# Generate the C++ API documentation, laid out like the published site:
#   build-docs/api/versions.js   <- shared version list (drives the dropdown)
#   build-docs/api/latest/       <- this build
# Serve build-docs/ over HTTP to preview it (the last line prints the command).
# Run from anywhere; defaults: build dir = build-docs, output = build-docs/api/latest
# Override: DOCS_BUILD_DIR=/path/to/build DOCS_HTML_DIR=/path/to/out ./docs-report.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${DOCS_BUILD_DIR:-$ROOT/build-docs}"
HTML_DIR="${DOCS_HTML_DIR:-$BUILD_DIR/api/latest}"
API_DIR="$(dirname "$HTML_DIR")"

need_cmd() {
	command -v "$1" >/dev/null 2>&1 || {
		echo "Error: required command not found: $1" >&2
		exit 1
	}
}

need_cmd cmake
need_cmd doxygen
need_cmd gcc

configure_docs_build() {
	cmake -B "$BUILD_DIR" \
		-DCMAKE_BUILD_TYPE=Release \
		-DBUILD_TESTING=OFF \
		-DBUILD_APP=OFF \
		-DBUILD_TOOLS=OFF \
		-DBUILD_EXAMPLES=OFF \
		-DWITH_QT=OFF \
		-DWITH_PYTHON=OFF \
		-DWITH_CERES=OFF \
		-DWITH_G2O=OFF \
		-DWITH_GTSAM=OFF \
		-DWITH_MRPT=OFF \
		-DWITH_VERTIGO=OFF \
		-DWITH_CVSBA=OFF \
		-DWITH_POINTMATCHER=OFF \
		-DWITH_CCCORELIB=OFF \
		-DWITH_OPEN3D=OFF \
		-DWITH_LOAM=OFF \
		-DWITH_FLOAM=OFF \
		-DWITH_LIOSAM=OFF \
		-DWITH_FLYCAPTURE2=OFF \
		-DWITH_ZED=OFF \
		-DWITH_ZEDOC=OFF \
		-DWITH_REALSENSE=OFF
}

if [[ ! -f "$BUILD_DIR/CMakeCache.txt" ]]; then
	echo "Configuring $BUILD_DIR (export headers for Doxygen)..."
	configure_docs_build
elif ! grep -q '^BUILD_TESTING:BOOL=OFF' "$BUILD_DIR/CMakeCache.txt" 2>/dev/null; then
	echo "Reconfiguring $BUILD_DIR for documentation..."
	configure_docs_build
fi

export_header="$BUILD_DIR/corelib/src/include/rtabmap/core/rtabmap_core_export.h"
if [[ ! -f "$export_header" ]]; then
	echo "Error: missing $export_header (CMake configure did not generate export header)" >&2
	exit 1
fi

# Overrides are appended to the generated Doxyfile and fed on stdin: a later
# assignment wins, and `doxygen -` is the only supported way to combine files
# (passing a second config file on the command line is silently ignored).
# INPUT is relative, so Doxygen must run from the source root.
mkdir -p "$HTML_DIR"
echo "Running Doxygen -> $HTML_DIR ..."
{
	cat "$BUILD_DIR/Doxyfile"
	printf 'INPUT = corelib/include utilite/include %s/corelib/src/include\n' "$BUILD_DIR"
	printf 'OUTPUT_DIRECTORY = %s\n' "$HTML_DIR"
	printf 'HTML_OUTPUT = .\n'
} | (cd "$ROOT" && doxygen -)

# The version list lives at the API root, one level above this build, so every
# published version shares it (see doxygen/versions.js).
cp "$ROOT/doxygen/versions.js" "$API_DIR/versions.js"

if [[ ! -f "$HTML_DIR/index.html" ]]; then
	echo "Error: expected $HTML_DIR/index.html after Doxygen run" >&2
	exit 1
fi

serve_dir="$(dirname "$API_DIR")"
echo ""
echo "Done: $HTML_DIR/index.html"
echo ""
echo "Preview (the version dropdown needs HTTP, not file://):"
echo "  python3 -m http.server 8899 --directory $serve_dir"
echo "  http://127.0.0.1:8899/api/$(basename "$HTML_DIR")/"
