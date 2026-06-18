#!/usr/bin/env bash
# Generate Doxygen HTML at doc/html/index.html
# Run from anywhere; defaults: build dir = build-docs, output = doc/html/
# Override build tree: DOCS_BUILD_DIR=/path/to/build ./docs-report.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${DOCS_BUILD_DIR:-$ROOT/build-docs}"
HTML_DIR="${DOCS_HTML_DIR:-$ROOT/doc/html}"
DOXY_INPUT_OVERRIDE="${DOCS_DOXY_INPUT:-$ROOT/.doxygen-input}"

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

default_build_dir="$ROOT/build-docs"
if [[ "$BUILD_DIR" == "$default_build_dir" ]]; then
	echo "Running Doxygen..."
	doxygen "$ROOT/Doxyfile"
else
	printf 'INPUT = corelib/include utilite/include %s/corelib/src/include\n' "$BUILD_DIR" \
		>"$DOXY_INPUT_OVERRIDE"
	echo "Running Doxygen (INPUT override: $DOXY_INPUT_OVERRIDE)..."
	doxygen "$ROOT/Doxyfile" "$DOXY_INPUT_OVERRIDE"
fi

if [[ ! -f "$HTML_DIR/index.html" ]]; then
	echo "Error: expected $HTML_DIR/index.html after Doxygen run" >&2
	exit 1
fi

echo ""
echo "Done: $HTML_DIR/index.html"
