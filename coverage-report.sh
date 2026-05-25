#!/usr/bin/env bash
# Generate an HTML coverage report at coverage-html/index.html
# Run from anywhere; defaults: build dir = build-coverage, output = coverage-html/
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${COVERAGE_BUILD_DIR:-$ROOT/build-coverage}"
HTML_DIR="${COVERAGE_HTML_DIR:-$ROOT/coverage-html}"
INFO_FILE="${COVERAGE_INFO_FILE:-$ROOT/lcov.info}"
JOBS="${JOBS:-$(nproc)}"

need_cmd() {
	command -v "$1" >/dev/null 2>&1 || {
		echo "Error: required command not found: $1" >&2
		exit 1
	}
}

need_cmd cmake
need_cmd lcov
need_cmd genhtml
need_cmd gcc

# lcov defaults to gcov-11 on Ubuntu; use gcov matching the active compiler.
detect_gcov_tool() {
	local ver
	ver="$(gcc -dumpversion 2>/dev/null | cut -d. -f1)"
	if [[ -n "$ver" ]] && command -v "gcov-${ver}" >/dev/null 2>&1; then
		echo "gcov-${ver}"
	else
		echo "gcov"
	fi
}
GCOV_TOOL="$(detect_gcov_tool)"

# lcov 1.14 (Ubuntu 22.04) only knows --ignore-errors gcov,source,graph.
# lcov 1.15+ also accepts mismatch,unused,format,deprecated. The category
# vocabulary is shared by lcov and geninfo so we parse the help output of
# geninfo (the tool that actually fails first) and use the intersection
# with what we want.
detect_lcov_ignore_categories() {
	local wanted=(gcov source graph mismatch unused)
	local help_line
	# geninfo prints the supported categories inline: "(gcov, source, graph)"
	# on 1.14; newer versions list more. Pull whatever's between the parens.
	help_line="$(geninfo --help 2>&1 | grep -m1 -- '--ignore-errors ERROR')"
	local available=()
	if [[ "$help_line" =~ \(([^\)]+)\) ]]; then
		IFS=', ' read -r -a available <<<"${BASH_REMATCH[1]}"
	fi
	local supported=()
	for w in "${wanted[@]}"; do
		for a in "${available[@]}"; do
			if [[ "$w" == "$a" ]]; then
				supported+=("$w")
				break
			fi
		done
	done
	# Fallback to the lcov-1.14 minimum if parsing failed.
	if [[ ${#supported[@]} -eq 0 ]]; then
		supported=(gcov source graph)
	fi
	(IFS=,; echo "${supported[*]}")
}
LCOV_IGNORE_CATEGORIES="$(detect_lcov_ignore_categories)"
LCOV_IGNORE=(--ignore-errors "$LCOV_IGNORE_CATEGORIES")

configure_coverage_build() {
	local -a gtest_args=()
	if [[ -d "$ROOT/build/_deps/googletest-src" ]]; then
		gtest_args+=(
			"-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=$ROOT/build/_deps/googletest-src"
			-DFETCHCONTENT_UPDATES_DISCONNECTED=ON
		)
	fi
	cmake -B "$BUILD_DIR" \
		-DCMAKE_BUILD_TYPE=Debug \
		-DENABLE_COVERAGE=ON \
		-DBUILD_TESTING=ON \
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
		-DWITH_REALSENSE=OFF \
		"${gtest_args[@]}"
}

if [[ ! -f "$BUILD_DIR/CMakeCache.txt" ]]; then
	echo "Configuring $BUILD_DIR (Debug + ENABLE_COVERAGE)..."
	configure_coverage_build
elif ! grep -q '^ENABLE_COVERAGE:BOOL=ON' "$BUILD_DIR/CMakeCache.txt" 2>/dev/null; then
	echo "Reconfiguring $BUILD_DIR with ENABLE_COVERAGE=ON..."
	configure_coverage_build
fi

echo "Building $BUILD_DIR (gcov tool: $GCOV_TOOL)..."
cmake --build "$BUILD_DIR" -j"$JOBS"

echo "Clearing old coverage runtime data..."
find "$BUILD_DIR" -name '*.gcda' -delete

echo "Running tests..."
# Prepend coverage libs: otherwise LD_LIBRARY_PATH (e.g. ROS) can load
# librtabmap from another build tree and .gcda is never written for corelib/src.
(
	export LD_LIBRARY_PATH="$BUILD_DIR/bin${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
	cd "$BUILD_DIR" && ctest --output-on-failure
)

echo "Capturing coverage..."
lcov --gcov-tool "$GCOV_TOOL" "${LCOV_IGNORE[@]}" \
	--capture \
	--directory "$BUILD_DIR/corelib/src" \
	--directory "$BUILD_DIR/utilite/src" \
	--output-file "$INFO_FILE"

echo "Filtering coverage data (repo sources only)..."
lcov "${LCOV_IGNORE[@]}" --extract "$INFO_FILE" \
	"${ROOT}/*" \
	--output-file "$INFO_FILE"
lcov "${LCOV_IGNORE[@]}" --remove "$INFO_FILE" \
	'*/sqlite3/*' \
	'*/rtflann/*' \
	--output-file "$INFO_FILE"

echo "Generating HTML..."
rm -rf "$HTML_DIR"
genhtml --ignore-errors source,mismatch "$INFO_FILE" --output-directory "$HTML_DIR" --legend --demangle-cpp

echo ""
echo "Done: $HTML_DIR/index.html"
lcov --summary "$INFO_FILE" 2>/dev/null || true
