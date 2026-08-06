#!/usr/bin/env bash
# Generate the C++ API documentation, laid out like the published site:
#   build-docs/api/versions.js   <- shared version list (drives the dropdown)
#   build-docs/api/index.html    <- redirect to the current version
#   build-docs/api/latest/       <- this build
# Serve build-docs/ over HTTP to preview it (the last line prints the command).
#
#   ./docs-report.sh                 API docs only
#   ./docs-report.sh --site          also build the landing page and serve the
#                                    whole site, the way CI assembles it
#
# Run from anywhere; defaults: build dir = build-docs, output = build-docs/api/latest
# Override: DOCS_BUILD_DIR=/path/to/build DOCS_HTML_DIR=/path/to/out DOCS_PORT=4000
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${DOCS_BUILD_DIR:-$ROOT/build-docs}"
HTML_DIR="${DOCS_HTML_DIR:-$BUILD_DIR/api/latest}"
API_DIR="$(dirname "$HTML_DIR")"
SITE_DIR="${DOCS_SITE_DIR:-$BUILD_DIR/site}"
PORT="${DOCS_PORT:-4000}"

BUILD_SITE=false
for arg in "$@"; do
	case "$arg" in
		--site)      BUILD_SITE=true ;;
		-h|--help)   sed -n '2,13p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
		*)           echo "Error: unknown argument: $arg (try --help)" >&2; exit 1 ;;
	esac
done

need_cmd() {
	command -v "$1" >/dev/null 2>&1 || {
		echo "Error: required command not found: $1" >&2
		exit 1
	}
}

need_cmd cmake
need_cmd doxygen
need_cmd gcc
need_cmd python3

configure_docs_build() {
	cmake -B "$BUILD_DIR" \
		-DCMAKE_BUILD_TYPE=Release \
		-DBUILD_TESTING=OFF \
		-DBUILD_DOCUMENTATION=ON \
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
elif ! grep -q '^BUILD_TESTING:BOOL=OFF' "$BUILD_DIR/CMakeCache.txt" 2>/dev/null ||
     ! grep -q '^BUILD_DOCUMENTATION:BOOL=ON' "$BUILD_DIR/CMakeCache.txt" 2>/dev/null; then
	echo "Reconfiguring $BUILD_DIR for documentation..."
	configure_docs_build
elif [[ "$ROOT/Doxyfile.in" -nt "$BUILD_DIR/Doxyfile" ]]; then
	# CMake generates Doxyfile from Doxyfile.in; nothing else here runs the
	# build system, so refresh it when the template is newer.
	echo "Doxyfile.in changed, reconfiguring $BUILD_DIR..."
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
# Doxygen never removes files from a previous run, so a page that is no longer
# generated (a deleted class, a list turned off) would linger in the preview
# while CI, building from scratch, would not have it. Only wipe a directory
# that is a previous Doxygen output, in case DOCS_HTML_DIR points elsewhere.
if [[ -f "$HTML_DIR/index.html" && -f "$HTML_DIR/doxygen.css" ]]; then
	rm -rf "$HTML_DIR"
fi
mkdir -p "$HTML_DIR"

# Parameter reference page: regenerated on every run so it cannot lag behind
# Parameters.h (CMake only generates it at configure time).
params_page="$BUILD_DIR/doxygen/parameters.md"
python3 "$ROOT/doxygen/generate_parameters_page.py" \
	--input "$ROOT/corelib/include/rtabmap/core/Parameters.h" \
	--output "$params_page"

echo "Running Doxygen -> $HTML_DIR ..."
{
	cat "$BUILD_DIR/Doxyfile"
	printf 'INPUT = %s/doxygen/mainpage.md %s/doxygen/tools.md %s corelib/include utilite/include %s/corelib/src/include\n' "$ROOT" "$ROOT" "$params_page" "$BUILD_DIR"
	printf 'USE_MDFILE_AS_MAINPAGE = %s/doxygen/mainpage.md\n' "$ROOT"
	printf 'OUTPUT_DIRECTORY = %s\n' "$HTML_DIR"
	printf 'HTML_OUTPUT = .\n'
} | (cd "$ROOT" && doxygen -)

# Doxygen's navigation tree descends into the contents of each topic and each
# namespace: the members of a topic (for our @defgroup topics, the same
# overloaded name four or five times) and, under the namespace, the 147 classes
# already listed under "Classes". Cutting the link to those children files keeps
# the tree two levels deep; the topic, namespace and class pages are untouched.
python3 - "$HTML_DIR" <<'PRUNE'
import os, re, sys

html_dir = sys.argv[1]
# file listing the nodes -> pattern of the reference to a node's children file
for name, pattern in (("topics.js", r'group__[A-Za-z0-9_]+'),
                      ("namespaces_dup.js", r'namespace[A-Za-z0-9_]+')):
    path = os.path.join(html_dir, name)
    if not os.path.exists(path):
        continue
    with open(path) as f:
        data = f.read()
    pruned, count = re.subn(r', "%s" \]' % pattern, ', null ]', data)
    if count:
        with open(path, 'w') as f:
            f.write(pruned)
    print("%s: %d node(s) collapsed to a single level" % (name, count))
PRUNE

# The version list lives at the API root, one level above this build, so every
# published version shares it (see doxygen/versions.js).
cp "$ROOT/doxygen/versions.js" "$API_DIR/versions.js"

# /api/ has no content of its own: send it to this build so a bare .../api/
# link lands somewhere useful instead of a 404 (relative target, so it works
# at the site root and under a preview prefix alike).
printf '%s\n' \
	'<!doctype html>' \
	'<meta charset="utf-8">' \
	'<title>RTAB-Map API documentation</title>' \
	"<meta http-equiv=\"refresh\" content=\"0; url=$(basename "$HTML_DIR")/\">" \
	"<link rel=\"canonical\" href=\"$(basename "$HTML_DIR")/\">" \
	"<p>Redirecting to the <a href=\"$(basename "$HTML_DIR")/\">latest API documentation</a>.</p>" \
	> "$API_DIR/index.html"

if [[ ! -f "$HTML_DIR/index.html" ]]; then
	echo "Error: expected $HTML_DIR/index.html after Doxygen run" >&2
	exit 1
fi

serve_dir="$(dirname "$API_DIR")"
echo ""
echo "Done: $HTML_DIR/index.html"

if ! $BUILD_SITE; then
	echo ""
	echo "Preview (the version dropdown needs HTTP, not file://):"
	echo "  python3 -m http.server 8899 --directory $serve_dir"
	echo "  http://127.0.0.1:8899/api/$(basename "$HTML_DIR")/"
	exit 0
fi

# --- Full site: landing page + API docs, assembled the way CI does -----------

if ! command -v jekyll >/dev/null 2>&1; then
	cat >&2 <<EOF
Error: jekyll not found. Install the same gem set GitHub Pages uses:
  sudo apt install ruby-dev build-essential   # native gems need the headers
  gem install --user-install github-pages
  export PATH="\$PATH:\$(ruby -e 'print Gem.user_dir')/bin"
EOF
	exit 1
fi

# github-pages enables these implicitly; calling jekyll directly does not, and
# without them index.md is copied verbatim instead of rendered with the theme.
# baseurl is emptied for the local preview: the site is served from the root
# here, while .github/workflows/docs.yml pins the real path per deployment.
jekyll_config="$BUILD_DIR/_config_local.yml"
{
	echo 'baseurl: ""'
	echo "plugins:"
	echo "  - jekyll-mentions"
	echo "  - jekyll-optional-front-matter"
	echo "  - jekyll-default-layout"
} > "$jekyll_config"

echo ""
echo "Building the landing page -> $SITE_DIR ..."
rm -rf "$SITE_DIR"
jekyll build -s "$ROOT/website" -d "$SITE_DIR" \
	--config "$ROOT/website/_config.yml,$jekyll_config"
cp -r "$API_DIR" "$SITE_DIR/api"

echo ""
echo "Serving the assembled site (Ctrl-C to stop):"
echo "  http://127.0.0.1:$PORT/"
echo "  http://127.0.0.1:$PORT/api/"
echo ""
# --skip-initial-build: a rebuild would wipe the destination, taking the api/
# tree copied above with it.
# --open-url: launch the browser on the served address (harmless when there is
# no browser to launch, e.g. over SSH -- jekyll just logs it).
exec jekyll serve -s "$ROOT/website" -d "$SITE_DIR" \
	--config "$ROOT/website/_config.yml,$jekyll_config" \
	--skip-initial-build --open-url --port "$PORT"
