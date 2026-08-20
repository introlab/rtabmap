#!/usr/bin/env bash
# Fetch test data assets listed in data/tests/manifest.txt. Each entry's
# source can be either a bare Google Drive file ID (assembled into a
# direct-download URL) or a full http(s):// URL (used as-is). An entry with a
# fourth column is an archive (.7z or .zip), which is extracted into data/tests/
# and then removed, its SHA-256 being the one of the file it holds. Skips files
# that are already present and whose SHA-256 matches the manifest. Intended for
# CI and local first-time setup.
#
# All linked files are public. The GDrive URL carries confirm=t, which is what
# answers the interstitial page Drive puts in front of a file it did not scan
# for viruses (it never scans an archive, whatever its size), so the bytes come
# down without needing a tool like gdown.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MANIFEST="${REPO_ROOT}/data/tests/manifest.txt"
DEST_DIR="${REPO_ROOT}/data/tests"

if [[ ! -f "$MANIFEST" ]]; then
	echo "Error: manifest not found at $MANIFEST" >&2
	exit 1
fi

# macOS ships `shasum`, not `sha256sum`. Linux and Git-Bash-on-Windows have
# sha256sum. Pick whichever is available.
if command -v sha256sum >/dev/null 2>&1; then
	sha256_of() { sha256sum "$1" | awk '{print $1}'; }
elif command -v shasum >/dev/null 2>&1; then
	sha256_of() { shasum -a 256 "$1" | awk '{print $1}'; }
else
	echo "Error: neither sha256sum nor shasum is on PATH" >&2
	exit 1
fi

# Extracts an archive into DEST_DIR. Which tool is there varies: every CI runner
# image ships 7-Zip, but it is 7z on some and 7zz on the ones carrying Debian's
# 7zip package, the slim ROS images have neither until the workflow installs one,
# and bsdtar reads 7z through libarchive where no 7-Zip is to be found (macOS tar
# is bsdtar). Same for zip: unzip where there is one, python3's zipfile where
# there is not (Git-Bash-on-Windows ships without unzip).
extract_archive() {
	local archive="$1"
	case "$archive" in
	*.7z)
		local sevenzip=""
		for candidate in 7z 7zz 7za 7zr; do
			if command -v "$candidate" >/dev/null 2>&1; then
				sevenzip="$candidate"
				break
			fi
		done
		if [[ -n "$sevenzip" ]]; then
			"$sevenzip" x -y -o"$DEST_DIR" "$archive" >/dev/null
		elif command -v bsdtar >/dev/null 2>&1; then
			bsdtar -x -f "$archive" -C "$DEST_DIR"
		elif tar --version 2>/dev/null | grep -qi bsdtar; then
			tar -x -f "$archive" -C "$DEST_DIR"
		else
			echo "Error: no 7z, 7zz, 7za, 7zr or bsdtar on PATH to extract $archive" >&2
			return 1
		fi
		;;
	*.zip)
		if command -v unzip >/dev/null 2>&1; then
			unzip -o -q "$archive" -d "$DEST_DIR"
		elif command -v python3 >/dev/null 2>&1; then
			python3 -m zipfile -e "$archive" "$DEST_DIR"
		else
			echo "Error: neither unzip nor python3 is on PATH to extract $archive" >&2
			return 1
		fi
		;;
	*)
		echo "Error: $archive is not an archive this script extracts (.7z, .zip)" >&2
		return 1
		;;
	esac
}

verify_sha() {
	local file="$1" expected="$2"
	if [[ "$expected" == "TODO_FILL_SHA256" || -z "$expected" ]]; then
		echo "  (no SHA in manifest yet for $file; skipping integrity check)" >&2
		return 0
	fi
	local actual
	actual="$(sha256_of "$file")"
	if [[ "$actual" != "$expected" ]]; then
		echo "  SHA mismatch for $file: expected $expected, got $actual" >&2
		return 1
	fi
}

while IFS=$'\t' read -r name source expected_sha extracted; do
	# Strip trailing CR so the script works when manifest.txt is checked out
	# with CRLF line endings (default on Windows Git unless core.autocrlf=input).
	# Without this, expected_sha keeps a trailing \r and even a byte-for-byte
	# match looks like "expected <sha>\r, got <sha>".
	name="${name%$'\r'}"
	source="${source%$'\r'}"
	expected_sha="${expected_sha%$'\r'}"
	extracted="${extracted-}"
	extracted="${extracted%$'\r'}"
	# Skip comments and blank lines.
	[[ -z "${name// }" || "$name" =~ ^# ]] && continue

	target="$DEST_DIR/$name"
	# An archive is not kept once it is extracted, so what has to be there, and
	# what the sha is of, is the file it held.
	kept="${extracted:-$name}"
	if [[ -f "$DEST_DIR/$kept" ]] && verify_sha "$DEST_DIR/$kept" "$expected_sha" 2>/dev/null; then
		echo "Already up-to-date: $kept"
		continue
	fi

	# If the source already looks like a URL, use it as-is. Otherwise treat
	# it as a Google Drive file ID and assemble the direct-download URL.
	if [[ "$source" =~ ^https?:// ]]; then
		url="$source"
	else
		url="https://drive.usercontent.google.com/download?id=${source}&export=download&confirm=t"
	fi
	echo "Fetching $name <- $url"
	mkdir -p "$(dirname "$target")"
	# -L follows the redirect, -f fails on HTTP errors, -S shows errors on stderr.
	curl -fsSL "$url" -o "$target.partial"

	if [[ -z "$extracted" ]]; then
		if ! verify_sha "$target.partial" "$expected_sha"; then
			rm -f "$target.partial"
			exit 1
		fi
		mv -f "$target.partial" "$target"
	else
		mv -f "$target.partial" "$target"
		echo "  Extracting $name"
		if ! extract_archive "$target"; then
			rm -f "$target"
			exit 1
		fi
		# The archive has served its purpose, and these are large files.
		rm -f "$target"
		if [[ ! -f "$DEST_DIR/$extracted" ]]; then
			echo "  $name does not hold $extracted" >&2
			exit 1
		fi
		if ! verify_sha "$DEST_DIR/$extracted" "$expected_sha"; then
			rm -f "$DEST_DIR/$extracted"
			exit 1
		fi
		echo "  Extracted $extracted ($(du -h "$DEST_DIR/$extracted" | cut -f1))"
	fi
done < "$MANIFEST"

echo "Test data ready under $DEST_DIR"

# --- Optional: trace SuperPoint *.pth -> *.pt for the tests ---
# The C++ side loads TorchScript (*.pt); upstream ships only *.pth, so we
# trace them locally if python3 + torch are on PATH. A failure is logged and
# silently skipped -- the test guards every SuperPoint variant with
# UFile::exists(*.pt) and skips when the trace didn't run.
trace_superpoint_pt() {
	local label="$1" script="$2" weights="$3" output="$4" model_dir="$5"
	if [[ -f "$output" ]]; then
		echo "  $label: already traced ($output)"
		return 0
	fi
	if [[ ! -f "$weights" || ! -f "$model_dir/$(basename "$script" | sed 's/^rtabmap_trace_superpoint\.py$/demo_superpoint.py/; s/^superpoint_to_torchscript\.py$/superpoint_pytorch.py/')" ]]; then
		echo "  $label: source files missing — skipping trace"
		return 0
	fi
	if ! command -v python3 >/dev/null 2>&1; then
		echo "  $label: python3 not on PATH — skipping trace"
		return 0
	fi
	if ! python3 -c "import torch" >/dev/null 2>&1; then
		echo "  $label: python3 doesn't have torch — skipping trace"
		return 0
	fi
	echo "Tracing $label: $weights -> $output"
	# Run in a subshell with PWD in DEST_DIR so demo_superpoint.py /
	# superpoint_pytorch.py (also under DEST_DIR) are found by the bare
	# `from X import ...` inside the trace scripts.
	if ( cd "$model_dir" && python3 "$script" --weights "$weights" --output "$output" ) >/tmp/sp_trace.log 2>&1; then
		echo "  $label: traced ($(du -h "$output" | cut -f1))"
	else
		echo "  $label: trace failed (see /tmp/sp_trace.log) — test will skip"
		rm -f "$output"
	fi
}

trace_superpoint_pt \
	"superpoint_v1" \
	"$REPO_ROOT/corelib/src/python/rtabmap_trace_superpoint.py" \
	"$DEST_DIR/superpoint_v1.pth" \
	"$DEST_DIR/superpoint_v1.pt" \
	"$DEST_DIR"

# rpautrat's SuperPoint backend (kFeatureSuperPointRpautrat) traces its own
# *.pth -> *.pt on first detect() inside the C++ class, so the fetch script
# doesn't pre-trace it. The *.pth and superpoint_pytorch.py downloaded above
# are what that runtime tracer consumes.
