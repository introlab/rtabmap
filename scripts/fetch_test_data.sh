#!/usr/bin/env bash
# Fetch test data assets listed in data/tests/manifest.txt. Each entry's
# source can be either a bare Google Drive file ID (assembled into the
# uc?export=download&id=... URL) or a full http(s):// URL (used as-is).
# Skips files that are already present and whose SHA-256 matches the manifest.
# Intended for CI and local first-time setup.
#
# All linked files are public and under ~100 MB, so the direct GDrive download
# URL streams the bytes directly without the virus-scan interstitial that
# would otherwise need a tool like gdown.
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

while IFS=$'\t' read -r name source expected_sha; do
	# Strip trailing CR so the script works when manifest.txt is checked out
	# with CRLF line endings (default on Windows Git unless core.autocrlf=input).
	# Without this, expected_sha keeps a trailing \r and even a byte-for-byte
	# match looks like "expected <sha>\r, got <sha>".
	name="${name%$'\r'}"
	source="${source%$'\r'}"
	expected_sha="${expected_sha%$'\r'}"
	# Skip comments and blank lines.
	[[ -z "${name// }" || "$name" =~ ^# ]] && continue

	target="$DEST_DIR/$name"
	if [[ -f "$target" ]] && verify_sha "$target" "$expected_sha" 2>/dev/null; then
		echo "Already up-to-date: $name"
		continue
	fi

	# If the source already looks like a URL, use it as-is. Otherwise treat
	# it as a Google Drive file ID and assemble the direct-download URL.
	if [[ "$source" =~ ^https?:// ]]; then
		url="$source"
	else
		url="https://drive.google.com/uc?export=download&id=${source}"
	fi
	echo "Fetching $name <- $url"
	mkdir -p "$(dirname "$target")"
	# -L follows the redirect, -f fails on HTTP errors, -S shows errors on stderr.
	curl -fsSL "$url" -o "$target.partial"
	if ! verify_sha "$target.partial" "$expected_sha"; then
		rm -f "$target.partial"
		exit 1
	fi
	mv -f "$target.partial" "$target"
done < "$MANIFEST"

echo "Test data ready under $DEST_DIR"
