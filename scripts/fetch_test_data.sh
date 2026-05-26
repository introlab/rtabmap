#!/usr/bin/env bash
# Fetch test data assets listed in data/tests/manifest.txt from Google Drive.
# Skips files that are already present and whose SHA-256 matches the manifest.
# Intended for CI and local first-time setup.
#
# All linked files are public and under ~100 MB, so the direct GDrive download
# URL (uc?export=download&id=...) streams the bytes directly without the
# virus-scan interstitial that would otherwise need a tool like gdown.
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

while IFS=$'\t' read -r name file_id expected_sha; do
	# Skip comments and blank lines.
	[[ -z "${name// }" || "$name" =~ ^# ]] && continue

	target="$DEST_DIR/$name"
	if [[ -f "$target" ]] && verify_sha "$target" "$expected_sha" 2>/dev/null; then
		echo "Already up-to-date: $name"
		continue
	fi

	url="https://drive.google.com/uc?export=download&id=${file_id}"
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
