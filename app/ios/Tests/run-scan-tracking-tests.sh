#!/usr/bin/env bash
set -euo pipefail
ios_dir="$(cd "$(dirname "$0")/.." && pwd)"
test_dir="$(mktemp -d)"
trap 'rm -rf "$test_dir"' EXIT
swiftc "$ios_dir/RTABMapApp/ScanTrackingState.swift" \
  "$ios_dir/Tests/ScanTrackingState/main.swift" -o "$test_dir/scan-tracking-tests"
"$test_dir/scan-tracking-tests"
