#!/usr/bin/env bash
# Compare the C library with the NumPy reference on simulated frames, then
# measure the centroid accuracy curve (SPEC §9.2, §12).
# Usage (inside WSL or CI): scripts/crosscheck.sh [--frames 40]
set -euo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"

cd "$REPO/tools/reference"
python3 crosscheck.py "$@"
python3 centroid_curve.py
