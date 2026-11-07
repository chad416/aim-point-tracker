#!/usr/bin/env bash
# Integration tests on rendered sequences (SPEC §9.3), then their plots.
# Until aim_host exists (week 3) they drive the C library through ctypes.
# Usage (inside WSL or CI): scripts/test-it.sh [extra pytest args]
set -euo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"

cd "$REPO"
python3 -m pytest tests/integration -v -p no:cacheprovider "$@"
python3 tools/reference/week2_plots.py
