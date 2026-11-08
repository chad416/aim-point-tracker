#!/usr/bin/env bash
# Integration tests on rendered sequences (SPEC §9.3), then their plots.
# IT01, IT02 and IT08 drive the C library through ctypes; IT03-IT07, IT09 and
# IT10 run aim_host (IT10 replays the weapon firmware recorded in Renode).
# By default the 100 000-frame run (IT09, marker long) and the Renode re-recording
# (marker cosim) are left out (pytest.ini); add -m long or -m cosim to run them.
# Usage (inside WSL or CI): scripts/test-it.sh [extra pytest args]
set -euo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"

cd "$REPO"
python3 -m pytest tests/integration -v -p no:cacheprovider "$@"
python3 tools/reference/it_plots.py
