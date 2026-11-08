#!/usr/bin/env bash
# Bind-mount this repository at a path without spaces (WSL or Linux, as root).
# Zephyr's build and twister break on paths containing spaces, and a symlink is
# not enough because twister resolves it to the real path.
# Usage: bash "<repo>/scripts/wsl-mount.sh" [mount point]
#        mount point default: $AIMTRK_SRC, else ~/aimtrk-src
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET="${1:-${AIMTRK_SRC:-$HOME/aimtrk-src}}"

if mountpoint -q "$TARGET"; then
	echo "already mounted: $TARGET"
	exit 0
fi

[ -L "$TARGET" ] && rm "$TARGET"
mkdir -p "$TARGET"
mount --bind "$REPO" "$TARGET"
echo "mounted $REPO -> $TARGET"
