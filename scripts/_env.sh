# Sourced by the other scripts. Sets REPO, WS and activates the Python venv.
# If the repository path contains a space (Windows folder "New CLI"), it is
# bind-mounted at $HOME/aimtrk-src first, because Zephyr builds break on spaces.

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$REPO" == *" "* ]]; then
	bash "$REPO/scripts/wsl-mount.sh" >/dev/null
	REPO="$HOME/aimtrk-src"
fi
# Shares the west workspace (Zephyr checkout and SDK) with S1 and S2.
WS="${AIMTRK_WS:-$HOME/trainer-ws}"

# shellcheck disable=SC1091
source "${AIMTRK_VENV:-$HOME/zvenv}/bin/activate"
