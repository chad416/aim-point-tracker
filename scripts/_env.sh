# Sourced by the other scripts. Sets REPO and WS and activates the Python venv.
#
# Environment (all optional; the defaults are one development set-up):
#   AIMTRK_WS    west workspace with zephyr/ and the SDK, shared with the
#                weapon firmware and motion-platform-controller repositories
#                (default ~/trainer-ws; only the Zephyr unit tests need it)
#   AIMTRK_VENV  Python venv to activate (default ~/zvenv; if it does not
#                exist, the python3 on PATH is used, e.g. after
#                pip install -r requirements.txt)
#   AIMTRK_SRC   where to bind-mount the repository if its path contains a
#                space (default ~/aimtrk-src; needs root, see wsl-mount.sh)
#
# Zephyr's build and twister break on paths with spaces (a Windows folder such
# as "New CLI" seen from WSL), so such a checkout is bind-mounted first.

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$REPO" == *" "* ]]; then
	bash "$REPO/scripts/wsl-mount.sh" >/dev/null
	REPO="${AIMTRK_SRC:-$HOME/aimtrk-src}"
fi
WS="${AIMTRK_WS:-$HOME/trainer-ws}"

VENV="${AIMTRK_VENV:-$HOME/zvenv}"
if [[ -f "$VENV/bin/activate" ]]; then
	# shellcheck disable=SC1091
	source "$VENV/bin/activate"
fi
