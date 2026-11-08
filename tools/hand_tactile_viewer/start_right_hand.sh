#!/usr/bin/env bash
set -euo pipefail
exec "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/start_hand.sh" right "$@"
