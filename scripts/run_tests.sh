#!/usr/bin/env bash
# Wrapper to build and run Zephyr tests (moved from tests/zephyr)

set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
proj_root="$(cd "$here/.." && pwd)"

cd "$proj_root"

if [[ "$#" -eq 0 ]]; then
  echo "Usage: $0 <board> [--clean]"
  exit 1
fi

BOARD="$1"
shift || true

if [[ "$*" == *--clean* ]]; then
  west build -b "$BOARD" --pristine
else
  west build -b "$BOARD"
fi

west flash
