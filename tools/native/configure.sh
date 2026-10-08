#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cmake -S "$ROOT" -B "$ROOT/build-player" -GNinja \
  -DNUBOTS_NATIVE_PLAYER=ON \
  -DPython3_EXECUTABLE="$ROOT/build-deps/venv/bin/python" "$@"
