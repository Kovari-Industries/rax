#!/usr/bin/env bash
# Check out the pinned upstream simulator sources (dependencies.lock) on the runtime storage tier
# and link runtime/ to it. Usage: bash scripts/bootstrap_sources.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$PROJECT_ROOT/dependencies.lock"

if [[ $# -ne 0 ]]; then
    echo "usage: $0" >&2
    exit 2
fi

if [[ -n "${RAX_RUNTIME_ROOT:-}" ]]; then
    runtime_target="$(realpath -m "$RAX_RUNTIME_ROOT")"
else
    runtime_target="$PROJECT_ROOT/.local-runtime"
fi

mkdir -p "$runtime_target"
runtime_target="$(readlink -f "$runtime_target")"
if [[ -L "$PROJECT_ROOT/runtime" ]]; then
    [[ "$(readlink -f "$PROJECT_ROOT/runtime")" == "$(readlink -f "$runtime_target")" ]] || {
        echo "runtime points somewhere else: $PROJECT_ROOT/runtime" >&2
        exit 2
    }
elif [[ -e "$PROJECT_ROOT/runtime" ]]; then
    echo "refusing to replace non-symlink runtime path" >&2
    exit 2
else
    ln -s "$runtime_target" "$PROJECT_ROOT/runtime"
fi

# DexJoCo: the official checkout, never modified, on the runtime storage
# tier and exposed at DexJoCo/.
dexjoco_target="$(dirname "$runtime_target")/DexJoCo"
if [[ ! -e "$PROJECT_ROOT/DexJoCo" ]]; then
    if [[ ! -d "$dexjoco_target/.git" ]]; then
        git clone "$DEXJOCO_URL" "$dexjoco_target"
    fi
    git -C "$dexjoco_target" checkout --detach "$DEXJOCO_COMMIT"
    ln -s "$dexjoco_target" "$PROJECT_ROOT/DexJoCo"
fi
[[ "$(git -C "$PROJECT_ROOT/DexJoCo" rev-parse HEAD)" == "$DEXJOCO_COMMIT" ]] || {
    echo "DexJoCo is not at $DEXJOCO_COMMIT" >&2
    exit 2
}

echo "Sources ready. Install the host environment (scripts/setup_host_env.sh), then run scripts/doctor.sh."
echo "For DexJoCo: scripts/setup_dexjoco_env.sh and scripts/setup_dexjoco_demos.py (see docs/DEXJOCO_BACKEND.md)."
