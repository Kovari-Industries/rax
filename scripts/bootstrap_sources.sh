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

# HumanoidArena: the pinned checkout lives on the runtime storage tier and
# is exposed at HumanoidArena/. Large-file pointers stay unfetched: assets come separately.
arena_target="$(dirname "$runtime_target")/HumanoidArena"
if [[ ! -e "$PROJECT_ROOT/HumanoidArena" ]]; then
    if [[ ! -d "$arena_target/.git" ]]; then
        GIT_LFS_SKIP_SMUDGE=1 git clone --filter=blob:none --no-checkout "$HUMANOID_ARENA_URL" "$arena_target"
        GIT_LFS_SKIP_SMUDGE=1 git -C "$arena_target" checkout --detach "$HUMANOID_ARENA_COMMIT"
    fi
    ln -s "$arena_target" "$PROJECT_ROOT/HumanoidArena"
fi
[[ "$(git -C "$PROJECT_ROOT/HumanoidArena" rev-parse HEAD)" == "$HUMANOID_ARENA_COMMIT" ]] || {
    echo "HumanoidArena is not at $HUMANOID_ARENA_COMMIT" >&2
    exit 1
}

echo "Sources ready. Install the host environment (scripts/setup_host_env.sh), then run scripts/doctor.sh."
echo "For HumanoidArena: scripts/setup_humanoid_assets.sh and scripts/setup_humanoid_demos.py (see docs/HUMANOID_BACKEND.md)."
