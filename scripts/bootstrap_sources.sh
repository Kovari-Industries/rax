#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$PROJECT_ROOT/dependencies.lock"

if [[ $# -ne 0 ]]; then
    echo "usage: $0" >&2
    exit 2
fi

if [[ -n "${ROBODOJO_RUNTIME_ROOT:-}" ]]; then
    runtime_target="$(realpath -m "$ROBODOJO_RUNTIME_ROOT")"
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

apply_patch_once() {
    local checkout="$1"
    local patch="$2"
    local expected_sha="$3"
    local actual_sha
    actual_sha="$(sha256sum "$patch" | awk '{print $1}')"
    [[ "$actual_sha" == "$expected_sha" ]] || {
        echo "patch checksum mismatch: $patch" >&2
        exit 2
    }
    if git -C "$checkout" apply --reverse --check "$patch" >/dev/null 2>&1; then
        echo "patch already applied: $patch"
    else
        git -C "$checkout" apply --check "$patch"
        git -C "$checkout" apply "$patch"
    fi
}

if [[ ! -d "$PROJECT_ROOT/RoboDojo/.git" ]]; then
    git clone --recursive "$ROBODOJO_URL" "$PROJECT_ROOT/RoboDojo"
fi
git -C "$PROJECT_ROOT/RoboDojo" checkout --detach "$ROBODOJO_COMMIT"
git -C "$PROJECT_ROOT/RoboDojo" submodule sync --recursive
git -C "$PROJECT_ROOT/RoboDojo" submodule update --init --recursive
apply_patch_once \
    "$PROJECT_ROOT/RoboDojo" \
    "$PROJECT_ROOT/patches/robodojo.patch" \
    "$ROBODOJO_PATCH_SHA256"
apply_patch_once \
    "$PROJECT_ROOT/RoboDojo" \
    "$PROJECT_ROOT/patches/robodojo-pr48-drive-reset.patch" \
    "$ROBODOJO_DRIVE_RESET_PATCH_SHA256"

# BEHAVIOR-1K (optional benchmark): the ~1 GB checkout lives on the runtime storage tier
# and is exposed at BEHAVIOR-1K/, like the other bulky sources.
behavior_target="$(dirname "$runtime_target")/BEHAVIOR-1K"
if [[ ! -e "$PROJECT_ROOT/BEHAVIOR-1K" ]]; then
    if [[ ! -d "$behavior_target/.git" ]]; then
        git clone --depth 1 --branch "$BEHAVIOR_TAG" "$BEHAVIOR_URL" "$behavior_target"
    fi
    # A runtime inside the project (.local-runtime) puts the checkout at BEHAVIOR-1K/ itself.
    [[ "$behavior_target" -ef "$PROJECT_ROOT/BEHAVIOR-1K" ]] || ln -s "$behavior_target" "$PROJECT_ROOT/BEHAVIOR-1K"
fi
[[ "$(git -C "$PROJECT_ROOT/BEHAVIOR-1K" rev-parse HEAD)" == "$BEHAVIOR_COMMIT" ]] || {
    echo "BEHAVIOR-1K is not at $BEHAVIOR_COMMIT ($BEHAVIOR_TAG)" >&2
    exit 2
}

echo "Sources ready. Install the native environment, then run scripts/robodojo_doctor.sh."
echo "For BEHAVIOR-1K: scripts/setup_behavior_env.sh and scripts/setup_behavior_assets.sh (see docs/BEHAVIOR_BACKEND.md)."
