#!/usr/bin/env bash
# Check that this machine can run RoboDojo auto-research sessions: pinned sources and patches,
# assets, the simulator environment, Docker and the agent image, an agent CLI and GPUs.
# Run scripts/behavior_doctor.py as well for the BEHAVIOR-1K backend.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/robodojo_env.sh"
source "$ROBODOJO_PROJECT_ROOT/dependencies.lock"
IMAGE="${ROBODOJO_AGENT_IMAGE:-robodojo-auto-research:dev}"
failures=0
pass() { echo "PASS  $*"; }
fail() { echo "FAIL  $*"; failures=$((failures+1)); }
note() { echo "NOTE  $*"; }

# Sources: the pinned commit, its submodules and both patches.
actual="$(git -C "$ROBODOJO_SOURCE" rev-parse HEAD 2>/dev/null)"
[[ "$actual" == "$ROBODOJO_COMMIT" ]] && pass "RoboDojo at $ROBODOJO_COMMIT" \
    || fail "RoboDojo is at ${actual:-nothing}, expected $ROBODOJO_COMMIT (scripts/bootstrap_sources.sh)"
moved="$(git -C "$ROBODOJO_SOURCE" submodule status 2>/dev/null | grep '^[-+U]')"
[[ -z "$moved" ]] && pass "RoboDojo submodules at their pinned commits" \
    || fail "RoboDojo submodules missing or moved: $moved (scripts/bootstrap_sources.sh)"
for patch in robodojo.patch robodojo-pr48-drive-reset.patch; do
    git -C "$ROBODOJO_SOURCE" apply --reverse --check "$ROBODOJO_PROJECT_ROOT/patches/$patch" >/dev/null 2>&1 \
        && pass "patches/$patch applied" || fail "patches/$patch not applied (scripts/bootstrap_sources.sh)"
done

# Assets.
missing=()
for name in Robots Object Material Eval_Layout; do
    [[ -d "$ROBODOJO_SOURCE/Assets/$name" ]] || missing+=("$name")
done
[[ ${#missing[@]} -eq 0 ]] && pass "RoboDojo assets present" \
    || fail "RoboDojo assets missing: ${missing[*]} (scripts/setup_assets.sh)"
assets="$(git -C "$ROBODOJO_RUNTIME_ROOT/robodojo-cache/robodojo_assets_repo" rev-parse HEAD 2>/dev/null)"
[[ -z "$assets" || "$assets" == "$ROBODOJO_ASSETS_REVISION" ]] \
    || fail "RoboDojo assets at $assets, expected $ROBODOJO_ASSETS_REVISION (scripts/setup_assets.sh)"

# Simulator environment.
if [[ -x "$ROBODOJO_PYTHON" ]]; then
    "$ROBODOJO_PYTHON" - "$ISAACSIM_VERSION" "$TORCH_VERSION" "$NUMPY_VERSION" <<'PY' && pass "simulator environment $ROBODOJO_ENV_ROOT" \
        || fail "simulator environment $ROBODOJO_ENV_ROOT (scripts/setup_robodojo_env.sh)"
import importlib.metadata as m, sys
want = dict(zip(("isaacsim", "torch", "numpy"), sys.argv[1:]))
bad = {name: m.version(name) for name, version in want.items() if m.version(name) != version}
import curobo, jinja2, yaml  # noqa: F401
print(f"      Isaac Sim {m.version('isaacsim')}, PyTorch {m.version('torch')}, NumPy {m.version('numpy')}")
sys.exit(f"      versions differ from dependencies.lock: {bad}" if bad else 0)
PY
else
    fail "no Python at $ROBODOJO_PYTHON (scripts/setup_robodojo_env.sh)"
fi
[[ -x "$ROBODOJO_ENV_ROOT/bin/nvcc" ]] && pass "nvcc in the environment (cuRobo kernels)" \
    || fail "no nvcc in $ROBODOJO_ENV_ROOT/bin (scripts/setup_robodojo_env.sh)"

# Docker and the agent image.
if docker info >/dev/null 2>&1; then
    pass "Docker usable without sudo"
    docker image inspect "$IMAGE" >/dev/null 2>&1 && pass "agent image $IMAGE" \
        || fail "agent image $IMAGE missing (scripts/build_agent_image.py)"
else
    fail "Docker not usable by $(id -un) (add the user to the docker group)"
fi
[[ -e /dev/loop-control ]] && pass "loop devices (capped session workspaces)" || fail "/dev/loop-control missing"
for tool in mkfs.ext4 findmnt git-lfs; do
    command -v "$tool" >/dev/null 2>&1 || [[ -x "/sbin/$tool" || -x "/usr/sbin/$tool" ]] \
        && pass "$tool" || fail "$tool not found"
done

# Agent CLIs: native binaries, since they run inside the agent container.
clis=0
for cli in codex claude; do
    path="$(command -v "$cli" 2>/dev/null)" || continue
    real="$(readlink -f "$path")"
    if [[ "$(head -c 4 "$real" | od -An -c | tr -d ' ')" == '177ELF' ]]; then
        pass "$cli $("$real" --version 2>&1 | tail -n 1) ($real)"
        clis=$((clis+1))
    else
        fail "$cli at $real is not a native executable (install the standalone binary, not the npm wrapper)"
    fi
done
[[ $clis -gt 0 ]] || fail "no agent CLI: install Codex CLI or Claude Code"
key="$HOME/.codex/secrets/openrouter_api_key"  # The launcher's default --key-file.
[[ -r "$key" || -n "${OPENROUTER_API_KEY:-}" ]] && pass "OpenRouter key (Gemini tool, OpenRouter models)" \
    || note "no OpenRouter key at $key or in OPENROUTER_API_KEY: the Gemini tool and OpenRouter providers are unavailable"

# GPUs.
if gpus="$(nvidia-smi --query-gpu=index,name,driver_version,memory.total --format=csv,noheader 2>/dev/null)"; then
    pass "GPUs:"; echo "$gpus" | sed 's/^/      /'
else
    fail "nvidia-smi failed"
fi
[[ -x "${BEHAVIOR_PYTHON:-$ROBODOJO_RUNTIME_ROOT/envs/behavior/bin/python}" ]] \
    && note "BEHAVIOR environment found; check it with: $ROBODOJO_PYTHON scripts/behavior_doctor.py --gpu 0"

if [[ $failures -eq 0 ]]; then echo "All checks passed."; else echo "$failures check(s) failed."; exit 1; fi
