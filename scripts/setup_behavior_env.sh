#!/usr/bin/env bash
# Build runtime/envs/behavior for the BEHAVIOR-1K simulator subprocess.
# Follows the pinned upstream setup.sh (OmniGibson, BDDL, JoyLo, eval, cuRobo primitives)
# inside a dedicated conda prefix with conda nvcc 12.8. Never touches runtime/envs/robodojo.
# Usage: bash scripts/setup_behavior_env.sh --accept-nvidia-eula
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/behavior_env.sh"

[[ "${1:-}" == "--accept-nvidia-eula" ]] || { echo "Pass --accept-nvidia-eula to accept the NVIDIA Isaac Sim EULA." >&2; exit 2; }
[[ -f "$BEHAVIOR_SOURCE/setup.sh" ]] || { echo "Missing BEHAVIOR-1K source; run scripts/bootstrap_sources.sh" >&2; exit 1; }
source "$(dirname "${BASH_SOURCE[0]}")/../dependencies.lock"
actual="$(git -C "$BEHAVIOR_SOURCE" rev-parse HEAD)"
[[ "$actual" == "$BEHAVIOR_COMMIT" ]] || { echo "BEHAVIOR-1K is at $actual, expected $BEHAVIOR_COMMIT" >&2; exit 1; }

# The same pinned Miniforge as scripts/setup_robodojo_env.sh; either script installs it.
MINIFORGE="$BEHAVIOR_RUNTIME_ROOT/miniforge3"
if [[ ! -x "$MINIFORGE/bin/conda" ]]; then
    installer="$(mktemp --suffix=.sh)"
    trap 'rm -f "$installer"' EXIT
    curl -fsSL -o "$installer" \
        "https://github.com/conda-forge/miniforge/releases/download/$MINIFORGE_VERSION/Miniforge3-$MINIFORGE_VERSION-Linux-x86_64.sh"
    echo "$MINIFORGE_SHA256  $installer" | sha256sum -c --quiet
    bash "$installer" -b -p "$MINIFORGE"
fi
CONDA="$MINIFORGE/bin/conda"

if [[ ! -x "$BEHAVIOR_PYTHON" ]]; then
    "$CONDA" create -y -p "$BEHAVIOR_ENV_ROOT" -c conda-forge python=3.11 pip "setuptools>=71,<81" wheel
    # Same nvcc the RoboDojo env uses; cuRobo JIT-compiles against it.
    "$CONDA" install -y -p "$BEHAVIOR_ENV_ROOT" -c nvidia/label/cuda-12.8.0 cuda-nvcc cuda-cudart-dev
fi

# Conda hooks reference unset variables. The binutils_linux-64 2.28.1 hook also
# unconditionally reports changes with `diff | ... | sort`, which fails under
# pipefail; GCC/G++ guard that report with CONDA_BUILD=1. Verify the resulting
# environment and build toolchain after restoring strict options immediately.
set +euo pipefail
source "$BEHAVIOR_RUNTIME_ROOT/miniforge3/bin/activate" "$BEHAVIOR_ENV_ROOT"
set -euo pipefail
[[ "${CONDA_PREFIX:-}" == "$BEHAVIOR_ENV_ROOT" && "$(command -v python)" == "$BEHAVIOR_ENV_ROOT/bin/python" ]] || {
    echo "Activation failed: expected env prefix and python" >&2; exit 1;
}
for compiler_var in CC CXX; do
    compiler="${!compiler_var:-}"
    [[ -n "$compiler" && "$(command -v "$compiler")" == "$BEHAVIOR_ENV_ROOT/bin/"* ]] && "$compiler" --version >/dev/null || {
        echo "Activation failed: $compiler_var must resolve to a working env compiler" >&2; exit 1;
    }
done
[[ "${CUDA_HOME:-}" == "$BEHAVIOR_ENV_ROOT" && "$(command -v nvcc)" == "$BEHAVIOR_ENV_ROOT/bin/nvcc" ]] || {
    echo "Activation failed: expected env CUDA_HOME and nvcc" >&2; exit 1;
}
nvcc -V | grep -q "release 12.8" || { echo "nvcc 12.8 not on PATH" >&2; exit 1; }

cd "$BEHAVIOR_SOURCE"
./setup.sh --omnigibson --bddl --joylo --eval --primitives --cuda-version 12.8 --accept-nvidia-eula
# Upstream asks for numpy<2; the working environment has the RoboDojo environment's numpy.
python -m pip install "numpy==$NUMPY_VERSION"

# Fail on a missing package or a version other than the pinned ones.
python - <<PY
import importlib.metadata as m, sys
expected = {"isaacsim": "$ISAACSIM_VERSION".rsplit(".", 1)[0], "torch": "$TORCH_VERSION", "numpy": "$NUMPY_VERSION"}
bad = []
for name in ("isaacsim", "torch", "numpy", "omnigibson", "bddl", "nvidia-curobo", "warp-lang"):
    try:
        version = m.version(name)
    except m.PackageNotFoundError:
        version = "MISSING"
    want = expected.get(name)
    wrong = version == "MISSING" or (want is not None and version != want and not version.startswith((want + ".", want + "+")))
    print(f"{name}=={version}" + (f"  (expected {want})" if wrong and want else ""))
    if wrong:
        bad.append(name)
sys.exit(f"BEHAVIOR environment check failed: {', '.join(bad)}" if bad else 0)
PY
echo "BEHAVIOR environment ready: $BEHAVIOR_ENV_ROOT"
