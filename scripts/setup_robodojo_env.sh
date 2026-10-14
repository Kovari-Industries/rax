#!/usr/bin/env bash
# Build runtime/envs/robodojo: Isaac Sim 5.1, Isaac Lab and cuRobo for the RoboDojo simulator,
# plus the harness's own Python dependencies. Follows the pinned upstream RoboDojo/scripts/install.sh
# inside a conda prefix with conda nvcc 12.8. It skips upstream's submodule step, which would move
# IsaacLab, cuRobo and XPolicyLab off their pinned commits.
# Usage: bash scripts/setup_robodojo_env.sh --accept-nvidia-eula
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/robodojo_env.sh"
source "$ROBODOJO_PROJECT_ROOT/dependencies.lock"

[[ "${1:-}" == "--accept-nvidia-eula" ]] || { echo "Pass --accept-nvidia-eula to accept the NVIDIA Isaac Sim EULA." >&2; exit 2; }
[[ "$(git -C "$ROBODOJO_SOURCE" rev-parse HEAD 2>/dev/null)" == "$ROBODOJO_COMMIT" ]] || {
    echo "RoboDojo is not at $ROBODOJO_COMMIT; run scripts/bootstrap_sources.sh first" >&2; exit 1;
}
if git -C "$ROBODOJO_SOURCE" submodule status | grep -q '^[-+U]'; then
    git -C "$ROBODOJO_SOURCE" submodule status >&2
    echo "RoboDojo submodules are missing or off their pinned commits; run scripts/bootstrap_sources.sh" >&2
    exit 1
fi

MINIFORGE="$ROBODOJO_RUNTIME_ROOT/miniforge3"
if [[ ! -x "$MINIFORGE/bin/conda" ]]; then
    installer="$(mktemp --suffix=.sh)"
    curl -fsSL -o "$installer" \
        "https://github.com/conda-forge/miniforge/releases/download/$MINIFORGE_VERSION/Miniforge3-$MINIFORGE_VERSION-Linux-x86_64.sh"
    echo "$MINIFORGE_SHA256  $installer" | sha256sum -c --quiet
    bash "$installer" -b -p "$MINIFORGE"
    rm -f "$installer"
fi
CONDA="$MINIFORGE/bin/conda"

if [[ ! -x "$ROBODOJO_PYTHON" ]]; then
    "$CONDA" create -y -p "$ROBODOJO_ENV_ROOT" -c conda-forge python=3.11 pip ninja git-lfs ffmpeg \
        xorg-libxt xorg-libx11 xorg-libxext xorg-libxrender xorg-libsm xorg-libice xorg-libxrandr \
        xorg-libxi xorg-libxcursor xorg-libxinerama libgl libegl libglu libglib
    # cuRobo compiles its CUDA kernels against this nvcc (robodojo_env.sh sets CUDA_HOME to the env).
    "$CONDA" install -y -p "$ROBODOJO_ENV_ROOT" --override-channels \
        -c nvidia/label/cuda-12.8.0 -c conda-forge cuda-nvcc=12.8.61
fi

# Conda activation hooks reference unset variables; restore strict options right after.
set +euo pipefail
source "$MINIFORGE/bin/activate" "$ROBODOJO_ENV_ROOT"
set -euo pipefail
[[ "${CONDA_PREFIX:-}" == "$ROBODOJO_ENV_ROOT" && "$(command -v python)" == "$ROBODOJO_PYTHON" ]] || {
    echo "Activation failed: expected $ROBODOJO_ENV_ROOT" >&2; exit 1;
}
export CUDA_HOME="$ROBODOJO_ENV_ROOT" PIP_USER=0 PYTHONNOUSERSITE=1
nvcc -V | grep -q "release 12.8" || { echo "nvcc 12.8 not on PATH" >&2; exit 1; }

# The steps of upstream install.sh (base_deps, isaacsim, isaaclab, curobo) with its pins. They run
# here rather than through install.sh, which activates any ~/miniconda3 env named RoboDojo and
# updates the submodules to their remote heads.
constraints="$(mktemp)"
trap 'rm -f "$constraints"' EXIT
printf '%s\n' numpy==1.26.0 packaging==23.0 typing_extensions==4.12.2 filelock==3.13.1 websockets==12.0 \
    scipy==1.15.3 warp-lang==1.11.0 click==8.1.7 psutil==5.9.8 wheel==0.45.1 starlette==0.45.3 \
    "stable-baselines3<2.8" "onnx>=1.18,<1.22" "ipython<9" virtualenv==20.30.0 > "$constraints"
pip() { python -m pip "$@"; }
pin_runtime_deps() {
    pip install -r "$constraints"
    pip uninstall -y python-discovery 2>/dev/null || true
}
pip install -r "$ROBODOJO_SOURCE/scripts/requirements.txt"
pip install opencv-python-headless==4.11.0.86 pillow matplotlib "scipy==1.15.3" scikit-learn
pip install "numpy==$NUMPY_VERSION"
if ! pip show isaacsim 2>/dev/null | grep -q "^Version: 5.1.0"; then
    pip install --upgrade pip
    pip install numpy==1.26.0 typing_extensions==4.12.2 filelock==3.13.1
    pip install torch==2.7.0 torchvision==0.22.0 torchaudio==2.7.0 --index-url https://download.pytorch.org/whl/cu128
    pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
    pin_runtime_deps
fi
if ! pip show isaaclab >/dev/null 2>&1; then
    (cd "$ROBODOJO_SOURCE/third_party/IsaacLab" && OMNI_KIT_ACCEPT_EULA=YES TERM=xterm-256color ./isaaclab.sh --install none)
    pip show torchaudio >/dev/null 2>&1 || pip install torch==2.7.0 torchvision==0.22.0 torchaudio==2.7.0 \
        --index-url https://download.pytorch.org/whl/cu128
    pin_runtime_deps
fi
if ! python -c 'from curobo.motion_planner import MotionPlanner' >/dev/null 2>&1; then
    pip uninstall -y nvidia-curobo curobo 2>/dev/null || true
    (cd "$ROBODOJO_SOURCE/third_party/curobo" && python -m pip install -e ".[cu12]" --no-build-isolation --constraint "$constraints")
    pin_runtime_deps
fi

# The harness's own dependencies, constrained to what Isaac Sim pins.
python -m pip install "jinja2==$JINJA2_VERSION" pyyaml h5py imageio imageio-ffmpeg msgpack omegaconf pytest \
    "numpy==$NUMPY_VERSION"

git -C "$ROBODOJO_SOURCE" submodule status | grep -q '^[-+U]' && {
    echo "A RoboDojo submodule moved during installation; rerun scripts/bootstrap_sources.sh" >&2; exit 1;
}
python - <<PY
import importlib.metadata as m, sys
expected = {"isaacsim": "$ISAACSIM_VERSION".rsplit(".", 1)[0], "torch": "$TORCH_VERSION", "numpy": "$NUMPY_VERSION",
            "jinja2": "$JINJA2_VERSION"}
bad = []
for name in ("isaacsim", "isaaclab", "torch", "numpy", "nvidia-curobo", "jinja2", "pyyaml", "h5py", "msgpack", "omegaconf"):
    try:
        version = m.version(name)
    except m.PackageNotFoundError:
        version = "MISSING"
    want = expected.get(name)
    print(f"{name}=={version}" + (f"  (expected {want})" if want and not version.startswith(want) else ""))
    if version == "MISSING" or (want and not version.startswith(want)):
        bad.append(name)
from curobo.motion_planner import MotionPlanner  # noqa: F401  (the compiled extension loads)
sys.exit(f"unexpected packages: {bad}" if bad else 0)
PY
echo "RoboDojo environment ready: $ROBODOJO_ENV_ROOT"
