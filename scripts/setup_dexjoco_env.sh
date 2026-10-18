#!/usr/bin/env bash
# Build runtime/envs/dexjoco for the DexJoCo simulator subprocess exactly as the official
# README does: the conda dependencies of environment-dexjoco.yaml (python 3.11, mujoco 3.4.0,
# gymnasium 1.0.0, numpy 1.26.4) and its pip section (editable dexjoco and openpi-client).
# Never touches runtime/envs/host. Usage: bash scripts/setup_dexjoco_env.sh
# CONDA_EXE selects the conda binary (default: the Miniforge scripts/setup_host_env.sh installs,
# else `conda` on PATH).
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/dexjoco_env.sh"
source "$(dirname "${BASH_SOURCE[0]}")/../dependencies.lock"
CONDA="${CONDA_EXE:-}"
if [[ -z "$CONDA" ]]; then
    CONDA="$DEXJOCO_RUNTIME_ROOT/miniforge3/bin/conda"
    [[ -x "$CONDA" ]] || CONDA="$(command -v conda || true)"
fi
[[ -x "$CONDA" ]] || { echo "No conda binary; set CONDA_EXE" >&2; exit 1; }
[[ -f "$DEXJOCO_SOURCE/environment-dexjoco.yaml" ]] || { echo "Missing DexJoCo source; run scripts/bootstrap_sources.sh" >&2; exit 1; }
actual="$(git -C "$DEXJOCO_SOURCE" rev-parse HEAD)"
[[ "$actual" == "$DEXJOCO_COMMIT" ]] || { echo "DexJoCo is at $actual, expected $DEXJOCO_COMMIT" >&2; exit 1; }
[[ -z "$(git -C "$DEXJOCO_SOURCE" status --porcelain --untracked-files=no)" ]] || { echo "DexJoCo checkout is modified" >&2; exit 1; }

if [[ ! -x "$DEXJOCO_PYTHON" ]]; then
    # The official yaml's conda dependencies from conda-forge only (some sites block the
    # `defaults` channel; every package is on conda-forge), then its pip section verbatim.
    spec="$(mktemp --suffix=.yaml)"
    awk '/^  - pip:/{exit} !/^  - defaults$/' "$DEXJOCO_SOURCE/environment-dexjoco.yaml" > "$spec"
    "$CONDA" env create -y -p "$DEXJOCO_ENV_ROOT" -f "$spec"
    rm -f "$spec"
fi
# The yaml's pip section: the official packages as editable installs from the pinned checkout.
"$DEXJOCO_ENV_ROOT/bin/pip" install -e "$DEXJOCO_SOURCE/dexjoco" -e "$DEXJOCO_SOURCE/openpi/packages/openpi-client"

"$DEXJOCO_PYTHON" - <<'PY'
import importlib.metadata as m
for name in ("mujoco", "gymnasium", "numpy", "scipy", "dexjoco", "openpi-client", "msgpack", "imageio-ffmpeg"):
    try:
        print(f"{name}=={m.version(name)}")
    except m.PackageNotFoundError:
        print(f"{name}: MISSING")
PY
