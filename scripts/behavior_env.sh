#!/usr/bin/env bash
# Shared native BEHAVIOR-1K/OmniGibson environment. Source this file from bash.
# Only the simulator subprocess uses this interpreter; host harness processes stay
# in runtime/envs/robodojo and never import OmniGibson.

BEHAVIOR_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Resolve symlinks: conda records its prefix in scripts, and worktree links are temporary.
BEHAVIOR_RUNTIME_ROOT="$(readlink -f "${BEHAVIOR_RUNTIME_ROOT:-${ROBODOJO_RUNTIME_ROOT:-$BEHAVIOR_PROJECT_ROOT/runtime}}")"
BEHAVIOR_ENV_ROOT="${BEHAVIOR_ENV_ROOT:-$BEHAVIOR_RUNTIME_ROOT/envs/behavior}"
BEHAVIOR_SOURCE="$(readlink -f "${BEHAVIOR_SOURCE:-$BEHAVIOR_PROJECT_ROOT/BEHAVIOR-1K}")"
BEHAVIOR_PYTHON="${BEHAVIOR_PYTHON:-$BEHAVIOR_ENV_ROOT/bin/python}"
export BEHAVIOR_PROJECT_ROOT BEHAVIOR_RUNTIME_ROOT BEHAVIOR_ENV_ROOT BEHAVIOR_SOURCE BEHAVIOR_PYTHON

export CUDA_HOME="$BEHAVIOR_ENV_ROOT"
export PATH="$BEHAVIOR_ENV_ROOT/bin:${PATH:-/usr/local/bin:/usr/bin:/bin}"
export OMNIGIBSON_DATA_PATH="${OMNIGIBSON_DATA_PATH:-$BEHAVIOR_RUNTIME_ROOT/behavior-data}"
export HF_HOME="${HF_HOME:-$BEHAVIOR_RUNTIME_ROOT/huggingface-cache}"
export PIP_CACHE_DIR="${PIP_CACHE_DIR:-$BEHAVIOR_RUNTIME_ROOT/pip-cache}"
export XDG_CACHE_HOME="${XDG_CACHE_HOME:-$BEHAVIOR_RUNTIME_ROOT/xdg-cache}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-$BEHAVIOR_RUNTIME_ROOT/xdg-runtime}"
export OMNI_KIT_ACCEPT_EULA="${OMNI_KIT_ACCEPT_EULA:-YES}"
export OMNIGIBSON_HEADLESS="${OMNIGIBSON_HEADLESS:-1}"
unset EXP_PATH CARB_APP_PATH ISAAC_PATH

if [[ -f /usr/share/vulkan/icd.d/nvidia_icd.json ]]; then
    export VK_ICD_FILENAMES="${VK_ICD_FILENAMES:-/usr/share/vulkan/icd.d/nvidia_icd.json}"
fi
if [[ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]]; then
    export __EGL_VENDOR_LIBRARY_FILENAMES="${__EGL_VENDOR_LIBRARY_FILENAMES:-/usr/share/glvnd/egl_vendor.d/10_nvidia.json}"
fi
mkdir -p "$XDG_RUNTIME_DIR" "$HF_HOME" "$XDG_CACHE_HOME" "$OMNIGIBSON_DATA_PATH"
chmod 700 "$XDG_RUNTIME_DIR"
