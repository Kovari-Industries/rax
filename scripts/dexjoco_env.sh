#!/usr/bin/env bash
# Shared native DexJoCo environment. Source this file from bash.
# Only the simulator subprocess uses this interpreter (the official DexJoCo package, unmodified);
# host harness processes stay in runtime/envs/host and never import dexjoco or mujoco.

DEXJOCO_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Resolve symlinks: conda records its prefix in scripts, and worktree links are temporary.
DEXJOCO_RUNTIME_ROOT="$(readlink -f "${DEXJOCO_RUNTIME_ROOT:-${RAX_RUNTIME_ROOT:-$DEXJOCO_PROJECT_ROOT/runtime}}")"
DEXJOCO_ENV_ROOT="${DEXJOCO_ENV_ROOT:-$DEXJOCO_RUNTIME_ROOT/envs/dexjoco}"
DEXJOCO_SOURCE="$(readlink -f "${DEXJOCO_SOURCE:-$DEXJOCO_PROJECT_ROOT/DexJoCo}")"
DEXJOCO_PYTHON="${DEXJOCO_PYTHON:-$DEXJOCO_ENV_ROOT/bin/python}"
export DEXJOCO_PROJECT_ROOT DEXJOCO_RUNTIME_ROOT DEXJOCO_ENV_ROOT DEXJOCO_SOURCE DEXJOCO_PYTHON
export PIP_CACHE_DIR="${PIP_CACHE_DIR:-$DEXJOCO_RUNTIME_ROOT/pip-cache}"
export HF_HOME="${HF_HOME:-$DEXJOCO_RUNTIME_ROOT/huggingface-cache}"
