#!/usr/bin/env bash
# Install the pinned BEHAVIOR-1K datasets into $OMNIGIBSON_DATA_PATH (SSD runtime).
# Zips are staged on the SSD with resumable downloads (upstream stages them in /tmp and
# cannot resume), downloaded at the lock's dataset revision, checked against the lock's
# sha256, then unpacked atomically. The encryption key is installed by upstream's
# own download_key(). Requires the operator's explicit acceptance of the BEHAVIOR-1K
# dataset terms (non-commercial research use, data used only within OmniGibson).
# Usage: bash scripts/setup_behavior_assets.sh --accept-dataset-tos [--keep-zips]
set -euo pipefail
here="$(dirname "${BASH_SOURCE[0]}")"
source "$here/behavior_env.sh"
source "$here/../dependencies.lock"

accept=false keep=false
for arg in "$@"; do
    case "$arg" in
        --accept-dataset-tos) accept=true ;;
        --keep-zips) keep=true ;;
        *) echo "Unknown argument: $arg" >&2; exit 2 ;;
    esac
done
$accept || { echo "Pass --accept-dataset-tos to accept the BEHAVIOR-1K dataset terms." >&2; exit 2; }
[[ -x "$BEHAVIOR_PYTHON" ]] || { echo "Missing $BEHAVIOR_PYTHON; run scripts/setup_behavior_env.sh" >&2; exit 1; }

export BEHAVIOR_STAGING="$BEHAVIOR_RUNTIME_ROOT/behavior-staging"
export BEHAVIOR_KEEP_ZIPS="$keep"
export TMPDIR="$BEHAVIOR_STAGING/tmp"
mkdir -p "$TMPDIR"

"$BEHAVIOR_PYTHON" - "$BEHAVIOR_DATASETS_REVISION" \
    "$BEHAVIOR_ROBOT_ASSETS" "$BEHAVIOR_ROBOT_ASSETS_SHA256" \
    "$BEHAVIOR_TASK_INSTANCES" "$BEHAVIOR_TASK_INSTANCES_SHA256" \
    "$BEHAVIOR_ASSETS" "$BEHAVIOR_ASSETS_SHA256" <<'PY'
import hashlib, os, shutil, sys, zipfile
from pathlib import Path
from huggingface_hub import hf_hub_download

data = Path(os.environ["OMNIGIBSON_DATA_PATH"])
staging = Path(os.environ["BEHAVIOR_STAGING"])
revision = sys.argv[1]
# Upstream unpacks each zip into DATA_PATH/<dataset name without version>.
names = ["omnigibson-robot-assets", "2026-challenge-task-instances", "behavior-1k-assets"]


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 24), b""):
            digest.update(block)
    return digest.hexdigest()


for versioned, expected, name in zip(sys.argv[2::2], sys.argv[3::2], names):
    target = data / name
    if target.exists():
        print(f"present: {target}", flush=True)
        continue
    archive = hf_hub_download("behavior-1k/zipped-datasets", f"{versioned}.zip",
                              repo_type="dataset", revision=revision, local_dir=staging)
    actual = sha256(archive)
    if actual != expected:
        # A kept mismatched zip would be reused by the next run instead of downloaded again.
        Path(archive).unlink()
        raise SystemExit(f"{versioned}.zip at revision {revision} has sha256 {actual}, expected {expected}; "
                         f"deleted it and installed nothing for {target}")
    partial = data / f".{name}.partial"
    shutil.rmtree(partial, ignore_errors=True)
    with zipfile.ZipFile(archive) as z:
        bad = z.testzip()
        if bad is not None:
            raise SystemExit(f"Corrupt member {bad} in {archive}")
        z.extractall(partial)
    partial.rename(target)
    print(f"installed: {target}", flush=True)
    if os.environ.get("BEHAVIOR_KEEP_ZIPS") != "true":
        Path(archive).unlink()

from omnigibson.utils.asset_utils import download_key, get_key_path
download_key()
print(f"key: {get_key_path()} ({'present' if os.path.exists(get_key_path()) else 'MISSING'})")
PY
du -sh "$OMNIGIBSON_DATA_PATH"/* 2>/dev/null
