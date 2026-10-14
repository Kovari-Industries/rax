#!/usr/bin/env bash
# Download RoboDojo's simulator assets (about 66 GB with the git-lfs cache) at the revision pinned in
# dependencies.lock, into runtime/robodojo-cache, linked as RoboDojo/Assets. Needs git-lfs.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/robodojo_env.sh"
source "$ROBODOJO_PROJECT_ROOT/dependencies.lock"

cache_root="$ROBODOJO_RUNTIME_ROOT/robodojo-cache"
mkdir -p "$cache_root"

if [[ ! -e "$ROBODOJO_SOURCE/.cache" ]]; then
    ln -s "$cache_root" "$ROBODOJO_SOURCE/.cache"
elif [[ ! -L "$ROBODOJO_SOURCE/.cache" ]]; then
    echo "Refusing to relocate non-symlink $ROBODOJO_SOURCE/.cache" >&2
    exit 2
fi

export HF_REPO_ID="${HF_REPO_ID:-RoboDojo-Benchmark/RoboDojo}"
export HF_REVISION="${HF_REVISION:-$ROBODOJO_ASSETS_REVISION}"
export PATH="$ROBODOJO_ENV_ROOT/bin:$PATH"
# Upstream init_assets.sh clones the dataset's current head and then checks out HF_REVISION, which
# fails for an older pinned commit. Given an existing repository it fetches exactly that commit.
repo="$cache_root/robodojo_assets_repo"
if [[ ! -d "$repo/.git" ]]; then
    git init -q "$repo"
    git -C "$repo" remote add origin "https://huggingface.co/datasets/$HF_REPO_ID"
fi
bash "$ROBODOJO_SOURCE/scripts/init_assets.sh"
actual="$(git -C "$repo" rev-parse HEAD)"
[[ "$actual" == "$HF_REVISION" ]] || {
    echo "Assets are at $actual, expected $HF_REVISION; move $repo aside and rerun" >&2; exit 1;
}
echo "RoboDojo assets at $HF_REVISION: $ROBODOJO_SOURCE/Assets"
