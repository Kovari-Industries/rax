#!/usr/bin/env bash
# Install the HumanoidArena simulation assets and the released SONIC models, hash-verified.
#
#   bash scripts/setup_humanoid_assets.sh [--arena-zip PATH] [--sonic-dir PATH]
#
# Without options it downloads the pinned archives from upstream (dependencies.lock). A local
# copy is accepted only when its sha256 matches the pin, so the result is the same either way.
# Installs into runtime/humanoid-assets/{arena,sonic} and links
# HumanoidArena/isaaclab_twist2_g1/assets to the arena assets. Run on a compute node: the
# archive is 2.7 GB and extracts to 3.9 GB.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$PROJECT_ROOT/dependencies.lock"

arena_zip="" sonic_dir=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --arena-zip) arena_zip="$2"; shift 2 ;;
        --sonic-dir) sonic_dir="$2"; shift 2 ;;
        *) echo "usage: $0 [--arena-zip PATH] [--sonic-dir PATH]" >&2; exit 2 ;;
    esac
done

[[ -d "$PROJECT_ROOT/HumanoidArena/isaaclab_twist2_g1" ]] || { echo "Run scripts/bootstrap_sources.sh first" >&2; exit 2; }
ASSETS="$(readlink -f "$PROJECT_ROOT/runtime")/humanoid-assets"
mkdir -p "$ASSETS"
ASSETS="$(readlink -f "$ASSETS")"
STAGING="$ASSETS/.staging"
mkdir -p "$STAGING"

check() {  # check FILE SHA256
    [[ "$(sha256sum "$1" | cut -d' ' -f1)" == "$2" ]]
}

# ── Arena scene and robot assets ────────────────────────────────────────────────
if [[ -f "$ASSETS/arena/installed.json" ]] && grep -q "$HUMANOID_ARENA_ASSETS_SHA256" "$ASSETS/arena/installed.json"; then
    echo "arena assets: present"
else
    archive="$STAGING/humanoidarena-assets.zip"
    if [[ -n "$arena_zip" ]]; then
        check "$arena_zip" "$HUMANOID_ARENA_ASSETS_SHA256" || { echo "$arena_zip does not match the pinned sha256" >&2; exit 1; }
        archive="$arena_zip"
    elif ! { [[ -f "$archive" ]] && check "$archive" "$HUMANOID_ARENA_ASSETS_SHA256"; }; then
        curl -L --fail --retry 3 -o "$archive.partial" "$HUMANOID_ARENA_ASSETS_URL"
        check "$archive.partial" "$HUMANOID_ARENA_ASSETS_SHA256" || { echo "Downloaded archive does not match the pin" >&2; exit 1; }
        mv "$archive.partial" "$archive"
    fi
    rm -rf "$STAGING/arena"
    mkdir -p "$STAGING/arena"
    unzip -q "$archive" -d "$STAGING/arena"
    # The archive holds assets/{objects,robots}; install that directory.
    top="$STAGING/arena"
    [[ -d "$top/assets" ]] && top="$top/assets"
    [[ -d "$top/objects" && -d "$top/robots" ]] || { echo "Unexpected archive layout" >&2; exit 1; }
    [[ -e "$ASSETS/arena" ]] && chmod -R u+w "$ASSETS/arena"
    rm -rf "$ASSETS/arena"
    mv "$top" "$ASSETS/arena"
    rm -rf "$STAGING/arena"
    printf '{\n  "source": "%s",\n  "archive_sha256": "%s",\n  "installed_at": "%s"\n}\n' \
        "$HUMANOID_ARENA_ASSETS_URL" "$HUMANOID_ARENA_ASSETS_SHA256" "$(date -u +%FT%TZ)" > "$ASSETS/arena/installed.json"
    echo "arena assets: installed"
fi
link="$PROJECT_ROOT/HumanoidArena/isaaclab_twist2_g1/assets"
if [[ -L "$link" || ! -e "$link" ]]; then
    ln -sfn "$ASSETS/arena" "$link"
else
    echo "refusing to replace non-symlink $link" >&2
    exit 2
fi

# ── SONIC models (encoder, decoder, observation config; the planner is not used) ─────────
mkdir -p "$ASSETS/sonic"
declare -A pins=([model_encoder.onnx]=$SONIC_ENCODER_SHA256 [model_decoder.onnx]=$SONIC_DECODER_SHA256
                 [observation_config.yaml]=$SONIC_OBSERVATION_CONFIG_SHA256 [LICENSE]=$SONIC_LICENSE_SHA256)
for name in "${!pins[@]}"; do
    target="$ASSETS/sonic/$name"
    if [[ -f "$target" ]] && check "$target" "${pins[$name]}"; then
        continue
    fi
    if [[ -n "$sonic_dir" ]]; then
        cp "$sonic_dir/$name" "$target.partial"
    else
        curl -L --fail --retry 3 -o "$target.partial" \
            "https://huggingface.co/$SONIC_REPOSITORY/resolve/$SONIC_REVISION/$name"
    fi
    check "$target.partial" "${pins[$name]}" || { echo "$name does not match the pinned sha256" >&2; exit 1; }
    mv "$target.partial" "$target"
done
cat > "$ASSETS/sonic/manifest.json" <<EOF
{
  "repository": "$SONIC_REPOSITORY",
  "revision": "$SONIC_REVISION",
  "files": {
    "encoder": {"path": "model_encoder.onnx", "sha256": "$SONIC_ENCODER_SHA256"},
    "decoder": {"path": "model_decoder.onnx", "sha256": "$SONIC_DECODER_SHA256"},
    "observation_config": {"path": "observation_config.yaml", "sha256": "$SONIC_OBSERVATION_CONFIG_SHA256"},
    "license": {"path": "LICENSE", "sha256": "$SONIC_LICENSE_SHA256"}
  }
}
EOF
chmod a-w "$ASSETS"/sonic/{model_encoder.onnx,model_decoder.onnx,observation_config.yaml,LICENSE}
chmod -R a-w "$ASSETS/arena"/{objects,robots}
echo "sonic models: verified ($SONIC_REPOSITORY@$SONIC_REVISION)"
echo "HumanoidArena assets ready under $ASSETS"
