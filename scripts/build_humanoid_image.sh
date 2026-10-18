#!/usr/bin/env bash
# Build the HumanoidArena simulator image (docker/humanoid/Dockerfile) from the pins in
# dependencies.lock. Accept the NVIDIA Isaac Sim EULA first; the base image is pulled from NGC.
# Usage: bash scripts/build_humanoid_image.sh --accept-nvidia-eula [--tag TAG]
# CONTAINER_CLI=podman builds with podman instead of Docker.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$PROJECT_ROOT/dependencies.lock"
CLI="${CONTAINER_CLI:-docker}"
tag="$HUMANOID_IMAGE"
[[ "${1:-}" == "--accept-nvidia-eula" ]] || { echo "Pass --accept-nvidia-eula to accept the NVIDIA Isaac Sim EULA." >&2; exit 2; }
shift
[[ "${1:-}" == "--tag" ]] && { tag="$2"; shift 2; }
[[ $# -eq 0 ]] || { echo "usage: $0 --accept-nvidia-eula [--tag TAG]" >&2; exit 2; }

patch="$PROJECT_ROOT/patches/isaaclab-isaacsim61.patch"
[[ "$(sha256sum "$patch" | cut -d' ' -f1)" == "$ISAACLAB_PATCH_SHA256" ]] || { echo "$patch differs from its pin" >&2; exit 1; }
context="$(mktemp -d)"
trap 'rm -rf "$context"' EXIT
cp "$PROJECT_ROOT/docker/humanoid/Dockerfile" "$PROJECT_ROOT/docker/humanoid/constraints.txt" "$patch" "$context/"
"$CLI" build $([[ "$CLI" == podman ]] && echo --format docker) \
    --build-arg ISAACSIM_IMAGE="$ISAACSIM_IMAGE" \
    --build-arg ISAACLAB_URL="$ISAACLAB_URL" --build-arg ISAACLAB_COMMIT="$ISAACLAB_COMMIT" \
    --build-arg CUROBO_URL="$CUROBO_URL" --build-arg CUROBO_COMMIT="$CUROBO_COMMIT" \
    -t "$tag" "$context"
echo "HumanoidArena simulator image ready: $tag"
