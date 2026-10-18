#!/usr/bin/env bash
# The HumanoidArena simulator "python" (HUMANOID_PYTHON's default): runs its arguments with the
# Isaac Sim interpreter of the simulator image (scripts/build_humanoid_image.sh).
#
# services/humanoid/process.py starts [HUMANOID_PYTHON, -m, services.humanoid.server, ...] with a
# working directory under the runtime and PYTHONPATH, CUDA_VISIBLE_DEVICES and XDG_RUNTIME_DIR set:
#   * the checkout, the runtime and artifact roots, the HumanoidArena checkout, its assets and the
#     working directory are mounted at their host paths, so every path the harness passes is valid;
#   * CUDA_VISIBLE_DEVICES (an ordinal or a UUID) selects the one GPU the container sees;
#   * Omniverse caches stay on local disk (their memory-mapped databases fail on network storage);
#   * host networking lets the harness reach the simulator's RPC port on 127.0.0.1.
# CONTAINER_CLI=podman runs the image with podman instead of Docker.
set -uo pipefail
HERE=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
PROJECT_ROOT=$(cd "$HERE/.." && pwd)
source "$PROJECT_ROOT/dependencies.lock"
CLI=${CONTAINER_CLI:-docker}
IMAGE=${HUMANOID_IMAGE_TAG:-$HUMANOID_IMAGE}
RUNTIME_ROOT=$(readlink -f "${HUMANOID_RUNTIME_ROOT:-${RAX_RUNTIME_ROOT:-$PROJECT_ROOT/runtime}}")
ARTIFACT_ROOT=$(readlink -f "${RAX_ARTIFACT_ROOT:-$RUNTIME_ROOT}")
ARENA=$(readlink -f "${HUMANOID_ARENA_ROOT:-$PROJECT_ROOT/HumanoidArena/isaaclab_twist2_g1}")
CACHE=${HUMANOID_CACHE:-/tmp/humanoid-cache-$USER}
mkdir -p "$CACHE"/{ov,nv,https} || { echo "humanoid_python.sh: cannot create $CACHE" >&2; exit 125; }
# Several simulators may run at once: the shader caches are shared, Kit's data and XDG runtime
# directories are private to each process.
INSTANCE=$(mktemp -d "$CACHE/inst.XXXXXX") || { echo "humanoid_python.sh: cannot create a directory under $CACHE" >&2; exit 125; }
mkdir -p "$INSTANCE/ovshare" "$INSTANCE/xdg"; chmod 700 "$INSTANCE/xdg"
trap 'rm -rf "$INSTANCE"' EXIT

# The GPU the harness chose (an ordinal or a UUID).
gpus=$(nvidia-smi --query-gpu=uuid --format=csv,noheader)
sel=${CUDA_VISIBLE_DEVICES:-0}; sel=${sel%%,*}
case "$sel" in
  GPU-*) uuid=$sel ;;
  ''|*[!0-9]*) uuid=$(head -1 <<< "$gpus") ;;
  *) uuid=$(sed -n "$((sel+1))p" <<< "$gpus") ;;
esac
grep -qx "$uuid" <<< "$gpus" || { echo "humanoid_python.sh: GPU $uuid is not on $(hostname)" >&2; exit 125; }
if [[ "$CLI" == podman ]]; then
  gpu=(--device "nvidia.com/gpu=$uuid")
  # Rootless podman keeps its state in the login session's runtime directory, not in the one the
  # harness gives the simulator (Kit's runtime directory is /tmp/xdg in the container).
  XDG_RUNTIME_DIR=/run/user/$(id -u)
  [[ -d "$XDG_RUNTIME_DIR" && -w "$XDG_RUNTIME_DIR" ]] || { XDG_RUNTIME_DIR=/tmp/podman-run-$(id -u); mkdir -p -m 700 "$XDG_RUNTIME_DIR"; }
  export XDG_RUNTIME_DIR
else
  gpu=(--gpus "device=$uuid")
fi
"$CLI" image inspect "$IMAGE" >/dev/null 2>&1 || { echo "humanoid_python.sh: image $IMAGE missing (scripts/build_humanoid_image.sh)" >&2; exit 127; }

envargs=()
for v in $(compgen -e | grep -E '^(HUMANOID_|RAX_|OMNI_|ACCEPT_EULA|PRIVACY_CONSENT|PYTHONUNBUFFERED)'); do
  envargs+=(-e "$v=${!v}")
done
cwd=$PWD
mounts=() roots=()
covered() { local d=$1 m; for m in "${roots[@]}"; do case "$d" in "$m"|"$m"/*) return 0 ;; esac; done; return 1; }
for d in "$PROJECT_ROOT" "$ARTIFACT_ROOT" "$RUNTIME_ROOT" "$(readlink -f "$RUNTIME_ROOT/humanoid-assets" 2>/dev/null)" \
         "$(dirname "$ARENA")" "$(readlink -f "$ARENA/assets" 2>/dev/null)" "$cwd"; do
  [[ -n "$d" ]] || continue
  d=$(readlink -f "$d" 2>/dev/null) || continue
  [[ -d "$d" && "$d" != / ]] || continue
  covered "$d" && continue
  roots+=("$d"); mounts+=(-v "$d:$d")
done

"$CLI" run --rm -i "${gpu[@]}" --shm-size=16g --network host --ulimit memlock=-1 --ulimit stack=67108864 \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e OMNI_KIT_ACCEPT_EULA=YES -e OMNI_KIT_ALLOW_ROOT=1 \
  -e PYTHONUNBUFFERED=1 -e XDG_RUNTIME_DIR=/tmp/xdg -e "PYTHONPATH=${PYTHONPATH:-$PROJECT_ROOT}:/opt/curobo" \
  "${envargs[@]}" \
  -v "$CACHE/ov:/root/.cache" -v "$CACHE/nv:/root/.nv" -v "$INSTANCE/ovshare:/root/.local/share/ov" \
  -v "$INSTANCE/xdg:/tmp/xdg" -v "$CACHE/https:/tmp/https" "${mounts[@]}" \
  -w "$cwd" --entrypoint /bin/bash "$IMAGE" -c '
# CDI (podman) mounts the host driver libraries at their host paths, /usr/lib64 on RHEL-family
# hosts, which the image does not search.
[ -e /usr/lib64/libcuda.so.1 ] && { echo /usr/lib64 > /etc/ld.so.conf.d/host-driver.conf; ldconfig; }
_icd=$(ls /etc/vulkan/icd.d/nvidia_icd*.json /usr/share/vulkan/icd.d/nvidia_icd*.json 2>/dev/null | head -1)
[ -n "$_icd" ] && export VK_DRIVER_FILES=$_icd VK_ICD_FILENAMES=$_icd
exec /isaac-sim/python.sh "$@"' _ "$@"
