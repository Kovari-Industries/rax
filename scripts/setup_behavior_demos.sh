#!/usr/bin/env bash
# Cache 1-3 official BEHAVIOR 2026 teleoperation demos per task as head-camera keyframes
# (runtime/reference-demos/behavior, about 2 MB per demo). Resumable: cached tasks are kept.
# The demos are part of the BEHAVIOR-1K dataset accepted by setup_behavior_assets.sh.
# Usage: bash scripts/setup_behavior_demos.sh [--episodes 2] [--tasks a,b] [--workers 4]
set -euo pipefail
here="$(dirname "${BASH_SOURCE[0]}")"
source "$here/behavior_env.sh"
source "$here/../dependencies.lock"
exec "$BEHAVIOR_PYTHON" "$here/fetch_behavior_demos.py" --revision "$BEHAVIOR_DEMOS_REVISION" \
    --root "$BEHAVIOR_RUNTIME_ROOT/reference-demos/behavior" "$@"
