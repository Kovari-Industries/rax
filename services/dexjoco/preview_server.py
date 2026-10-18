"""DexJoCo robot-only motion preview service: ``python -m services.dexjoco.preview_server``.

Runs in runtime/envs/dexjoco. One persistent process per backend serves any number of preview
runs of one task's robot over the same msgpack RPC as the episode simulator.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", required=True)
    parser.add_argument("--port", type=int, default=0)
    parser.add_argument("--output", help="owning output root; identifies the process for cleanup")
    args = parser.parse_args(argv)
    os.environ.setdefault("MUJOCO_GL", "egl")
    from services.dexjoco.preview_session import PreviewSession
    from services.simulator.rpc import serve

    session = PreviewSession(args.task)
    print(json.dumps({"event": "environment_loaded", "preview": True, "task": args.task}), flush=True)
    try:
        serve(session, args.port)
    finally:
        session.close()


if __name__ == "__main__":
    main()
