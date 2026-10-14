"""BEHAVIOR robot-only motion preview service: ``python -m services.behavior.preview_server``.

Runs in runtime/envs/behavior. One persistent process serves any number of preview runs
over the same msgpack RPC as the episode simulator.
"""

from __future__ import annotations

import argparse
import json


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=0)
    parser.add_argument("--output", help="owning output root; identifies the process for cleanup")
    args = parser.parse_args(argv)
    from services.behavior.preview_session import PreviewSession
    from services.simulator.rpc import serve

    session = PreviewSession()
    print(json.dumps({"event": "environment_loaded", "preview": True}), flush=True)
    try:
        serve(session, args.port)
    finally:
        import omnigibson as og
        session.env.close()
        og.shutdown()


if __name__ == "__main__":
    main()
