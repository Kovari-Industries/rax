"""HumanoidArena simulator process: one Isaac Sim app, one task, episodes reset by seed.

Runs only in the HumanoidArena simulator interpreter (``HUMANOID_PYTHON``: Isaac Sim 6.1 with
the pinned Isaac Lab 2.3 and HumanoidArena sources, scripts/humanoid_python.sh). The host talks to
it over ``services.simulator`` RPC.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

MODES = {0: "exploration", 1: "official_test"}
SONIC_FILES = ("encoder", "decoder", "observation_config")
DEFAULT_EXPERIENCE = "/isaac-sim/apps/isaacsim.exp.base.python.kit"


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--port", type=int, default=0)
    parser.add_argument("--collection", type=int, choices=sorted(MODES), required=True,
                        help="0: exploration layouts; 1: official test episodes (held out)")
    parser.add_argument("--seed", type=int, required=True, help="Environment seed of the first episode")
    parser.add_argument("--arena-root", required=True, help="The pinned isaaclab_twist2_g1 directory")
    parser.add_argument("--sonic-manifest", required=True, help="SONIC release manifest (paths and sha256)")
    parser.add_argument("--episode-timeout-seconds", type=int, default=0)
    parser.add_argument("--depth", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args(argv)


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def frozen_sonic(manifest_path):
    """The released SONIC files, verified against their pinned hashes before any model loads."""
    manifest_path = Path(manifest_path)
    manifest = json.loads(manifest_path.read_text())
    files = {}
    for key in SONIC_FILES:
        entry = manifest["files"][key]
        path = (manifest_path.parent / entry["path"]).resolve()
        actual = sha256(path)
        if actual != entry["sha256"]:
            raise ValueError(f"SONIC {key} does not match its pinned hash: {path}")
        files[key] = path
    return {**files, "sha256": {key: manifest["files"][key]["sha256"] for key in SONIC_FILES}}


def main(argv=None):
    args = parse_args(argv)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    arena_root = Path(args.arena_root).resolve()
    if str(arena_root) not in sys.path:
        sys.path.insert(0, str(arena_root))  # tasks, action_provider, common_env_objects, tools, ...
    sonic = frozen_sonic(args.sonic_manifest)
    from isaaclab.app import AppLauncher
    experience = os.environ.get("HUMANOID_KIT_EXPERIENCE",
                                DEFAULT_EXPERIENCE if Path(DEFAULT_EXPERIENCE).is_file() else "")
    launcher = AppLauncher(headless=True, enable_cameras=True, **({"experience": experience} if experience else {}))
    app = launcher.app

    from services.humanoid.sim.session import HumanoidSession
    from services.simulator.rpc import serve

    session = None
    try:
        session = HumanoidSession(task=args.task, output=output, mode=MODES[args.collection],
                                  include_depth=args.depth, arena_root=arena_root, sonic=sonic, seed=args.seed)
        print(json.dumps({"event": "environment_loaded", "task": args.task, "mode": MODES[args.collection]}), flush=True)
        serve(session, args.port, episode_timeout_seconds=args.episode_timeout_seconds)
    finally:
        try:
            if session is not None:
                if session.episode_id is not None and not session.finish_reason:
                    session.finish("process_exit")
                session.close()
        finally:
            app.close()


if __name__ == "__main__":
    main()
