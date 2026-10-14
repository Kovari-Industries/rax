"""BEHAVIOR-1K simulator process: one OmniGibson app, one task instance, one episode.

Runs only in ``runtime/envs/behavior``. The environment is constructed by the official
2026 ``BatchedEvaluator`` with the same configuration ``omnigibson.eval.eval`` builds
(partial scene load, human-demo-derived step limit, full-resolution RGB-D wrapper, fixed
evaluation seed), with a zero-action local policy because actions arrive over RPC.
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

MODES = {0: "train", 1: "public_test"}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--port", type=int, default=0)
    parser.add_argument("--collection", type=int, choices=sorted(MODES), required=True,
                        help="0: train instances (exploration); 1: public test instances (held out)")
    parser.add_argument("--instruction", default=None)
    parser.add_argument("--episode-timeout-seconds", type=int, default=0)
    parser.add_argument("--depth", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args(argv)


def preload_planner_libraries():
    """Import the pinned warp and cuRobo before the Kit app starts: Isaac Sim bundles an older
    omni.warp.core whose submodules would otherwise shadow warp-lang's and break cuRobo."""
    import warp  # noqa: F401
    import warp.context  # noqa: F401
    import warp.torch  # noqa: F401
    import curobo.geom.sdf.world_mesh  # noqa: F401


def pin_render_gpu():
    """Render on the simulator GPU. OmniGibson passes OMNIGIBSON_GPU_ID as both the CUDA
    physics device and Kit's Vulkan renderer device; under CUDA_VISIBLE_DEVICES only the
    former is remapped, so the renderer would land on physical GPU 0 and PhysX would find
    no matching CUDA device. ROBODOJO_RENDER_GPU carries the physical index instead."""
    gpu = os.environ.get("ROBODOJO_RENDER_GPU")
    if gpu is None:
        return
    import isaacsim

    base = isaacsim.SimulationApp

    class PinnedSimulationApp(base):
        def __init__(self, launch_config=None, *args, **kwargs):
            super().__init__({**(launch_config or {}), "active_gpu": int(gpu)}, *args, **kwargs)

    isaacsim.SimulationApp = PinnedSimulationApp


def build_evaluator(task, mode):
    from omegaconf import OmegaConf
    from omnigibson.macros import gm

    pin_render_gpu()
    gm.HEADLESS = True
    gm.RENDER_VIEWER_CAMERA = False
    from omnigibson.eval.evaluator import BatchedEvaluator
    from omnigibson.eval.utils.eval_utils import DEFAULT_EVAL_SEED, seed_everything

    seed = seed_everything(DEFAULT_EVAL_SEED)
    cfg = OmegaConf.create({
        "env_wrapper": {"_target_": "omnigibson.eval.wrappers.RGBDFullResWrapper"},
        "policy_name": "local",
        "model": {"_target_": "omnigibson.eval.policies.LocalPolicy", "action_dim": None},
        "headless": True,
        "partial_scene_load": True,
        "max_steps": None,
        "write_video": False,
        "mode": mode,
        "seed": seed,
        "num_envs": 1,
        "task": {"name": task},
        "robot": None,
    })
    return BatchedEvaluator(cfg)


def main(argv=None):
    args = parse_args(argv)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    mode = MODES[args.collection]
    preload_planner_libraries()
    evaluator = build_evaluator(args.task, mode)

    from services.behavior.session import BehaviorSession
    from services.simulator.rpc import serve

    session = BehaviorSession(evaluator, output, args.task, mode=mode, include_depth=args.depth,
                              instruction=args.instruction)
    print(json.dumps({"event": "environment_loaded", "task": args.task, "mode": mode}), flush=True)
    try:
        serve(session, args.port, episode_timeout_seconds=args.episode_timeout_seconds)
    finally:
        try:
            if session.episode_id is not None and session.truth_log is not None:
                session.finish(session.finish_reason or "process_exit")
        finally:
            import omnigibson as og
            evaluator.env.close()
            og.shutdown()


if __name__ == "__main__":
    main()
