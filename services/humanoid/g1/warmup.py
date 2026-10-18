"""Build the G1 planner solvers once (``python -m services.humanoid.g1.warmup``).

``--prebuild`` (installer only, needs write access to the package directory) stores every
compiled cuRobo NVRTC kernel and warp module in the kernel cache so that later
processes, including read-only sandboxed ones, load kernels instead of compiling them. The
solver variants share kernel instantiations, so a representative subset covers all of them
(the validation script reports any kernel that still had to be compiled).
"""
from __future__ import annotations

import argparse
import json
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prebuild", action="store_true")
    parser.add_argument("--quick", action="store_true", help="right-arm palm IK only")
    args = parser.parse_args()
    from . import kernel_cache
    kernel_cache.install(write_prebuilt=args.prebuild)
    from services.humanoid import g1
    started = time.perf_counter()
    report = {"solvers": []}
    t0 = time.perf_counter()
    g1.forward_kinematics({}, backend="curobo")
    report["fk_curobo_s"] = round(time.perf_counter() - t0, 3)
    t0 = time.perf_counter()
    g1.check_collision({}, obstacles=[{"cuboid": {"dims": [0.4, 0.8, 0.04],
                                                   "pose_wxyz": [0.6, 0.0, 0.0, 1, 0, 0, 0]}}])
    report["collision_s"] = round(time.perf_counter() - t0, 3)
    if args.quick:
        report["solvers"] += g1.warmup(sides=("right",))
    else:
        report["solvers"] += g1.warmup(sides=("right",), use_waist=(False, True), position_only=(False, True))
        report["solvers"] += g1.warmup(sides=("left",))
        report["solvers"] += g1.warmup(sides=("right",), end_effectors=("wrist", "hand_center"))
        report["solvers"] += g1.warmup(sides=("right",), kinds=("plan",), position_only=(False, True))
    report["total_s"] = round(time.perf_counter() - started, 3)
    report["kernel_cache"] = dict(kernel_cache.STATS)
    print(json.dumps(report, indent=1))


if __name__ == "__main__":
    main()
