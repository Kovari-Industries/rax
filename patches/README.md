# RoboDojo patches

`scripts/bootstrap_sources.sh` clones RoboDojo into `RoboDojo/` at the commit pinned in
`dependencies.lock` and applies these two patches to it. Task rewards, success checks,
thresholds, layouts and assets are unchanged.

## `robodojo.patch`

What the harness needs from the simulator:

- **Cameras** (`env/camera_manager/camera_manager.py`): the intrinsic matrix and the
  camera-to-world pose are read from the live optical camera prim, so they include
  Isaac's square-pixel aperture adjustment and any fixed mount-to-optics offset. The
  configured formula stays as the fallback, now with separate horizontal and vertical
  focal lengths.
- **Planner** (`env/planner_manager/curobo_planner.py`): a failed plan reports its
  `failure_stage` (inverse kinematics or graph search, or trajectory optimization), a
  `reason` and the smallest position and rotation errors reached; every plan reports its
  `interpolation_dt`. It reads the cuRobo v2 kinematics schema, falls back to the robot
  directory for missing asset and URDF paths, treats a missing `frame_bias` as zero, and
  plans to a 1 mm position and 0.005 orientation tolerance.
- **Robot** (`env/robot_manager/robot_class/x5.py`): when the downloaded robot assets
  have no `curobo.yml`, the checkout's vendored `third_party/curobo/robot/x5_v2/curobo.yml`
  is used.
- **Evaluation** (`src/eval_client/eval_env.py`): an `enforce_step_limit` switch (on by
  default) that can lift only the step limit while keeping the task's completion and
  failure checks. The harness keeps the native step limit on in every episode.

## `robodojo-pr48-drive-reset.patch`

A backport of upstream
[RoboDojo PR #48](https://github.com/RoboDojo-Benchmark/RoboDojo/pull/48) (commit
`487e425`). For articulations configured with `reset_drive: true`, such as the button in
Swap Blocks and Press by Number, it restores the authored spring drive targets in PhysX,
not only in USD. Positions start from the limit-clipped targets, and angular targets are
converted from degrees to radians. A missing or ambiguous target mapping keeps the
previous behavior. Revisit the backport when the upstream pin moves.

## Applying and checking

Each patch is pinned by its SHA-256 in `dependencies.lock`
(`ROBODOJO_PATCH_SHA256`, `ROBODOJO_DRIVE_RESET_PATCH_SHA256`). The bootstrap refuses a
patch whose checksum differs, skips a patch that is already applied, and otherwise
applies it after a dry run, so rerunning it is safe. `bash scripts/robodojo_doctor.sh`
reports whether both are applied.

To change a patch, apply it alone to a clean checkout of the pinned commit, edit,
regenerate it with `git diff`, and update its checksum in `dependencies.lock`.

`local_tests/unit/test_articulation_drive_reset.py` checks the drive-target mapping of
the second patch against the pinned checkout.
