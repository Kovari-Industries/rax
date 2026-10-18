"""DexJoCo task catalog, official evaluation layouts and budgets (stdlib only).

Each official task has two harness tasks: ``<task>`` uses the official ``configs/rand_obj``
evaluation config and ``<task>_random`` the official ``configs/rand_full`` one (camera, lighting
and table texture also randomized). The official evaluation (``dexjoco-openpi-eval``) builds one
environment per seed S in {0, 1, 2} and plays 50 episodes in a row, so its episode k is the
(k+1)-th reset of the environment seeded with S. Formal layouts (collection 1) are exactly those
150 official episodes; exploration layouts (collection 0) are first resets of other seeds.
"""

from __future__ import annotations

import os
from functools import lru_cache
from pathlib import Path

EXPLORATION_COLLECTION, FORMAL_COLLECTION = 0, 1
OFFICIAL_SEEDS = (0, 1, 2)
OFFICIAL_EPISODES_PER_SEED = 50
FORMAL_LAYOUTS = len(OFFICIAL_SEEDS) * OFFICIAL_EPISODES_PER_SEED
# Exploration layout i is the first reset of seed EXPLORATION_SEED_BASE + i, never an official seed.
EXPLORATION_SEED_BASE = 1000
RANDOM_SUFFIX = "_random"

# Official task name -> (robot type, the environment's own episode cap in control steps:
# `env_step >= cap` sets terminated in each official env's step()).
OFFICIAL_TASKS = {
    "hammer_nail": ("single_arm", 1000),
    "click_mouse": ("single_arm", 1200),
    "pick_bucket": ("single_arm", 1200),
    "pinch_tongs": ("single_arm", 1000),
    "fold_glasses": ("single_arm", 1200),
    "water_plant": ("single_arm", 1000),
    "bimanual_unlock_ipad": ("dual_arm", 1200),
    "bimanual_hanoi": ("dual_arm", 1500),
    "bimanual_assembly": ("dual_arm", 1500),
    "bimanual_microwave_cook": ("dual_arm", 1100),
    "bimanual_photograph": ("dual_arm", 1000),
}
# Steps the official evaluation runs after reset before the policy acts (they count toward the
# environment's cap): click_mouse aligns with its dataset by 30 fixed steps.
PREROLL_STEPS = {"click_mouse": 30}
# The official env paces step() to 30 Hz of wall clock; each step is 10 physics substeps of 2 ms.
CONTROL_HZ = 30
PHYSICS_SECONDS_PER_STEP = 0.02


def project_root() -> Path:
    return Path(__file__).resolve().parents[2]


def source_root() -> Path:
    """The pinned official checkout (scripts/bootstrap_sources.sh)."""
    return Path(os.environ.get("DEXJOCO_SOURCE") or project_root() / "DexJoCo").resolve()


def task_names():
    return sorted([*OFFICIAL_TASKS, *(f"{name}{RANDOM_SUFFIX}" for name in OFFICIAL_TASKS)])


def official_task(task):
    """Harness task -> (official env name, official config regime)."""
    if task in OFFICIAL_TASKS:
        return task, "rand_obj"
    if task.endswith(RANDOM_SUFFIX) and task[: -len(RANDOM_SUFFIX)] in OFFICIAL_TASKS:
        return task[: -len(RANDOM_SUFFIX)], "rand_full"
    raise ValueError(f"Unknown DexJoCo task: {task!r}; supported: {', '.join(task_names())}")


@lru_cache(maxsize=None)
def official_config(task):
    """The official evaluation config (configs/<regime>/<env>.yaml) of a harness task."""
    import yaml

    name, regime = official_task(task)
    path = source_root() / "configs" / regime / f"{name}.yaml"
    config = yaml.safe_load(path.read_text())
    if config.get("env_name") != name:
        raise ValueError(f"{path} names env {config.get('env_name')!r}, expected {name!r}")
    return config


def task_info(task):
    name, regime = official_task(task)
    robot_type, cap = OFFICIAL_TASKS[name]
    return {"task": task, "env_name": name, "regime": regime, "robot_type": robot_type,
            "dual_arm": robot_type == "dual_arm", "env_step_cap": cap,
            "preroll_steps": PREROLL_STEPS.get(name, 0)}


def native_step_limit(task):
    """Steps the agent may take: the env's own cap minus the official pre-roll."""
    info = task_info(task)
    return info["env_step_cap"] - info["preroll_steps"]


def action_dim(task):
    return 44 if task_info(task)["dual_arm"] else 22


def state_dim(task):
    return 46 if task_info(task)["dual_arm"] else 23


def layout(task, collection, layout_id):
    """A harness layout -> the official environment seed and how many resets reach it."""
    official_task(task)
    if not isinstance(layout_id, int) or isinstance(layout_id, bool) or layout_id < 0:
        raise ValueError(f"layout ids are nonnegative integers, got {layout_id!r}")
    if collection == FORMAL_COLLECTION:
        if layout_id >= FORMAL_LAYOUTS:
            raise ValueError(f"formal layout {layout_id} is outside the {FORMAL_LAYOUTS} official evaluation "
                             f"episodes (0..{FORMAL_LAYOUTS - 1})")
        return {"seed": OFFICIAL_SEEDS[layout_id // OFFICIAL_EPISODES_PER_SEED],
                "episode": layout_id % OFFICIAL_EPISODES_PER_SEED}
    if collection == EXPLORATION_COLLECTION:
        return {"seed": EXPLORATION_SEED_BASE + layout_id, "episode": 0}
    raise ValueError("DexJoCo layout collections are 0 (exploration) and 1 (official evaluation)")


def validate_layouts(task, collection, layout_ids):
    for layout_id in layout_ids:
        layout(task, collection, layout_id)


# Wall-clock model: the official step paces to 30 Hz; agents think between calls. Calibrated
# by scripts/dexjoco_doctor.py (see docs/DEXJOCO_BACKEND.md).
STARTUP_SECONDS = 300
EPISODE_BASE_SECONDS = 600
SECONDS_PER_STEP = 5


def task_timeouts(task):
    steps = native_step_limit(task)
    return {"native_step_limit": steps, "episode_seconds": EPISODE_BASE_SECONDS + steps * SECONDS_PER_STEP,
            "startup_seconds": STARTUP_SECONDS}
