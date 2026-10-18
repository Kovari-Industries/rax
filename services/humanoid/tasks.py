"""HumanoidArena tasks with official test episodes; importable without the simulator.

The pinned upstream SONIC evaluation (``script/eval_scripts/sonic/eval_vla_suite*.py`` with
``tasks/common_test_config/base_test/<task>_sonic_test.yaml``) runs group seeds [0, 1, 2] with
20 repeats each: 60 episodes per task, group-major. Each episode seed is the last four bytes of
sha256("<gym id>|<group seed>|<repeat>"), big-endian, masked to a positive int32, and the
environment resets objects from that seed (``object_reset_seed_source: env_seed``). Upstream
ends an episode after ``MAX_STEPS`` 50 Hz control steps.

Layout collections: 1 = the 60 official test episodes (formal; layout id = episode index);
0 = exploration, whose layout ids map to seeds drawn from the same distribution by a fixed
derivation that never yields an official seed.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math

EXPLORATION_COLLECTION, FORMAL_COLLECTION = 0, 1
GROUP_SEEDS = (0, 1, 2)
REPEATS_PER_SEED = 20
FORMAL_EPISODES = len(GROUP_SEEDS) * REPEATS_PER_SEED
CONTROL_HZ = 50


@dataclass(frozen=True)
class Task:
    slug: str
    gym_id: str
    package: str          # Upstream task package under isaaclab_twist2_g1/tasks.
    profile: str          # Upstream task_runtime_profiles entry.
    base_test: str        # Pinned base_test YAML (official test episodes).
    max_steps: int        # Upstream SONIC eval-script MAX_STEPS (50 Hz control steps).
    objective: str
    success: str

    @property
    def runtime_profile(self):
        # Pinned Arena rejects explicit inference profiles for other tasks.
        return "inference" if self.slug in {"opendoor", "warehouse_navigation"} else "auto"

    def profile_file(self):
        return f"{self.profile}_sonic.yaml"


TASKS = {task.slug: task for task in (
    Task("opendoor", "Isaac-Move-Open-Door-G129-Dex3-Wholebody",
         "move_open_door_g1_29dof_dex3_wholebody", "opendoor", "open_door_sonic_test.yaml", 1800,
         "Physically operate the handle or latch and open the door leaf while remaining standing.",
         "Arena evaluator: the robot stands, the latch is unlocked and the door leaf is open at least 60 degrees."),
    Task("boxing_bag", "Isaac-Move-Boxing-Bag-G129-Dex3-Wholebody",
         "move_boxing_bag_g1_29dof_dex3_wholebody", "boxing_bag", "boxing_sonic_test.yaml", 900,
         "Move a punching body into the boxing bag's designated target region.",
         "Arena compute_success_mask, with unchanged task thresholds."),
    Task("pickplace_box", "Isaac-Move-PickPlace-Box-G129-Dex3-Wholedoby",
         "move_pickplace_box_g1_29dof_dex3_wholedoby", "pickplace_box", "pp_box_sonic_test.yaml", 1450,
         "Pick up the box and place it on the designated support surface.",
         "Arena compute_success_mask, with unchanged task thresholds."),
    Task("doubledesk", "Isaac-Move-PickPlace-DoubleDesk-G129-Dex3-Wholebody",
         "move_pickplace_doubledesk_g1_29dof_dex3_wholebody", "doubledesk", "doubledesk_sonic_test.yaml", 2000,
         "Pick up the hammer and place it in the basket on the other desk.",
         "Arena sparse placement reward is positive; placement on the ground does not count."),
    Task("sit_sofa", "Isaac-Move-Sit-Sofa-G129-Dex3-Wholebody",
         "move_sit_sofa_g1_29dof_dex3_wholebody", "livingroom_sitsofa", "sit_sofa_sonic_test.yaml", 2000,
         "Sit on the sofa in the task's target region and posture.",
         "Arena compute_success_mask, with unchanged task thresholds."),
    Task("football_single", "Isaac-Move-Football-Single-G129-Dex3-Wholebody",
         "move_football_single_g1_29dof_dex3_wholebody", "football_single", "football_single_sonic_test.yaml", 2000,
         "Move the football into the single goal.",
         "Arena sparse goal reward is positive."),
    Task("warehouse_navigation", "Isaac-Move-SmallWarehouse-VisionNavigation-G129-Dex3-Wholebody",
         "move_small_warehouse_vision_navigation_g1_29dof_dex3_wholebody", "small_warehouse_vision_navigation",
         "vision_navi_sonic_test.yaml", 1800,
         "Navigate through the warehouse to the designated destination.",
         "Arena compute_success_mask, with unchanged task thresholds."),
)}


def task_names():
    return sorted(TASKS)


def task_info(task):
    if task not in TASKS:
        raise ValueError(f"Unknown HumanoidArena task: {task!r}; supported: {', '.join(sorted(TASKS))}")
    return TASKS[task]


def native_step_limit(task):
    return task_info(task).max_steps


def episode_seed(gym_id, group_seed, repeat_idx):
    """Upstream ``_derive_episode_seed``, byte for byte."""
    payload = f"{gym_id}|{int(group_seed)}|{int(repeat_idx)}".encode("utf-8")
    return int.from_bytes(hashlib.sha256(payload).digest()[-4:], "big", signed=False) & 0x7FFFFFFF


def official_seeds(task):
    """The 60 official test episode seeds of a task, in upstream episode order."""
    gym_id = task_info(task).gym_id
    return tuple(episode_seed(gym_id, group, repeat) for group in GROUP_SEEDS for repeat in range(REPEATS_PER_SEED))


def exploration_seed(task, layout_id):
    """Seed of an exploration layout: same distribution as the tests, never an official seed."""
    gym_id, official = task_info(task).gym_id, set(official_seeds(task))
    for salt in range(64):
        payload = f"rax-exploration|{gym_id}|{int(layout_id)}|{salt}".encode("utf-8")
        seed = int.from_bytes(hashlib.sha256(payload).digest()[-4:], "big", signed=False) & 0x7FFFFFFF
        if seed and seed not in official:
            return seed
    raise RuntimeError("No exploration seed outside the official seeds")  # Unreachable in practice.


def layout_seed(task, collection, layout_id):
    """Environment seed of a harness layout (collection, layout id)."""
    if not isinstance(layout_id, int) or isinstance(layout_id, bool) or layout_id < 0:
        raise ValueError(f"layout ids are nonnegative integers, got {layout_id!r}")
    if collection == FORMAL_COLLECTION:
        seeds = official_seeds(task)
        if layout_id >= len(seeds):
            raise ValueError(f"layout {layout_id} is outside the {len(seeds)} official test episodes of {task}")
        return seeds[layout_id]
    if collection == EXPLORATION_COLLECTION:
        return exploration_seed(task, layout_id)
    raise ValueError("HumanoidArena layout collections are 0 (exploration) and 1 (official test episodes)")


def validate_layouts(task, collection, layout_ids):
    for layout_id in layout_ids:
        layout_seed(task, collection, layout_id)


# Wall-clock model: startup of one Isaac Sim 6.1 process plus a per-step allowance for physics,
# rendering and the agent's own time. Calibrate with scripts/humanoid_doctor.py.
STARTUP_SECONDS = 600
SECONDS_PER_STEP = 1.0


def task_timeouts(task):
    steps = native_step_limit(task)
    episode = STARTUP_SECONDS + math.ceil(steps * SECONDS_PER_STEP)
    return {"native_step_limit": steps, "episode_seconds": episode, "session_seconds": 24 * episode,
            "startup_seconds": STARTUP_SECONDS}
