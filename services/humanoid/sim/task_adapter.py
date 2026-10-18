"""Task lifecycle and success around the unchanged Arena implementations."""
import importlib
from pathlib import Path


def missing_scene_assets(cfg, arena_root):
    paths = []
    for value in vars(cfg.scene).values():
        path = getattr(getattr(value, "spawn", None), "usd_path", None)
        if path and str(path).startswith(str(Path(arena_root)/"assets")) and not Path(path).is_file():
            paths.append(str(path))
    return sorted(set(paths))


class TaskAdapter:
    def __init__(self, task, env, cfg):
        self.task, self.env, self.cfg = task, env, cfg
        self.door = None
        self.prim_repairs = []
        if task.slug == "opendoor":
            # The inference profile spawns the door unmanaged: restore it at episode boundaries.
            from .door_reset import DoorEpisodeReset
            self.door = DoorEpisodeReset(env, cfg)

    def reset(self):
        manager = getattr(self.cfg, "event_manager", None)
        if manager is not None:
            manager.trigger("reset_all_self", self.env)
            # Upstream's prim poses are kept exactly; prim_reset.py reports them and flags the
            # static colliders PhysX must re-read (Isaac Sim 6.1 does not follow USD edits).
            from .prim_reset import upstream_prim_resets
            self.prim_repairs = upstream_prim_resets(self.env, self.cfg)
        else:
            import torch
            from isaaclab.envs.mdp import reset_scene_to_default
            reset_scene_to_default(self.env, torch.arange(self.env.num_envs, device=self.env.device))
        if self.door is not None:
            self.door.restore()
            self.cfg.reset_open_door_latch(self.env, reason="episode_reset")

    def needs_physics_restart(self):
        """A static collider written by upstream's reset is only re-read when PhysX parses again."""
        return any(item.get("upstream_written") and item.get("static_collider") for item in self.prim_repairs)

    def success(self):
        if self.task.slug in {"football_single", "doubledesk"}:
            # Reward computation has already run exactly once after this tick. Do not call
            # stateful reward functions again to obtain success. Isaac Lab assigns
            # env.reward_buf only inside env.step(), which the SONIC path bypasses; the
            # manager keeps the total it just computed.
            buffer = getattr(self.env, "reward_buf", None)
            if buffer is None:
                buffer = self.env.reward_manager._reward_buf
            return bool(buffer[0].item() > 1e-6)
        module = importlib.import_module(f"tasks.g1_tasks.{self.task.package}.mdp.rewards")
        return bool(module.compute_success_mask(self.env)[0].item())

    def layout_objects(self):
        """Operator-only reset layout (object poses), for the layout fingerprint."""
        from common_env_objects import collect_recordable_env_object_states
        from services.humanoid.sim.session import plain
        objects = plain(collect_recordable_env_object_states(self.env, self.cfg))
        if any(v is None for v in objects.values()):
            raise RuntimeError("Cannot fingerprint actual reset object layout")
        for kind in ("rigid_objects", "articulations"):
            for name, asset in getattr(self.env.scene, kind, {}).items():
                if name != "robot":
                    objects[f"physical:{name}"] = plain(asset.data.root_state_w[0, :7])
        # Upstream reads prim orientations from scaled matrices (|q| != 1 on centimetre assets);
        # add the composed pose with scale removed for every prim-driven record.
        for item in self.prim_repairs:
            objects[f"prim:{item['record']}"] = {"position": item["realized_position_m"],
                                                 "orientation_wxyz": item["realized_orientation_wxyz"]}
        return objects
