"""Trusted auto-research backend for HumanoidArena; same interface as controller NativeBackend.

Runs in the host harness interpreter and never imports Isaac Sim. Each episode is a fresh
simulator process (``services.humanoid.process``); exploration uses fresh layouts of the test
distribution (collection 0) and formal uses the official test episodes (collection 1).
"""

from __future__ import annotations

import base64
import json
import time
import uuid
from pathlib import Path

import numpy as np

from services.artifact_io import write_artifacts
from services.humanoid import robot as R
from services.humanoid import tools as T
from services.humanoid.process import HumanoidProcess
from services.humanoid.tasks import layout_seed, task_info, task_timeouts
from services.mcp_reply import _agent_safe, _depth_png_data, _jsonable, _png_data


class HumanoidBackend:
    def __init__(self, config):
        from services.mcp_contract import Contract

        self.config = config
        task_info(config.task)
        root = Path(config.root) / "native"
        root.mkdir(mode=0o700, exist_ok=True)
        self.contract = Contract.from_config(config, isolated=True)
        self.observation_profile = self.contract.observation_profile
        self.frame_workspace = root / "workspace"
        self.frame_root = self.frame_workspace / "runtime" / "frames"
        self.sim = HumanoidProcess(output_root=root / "results")
        self._evaluation = None
        self._raw = None  # Last raw observation (host memory only), for pixel geometry.
        self._environment_spec = None

    # ------------------------------------------------------------------ tools
    def tools(self):
        return [t for t in self.contract.select(T.tool_definitions()) if t["name"] in self.contract.robot_names]

    @staticmethod
    def validate(name, arguments):
        T.validate_request(name, arguments)

    def _geometry(self):
        return self.contract.observation.geometry

    def call(self, name, arguments):
        self.contract.require_robot(name, arguments)
        T.validate_request(name, arguments)
        if name == T.STATUS:
            return self._status(), []
        sim = self.sim
        if not sim.active:
            raise RuntimeError("No active HumanoidArena episode; start an episode first")
        before = (sim.episode_id, sim.step_id)
        if name == T.OBSERVE:
            return self._observation_packet(sim.request("observe"))
        if name == T.PIXEL_TO_POSITION:
            if not self._geometry():
                raise PermissionError("Pixel-to-position is unavailable with RGB-only observations")
            return self._pixel_to_position(arguments), []
        try:
            if name == T.STEP:
                result = sim.request("chunk_step", actions=np.asarray(arguments["actions"], dtype=np.float32))
            elif name == T.FREE_SPACE_MOVE:
                result = sim.request("plan_arm", **arguments)
            elif name == T.EXECUTE_PLAN:
                result = sim.request("execute_motion_plan", motion_plan_id=arguments["motion_plan_id"])
            else:
                raise ValueError(f"Unknown HumanoidArena tool: {name}")
        finally:
            if (sim.episode_id, sim.step_id) != before:
                self._evaluation = None
        return self._motion_packet(result)

    # ---------------------------------------------------------------- packets
    def _status(self):
        sim = self.sim
        metadata = sim.metadata or {}
        return {"active": sim.active, "finished": sim.finished, "task": self.config.task,
                "episode_id": sim.episode_id, "step_id": sim.step_id if sim.episode_id else None,
                "environment_spec": self._environment_spec,
                "control": "semantic40 reference frames tracked by the frozen SONIC whole-body controller",
                "action_layout": R.action_layout(), "max_action_rows": R.MAX_ACTION_ROWS,
                "joint_names_canonical29": list(R.JOINT_NAMES), "cameras": list(R.CAMERAS),
                **({"joint_limits_rad": {"lower": metadata["joint_lower"], "upper": metadata["joint_upper"]}}
                   if "joint_lower" in metadata else {})}

    def _attachments(self, observation):
        content, attachments, depth = [], [], {}
        for camera in R.CAMERAS:
            if camera in observation:
                content.append({"type": "image", "data": _png_data(np.asarray(observation[camera], dtype=np.uint8)),
                                "mimeType": "image/png"})
                attachments.append({"kind": "rgb", "camera": camera, "content_index": len(content)})
        if self._geometry():
            for camera in R.CAMERAS:
                key = f"{camera}_depth_m"
                if key in observation:
                    encoded, stats = _depth_png_data(observation[key])
                    content.append({"type": "image", "data": encoded, "mimeType": "image/png"})
                    depth[camera] = dict(stats, content_index=len(content))
                    attachments.append({"kind": "depth", "camera": camera, "content_index": len(content)})
        return content, attachments, depth

    def _observation_fields(self, observation, attachments, depth):
        fields = {"proprio": observation.get("proprio"), "sim_time_s": observation.get("sim_time_s"),
                  "images": [a["camera"] for a in attachments if a["kind"] == "rgb"], "attachments": attachments}
        if self._geometry():
            fields.update(depth=depth, camera_intrinsics=observation.get("camera_intrinsics"),
                          camera_poses=observation.get("camera_poses"))
        return fields

    def _observation_packet(self, observation, *, transition=None):
        self._raw = (self.sim.episode_id, self.sim.step_id, observation)
        content, attachments, depth = self._attachments(observation)
        packet = {"episode_id": self.sim.episode_id, "step_id": self.sim.step_id, "task": self.config.task,
                  "environment_spec": self._environment_spec, **self._observation_fields(observation, attachments, depth)}
        if transition is not None:
            packet["transition"] = _jsonable(transition)
        return self.contract.observation.clean(_agent_safe(packet)), content

    def _motion_packet(self, result):
        frames = result.pop("frames", []) or []
        transition = dict(result)
        if not frames:
            return self._observation_packet(self.sim.request("observe"), transition=transition)
        records, contents = [], []
        for index, observation in enumerate(frames):
            content, attachments, depth = self._attachments(observation)
            records.append({"frame_index": index, "step_id": int(observation["step_id"]),
                            **self._observation_fields(observation, attachments, depth)})
            contents.append(content)
        sequence_id, manifest_path = self._save_sequence(records, contents, transition)
        self._raw = (self.sim.episode_id, self.sim.step_id, frames[-1])
        final, final_content = records[-1], contents[-1]
        directory = Path(manifest_path).parent.as_posix()
        packet = {"episode_id": self.sim.episode_id, "step_id": self.sim.step_id, "task": self.config.task,
                  "environment_spec": self._environment_spec, **{k: v for k, v in final.items() if k != "frame_index"},
                  "transition": _jsonable(transition),
                  "frame_sequence": {
                      "sequence_id": sequence_id, "frequency_hz": R.CONTROL_HZ / R.FRAME_STRIDE,
                      "frame_stride_steps": R.FRAME_STRIDE, "frame_count": len(records),
                      "manifest_path": manifest_path,
                      "frame_path_pattern": f"{directory}/frame_{{frame_index:06d}}_{{camera}}_{{kind}}.png",
                      "frame_index_range": [0, len(records) - 1],
                      "streams": [{"camera": a["camera"], "kind": a["kind"]} for a in final["attachments"]],
                      "returned_images": "final_frame_only"}}
        return self.contract.observation.clean(_agent_safe(packet)), final_content

    def _save_sequence(self, records, contents, transition, *, kind="humanoid"):
        run_id = self.sim.run_id or "unscoped"
        sequence_id = f"seq_{self.sim.step_id:06d}_{uuid.uuid4().hex[:8]}"
        logical = Path("runtime") / "frames" / kind / run_id / sequence_id
        artifacts, saved = {}, []
        for record, content in zip(records, contents, strict=True):
            files = []
            for attachment in record["attachments"]:
                item = content[attachment["content_index"] - 1]
                filename = f"frame_{record['frame_index']:06d}_{attachment['camera']}_{attachment['kind']}.png"
                artifacts[filename] = base64.b64decode(item["data"])
                files.append({"kind": attachment["kind"], "camera": attachment["camera"],
                              "path": (logical / filename).as_posix()})
            entry = {k: v for k, v in record.items() if k not in ("attachments",)}
            if "depth" in entry:
                entry["depth"] = {c: {k: v for k, v in d.items() if k != "content_index"} for c, d in entry["depth"].items()}
            entry["files"] = files
            saved.append(entry)
        manifest = {"sequence_id": sequence_id, "source": kind, "artifact_id": run_id,
                    "frequency_hz": R.CONTROL_HZ / R.FRAME_STRIDE, "frame_count": len(saved),
                    "transition": _agent_safe(transition), "frames": saved}
        artifacts["manifest.json"] = (json.dumps(self.contract.observation.clean(_jsonable(manifest)),
                                                 ensure_ascii=False, indent=2) + "\n").encode()
        write_artifacts(self.frame_root / kind / run_id / sequence_id, artifacts)
        return sequence_id, (logical / "manifest.json").as_posix()

    def _pixel_to_position(self, arguments):
        if self._raw is None or self._raw[0] != self.sim.episode_id:
            raise ValueError("Observe first; no current observation")
        _, step, observation = self._raw
        if arguments["observation_step_id"] != self.sim.step_id or step != self.sim.step_id:
            raise ValueError(f"Stale pixel selection: observation_step_id={arguments['observation_step_id']}, "
                             f"current step_id={self.sim.step_id}; observe again and reselect")
        depth = observation.get("cam_front_depth_m")
        if depth is None:
            raise ValueError("The current observation has no depth")
        intrinsics, pose = observation["camera_intrinsics"]["cam_front"], observation["camera_poses"]["cam_front"]
        root = observation["proprio"]["root_orientation_world_wxyz"]
        results = []
        for item in arguments["selections"]:
            point = T.pixel_to_pelvis(depth, intrinsics, pose, float(item["u"]), float(item["v"]),
                                      int(arguments.get("depth_window_radius", 1)))
            if point["valid"]:
                point["position_level_m"] = T.level_from_pelvis(point["position_pelvis_m"], root).tolist()
            results.append({"u": item["u"], "v": item["v"], **point})
        return {"episode_id": self.sim.episode_id, "observation_step_id": step, "camera": "cam_front",
                "frames": {"pelvis": "robot pelvis", "level": "origin at the pelvis, z up, x along the measured heading"},
                "selections": results}

    # -------------------------------------------------------------- lifecycle
    def start(self, *, formal, seed):
        self._evaluation = None
        self._raw = None
        c = self.config
        collection = c.formal_collection if formal else c.eval_seed
        environment_seed = layout_seed(c.task, collection, int(seed))
        if self.sim.process is not None:
            try:
                if self.sim.active:
                    self.sim.finish("formal_episode" if formal else "reset")
            finally:
                self.sim.stop()
        mode = "formal" if formal else "exploration"
        run_id = f"{c.task}_{mode}_{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}_{uuid.uuid4().hex[:8]}"
        timeouts = task_timeouts(c.task)
        reset = self.sim.start(task=c.task, collection=collection, seed=environment_seed, run_id=run_id,
                               sim_gpu=c.sim_gpu, startup_timeout=timeouts["startup_seconds"],
                               include_depth=self._geometry(),
                               # The supervisor's own wall limit also bounds the episode.
                               episode_timeout_seconds=int(c.formal.wall_seconds))
        limit = int(reset["native_step_limit"])
        self._environment_spec = {"benchmark": "humanoid", "episode_mode": mode, "episode_step_limit": limit,
                                  "native_step_limit": limit, "control_frequency_hz": R.CONTROL_HZ,
                                  "startup_seconds": round(self.sim.startup_seconds or 0.0, 1)}
        packet, images = self._observation_packet(self.sim.request("observe"))
        packet.update(reset=_agent_safe({"episode_id": reset["episode_id"], "step_id": reset["step_id"]}),
                      environment_spec=self._environment_spec, artifact_id=self.sim.run_id,
                      artifact_storage="service_managed_not_agent_accessible")
        return packet, images

    def evaluate(self, agent_assessment=None, confidence=None):
        sim = self.sim
        identity = (sim.episode_id, sim.step_id)
        if self._evaluation is not None and self._evaluation[0] == identity:
            return dict(self._evaluation[1])
        if sim.output_dir is None or sim.episode_id is None:
            raise RuntimeError("No HumanoidArena episode to evaluate")
        state = json.loads((sim.output_dir / "sim" / "operator_state.json").read_text())
        if state.get("episode_id") != sim.episode_id or state.get("step_id") != sim.step_id:
            raise RuntimeError("Native evaluation identity mismatch")
        success = (state.get("reward") or {}).get("native_success")
        if type(success) is not bool:
            raise RuntimeError("Native evaluation unavailable")
        evaluation = {"status": "completed", "task_complete": success, "episode_id": sim.episode_id,
                      "step_id": sim.step_id, "task": self.config.task, "evaluation_kind": "binary_task_completion",
                      "decided_at_unix_s": time.time()}
        if agent_assessment:
            evaluation["agent_assessment"] = str(agent_assessment)[:2000]
        self._evaluation = (identity, dict(evaluation))
        return dict(evaluation)

    def close(self):
        try:
            if self.sim.active:
                self.sim.finish("close")
        finally:
            self.sim.stop()
