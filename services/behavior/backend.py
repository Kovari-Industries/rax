"""Trusted auto-research backend for BEHAVIOR-1K; same interface as controller NativeBackend.

Runs in the host harness interpreter and never imports OmniGibson. Each episode is a fresh
simulator process (``services.behavior.process``); exploration uses train instances
(collection 0) and formal uses the held-out public test instances (collection 1).
"""

from __future__ import annotations

import base64
import json
import time
import uuid
from pathlib import Path

import numpy as np

from services.artifact_io import write_artifacts
from services.behavior import robot as R
from services.behavior import tools as T
from services.behavior.preview import PreviewProcess
from services.behavior.process import BehaviorProcess
from services.behavior.tasks import layout_instance, task_info, task_timeouts
from services.robodojo.mcp_server import _agent_safe, _depth_png_data, _jsonable, _png_data
from services.simulator.io import action_execution_scope, exit_report


class BehaviorBackend:
    # evaluate() reads the simulator's operator_state.json, which outlives its process.
    evaluates_from_record = True

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
        self.sim = BehaviorProcess(output_root=root / "results")
        # Robot-only preview on the simulator GPU, started on first use.
        self.preview = PreviewProcess(output_root=root / "preview", sim_gpu=config.sim_gpu)
        self._evaluation = None
        self._raw = None  # Last raw observation (host memory only), for pixel geometry.
        self._environment_spec = None
        self._formal = False

    # ------------------------------------------------------------------ tools
    def tools(self):
        return [t for t in self.contract.select(T.tool_definitions()) if t["name"] in self.contract.robot_names]

    @staticmethod
    def validate(name, arguments):
        T.validate_request(name, arguments)

    def _geometry(self):
        return self.contract.observation.geometry

    def call(self, name, arguments):
        # Each motion sends one mutating RPC, followed by packet assembly, which
        # may request an observation. Cover the entire tool, including that request.
        with action_execution_scope(self.sim, name):
            return self._call(name, arguments)

    def _call(self, name, arguments):
        self.contract.require_robot(name, arguments)
        T.validate_request(name, arguments)
        if name == T.STATUS:
            return self._status(), []
        if name == T.PREVIEW_DESCRIBE:
            return self._preview_description(), []
        if name == T.PREVIEW_RUN:
            return self._preview_run(arguments)
        sim = self.sim
        if not sim.active:
            raise RuntimeError("No active BEHAVIOR episode; start an episode first")
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
            elif name == T.BASE_MOVE:
                result = sim.request("base_move", **arguments)
            elif name == T.STEP_EEF:
                result = sim.request("eef_move", **arguments)
            elif name == T.FREE_SPACE_MOVE:
                result = sim.request("plan_arm", **arguments)
            elif name == T.EXECUTE_PLAN:
                result = sim.request("execute_motion_plan", motion_plan_id=arguments["motion_plan_id"])
            else:
                raise ValueError(f"Unknown BEHAVIOR tool: {name}")
        finally:
            if (sim.episode_id, sim.step_id) != before:
                self._evaluation = None
        return self._motion_packet(result)

    # ---------------------------------------------------------------- packets
    def _status(self):
        sim = self.sim
        return {"active": sim.active, "finished": sim.finished, "task": self.config.task,
                "episode_id": sim.episode_id, "step_id": sim.step_id if sim.episode_id else None,
                "environment_spec": self._environment_spec,
                "action_layout": R.action_layout(), "cameras": list(R.CAMERAS),
                "max_action_rows": R.MAX_ACTION_ROWS,
                # Trunk/arm entries are joint limits (rad); base and grippers are normalized.
                **({"action_limits": {"lower": sim.metadata["action_lower"], "upper": sim.metadata["action_upper"]}}
                   if sim.metadata else {})}

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
        fields = {
            "proprio": observation.get("proprio_fields"),
            "proprio_vector": observation.get("proprio"),
            "joint_limit_violations": observation.get("joint_limit_violations", []),
            "odometry": observation.get("odometry"),
            "gripper_commands": observation.get("gripper_commands"),
            "gripper_openings": observation.get("gripper_openings"),
            "images": [a["camera"] for a in attachments if a["kind"] == "rgb"],
            "attachments": attachments,
        }
        if self._geometry():
            fields.update(depth=depth, camera_intrinsics=observation.get("camera_intrinsics"),
                          camera_poses=observation.get("camera_poses"))
        return fields

    def _observation_packet(self, observation, *, transition=None):
        self._raw = (self.sim.episode_id, self.sim.step_id, observation)
        content, attachments, depth = self._attachments(observation)
        packet = {"episode_id": self.sim.episode_id, "step_id": self.sim.step_id, "task": self.config.task,
                  "instruction": observation.get("instruction"), "environment_spec": self._environment_spec,
                  **self._observation_fields(observation, attachments, depth)}
        if transition is not None:
            packet["transition"] = _jsonable(transition)
        return self.contract.observation.clean(_agent_safe(packet)), content

    def _motion_packet(self, result):
        frames = result.pop("frames", []) or []
        transition = {k: v for k, v in result.items()}
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
                  "instruction": frames[-1].get("instruction"), "environment_spec": self._environment_spec,
                  **{k: v for k, v in final.items() if k != "frame_index"},
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

    # ---------------------------------------------------------------- preview
    @staticmethod
    def _preview_description():
        return {"tool": T.PREVIEW_RUN, "simulator": "robot-only OmniGibson: the episode R1Pro on a floor plane",
                "cameras": {name: "fixed third-person RGB, 480x480" for name in T.PREVIEW_CAMERAS},
                "frame": "the base starts at the preview origin facing +x; cameras stay fixed while it moves",
                "inputs": {"actions": f"1..{R.MAX_ACTION_ROWS} behavior_step rows (23-D, {R.CONTROL_HZ} Hz)",
                           "motion_plan_id": "a behavior_free_space_move preview_only plan, still executable after",
                           "initial_state": "optional joints (trunk, arm_left, arm_right) and gripper commands; "
                                            "default: the current episode's measured state, with drift past a joint limit "
                                            "clamped and reported"},
                "outputs": ["end proprio (joints, end-effector poses in the base frame)", "base motion (odometry)",
                            f"third-person frames every {R.FRAME_STRIDE}th row, final frames attached"],
                "not_available": ["scene, furniture or objects", "contact with the world, grasps or task outcome"],
                "counts_toward_episode_steps": False}

    def _preview_run(self, arguments):
        sim = self.sim
        live = sim.active
        if "motion_plan_id" in arguments:
            if not live:
                raise RuntimeError("motion_plan_id previews need the active episode that planned it")
            rows = np.asarray(sim.request("motion_plan_rows", motion_plan_id=arguments["motion_plan_id"])["rows"],
                              dtype=np.float32)
            source = {"motion_plan_id": arguments["motion_plan_id"]}
        else:
            rows = np.asarray(arguments["actions"], dtype=np.float32)
            source = {"actions": int(len(rows))}
        if "initial_state" in arguments:
            state = {k: list(map(float, v)) for k, v in arguments["initial_state"].items()}
        else:
            if not live:
                raise RuntimeError("No active episode to take the start state from; pass initial_state")
            observation = sim.request("observe")
            fields = observation["proprio_fields"]
            commands = observation["gripper_commands"]
            state = {"trunk": fields["trunk_qpos"], "arm_left": fields["arm_left_qpos"],
                     "arm_right": fields["arm_right_qpos"], "grippers": [commands["left"], commands["right"]],
                     "gripper_left_qpos": fields["gripper_left_qpos"], "gripper_right_qpos": fields["gripper_right_qpos"]}
        result = self.preview.run(state, rows.tolist(), measured="initial_state" not in arguments)
        records, contents = [], []
        for index, frame in enumerate(result["frames"]):
            content, attachments = [], []
            for camera in T.PREVIEW_CAMERAS:
                content.append({"type": "image", "data": _png_data(np.asarray(frame["images"][camera], dtype=np.uint8)),
                                "mimeType": "image/png"})
                attachments.append({"kind": "rgb", "camera": camera, "content_index": len(content)})
            records.append({"frame_index": index, "preview_row": int(frame["step"]), "attachments": attachments})
            contents.append(content)
        end = result["end_proprio"]
        summary = {"status": result["status"], "rows": result["rows"], "source": source,
                   "start_state": {k: v for k, v in result.get("initial_state", state).items() if not k.endswith("_qpos")},
                   "end_state": {k: end[k] for k in ("trunk_qpos", "arm_left_qpos", "arm_right_qpos", "eef_left_pos",
                                                     "eef_left_quat", "eef_right_pos", "eef_right_quat",
                                                     "gripper_left_qpos", "gripper_right_qpos")},
                   "base_motion": result["base_motion"], "preview_seconds": result["seconds"]}
        for key in ("joint_target_clamping", "initial_state_joint_clamping"):
            if key in result:
                summary[key] = result[key]
        sequence_id, manifest_path = self._save_sequence(records, contents, summary, kind="behavior_preview")
        directory = Path(manifest_path).parent.as_posix()
        packet = {**summary, "simulator": "robot-only preview (floor plane; no scene, objects or contact)",
                  "cameras": (self.preview.metadata or {}).get("cameras"),
                  # Not `step_id`: the preview never advances or records the live episode.
                  "live_episode_id": sim.episode_id if live else None, "live_step_id": sim.step_id if live else None,
                  "frame_sequence": {
                      "sequence_id": sequence_id, "frame_stride_rows": R.FRAME_STRIDE, "frame_count": len(records),
                      "manifest_path": manifest_path,
                      "frame_path_pattern": f"{directory}/frame_{{frame_index:06d}}_{{camera}}_{{kind}}.png",
                      "frame_index_range": [0, len(records) - 1],
                      "streams": [{"camera": camera, "kind": "rgb"} for camera in T.PREVIEW_CAMERAS],
                      "returned_images": "final_frame_only"}}
        return _agent_safe(_jsonable(packet)), contents[-1]

    def _save_sequence(self, records, contents, transition, *, kind="behavior"):
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
        episode, step, observation = self._raw
        if arguments["observation_step_id"] != self.sim.step_id or step != self.sim.step_id:
            raise ValueError(f"Stale pixel selection: observation_step_id={arguments['observation_step_id']}, "
                             f"current step_id={self.sim.step_id}; observe again and reselect")
        results = []
        for item in arguments["selections"]:
            camera = item["camera"]
            depth = observation.get(f"{camera}_depth_m")
            if depth is None:
                results.append({"camera": camera, "valid": False, "reason": "no depth"})
                continue
            point = T.pixel_to_base(depth, observation["camera_intrinsics"][camera], observation["camera_poses"][camera],
                                    float(item["u"]), float(item["v"]), int(arguments.get("depth_window_radius", 1)))
            results.append({"camera": camera, "u": item["u"], "v": item["v"], **point})
        return {"episode_id": episode, "observation_step_id": step, "selections": results}

    # -------------------------------------------------------------- lifecycle
    def start(self, *, formal, seed):
        self._evaluation = None
        self._raw = None
        c = self.config
        collection = c.formal_collection if formal else c.eval_seed
        instance = layout_instance(c.task, collection, int(seed))
        if self.sim.process is not None:
            try:
                if self.sim.active:
                    self.sim.finish("formal_episode" if formal else "reset")
            finally:
                self.sim.stop()
        mode = "formal" if formal else "exploration"
        run_id = f"{c.task}_{mode}_{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}_{uuid.uuid4().hex[:8]}"
        timeouts = task_timeouts(c.task)
        reset = self.sim.start(task=c.task, collection=collection, instance_id=instance, run_id=run_id,
                               sim_gpu=c.sim_gpu, startup_timeout=timeouts["startup_seconds"],
                               include_depth=self._geometry(),
                               # The supervisor's own wall limit, if any, also bounds the episode.
                               episode_timeout_seconds=int(c.formal.wall_seconds or 0))
        self._formal = formal
        limit = int(reset["native_step_limit"])
        self._environment_spec = {"benchmark": "behavior", "episode_mode": mode, "episode_step_limit": limit,
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
            raise RuntimeError("No BEHAVIOR episode to evaluate")
        state = json.loads((sim.output_dir / "sim" / "operator_state.json").read_text())
        if (sim.finished and state.get("episode_id") == sim.episode_id and state.get("reason") is not None
                and isinstance(state.get("step_id"), int) and state["step_id"] > sim.step_id):
            # The simulator ended the episode inside a call whose reply never arrived (its wall
            # limit, then exit): its final summary is the record of the rows it ran.
            sim.step_id = state["step_id"]
            identity = (sim.episode_id, sim.step_id)
        if state.get("episode_id") != sim.episode_id or state.get("step_id") != sim.step_id:
            raise RuntimeError(
                f"Native evaluation identity mismatch: the simulator last recorded episode "
                f"{state.get('episode_id')!r} at step {state.get('step_id')!r}, but this episode is "
                f"{sim.episode_id!r} at step {sim.step_id!r}; "
                + ("the simulator process was already stopped" if sim.process is None
                   else exit_report(sim.process, sim.log_path, since=sim.log_mark)))
        success = (state.get("reward") or {}).get("native_success")
        if type(success) is not bool:
            raise RuntimeError("Native evaluation unavailable")
        # Only binary completion leaves the host: partial credit stays in operator files.
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
            try:
                self.sim.stop()
            finally:
                self.preview.stop()
