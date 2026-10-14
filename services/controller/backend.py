"""Trusted adapter to existing MCP implementations, never mounted in containers."""
from __future__ import annotations

import json
import os
import sys
import time
import uuid


class NativeBackend:
    def __init__(self, config):
        from services.robodojo.mcp_server import RoboDojoMCP
        from services.robot_preview.mcp_server import RobotPreviewMCP

        root = config.root / "native"
        root.mkdir(mode=0o700, exist_ok=True)
        # This adapter runs in a dedicated supervisor process, not a shared server.
        if os.environ.get("ROBODOJO_AGENT_TRIAL_ID"):
            raise RuntimeError("Controller service must not inherit an interactive trial identity")
        os.environ.setdefault("ROBODOJO_PYTHON", sys.executable)
        os.environ.update({
            "ROBODOJO_AGENT_WORKSPACE": str(root / "workspace"),
            "ROBODOJO_HUMAN_SETUP_PATH": str(root / "operator" / "setup.json"),
            "ROBODOJO_OPERATOR_SETUP_REQUEST_PATH": str(root / "operator" / "setup-request.json"),
            "ROBODOJO_OPERATOR_EVALUATION_REQUEST_PATH": str(root / "operator" / "evaluation.json"),
            "ROBODOJO_MCP_OUTPUT_ROOT": str(root / "results"),
            "ROBODOJO_RUNTIME_ROOT": str(root),
            "ROBODOJO_SIM_GPU": config.sim_gpu,
            "ROBODOJO_TRIAL_GPU": config.sim_gpu,
        })

        class ManagedBridge(RoboDojoMCP):
            # Includes interactive exploration; isolated runs also have the
            # supervisor's stricter startup-inclusive deadline.
            episode_timeout_seconds = config.formal.wall_seconds

            def _validate_trial_task(self, task):
                # The new supervisor binds identity/configuration durably instead
                # of relying on the legacy Codex workspace's trial.json.
                if task != config.task:
                    raise RuntimeError("Controller task differs from operator configuration")

            def _await_human_setup(self, **kwargs):
                return dict(self.pending_setup)

            def _await_human_evaluation(self, record):
                # Same trusted evidence used by dashboard automatic evaluation.
                # Check identity, task AND step; never accept a stale result.
                state = json.loads((self.sim.output_dir / "sim" / "operator_state.json").read_text())
                for key in ("episode_id", "task", "step_id"):
                    if state.get(key) != record.get(key):
                        raise RuntimeError(f"Native evaluation identity mismatch: the simulator last recorded "
                                           f"{key} {state.get(key)!r}, but this episode's is {record.get(key)!r}")
                success = state.get("reward", {}).get("native_success")
                if type(success) is not bool:
                    raise RuntimeError("Native evaluation unavailable")
                return {**record, "status": "completed", "task_complete": success,
                        "decided_at_unix_s": time.time()}

        from services.mcp_contract import Contract
        self.contract = Contract.from_config(config, isolated=True)
        self.config = config
        self.robot = ManagedBridge()
        self.preview = RobotPreviewMCP()
        self.robot.observation_profile = self.contract.observation_profile
        self.preview.observation_profile = self.contract.observation_profile
        self.frame_workspace = self.robot.agent_workspace
        self._evaluation = None

    def tools(self):
        return [t for t in self.contract.select(self.robot.tool_definitions() + self.preview.tools())
                if t["name"] in self.contract.robot_names]

    @staticmethod
    def validate(name, arguments):
        from services.robodojo.action_validation import validate_motion_request
        validate_motion_request(name, arguments)

    def call(self, name, arguments):
        self.contract.require_robot(name, arguments)
        from .gateway import MUTATING
        planning_only = name == 'robodojo_free_space_move' and arguments.get('preview_only') is True
        if name == "robot_preview_describe":
            return self.preview.description(self.contract.observation_profile), []
        if name == "robot_preview_run":
            return self.preview.preview(arguments)
        if name not in MUTATING or planning_only:
            return self.robot.call(name, arguments)
        sim = getattr(self.robot, 'sim', None)
        before = (getattr(sim, "episode_id", None), getattr(sim, "step_id", None))
        try:
            result = self.robot.call(name, arguments)
        except BaseException:
            # A failed motion can change physical state before step_id updates.
            self._evaluation = None
            raise
        if (getattr(sim, "episode_id", None), getattr(sim, "step_id", None)) != before:
            self._evaluation = None
        # A motion that executed nothing (Planning_Failed, Plan_Stale) keeps the
        # cached evaluation: the native evaluator refuses a repeat at the same step.
        return result

    def start(self, *, formal, seed):
        self._evaluation = None
        c = self.config
        self.robot.pending_setup = {
            "revision": uuid.uuid4().hex, "task": c.task, "seed": seed,
            "eval_seed": c.formal_collection if formal else c.eval_seed,
            "sim_gpu": c.sim_gpu, "sim_port": 0, "startup_timeout": 180,
            "include_depth": self.contract.observation.geometry,
            "include_camera_parameters": self.contract.observation.geometry,
            # Existing backend uses this legacy field for phase selection, but
            # SimulatorProcess enforces the native horizon in BOTH phases.
            # Visual demos are provisioned once into the development workspace at
            # launch, not recreated in this private native workspace each episode.
            "enforce_step_limit": formal, "demonstration_context": "none",
            "max_exploration_episodes": len(c.exploration_seeds),
        }
        return self.robot.call("robodojo_request_formal_episode" if formal else "robodojo_request_human_setup", {})

    def evaluate(self, agent_assessment=None, confidence=None):
        # Legacy interactive review rejects repeat requests at the same step.
        # Auto-research lifecycle transitions may evaluate after an explicit check;
        # reuse only this process's trusted result for this exact episode/state.
        sim = self.robot.sim
        identity = (sim.episode_id, sim.step_id)
        if self._evaluation is not None and self._evaluation[0] == identity:
            return dict(self._evaluation[1])
        request = {"agent_assessment": agent_assessment or "Automatic controller supervisor terminal evaluation"}
        if confidence is not None:
            request["confidence"] = confidence
        value, _ = self.robot.call("robodojo_request_human_evaluation", request)
        evaluation = value["human_evaluation_request"]
        self._evaluation = (identity, dict(evaluation))
        return dict(evaluation)

    def close(self):
        try:
            self.robot.close()
        finally:
            self.preview.close()


def make_backend(config):
    """The configured benchmark's trusted backend (RoboDojo when unspecified)."""
    from services.benchmarks import for_config
    return for_config(config).backend(config)
