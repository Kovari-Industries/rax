"""Evo action helpers. All calls use the same MCP tools as interactive control."""
import math
import time
from .runtime import Context


def _validate_timeout(timeout_s):
    if (
        type(timeout_s) not in (int, float)
        or not math.isfinite(timeout_s)
        or timeout_s <= 0
    ):
        raise ValueError("timeout_s must be finite and positive")


class Evo:
    def __init__(self, context=None):
        self.context = context or Context()
        self._owns_context = context is None

    def wait(self, action, timeout_s=40, cancelled=None):
        _validate_timeout(timeout_s)
        action_id = action["action_id"]
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if cancelled is not None and cancelled():
                result = self._stop_and_wait(action_id)
                raise EvoActionFailed({**result, "state": "cancelled", "reason": "caller cancelled"})
            result = self.context.value("evo_action_status", action_id=action_id)
            if result["state"] in ("succeeded", "cancelled", "failed"):
                return result
            time.sleep(0.05)
        result = self._stop_and_wait(action_id)
        raise TimeoutError(
            f"Evo action {action_id} timed out; physical_stop_verified="
            f"{result.get('physical_stop_verified', False)}"
        )

    def _stop_and_wait(self, action_id):
        terminal = ("succeeded", "cancelled", "failed")
        result = self.context.value("evo_action_status", action_id=action_id)
        if result.get("state") in terminal:
            if result.get("physical_stop_verified") is not True:
                raise EvoActionFailed(result)
            return result
        try:
            self.context.value("evo_cancel", action_id=action_id)
        except RuntimeError:
            # Completion can race cancellation. Never resend a motion or stop.
            result = self.context.value("evo_action_status", action_id=action_id)
            if result.get("state") in terminal and result.get("physical_stop_verified") is True:
                return result
            raise
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            result = self.context.value("evo_action_status", action_id=action_id)
            if result.get("state") in ("succeeded", "cancelled", "failed"):
                if result.get("physical_stop_verified") is not True:
                    raise EvoActionFailed(result)
                return result
            time.sleep(0.05)
        raise EvoActionFailed({"state": "failed", "reason": "stop verification timeout",
                               "physical_stop_verified": False})

    def move(self, tool, **arguments):
        return self.wait(self.context.value(tool, **arguments))

    def execute(self, tool, *, wait_timeout_s=40, cancelled=None, **arguments):
        """Preview, execute, and require measured success before the next stage."""
        allowed = {
            "evo_move_joints",
            "evo_move_eef",
            "evo_gripper",
            "evo_set_grip",
            "evo_move_body",
            "evo_follow_path",
            "evo_move_pair",
            "evo_move_carriage",
            "evo_move_neck",
            "evo_drive",
        }
        _validate_timeout(wait_timeout_s)
        if tool not in allowed:
            raise ValueError("not an Evo action tool")
        if cancelled is not None and cancelled():
            raise EvoActionFailed({"state": "cancelled", "reason": "cancelled before motion"})
        preview = self.context.value(
            "evo_preview", action={"kind": tool.removeprefix("evo_"), **arguments}
        )
        if preview.get("valid") is not True:
            raise ValueError("Evo preview did not validate the path")
        if cancelled is not None and cancelled():
            raise EvoActionFailed({"state": "cancelled", "reason": "cancelled after preview"})
        result = self.wait(self.context.value(tool, **arguments), wait_timeout_s, cancelled)
        if (
            result.get("state") != "succeeded"
            or result.get("physical_stop_verified") is not True
        ):
            raise EvoActionFailed(result)
        return result

    def close(self):
        if self._owns_context:
            self.context.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


class EvoActionFailed(RuntimeError):
    def __init__(self, result):
        self.result = result
        super().__init__(
            f"Evo action did not succeed: {result.get('reason', result.get('state'))}"
        )
