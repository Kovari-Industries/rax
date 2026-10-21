"""Evo action helpers. All calls use the same MCP tools as interactive control."""
import math
import time
from .runtime import Context


class Evo:
    def __init__(self, context=None):
        self.context = context or Context()
        self._owns_context = context is None

    def wait(self, action, timeout_s=40):
        if (
            type(timeout_s) not in (int, float)
            or not math.isfinite(timeout_s)
            or timeout_s <= 0
        ):
            raise ValueError("timeout_s must be finite and positive")
        action_id = action["action_id"]
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            result = self.context.value("evo_action_status", action_id=action_id)
            if result["state"] in ("succeeded", "cancelled", "failed"):
                return result
            time.sleep(0.05)
        self.context.value("evo_cancel", action_id=action_id)
        raise TimeoutError(
            f"Evo action {action_id} timed out; cancellation requested, stop not verified"
        )

    def move(self, tool, **arguments):
        return self.wait(self.context.value(tool, **arguments))

    def close(self):
        if self._owns_context:
            self.context.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
