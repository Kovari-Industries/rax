"""Evo SAM jobs through the same MCP interface as interactive tools.

This module has no ML dependencies. Masks supply pixel evidence only.
"""
import math
import time


class Sam:
    def __init__(self, context):
        self.context = context

    def segment(self, observation_id, camera="base", **prompts):
        return self.context.value(
            "evo_sam_segment", observation_id=observation_id, camera=camera, **prompts
        )

    def track(self, observation_ids, camera="base", **prompts):
        return self.context.value(
            "evo_sam_track",
            observation_ids=list(observation_ids),
            camera=camera,
            **prompts,
        )

    def cancel(self, job):
        return self.context.value("evo_sam_cancel", job_id=job["job_id"])

    def wait(self, job, timeout_s=125):
        if (
            type(timeout_s) not in (int, float)
            or not math.isfinite(timeout_s)
            or timeout_s <= 0
        ):
            raise ValueError("timeout_s must be finite and positive")
        deadline = time.monotonic() + timeout_s

        def check_deadline():
            if time.monotonic() >= deadline:
                try:
                    self.cancel(job)
                except Exception as error:
                    raise TimeoutError(
                        "SAM wait timed out; cancellation failed"
                    ) from error
                raise TimeoutError("SAM wait timed out; cancellation requested")

        # Check each response boundary, including artifact retrieval. A synchronous
        # RPC cannot be interrupted here; use a separate Context for urgent calls.
        while True:
            check_deadline()
            status = self.context.value("evo_sam_result", job_id=job["job_id"])
            check_deadline()
            if status["state"] in ("cancelled", "failed"):
                raise RuntimeError(f"SAM job {status['state']}: {status.get('error')}")
            if status["state"] == "succeeded":
                count = status["frame_count"]
                if (
                    not isinstance(count, int)
                    or isinstance(count, bool)
                    or not 1 <= count <= 64
                ):
                    raise ValueError("invalid SAM frame count")
                frames = []
                for index in range(count):
                    check_deadline()
                    page = self.context.value(
                        "evo_sam_result", job_id=job["job_id"], frame_index=index
                    )
                    check_deadline()
                    if (
                        page.get("state") != "succeeded"
                        or page.get("result") != status["result"]
                    ):
                        raise ValueError("SAM result changed during retrieval")
                    frames.append(page["frame"])
                return {**status["result"], "frames": frames}
            time.sleep(min(0.05, max(0, deadline - time.monotonic())))
