"""Separate Evo v4 registration; shared transport, loaded v4 model limits."""
import math
from services.evo.backend import EvoBackend, EvoRefused


class EvoV4Backend(EvoBackend):
    @staticmethod
    def _check_model(capabilities):
        model = capabilities.get("model", {})
        joints = model.get("joint_names", [])
        limits = model.get("limits", {})
        if "hip_pitch_joint" not in joints:
            raise EvoRefused("evo-v4 requires a model with hip_pitch_joint; a v3 model cannot be used")
        for joint in ("hip_pitch_joint", "carriage_joint"):
            bound = limits.get(joint, {})
            low, high = bound.get("lower"), bound.get("upper")
            if any(type(x) not in (int, float) or not math.isfinite(x) for x in (low, high)) or low >= high:
                raise EvoRefused(f"evo-v4 requires finite loaded model limits for {joint}")
        scene = capabilities.get("scene", {})
        if scene.get("configured") and "hip_pitch_joint" not in scene.get("settings", {}).get("travel_joints", {}):
            raise EvoRefused("evo-v4 elevator travel posture must include hip_pitch_joint")

    def _ensure_session(self):
        # Check the loaded model before opening motion authority. Do not infer
        # v4 identity from the selected UI name or reuse v3 model constants.
        if self.session_id is None:
            self._check_model(self._rpc("capabilities", {}))
        return super()._ensure_session()

    def call(self, name, arguments):
        value, images = super().call(name, arguments)
        if name == "evo_capabilities":
            self._check_model(value)
            value["rax_robot"] = "evo-v4"
        return value, images
