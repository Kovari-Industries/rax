"""Upstream's fall rule (HumanoidArena ``script/eval_scripts/sonic/sim_eval_vla.py``).

A control step is a fall candidate when the root's up axis tilts more than 75 degrees, or more
than 60 degrees while a critical body (pelvis, torso, waist, hip, thigh, knee, calf, shin or
head, but not ankle or foot) carries at least 50 N of contact force. A fall is declared after
5 consecutive candidate steps. Contact forces come from ``robot.data.body_net_contact_force_w``
exactly as upstream reads them; when that field is absent the contact branch is inactive, as
it is upstream. Success is checked before falls, also as upstream does.
"""
import math

SOFT_TILT_DEG, HARD_TILT_DEG = 60.0, 75.0
CONTACT_FORCE_N, CONFIRM_STEPS = 50.0, 5
CRITICAL_TOKENS = ("pelvis", "torso", "waist", "hip", "thigh", "knee", "calf", "shin", "head")
SUPPORTED_TOKENS = ("ankle", "foot")


def critical_bodies(body_names):
    names = [str(name) for name in body_names]
    return [index for index, name in enumerate(names)
            if not any(token in name.lower() for token in SUPPORTED_TOKENS)
            and any(token in name.lower() for token in CRITICAL_TOKENS)]


def up_alignment_from_quat_wxyz(quat_wxyz):
    _, x, y, _ = (float(v) for v in quat_wxyz)
    return 1.0 - 2.0 * (x * x + y * y)


def is_candidate(up_alignment, critical_contact_forces_n=()):
    up = max(-1.0, min(1.0, float(up_alignment)))
    soft = up < math.cos(math.radians(SOFT_TILT_DEG))
    hard = up < math.cos(math.radians(HARD_TILT_DEG))
    return hard or (soft and any(force >= CONTACT_FORCE_N for force in critical_contact_forces_n))


class FallDetector:
    """Streak-confirmed fall state across the control steps of one episode."""

    def __init__(self, body_names):
        self.critical = critical_bodies(body_names)
        self.streak = 0

    def reset(self):
        self.streak = 0

    def update(self, robot_data):
        gravity = getattr(robot_data, "projected_gravity_b", None)
        if gravity is not None:
            up = float(-gravity[0, 2].item())
        else:
            up = up_alignment_from_quat_wxyz(robot_data.root_state_w[0, 3:7].tolist())
        forces = ()
        contacts = getattr(robot_data, "body_net_contact_force_w", None)
        if contacts is not None and self.critical:
            import torch
            forces = torch.linalg.vector_norm(contacts[0, self.critical], dim=-1).tolist()
        self.streak = self.streak + 1 if is_candidate(up, forces) else 0
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, up))))
        return self.streak >= CONFIRM_STEPS, {"tilt_deg": round(tilt, 2), "fall_streak": self.streak}
