"""Composable cloth interactions. Task strategies and learned policies share these calls.

A completed robot action does not prove a cloth outcome. Inspect again after motion.
No rectangle, fold planner, force estimate, or automatic recovery is required.
"""
import math
from .evo import Evo
from .cloth_vision import ClothVision


class ClothActions:
    def __init__(self, context, *, workspace="."):
        self.context = context
        self.evo = Evo(context)
        self.vision = ClothVision(context, workspace)

    def observe(self, **arguments):
        return self.context.value("evo_observe", **arguments)

    def inspect(self, observation, checks, *, target=None):
        return self.vision.inspect(observation, checks, target=target)

    def move(self, targets, *, duration_s=3):
        """Move named left/right palm poses in base_link. Evo validates the path."""
        if not targets or set(targets) - {"left", "right"}:
            raise ValueError("select left and/or right palm targets")
        return self.evo.execute(
            "evo_follow_path",
            frame="base_link",
            waypoints=[{**targets, "duration_s": duration_s}],
        )

    def grip(self, side, position_rad, *, retain):
        """Command position only; retain does not prove cloth is held."""
        return self.evo.execute(
            "evo_set_grip", side=side, position_rad=position_rad, retain=retain
        )

    def release_checked(self, side, position_rad, assessment, *, checks):
        """Require explicit fresh visual checks before opening one gripper."""
        if not checks:
            raise ValueError("release requires explicit checks")
        assessment.require(*checks)
        return self.grip(side, position_rad, retain=False)

    def lift(self, hands, distance_m, *, duration_s=3):
        """One bounded vertical increment; inspect cloth response before repeating."""
        if not hands or len(set(hands)) != len(hands) or set(hands) - {"left", "right"}:
            raise ValueError("select distinct hands")
        if (
            type(distance_m) not in (int, float)
            or not math.isfinite(distance_m)
            or not 0 < distance_m <= 0.04
        ):
            raise ValueError("lift increment must be in (0, .04] metres")
        poses = self.context.value("evo_fk", links=[s + "_palm" for s in hands])[
            "poses"
        ]
        targets = {
            s: {
                "position": [
                    *poses[s + "_palm"]["position"][:2],
                    poses[s + "_palm"]["position"][2] + distance_m,
                ],
                "quaternion_xyzw": poses[s + "_palm"]["quaternion_xyzw"],
            }
            for s in hands
        }
        return self.move(targets, duration_s=duration_s)

    def spread_increment(self, distance_m, *, duration_s=3):
        """Increase hand separation by at most 4 cm. This does not measure tension."""
        if (
            type(distance_m) not in (int, float)
            or not math.isfinite(distance_m)
            or not 0 < distance_m <= 0.04
        ):
            raise ValueError("spread increment must be in (0, .04] metres")
        poses = self.context.value("evo_fk", links=["left_palm", "right_palm"])["poses"]
        left, right = poses["left_palm"], poses["right_palm"]
        delta = [a - b for a, b in zip(left["position"], right["position"])]
        span = math.sqrt(sum(x * x for x in delta))
        if span < 0.01:
            raise ValueError("hands are too close to define spread direction")
        return self.evo.execute(
            "evo_move_pair",
            frame="base_link",
            waypoints=[
                {
                    "midpoint": [
                        (a + b) / 2 for a, b in zip(left["position"], right["position"])
                    ],
                    "separation": [x * (span + distance_m) / span for x in delta],
                    "left_quaternion_xyzw": left["quaternion_xyzw"],
                    "right_quaternion_xyzw": right["quaternion_xyzw"],
                    "duration_s": duration_s,
                }
            ],
        )
