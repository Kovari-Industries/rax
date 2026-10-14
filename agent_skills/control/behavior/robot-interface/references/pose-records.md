# Pose and action records: R1Pro

{% if formal_episode %}
How [experience memory](../../experience-memory/SKILL.md) records a BEHAVIOR motion for
reuse.
{% else %}
How [experience memory](../../experience-memory/SKILL.md) records a BEHAVIOR motion for
reuse. These fields fill the robot-specific part of an example in
[its format](../../experience-memory/references/format.md).
{% endif %}

## Encode poses

- Record the executed rows separately from the reusable pose recipe. Free-space
  targets are robot-base-frame end-effector poses with `xyzw` quaternions.
- Store targets relative to a visible anchor or the measured end-effector pose,
  together with how to find the anchor again.
- Record base motions as the viewpoint they reach (what is visible, at what
  distance), not as odometry coordinates, which start over each episode.
- Do not hand-edit quaternion components; keep measured or computed unit quaternions.

## Example fields

```json
{
  "action_kind": "rows | base_move | free_space_move",
  "executed_action": {
    "tool": "behavior_step",
    "representation": "r1pro_action_rows_23d_30hz",
    "rows": "transition.steps[*].executed_action"
  },
  "pose_recipe": {
    "representation": "relative_base",
    "anchor": "<visible point or measured end-effector pose>",
    "frame": "robot_base",
    "position_m": [0.0, 0.0, 0.05],
    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]
  },
  "point_reacquisition": {
{% if camera_geometry %}
    "tool": "behavior_pixel_to_position",
{% else %}
    "method": "RGB visual alignment with measured robot poses",
{% endif %}
    "visible_feature": "<feature description>",
    "preferred_views": ["cam_head"],
    "offset_m": [0.0, 0.0, 0.05]
  }
}
```

Record executed rows as `r1pro_action_rows_23d_30hz`: the 23-D rows from
`transition.steps[*].executed_action` of any motion tool, paired with the preceding
`observation_step_id`. Keep requested end-effector poses in `pose_recipe`; they are
goals, not the executed representation. Replaying rows only reproduces an effect
from a matching measured state.

`relative_base` stores a target relative to a visible anchor or the measured
end-effector pose in the robot base frame at that moment, plus how to re-find the
anchor.
