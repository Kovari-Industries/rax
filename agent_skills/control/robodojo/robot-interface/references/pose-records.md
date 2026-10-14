# Pose and action records: dual ARX X5

{% if formal_episode %}
How [experience memory](../../experience-memory/SKILL.md) records a RoboDojo motion for
reuse.
{% else %}
How [experience memory](../../experience-memory/SKILL.md) records a RoboDojo motion for
reuse. These fields fill the robot-specific part of an example in
[its format](../../experience-memory/references/format.md).
{% endif %}

## Encode poses

- Record the executed action representation separately from the reusable pose
  recipe. EEF and cuRobo requests remain absolute environment-frame `link6` poses;
  only the learned recipe may be relative. Prefer `relative_start_eef` plus
  `delta_frame: "local"` for a motion expressed once in the starting EEF frame;
  derive it with `relative_pose`.
- Store `relative_environment` with `delta_frame: "environment"` for a delta
  in fixed environment axes.
- Save an absolute environment pose only for a genuinely fixed robot/table
  waypoint and label it `absolute_environment`. Use a reacquired relative target
  for randomized objects.
- A mixed recipe may use an absolute safe waypoint plus relative manipulation
  steps; store ordered `pose_recipe.segments`. Reconstruct relative recipes with
  `compose_pose` and preserve its normalized quaternion output. The starting-EEF
  frame is fixed for that recipe, not a continuously moving control frame.
- If the anchor is only a visible surface point, store the offset and how the
  point was obtained under `point_reacquisition`.
- Preserve exact pose-math output; do not hand-edit quaternion components.

## Example fields

```json
{
  "action_kind": "step_eef | joint | curobo",
  "executed_action": {
    "tool": "robodojo_step_eef",
    "representation": "absolute_joint_positions_gripper_14d_25hz",
    "joint_target_contract_version": 2,
    "rows": "transition.steps[*].executed_action"
  },
  "pose_recipe": {
    "representation": "mixed",
    "anchor": "<observable anchor semantics>",
    "segments": [
      {
        "representation": "absolute_environment",
        "anchor": "<fixed safe waypoint>",
        "position_m": [0.0, 0.0, 0.8],
        "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0]
      },
      {
        "representation": "relative_environment",
        "anchor": "<visible point>",
        "delta_frame": "environment",
        "position_m": [0.0, 0.0, 0.05],
        "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0]
      }
    ]
  },
  "point_reacquisition": {
{% if camera_geometry %}
    "tool": "robodojo_pixel_to_position",
    "method": "auto | depth | triangulation",
{% else %}
    "method": "RGB visual alignment with measured robot poses",
{% endif %}
    "visible_feature": "<feature description>",
    "preferred_views": ["cam_high"],
    "offset_m": [0.0, 0.0, 0.05]
  }
}
```

Use `absolute_joint_positions_gripper_14d_25hz` for the executed low-level
actions from every motion tool, with `joint_target_contract_version: 2`.
The source is `transition.steps[*].executed_action`, not measured states or
the full planned sequence after partial execution. Pair commands with their
preceding `observation_step_id`. Keep requested EEF poses in `pose_recipe`;
they are goals, not the executed joint-action representation.
Only recorded commands with this contract are replay targets; sampled measured
joint states are not interchangeable with those commands.

`relative_start_eef` means one transform expressed in the observed starting
`link6` frame and replayed as `T_target=T_current*T_delta`; it is not updated
continuously during the move. `relative_environment` means translation and
rotation deltas in fixed environment axes. For `absolute_environment`, omit
`delta_frame` and identify the fixed environment anchor. For `mixed`, omit the single
delta fields and store ordered `segments`, each using one non-mixed representation.
