# Pose and action records: G1 + Dex3

{% if formal_episode %}
How [experience memory](../../experience-memory/SKILL.md) records a HumanoidArena motion for
reuse.
{% else %}
How [experience memory](../../experience-memory/SKILL.md) records a HumanoidArena motion for
reuse. These fields fill the robot-specific part of an example in
[its format](../../experience-memory/references/format.md).
{% endif %}

## Encode poses and motions

- Record the executed frames separately from the reusable recipe. Executed frames are
  `transition.steps[*].executed_action`: 40 numbers per 50 Hz tick, paired with the preceding
  `observation_step_id`.
- A reusable motion is a primitive: a function from parameters to frames, with the measured
  effect (forward, lateral and heading change, or where a hand ended) over several executions.
  Record the parameter range you tested.
- Hand targets are poses in the pelvis or torso frame. Store them relative to something you can
  find again (a visible edge, a measured hand pose), never as an episode's absolute numbers.
- Headings are relative to the episode's initial heading; record a turn as the change it made,
  checked against the scene.
- Do not hand-edit rotation components; keep measured or computed values.

## Example fields

```json
{
  "action_kind": "frames | free_space_move",
  "executed_action": {
    "tool": "humanoid_step",
    "representation": "g1_semantic40_frames_50hz",
    "rows": "transition.steps[*].executed_action"
  },
  "primitive": {
    "name": "<walk cycle, turn, reach>",
    "parameters": {"duration_s": 2.0},
    "measured_effect": {"forward_m": [0.55, 0.65], "n": 6}
  },
  "pose_recipe": {
    "representation": "relative_pelvis",
    "anchor": "<visible feature or measured hand pose>",
    "frame": "pelvis",
    "position_m": [0.35, -0.20, 0.10],
    "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0]
  },
  "point_reacquisition": {
{% if camera_geometry %}
    "tool": "humanoid_pixel_to_position",
{% else %}
    "method": "RGB visual alignment with measured robot poses",
{% endif %}
    "visible_feature": "<feature description>",
    "preferred_views": ["cam_front"],
    "offset_m": [0.0, 0.0, 0.05]
  }
}
```

Record executed frames as `g1_semantic40_frames_50hz`. Keep requested hand targets in
`pose_recipe`; they are goals, not the executed representation. Replaying frames reproduces an
effect only from a matching measured state, and the controller's drift makes even that loose.
