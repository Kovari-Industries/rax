# Pose and action records: Panda + Allegro

{% if formal_episode %}
How [experience memory](../../experience-memory/SKILL.md) records a DexJoCo motion for
reuse.
{% else %}
How [experience memory](../../experience-memory/SKILL.md) records a DexJoCo motion for
reuse. These fields fill the robot-specific part of an example in
[its format](../../experience-memory/references/format.md).
{% endif %}

## Encode poses

- Record the executed rows separately from the reusable pose recipe. Flange targets
  are world-frame positions with rotation vectors (actions) or `[w, x, y, z]`
  quaternions (state); say which.
- Store targets relative to a visible anchor or the measured flange pose, together
  with how to find the anchor again; object placement and the table height change
  between layouts, so absolute coordinates do not transfer.
- Record a hand posture as the 16 joint targets with what each finger does (which
  fingers close, how far past contact), in the hand's own joint order.
- Do not hand-edit quaternion or rotation-vector components; keep measured or
  computed values.

## Example fields

```json
{
  "action_kind": "rows | hold",
  "executed_action": {
    "tool": "dexjoco_step",
    "representation": "dexjoco_official_action_rows_30hz",
    "rows": "transition.steps[*].executed_action"
  },
  "pose_recipe": {
    "representation": "relative_world",
    "anchor": "<visible point or measured flange pose>",
    "frame": "world",
    "position_offset_m": [0.0, 0.0, 0.05],
    "flange_rotvec_rad": [-3.1416, 0.0, 0.0]
  },
  "hand_recipe": {
    "joint_targets_rad": "<16 values in the hand's joint order>",
    "fingers": "<which fingers close and how far past contact>"
  },
  "point_reacquisition": {
{% if camera_geometry %}
    "tool": "dexjoco_pixel_to_position",
{% else %}
    "method": "RGB visual alignment with measured flange poses",
{% endif %}
    "visible_feature": "<feature description>",
    "preferred_views": {{ robot.cameras | list | tojson }},
    "offset_m": [0.0, 0.0, 0.05]
  }
}
```

Record executed rows as `dexjoco_official_action_rows_30hz`: the rows from
`transition.steps[*].executed_action` (a held step reports the row it held), each
paired with the observation it was sent from, step `step_id - 1`. Replaying rows only
reproduces an effect from a matching measured state and layout.
