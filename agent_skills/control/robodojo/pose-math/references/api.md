# `robodojo_pose_math` API

{% if formal_episode %}
This tool is pure geometry. It reads no scene state and advances no physics.
{% else %}
This tool is pure geometry. It can run before
`start_episode`, reads no scene state, and advances no physics.
{% endif %}

`compose_pose` and `relative_pose` use the explicit `base_pose` as a fixed
reference for that call. Reusing the same base expresses every target relative
to that same reference, not the preceding action. No measured pose, previous
command, or accumulated trajectory is selected implicitly. `delta_frame`
chooses the coordinate axes/composition rule, not a different reference pose.
These are geometry operations, not delta-action execution modes;
`robodojo_step_eef` accepts absolute targets only.

## Rotation values

Every rotation is:

```json
{"representation": "quaternion_wxyz", "value": [1, 0, 0, 0]}
```

Supported representations:

- `quaternion_wxyz`: `[qw,qx,qy,qz]`. Non-zero input is normalized.
- `quaternion_xyzw`: `[qx,qy,qz,qw]`; only component order changes.
- `euler_xyz_extrinsic_rad`: fixed environment-axis rotations
  `[roll_x,pitch_y,yaw_z]` in radians. Composition is `qz*qy*qx`.
- `axis_angle_vector_rad`: direction is the right-hand rotation axis; vector
  magnitude is the angle in radians. `[0,0,0]` is identity.
- `rotation_matrix_row_major`: flat nine-element 3x3 matrix. It maps vectors
  from the represented local frame into its parent frame and must be
  orthonormal with determinant `+1`.

Every result also includes normalized `quaternion_wxyz`, standardized to
`qw >= 0`. Quaternion signs are not physical: `q` and `-q` are the same
orientation. Euler angles are non-unique and become ill-conditioned near
pitch `±pi/2`; use returned `quaternion_wxyz` for control.

Pose inputs accept only `position_m` and `rotation`. When chaining a returned
`pose` or `delta_pose` into another call, copy those two fields; omit its
extra `quaternion_wxyz` convenience field.

## `convert_rotation`

Inputs: `rotation`, optional `output_representation`.

The input is decoded to normalized `wxyz`, then encoded in the requested
representation. No frame change is performed.

```json
{
  "operation": "convert_rotation",
  "rotation": {
    "representation": "euler_xyz_extrinsic_rad",
    "value": [0, 0, 1.57079632679]
  },
  "output_representation": "quaternion_wxyz"
}
```

## `compose_pose`

Inputs: complete `base_pose`, partial `delta_pose`, and `delta_frame`. A missing
delta position means `[0,0,0]`; a missing delta rotation means identity.
`delta_pose: {}` is an identity delta.

For `delta_frame: "local"`:

```text
p_out = p_base + R(q_base) * dp_local
q_out = q_base * q_delta
T_out = T_base * T_delta
```

For `delta_frame: "environment"`:

```text
p_out = p_base + dp_environment
q_out = q_delta * q_base
```

The environment rotation changes orientation about a fixed environment axis in
place; it does not orbit `p_base` around the environment origin.

## `relative_pose`

Inputs: complete `base_pose`, complete `target_pose`, and `delta_frame`.

For `local`:

```text
dp_local = R(q_base)^T * (p_target - p_base)
q_delta = inverse(q_base) * q_target
```

For `environment`:

```text
dp_environment = p_target - p_base
q_delta = q_target * inverse(q_base)
```

Pass the returned `delta_pose.position_m` and `delta_pose.rotation` back as
`delta_pose` to `compose_pose` with the same frame to recover the target.

## `format_target`

Inputs: complete `pose`, `target_kind`, and gripper fields.

- `target_kind: "step_eef"` requires `gripper_closed`; optional
  `gripper_opening` is continuous in `[0,1]` and overrides the boolean fallback.
- `target_kind: "free_space_move"` omits `gripper_closed`; optional
  `gripper_opening` remains continuous. If omitted, execution holds the
  measured opening from the plan's start state.

It emits only the exact target keys accepted by the selected motion tool:
`position` in environment-origin metres and unit `quaternion_wxyz` for `link6`.
The result is `{"target_kind": ..., "target": {...}}`. For `step_eef`, call
the operation separately for each arm and assemble the two returned `target`
objects under `left` and `right`. For `free_space_move`, pass the selected
arm's returned `target` directly.

## `extract_target`

Inputs: `eef_target`, optional `output_representation`.

It validates and separates `position`, rotation, and any gripper fields. Use
this to inspect a target or round-trip it through another representation before
control.
