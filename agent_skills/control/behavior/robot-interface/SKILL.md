---
name: robot-interface
description: The R1Pro mobile bimanual robot in BEHAVIOR-1K - its 23-D action rows, proprioception, frames, cameras and odometry, and which motion tool to use for which objective. Read before composing robot actions.
---

# Robot interface: R1Pro

The robot is a Galaxea R1Pro: a holonomic mobile base, a 4-joint trunk, two 7-joint
arms with parallel grippers, a head camera and a camera on each wrist. The episode
runs in a household scene that may span several rooms. `{{ robot.status }}` reports
the live action layout and joint limits; this skill explains them.

## One action row

Every `behavior_step` row is one native control step at 30 Hz (23 numbers):

| Slice | Part | Meaning |
|---|---|---|
| `[0:3]` | base | Robot-frame velocity `[vx, vy, wz]`, each in [-1, 1]. Scaled to +-0.75 m/s forward/left and +-1 rad/s yaw. Zero stops the base. |
| `[3:7]` | trunk | Absolute joint targets (rad) within `action_limits`. |
| `[7:14]` | left arm | Absolute joint targets (rad). |
| `[14]` | left gripper | Normalized command in [-1, 1]: -1 closes, +1 opens. |
| `[15:22]` | right arm | Absolute joint targets (rad). |
| `[22]` | right gripper | Normalized command in [-1, 1]: -1 closes, +1 opens. |

Trunk and arm entries are position targets, not deltas: to keep a joint still, repeat
its measured value from the latest observation. A row of zeros is not "hold still"; it
drives every arm joint toward zero. The base entries are velocities: they act for one
step, so a base motion needs a sequence of rows. Build rows from the measured state:
copy `proprio.trunk_qpos`, `proprio.arm_left_qpos` and `proprio.arm_right_qpos`, set the
grippers to `gripper_commands`, then change only what you intend to move.

Measured joints are raw physics observations and can lie slightly outside
`action_limits`; keep the measurements when reasoning about the robot's state.
When building a hold or an interpolation's first row, bound the joint targets to
the reported limits. The tools accept excursions of at most **1e-4 rad** (about
0.006 degrees), clamp them to the actual bound, and report `joint_target_clamping`
in the motion reply (`transition` for episode motion). Larger excursions reject
the whole batch before any action. These rules apply to `behavior_step` and
`behavior_preview_run`, including the preview's measured or explicit initial state
(`initial_state_joint_clamping`). Base and gripper command ranges are unchanged.
For `behavior_base_move` and `behavior_step_eef`, the service generates hold targets
for joints you are not commanding. It always clamps these measured holds to the
limits, even when drift exceeds 1e-4 rad, and reports the correction in
`transition.joint_target_clamping`, marked `clamp_measured_hold`. Measured drift
does not reject a convenience motion; explicit agent-supplied joint targets retain
the strict tolerance check. Observed proprioception stays unchanged.
`joint_limit_violations` in observations reports each measured joint outside its
bound, with zero-based joint/action indices, bounds and excess in radians. A
rejection identifies the offending row and joint and the exact excess; correct the
targets before retrying. A typed `RejectedBeforeAction` keeps control trusted and
the connection available only when no native rows ran in the whole tool call;
a partial-execution failure or timeout still blocks motion.

Each row counts once against the episode step limit (`episode_limits`), however many
rows a call carries.

## Observations

- `proprio` holds the 61-D proprioception, split into named fields:
  base velocity, joint positions and velocities, gripper finger positions, and each
  end effector's position and `xyzw` quaternion **relative to the robot base**.
  `proprio_vector` is the same data as one list.
- `gripper_openings` maps the measured finger opening onto the command range;
  `gripper_commands` is the last command sent.
- `odometry` is the base pose relative to where this episode started (`x_m`, `y_m`,
  `yaw_rad`), integrated from the base velocity. There is no map and no world pose.
  Odometry resets with every episode, so coordinates from an earlier episode do not
  transfer: re-find objects in each episode. It drifts by about 2% of the distance
  driven; re-observe landmarks after long moves.
- Cameras: `cam_head` (720x720, the overview) and `cam_left_wrist` / `cam_right_wrist`
  (480x480).
{% if camera_geometry %}
  For metric positions from these images, see
  [object-localization](../object-localization/SKILL.md).
{% else %}
  For locating objects without depth, see
  [object-localization](../object-localization/SKILL.md).
{% endif %}

## Aiming the head camera

The robot has no neck: `cam_head` is fixed to the top of the trunk, so the trunk joints
aim it (sent as `behavior_step` rows with everything else held).

- **Tilt.** Trunk joints 1-3 pitch the upper body. The downward tilt grows by about
  57 degrees per radian of `trunk[0] + trunk[1] - trunk[2]`; the start pose already looks
  about 20 degrees down. Joint 1 also lowers the head, joint 2 leans it forward at nearly
  constant height, and joint 3 tilts it with the least head travel. Tilt down to see a
  surface just ahead, and back up to search a room.
- **Pan.** Trunk joint 4 yaws the whole upper body, both arms included (+ turns left).
  For wide or lasting turns, rotate the base instead.
- The arms ride on the trunk: any trunk motion moves both hands and every camera in the
  base frame. Aim the head before reaching, keep the trunk still while a hand holds or
  touches something, and re-observe after aiming (`camera_poses` gives the new head pose).

## Frames

- **Robot base frame:** origin at the base, +x forward, +y left, +z up.
{% if camera_geometry %}
  End-effector poses, camera poses, pixel-to-position results and free-space targets
  all use it.
{% else %}
  End-effector poses and free-space targets use it.
{% endif %}
  It moves with the robot: after any base motion, earlier base-frame points are stale.
- **Odometry frame:** the robot base frame at the start of the episode. To keep a
  point across base motion, convert it with the current odometry pose:
  `p_odom = (x + cos(yaw) px - sin(yaw) py, y + sin(yaw) px + cos(yaw) py, pz)`.
- Quaternions are `[x, y, z, w]` in all tools and observations.

## Choose the tool to match the objective

If the objects are out of view or reach, search and approach with the base first
([base-navigation](../base-navigation/SKILL.md)), then work with the arms.

- **Base:** `behavior_base_move` for bounded relative moves and turns, `(dx, dy, dyaw)`
  from the current pose, driven by odometry; it knows no obstacles, so check the head
  camera first.
- **A hand to a pose in open space:** `behavior_free_space_move` plans the arm (and,
  with `use_trunk`, the trunk) around the robot itself only; tables and objects are
  invisible to it. See [free-space-move](../free-space-move/SKILL.md).
- **Approach, contact and small corrections:** `behavior_step_eef` moves one arm only,
  with the trunk and base still (closed-loop IK, not collision-aware).
- **Grippers, trunk poses and explicit joints:** `behavior_step` rows built from the
  measured joints (see the action row above). Change only the joints you intend to
  move, and close or open grippers over several rows while holding the arm still.
- **Dry run:** `behavior_preview_run` shows rows or a plan from third-person cameras
  on a robot-only simulator without changing the episode
  ([motion-preview](../motion-preview/SKILL.md)).
- **Replay:** every motion reply includes the executed rows
  (`transition.steps[*].executed_action`), which `behavior_step` can send again from
  a matching state.

Verify object contact or retention visually rather than inferring it from the
commanded gripper value. Frames are saved every 6th step and at the final step;
only the final frame is returned inline.

{% if multi_env %}
## Environment selection

- Pass the selected `env_id` on every `behavior_*` call and on `start_episode`,
  `evaluate` and `finish`; for example `behavior_step(env_id=i, actions=...)` or
  `behavior_base_move(env_id=i, dx=..., dy=..., dyaw=...)`. `behavior_status`
{% if agent_formal %}
  reports one environment; `status` and `gemini_generate` take no `env_id`. Live
  schemas define the other arguments.
{% else %}
  reports one environment; `status`, `gemini_generate`, `register`, `rehearse` and
  `submit` take no `env_id`. Live schemas define the other arguments.
{% endif %}
- Keep multi-call sequences in one environment: execute a previewed plan with
  `behavior_execute_motion_plan` in the environment that planned it, as
  [multi-env exploration](../multi-env-exploration/SKILL.md) describes.
{% endif %}

## Related skills

- [base-navigation](../base-navigation/SKILL.md) to find objects and move the base between
  rooms and workspaces.
- [free-space-move](../free-space-move/SKILL.md) for moving a hand to a base-frame
  pose; inspect the measured result, not just planning success.
{% if not formal_episode %}
- [references/pose-records.md](references/pose-records.md) for how poses and executed
  actions are recorded for reuse.
{% endif %}
