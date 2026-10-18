---
name: robot-interface
description: The Unitree G1 humanoid with two Dex3 hands under the frozen SONIC whole-body controller - its semantic40 reference frames, observations, feedback, frames, body facts and which tool to use for which objective. Read before composing robot actions.
---

# Robot interface: G1 + Dex3 (SONIC, semantic40)

The robot is a Unitree G1 humanoid with 29 body joints and two three-finger Dex3 hands. You
do not command motors: you send whole-body **reference frames**, and the frozen SONIC
whole-body controller turns them into motor commands and keeps the robot balanced. It has one
camera, `cam_front`, on the torso, looking forward and down. `{{ robot.status }}` reports the
live action layout, joint names and joint limits; this skill explains them.

## One reference frame

Every `humanoid_step` row is one frame for one 50 Hz control tick (40 numbers); a call carries
1 to 100 rows (up to 2 s):

| Slice | Meaning |
|---|---|
| `[0:2]` | Reference root displacement for this tick (metres, in the reference root's own frame). Required by the layout; **the controller ignores it**. |
| `[2]` | Reference pelvis height (metres). Required by the layout; **ignored** as well. |
| `[3:9]` | Reference root rotation relative to the episode's initial heading: the first two columns of the rotation matrix, row-major. Identity is `[1, 0, 0, 1, 0, 0]`; a pure turn by `psi` is `[cos psi, -sin psi, sin psi, cos psi, 0, 0]`. Large jumps are projected or clipped (`max_root_orientation_projection_deg`). |
| `[9:38]` | Absolute angles (rad) of the 29 body joints, in the order of `proprio.joint_names_canonical29`. |
| `[38:40]` | Left and right hand: `0` open, `1` closed. Nothing in between. |

The controller receives the 29 joint angles, their change from tick to tick (as joint
velocities) and the root rotation. It receives no root position: **the robot moves through the
room only by stepping**, through the leg joint references. Walking therefore means sending
stepping leg references ([controller-field-guide](../controller-field-guide/SKILL.md)).

- **Frames are references, not motor targets.** On an empty floor and without contact the
  controller tracks the joints to about 6 degrees, and over a walk the root drifts from the
  commanded motion by tens of centimetres, up to about a metre. A replayed motion therefore does
  not reproduce its recorded displacement in another layout, a part of a motion may do
  something quite different from the whole, and a very short slice may hardly move the robot
  ([controller-tracking](../controller-tracking/SKILL.md)).
- **Holding still.** A row of zeros is not "hold": it drives every joint toward zero and the root
  toward the initial heading. Your first frame of an episode starts from the measured
  `proprio.joint_positions_rad` and the measured heading; later frames continue from the frame
  you last sent, changing only what you intend to move.
- **Each row is one step** against the episode step limit (`episode_limits`). Rows past the
  limit are cut, not refused (`transition.feedback.rows_cut_at_step_limit`), so the end of an
  episode stays usable.

## Observations

`{{ robot.observe }}` and every motion reply carry:

- `proprio.root_orientation_world_wxyz`: the measured pelvis orientation. Its yaw is your
  heading; there is **no world position** of the robot or of any object.
- `proprio.joint_names_canonical29`, `joint_positions_rad`, `joint_velocities_rad_s`: the
  measured body joints.
- `proprio.hand_state`: the 14 Dex3 finger joints, their positions, velocities and applied
  torques.
- `cam_front`: the front camera image, 640 x 480 RGB.
{% if camera_geometry %}
- Depth of `cam_front` (distance along the optical axis, uint16 PNG in millimetres, `0`
  invalid), `camera_intrinsics` and `camera_poses.cam_front`: the camera pose in the pelvis
  frame (camera x along the optical axis, y left, z up). The camera sits on the torso, so it
  turns with the waist joints.
{% endif %}

## Feedback and the end of an episode

Every motion reply has `transition.feedback`: the ticks executed, `joint_tracking_rmse_rad` and
`max_joint_tracking_error_rad` of the last tick, `max_root_orientation_projection_deg`,
`tilt_deg` and `fallen`. `transition.steps[*].executed_action` holds every executed frame,
paired with the preceding `observation_step_id`; `humanoid_step` can send them again from a
matching state.

The episode ends at the first tick where the task's success condition holds
(`task_complete: true`), when the robot falls, or at the step limit (`episode_ended`). A fall
is the task's own rule: the torso tilted more than 75 degrees, or more than 60 degrees while
the pelvis, torso, a hip, thigh, knee or the head carries a hard contact, for five consecutive
ticks. It cannot be undone. Frames are saved every 10th step and at the last one; only the
final frame is returned inline.

## Frames

- **pelvis**: the robot's root body; x forward, y left, z up. It moves and tilts with the robot.
- **level**: the pelvis origin, but gravity-aligned: z straight up, x the pelvis forward
  direction projected onto the floor. It needs the measured root orientation; heights in it are
  relative to the pelvis.
- **torso**: `torso_link`, which carries the arms and the camera; the waist joints move it
  relative to the pelvis.

Every pose any tool returns or takes is in one of these body frames. A point in them moves when
you walk or turn; re-measure after moving.

## The body you have

- **Hands are binary.** Open is every finger joint at 0 (index and middle straight, the thumb
  spread about 90 degrees from them); closed is a fixed fist. There is no partial or per-finger
  command. When closed, the fingertips curl back to a few centimetres in front of the palm. An
  object is held only if it lies between the thumb and the two fingers before they close, or if
  open fingers and thumb clamp it from outside.
- **Measured joints tell where the body is**; the angles you commanded do not. Compute hand and
  foot positions from measured joints.
- **The camera looks down and forward**: objects near your feet or behind you are out of view,
  and your own hands, arms, knees and feet appear in the lower image.
{% if camera_geometry %}

## Robot model

`scripts/g1_model.py` (with its `robot.json`) is the robot's kinematic model, exported from the
simulated robot itself and checked against its measured link poses (differences of
micrometres). It describes the robot only, not the scene. Save an observation's JSON (the
first text block of a `{{ robot.observe }}` or motion reply) to a file, then:

```bash
python3 .agents/skills/robot-interface/scripts/g1_model.py pose --observation obs.json --png /tmp/pose.png
python3 .agents/skills/robot-interface/scripts/g1_model.py project 0.45 -0.2 -0.1 --observation obs.json
python3 .agents/skills/robot-interface/scripts/g1_model.py deproject 320 300 --observation obs.json --depth-png frame_depth.png
python3 .agents/skills/robot-interface/scripts/g1_model.py semantic40
python3 .agents/skills/robot-interface/scripts/g1_model.py hand
```

- `pose` prints the key points (both hand centres, palms, thumb, index and middle tips, both
  soles, the head and the camera) in the level frame, with heights above your lowest sole, and
  draws side, rear and top views; open the picture after any posture change you did not plan.
- `project` puts a planned 3-D point into the image, so you can check that it lands where you
  mean before you move; `deproject` turns a pixel and its depth into a 3-D point.
- `semantic40` names all 40 dimensions; from Python, `g1_model.semantic40_frame(joints29,
  hands=(False, False), ...)` builds a frame and `describe_semantic40(frame)` names every value of
  one you are about to send. Left and right are easy to swap by index; names are not.
- `hand` lists the Dex3 joints, the open and closed poses, the fingertip positions in the palm
  frame and the thumb-to-finger gap.
{% endif %}

## Choose the tool to match the objective

- **Whole-body motion, walking, turning, posture, hands:** `humanoid_step` frames built from the
  measured state, then from your last frame. The [controller-field-guide](../controller-field-guide/SKILL.md)
  describes walking, turning and contact.
- **A hand to a pose in open space:** `humanoid_free_space_move` plans one arm (optionally the
  waist) around the robot itself only, and keeps the legs and everything else at your last
  frame ([free-space-move](../free-space-move/SKILL.md)).
{% if camera_geometry %}
- **Metric points from the image:** `humanoid_pixel_to_position`
  ([object-localization](../object-localization/SKILL.md)).
{% else %}
- **Locating objects without depth:** [object-localization](../object-localization/SKILL.md).
{% endif %}

{% if multi_env %}
## Environment selection

- Pass the selected `env_id` on every `humanoid_*` call and on `start_episode`, `evaluate` and
  `finish`; for example `humanoid_step(env_id=i, actions=...)`. `humanoid_status`
{% if agent_formal %}
  reports one environment; `status` and `gemini_generate` take no `env_id`. Live schemas define
  the other arguments.
{% else %}
  reports one environment; `status`, `gemini_generate`, `register`, `rehearse` and `submit`
  take no `env_id`. Live schemas define the other arguments.
{% endif %}
- Keep multi-call sequences in one environment: execute a previewed plan with
  `humanoid_execute_motion_plan` in the environment that planned it, as
  [multi-env exploration](../multi-env-exploration/SKILL.md) describes.
{% endif %}

## Related skills

- [controller-tracking](../controller-tracking/SKILL.md): how the references are tracked, and
  how to measure and correct.
- [controller-field-guide](../controller-field-guide/SKILL.md): practical motion limits and
  feedback checks for this controller.
{% if expert_demonstrations %}
- [learn-from-experts](../learn-from-experts/SKILL.md): the expert episodes in `runtime/experts/`.
{% endif %}
{% if not formal_episode %}
- [references/pose-records.md](references/pose-records.md) for how poses and executed frames are
  recorded for reuse.
{% endif %}
