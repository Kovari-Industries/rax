# Field guide, upper body: posture, arms and hands

Use [the field guide](../SKILL.md) to check each motion in the current posture and load.

## Posture

- **Waist axes track differently.** Do not assume a waist-pitch reference gives the requested
  lean. Check measured joints and body pose before using it in inverse kinematics.
- **Root pitch affects reach and balance.** A larger lean can make the robot step; verify the
  resulting hand position and support before contact.
- **Root height alone does not define a crouch.** Coordinate hip and knee references, and check
  the measured pelvis height.
- **Swing-leg motion can saturate.** Measure actual flexion and foot clearance.

## Arms

- **Check the swept path.** A joint-space move from a low arm pose can cross nearby surfaces;
  raise the arm into free space before following Cartesian waypoints.
- **Allow for lag and tracking error.** Check forward kinematics on measured joints and make
  small corrections after settling.
- **Reach depends on posture and load.** Straight elbows and extreme wrist or shoulder pitch
  can be under-tracked; contact can also push the body away from the target.
- **Force needs pre-load.** Commanded references beyond a contact surface produce pressure, and
  the reaction affects balance. Check both the hand and the body.
- **A low IK residual is not arrival.** A planned pose says nothing about whether a blocked arm
  reached it; inspect the measured hand pose.
- **Arm motion can change the gait.** Re-characterize a walk when changing arm posture or load.
- **Body width depends on the arms.** Check the full swept body, including the hands and load,
  when choosing clearance.

## Hands and holding

- **Closing is binary.** The fingertips sweep while closing; a plausible open-hand pose does
  not guarantee the closing path captures an object.
- **Closure profiles affect contact.** Partial duty, a ramp to full closure and an immediate full
  close can each move an object differently. Check finger feedback and the object's motion,
  then use full closure to maintain a grip during a lift.
- **Finger torque is contact evidence.** Calibrate a threshold from your own open, touching and
  holding states; do not assume the same value works for every posture and object.
- **Carrying changes balance and grip.** Limit abrupt turns, hold a stable arm pose and inspect
  the object after each action.
