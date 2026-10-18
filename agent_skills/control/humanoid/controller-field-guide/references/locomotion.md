# Field guide, locomotion: walking, turning and obstacles

Use [the field guide](../SKILL.md) with current observations to characterize each motion.

## Walking

- **Walking needs dynamic stepping leg references.**
{% if expert_recordings %}
  A repeatable walk cycle can be cut from an expert recording, with a yaw-only rot6d of your own.
{% elif expert_demonstrations %}
  Your expert episodes show walking on camera without commanded references. Build a repeatable
  cycle from your own commanded frames and measured motion.
{% else %}
  Build a repeatable walk cycle from your commanded frames and measured motion.
{% endif %}
- **Small commands may not produce small steps.** Quasi-static or bent-knee motions may leave the
  body in place. Characterize the smallest dynamic stride, end the last walk within tolerance,
  and use the arms or torso for the remaining correction.
- **Starts and stops have transients.** Blend from the measured pose into the cycle and back to
  a stand. Allow for coasting and for the old direction to persist briefly after a reversal.
- **Asymmetric cycles can crab sideways.** Check forward and lateral movement against the scene.
- **Pelvis pitch affects balance at gait start.** Check whether a yaw-only root orientation is
  sufficient before carrying roll and pitch from a recorded gait into a new motion.

## Turning

- **Heading is a closed-loop target.** Allow for lag, saturation and a minimum effective command;
  measure the resulting heading rather than adding commanded angles.
- **Turning in place depends on rate and amplitude.** A small, slow rot6d ramp may twist the hips
  without making the feet step. Characterize a range before choosing a turning primitive.
- **Turning can translate the body.** Re-localize afterwards, and allow clearance for the body's
  sideways swing while turning during a walk.
- **Sidesteps and backward walking need their own characterization.** Check direction, distance
  and balance before using them for precise positioning.

## Obstacles

- **Check foot clearance and weight transfer.** Raised steps and low obstacles can block a foot
  or destabilize a gait. Prefer a clear route when one exists; characterize an unavoidable
  crossing separately before composing it into a longer controller.
