## Measuring and correcting motion

- **Characterizing before building.** Before designing a motion of your own, measuring what the
  controller does with the commands it already offers, at two or more rates or magnitudes
{% if formal_episode %}
  saves attempts. A single probe may not represent the available motion range.
{% else %}
  ([evidence-and-regression](../evidence-and-regression/SKILL.md)), saves episodes. A single
  probe may not represent the available motion range.
{% endif %}
- **A measured table of the primitives you rely on:** for each, the measured forward, lateral and
  heading change over several executions, with mean and spread, in `memory/`. Plans made with the
  table, not with commanded values, hold up; a change of the primitive, the posture, the load or the
  speed calls for a re-measurement.
- **Closing the loop between actions.** Re-perceiving the target and re-planning from the measured
  state after each action works better than long open-loop sequences.
- **Systematic offsets in a measured model.** If the hand consistently lands 3 cm short, aiming 3 cm
  further and checking that this helped beats patching a threshold after each failure.
- **Contact in small corrective steps,** with settling time before you judge where a hand or foot
  ended up.
- **Turns and lateral motion checked against the scene** (an edge, a floor line, a fixed object),
  not against summed commands.
- **Balance.** Large sudden command changes, stepping while the hands carry a load and leaning far
  out can all cause a fall, and a fall ends the episode.
- **Your body's signals.** The normal range of the tracking error in `transition.feedback` is worth
  measuring in your own episodes. A sudden rise means contact, a blocked limb or a coming fall, and
  settling, backing off and re-perceiving is the usual answer. Read this feedback while moving;
  a rising tracking error can precede a fall.
- **Feet and hands from the measured joints,** not from commanded positions, related to nearby
  objects.
- **The fingers.** `proprio.hand_state` gives the measured finger angles, velocities and torques.
  Fingers closing on nothing reach their closed targets quickly; fingers that stop early, with
  torque, are touching something. The finger state before and after closing is contact evidence.
