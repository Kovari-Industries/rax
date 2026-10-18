- **Re-projection.** A 3-D estimate (an outline, a contact point, the hand) projected back into the
  image lands on the object when the pipeline is right. A hand that occludes part of an object can
  look like contact.
- **Consistency over frames.** A static object seen from two poses gives the same estimate once your
  own motion is accounted for.
{% if expert_recordings %}
- **Expert episodes** contain their own front-camera frames and the recorded object state for each
  step. Your detector run on their frames and compared with the recorded pose gives an error
  distribution across layouts and viewpoints.
{% elif expert_demonstrations %}
- **Expert episodes** contain their own front-camera video and depth, without object states. Your
  detector run on their frames shows how often it finds the object across other layouts and
  viewpoints, though not its error.
{% endif %}
- **A residual log** in `memory/` (episode, posture, residuals, frames) shows when to {{ redo }};
  investigate residuals before changing a threshold that rejects them.
