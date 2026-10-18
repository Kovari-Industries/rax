# Field guide, primitives: reusable motion segments

Use [the field guide](../SKILL.md) to check the controller's response before composing motions.

## The library pattern

Represent each primitive as a function from parameters to 50 Hz reference frames. Keep its tested
parameter range, entry and exit conditions, measured effect and duration beside the code. Budget
time for approach, contact, recovery and settling before composing a complete sequence.

## Retargeting contact

{% if expert_recordings %}
- **Retarget commanded references around a contact.** Use the recording's joint references and
  hand-closing profile. Compute the hand path by forward kinematics, shift and rotate it onto the
  perceived target, solve arm inverse kinematics at keyframes and interpolate the corrections.
  Add corrections to the commanded references to preserve pre-load into the surface.
{% elif expert_demonstrations %}
- **Reuse your own commanded contacts.** The expert episodes hold camera observations only.
  Your action records contain `transition.steps[*].executed_action`: retarget a contact that held
  by inverse kinematics at keyframes, adding corrections to preserve its pre-load.
{% else %}
- **Reuse your own commanded contacts.** Your action records contain
  `transition.steps[*].executed_action`: retarget a contact that held by inverse kinematics at
  keyframes, adding corrections to preserve its pre-load.
{% endif %}
- **Check pre-load and hand timing separately.** The commanded-minus-measured offset describes
  how far the references press past a surface. The hand-command profile sets when closure starts,
  its speed and its level. Compare both with a contact that held when diagnosing a failure.

## Graded closure from the binary command

A duty cycle can produce partial closure: add duty d (0 to 1) to an accumulator every tick,
send closed and subtract 1 whenever the accumulator reaches 1, and send open otherwise. The
fingers follow the running mean. Check actual finger motion and contact as you tune the profile;
use a fully closed command to maintain a grip during a lift.
{% if expert_recordings %}
Recorded hand commands are continuous, so their closing onset, speed and level can guide this
duty profile.
{% endif %}

## Locomotion segments

{% if expert_recordings %}
Walk cycles and turns can be cut from recordings and retargeted:
{% else %}
A segment of your own that produces repeatable motion can become a primitive:
{% endif %}

- blend from the measured pose into the segment and back to a stand;
- use a yaw-only rot6d ramp toward the target heading for a walk; retain roll and pitch only
  when the motion needs them and the resulting balance has been checked;
- identify matching loop frames for walks of different lengths;
- check the model's roll and yaw sign convention before mirroring leg joints;
- compare clips with matching timing before blending their amplitudes;
- keep actions short enough to observe and correct between them;
- for obstacle crossings, verify the landing foot supports the weight before lifting the
  trailing foot, and measure a usable approach window.
