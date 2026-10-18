## Robust detection

- **Support.** A minimum point count, spatial extent, a height band and a size consistent with
  TASK.md help reject spurious small blobs.
- **Your own body and what you hold.** Hands, arms, knees and a carried object appear in the image
  and look like scenery. Masking them by their predicted image region from forward kinematics on the
  measured joints, before detecting supports, edges or landmarks, worked; a blanket near-range mask
  also hides the objects you touch.
- **Gating.** A new detection that agrees with the tracked estimate and with physical priors (a
  change your own motion and the object's possible motion can explain, a size and height within
  TASK.md's ranges, a location inside the object's region) can be trusted; one that does not is worth
  confirming in another frame before it replaces a stable estimate. A wrongly matched landmark
  can corrupt your position estimate even when its appearance is plausible.
- **Tracking across frames.** The last confident estimate, with its age, carries a briefly hidden
  target; a prediction from your measured motion is good for a short time only and is best labelled
  as predicted in your log.
- **Recurring anomalies may be scene features.** The same unexplained event at the same place can be
  geometry TASK.md does not mention; careful probing tells more than thresholds tuned around it.

## Localizing relative to the scene

- **Static structure as anchor:** walls, edges of fixed objects, lines on the floor. Fitted planes,
  lines and edges give your position relative to them; a wall gives both a distance and a relative
  heading. Dead reckoning from commanded or even measured steps drifts by metres over an episode.
- **Servoing on the target itself.** Near the target, control on its measured relative pose ("0.42 m
  ahead, 0.05 m left, 12 degrees off"), re-measured after each action, beats walking to a computed
  map coordinate.
- **Measured heading** for turns, checked against a scene edge as well. Headings average with a
  circular mean; an arithmetic mean of angles near ±180 degrees is wrong.
- **Position after contact is uncertain.** Pushing, striking, bumping or being pushed moves you in
  ways dead reckoning does not see. Until a landmark confirms the estimate, rules that depend on map
  position (keep-outs, waypoints, boundaries) have nothing reliable to act on.
- **The scene's hard bounds.** An estimate of your own or an object's position outside the scene's
  fixed structure (beyond a wall, inside a solid object, below the floor) is a perception error;
  re-localizing or searching is the way out, not walking toward it.
- **What you can no longer see.** The camera looks down and forward: objects near your feet or behind
  you are invisible. An object's estimate from its last sighting and your measured motion carries
  you, but stepping or reaching into the space where it may be is risky until you see it again.
  Approach paths that keep the target in view until the last step avoid the problem.
- **Searching when lost:** turn back toward the last measured bearing, then widen step by step
  (waist, then body), then step back to enlarge the view. A remembered pose is not a fallback.

## Around contact

Perception of an object in clear view can be accurate while the contact still misses:

- **Re-measure the hand-to-object offset in the current frame right before contact.** The body keeps
  moving during an approach and a grasp, so a contact steered on an object position remembered from
  an earlier frame inherits that drift. Quantities the hand does not hide, such as the object's
  position across your line of approach and its height, often stay measurable while the hand
  partly covers it.
- **A hand in front of the object corrupts what it hides.** Occlusion mostly corrupts the coordinate
  along the hidden part; estimating that coordinate from the visible part and the object's known
  extent is more reliable than trusting the shrunken outline.
- **Very close objects are only partly in view.** Near the body the camera sees only part of an
  object; stepping back a little or using the object's known extent from TASK.md fills in the rest.
- **Re-perceive after closing the hand** to see whether the object moved with it, moved away, or
  stayed.

A record in `memory/` of what each detector assumes (support plane, height band, size, colour) and
its hit rate over your episodes, with the conditions of its misses, shows where it can be trusted.
