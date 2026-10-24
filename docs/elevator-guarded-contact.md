# Guarded final elevator contact

This mode is disabled by default. It is an explicit contact transaction for a
camera and tool configuration whose label becomes occluded near contact. It is
not a fallback after failed recognition. CPU contract tests do not qualify the
optics, collision model, dynamics, or hardware.

```text
visible alignment -> fresh image target -> complete preview -> freshness gate
                  -> guarded approach/press/dwell/withdraw
                  -> fresh visible target confirmation -> stow
```

Use `FinalContactPolicy(visible_gap_m, max_inward_travel_m,
occlusion_qualification_id)` as `ElevatorSettings.final_contact`. The runtime must
advertise both `evo_capabilities.scene.guarded_contact = true` and the
`evo_guarded_contact` tool. It must also advertise exact records in
`evo_capabilities.scene.guarded_contact_profiles`:

```json
[{"panel_id":"cabin-panel","camera":"right_arm","occlusion_qualification_id":"reviewed-right-id"}]
```

The policy qualification ID can be the existing string, or an explicit mapping
such as `{"left": "reviewed-left-id", "right": "reviewed-right-id"}`. Mapping keys
must be `left` or `right`; an absent side has no fallback. Each selected side must
match the configured panel ID, wrist camera, and configured qualification ID.
Missing, malformed, or duplicate advertisement records are refused. A string
still requires a matching camera record for each eligible arm.

Search and arm selection exclude unqualified arms before viewing motion or reach
preview. Fresh capability reads check admission again before preparation and
after each preparation preview, then after the complete contact preview. The
final action contains the selected arm's scalar qualification ID. Hall and cabin
tasks have distinct panel IDs; the adapter must keep a durable attempt latch per
panel across images and arm choices. Changing arms cannot retry an uncertain
contact on that panel.

The qualification must bind the camera, pressing tool, source profile, and
visibility at the initial and withdrawal gaps. Configure no policy until that
qualification exists.

The guarded route finishes orientation, carriage, and tangential alignment while
outside the visible gap. It preserves the existing orientation sweep check. It
then uses normal observed approach steps only as far as the visible gap. It does
not call the legacy inward preparation path. The visible approach plus stroke
must fit the existing approach budget.

The preview is `evo_preview(action={"kind": "guarded_contact", **args})`.
Execution is `evo_guarded_contact(**args)` with identical arguments:

| Field | Content |
| --- | --- |
| `frame`, `side` | `base_link`, and the selected arm |
| `observation` | `observation_id`, `camera`, `capture_us`, `panel_id`, `label`, `receipt_id`, `image_sha256`, `position`, `normal`, `uncertainty_m`, `scene_pose`, `carriage_position_m`, `surface_face` |
| `surface_face` | Captured finite face: `center`, `horizontal_axis`, `vertical_axis`, `width_m`, `height_m`, `source` |
| `tip_in_palm_m`, `press_quaternion_xyzw` | Selected profile calibration |
| `occlusion_qualification_id` | Explicit runtime qualification identity |
| `visible_gap_m`, `max_inward_travel_m` | Explicit bounded final approach |
| `prepress_gap_m`, `approach_step_m`, `segment_s`, `stroke_m`, `dwell_s` | Existing settings, unchanged |
| `retract_gap_m` | Maximum of existing retract gap and qualified visible gap |
| `position_tolerance_m` | Minimum of existing tolerance and 0.004 m |
| `angle_tolerance_rad` | Minimum of existing tolerance and 0.01 rad |

At least one of `receipt_id` and a valid `image_sha256` is required. The adapter
must bind that identity to the actual camera capture and the finite surface. The
captured target uncertainty must fit inside the face. A known simulation face
must remain explicitly labeled as simulation geometry, not measured depth.

Rax checks the original image age and post-motion timestamp after preview and
again after the final state queries. It does not change the capture time. It
reserves one attempt before dispatch. The runtime must continuously enforce
source limits, fixed base/carriage, panel identity and geometry, and the bounded
path. It must apply the existing measured endpoint and stop gates at prepress,
press, and withdrawal; an ordinary multiwaypoint path alone does not supply
those intermediate stop gates.

The action result must have `state="succeeded"`, `physical_stop_verified=true`,
and a `guarded_contact` receipt with matching `observation_id`, `capture_us`,
`side`, and `target`. These fields must each be true: `prepress_stop_verified`,
`press_stop_verified`, `withdrawal_stop_verified`, `source_limits_verified`,
`fixed_geometry_verified`, `single_correct_press`, and `released`.

Rax independently checks the final source FK, base/carriage, finite panel, and
withdrawal orientation. It then requires fresh unambiguous OCR of the same
metric target after withdrawal. Only then can it clear contact state and stow.
Failure leaves contact state latched. It cannot retry, stow, or clear the state
through generic retraction. Explicit recovery requires a separate reviewed
procedure. Successful completion also consumes the instance's guarded attempt.

`ElevatorSettings.coarse_panel_lateral_offset_m` is separately optional and
normally used for the exterior hall call. In `press_elevator_button`, it stages
the compact robot once relative to the identified finite panel frame, before the
first image search. It uses existing base preview, step, endpoint, and stop
checks. Fresh image recognition and reach preview still select the arm. This
option skips the later legacy lateral reposition and second search. It does not
infer a label or physical button center. `CabinRide` retains its separate checked
inside-cabin staging route.
