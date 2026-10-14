---
name: base-navigation
description: Find task objects and move the R1Pro base between rooms and workspaces using only cameras and odometry. Use whenever a target is out of view or out of arm's reach, before manipulation, and when returning to a previous place.
---

# Navigation with cameras and odometry

There is no map, object list or world pose. The planner and `behavior_base_move` do
not know where walls or furniture are. Navigation is observe, move a bounded amount,
observe again.

## Search

1. Look with `cam_head` first. If the target is not visible, rotate in place in steps
   of about 0.5-0.8 rad with `behavior_base_move` (`dx=0, dy=0, dyaw=...`) and inspect
   each new view. A full turn is about 8-12 views. Tilt the head with the trunk to look
   at the floor or a surface nearby ([robot-interface](../robot-interface/SKILL.md)).
2. Rooms named in `TASK.md` narrow the search. Doorways and corridors connect rooms;
   head toward an open doorway rather than a wall.
3. Within the episode, note where you saw things in odometry coordinates, so you can
   return to a viewpoint without searching again (convert base-frame points with the
   odometry pose; see [robot-interface](../robot-interface/SKILL.md)).
{% if not formal_episode %}
   Odometry starts over each episode: records kept across episodes describe the
   viewpoint instead, as [pose records](../robot-interface/references/pose-records.md)
   say.
{% endif %}

## Approach

{% if camera_geometry %}
1. Select a pixel on the target or its support surface and convert it with
   `behavior_pixel_to_position`; the result is in the current base frame.
{% else %}
1. Estimate direction from the target's horizontal position in `cam_head` and its
   distance from its apparent size and floor contact; approach in shorter steps than
   you would with depth, re-observing after each.
{% endif %}
2. Turn to face it (`dyaw = atan2(y, x)`), then drive forward to leave a standoff: stop
   with the target about 0.5-0.7 m ahead for table-height manipulation. Split moves
   longer than about 1.5 m and re-observe between them.
3. Before every forward move, check `cam_head` for obstacles along the path. Prefer
   rotating then driving straight over diagonal moves near furniture.
4. After arriving, re-acquire the target in the new base frame; do not reuse points
   measured before the move.

## Reading `behavior_base_move` results

- `reached: true` means odometry reached the goal within tolerance.
- `stalled: true` means the base stopped making progress: it is probably touching
  something. Back off (`dx` negative), rotate, and choose a clearer path.
- `final_position_error_m` and `final_yaw_error_rad` report the remaining odometry error.
- The trunk and arms hold their joints during the move. Raise or tuck an arm that
  could hit furniture before moving.

A move spends one native step per row, like `behavior_step`. Long searches consume the
episode budget: search deliberately and keep notes of what you have seen.
