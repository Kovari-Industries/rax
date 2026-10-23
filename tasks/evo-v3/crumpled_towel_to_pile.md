# Unfold, fold, and stack a crumpled towel

Start with a crumpled towel in the robot workspace and a measured empty placement
region or towel pile. Read the `robot-interface` and `cloth-handling` skills. Use
fresh calibrated images, measured palm grasp targets, measured surface height,
and a fresh pile top. Do not reuse historical contact coordinates.

```mermaid
flowchart LR
    A[Pick visible point] --> B[Lift and inspect]
    B --> C[Regrasp visible edge]
    C --> D[Spread and lay down]
    D --> E[Verify flat rectangle]
    E --> F[Fold and verify each step]
    F --> G[Pick complete packet]
    G --> H[Place on pile and verify release]
```

Use `Cloth.crumpled_to_pile` with measured landing palm poses, a bounded span, and
a fold pattern that fits the robot workspace. Its perception function must provide
current scene and grasp evidence. Keep the base fixed. Record the observations,
actions, and first failed check. Stop when a second grasp is not visible, the
unfold attempt budget is exhausted, a motion fails, or a grasp or layer check is
uncertain. Use a new observation to choose a recovery; do not retry blindly.

Success requires a released folded packet, verified support on the pile, and a
stable final image. Report success, failure, uncertain, or blocked before motion.
Include image and recording paths. Motion completion is not cloth success.
