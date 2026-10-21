---
name: live-stream
description: Read bounded Evo state history and observation metadata. Use a fresh observation to inspect the current scene.
---
# Evo observations and recent history

Call `evo_observe {}` before motion after a pause. It returns fresh RGB images,
capture timestamps, calibration validity, and associated measured state. Inspect
camera errors. Images are saved under `runtime/frames/` in the agent workspace.

Call `evo_history {}` to read the bounded recent history. This tool takes no
arguments. It returns `states`, `observations`, and `recording`. The observations
are metadata from earlier observation calls; they do not include image bytes.
The recording value is an operator-owned bag path, not an agent-readable image
folder. History is bounded by sample counts, not a fixed time interval. Read the
timestamps to find its actual coverage.

```mermaid
flowchart LR
    A[evo_observe] --> B[Fresh images and capture state]
    C[evo_history] --> D[Recent state and observation metadata]
```

Python programs use the same tools:

```python
from api.runtime import Context

with Context() as ctx:
    observation = ctx.value("evo_observe")
    history = ctx.value("evo_history")
```

Use saved observation IDs with `evo_camera_ray` only when camera geometry is valid.
Rays do not supply depth. Save the images and results needed for task evidence in
the workspace. Request a new observation to check scene changes; state history
alone cannot show a moved object.
