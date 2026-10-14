**Goal:** Attach the digital camera to the camera tripod. Both start on the bedroom floor.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (attached ?digital_camera.n.01_1 ?camera_tripod.n.01_1)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop digital_camera.n.01_1 floor.n.01_1)
        (ontop camera_tripod.n.01_1 floor.n.01_1)
        (inroom floor.n.01_1 bedroom) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
