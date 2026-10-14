**Goal:** Hang the poster on one of the wall nails: the poster must end up attached to a nail. The poster starts on the kitchen countertop; the nails are in the kitchen.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (exists
                (?wall_nail.n.01 - wall_nail.n.01)
                (attached ?poster.n.01_1 ?wall_nail.n.01)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop poster.n.01_1 countertop.n.01_1)
        (inroom floor.n.01_1 kitchen)
        (inroom countertop.n.01_1 kitchen)
        (inroom wall_nail.n.01_1 kitchen)
        (inroom wall_nail.n.01_* kitchen)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
