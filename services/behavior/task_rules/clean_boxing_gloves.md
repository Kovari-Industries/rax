**Goal:** Wash the two dusty boxing gloves from the countertop in the utility room in the washer until they are no longer covered with dust.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall 
                (?boxing_glove.n.01 - boxing_glove.n.01)
                (not 
                    (covered ?boxing_glove.n.01 ?dust.n.01_1)
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop boxing_glove.n.01_1 countertop.n.01_1) 
        (ontop boxing_glove.n.01_2 countertop.n.01_1) 
        (covered boxing_glove.n.01_1 dust.n.01_1)
        (covered boxing_glove.n.01_2 dust.n.01_1)
        (inroom washer.n.03_1 utility_room)
        (inroom countertop.n.01_1 utility_room)
        (inroom floor.n.01_1 utility_room)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
