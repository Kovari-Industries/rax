**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal
        (and
            (forall
                (?taco.n.02 - taco.n.02)
                (inside ?taco.n.02 ?tupperware.n.01_1)
            )
	    (ontop ?plate.n.04_1 sink.n.01_1)
	    (ontop ?plate.n.04_2 sink.n.01_1)
            (inside ?tupperware.n.01_1 ?electric_refrigerator.n.01_1)
            (not
                (open ?electric_refrigerator.n.01_1)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init
        (ontop taco.n.02_1 plate.n.04_1)
        (ontop taco.n.02_2 plate.n.04_2)
        (ontop plate.n.04_1 countertop.n.01_1)
        (ontop plate.n.04_2 countertop.n.01_1)
        (inroom countertop.n.01_1 kitchen)
        (inside tupperware.n.01_1 cabinet.n.01_1)
        (inroom cabinet.n.01_1 kitchen)
        (inroom sink.n.01_1 kitchen)
        (inroom electric_refrigerator.n.01_1 kitchen)
        (inroom floor.n.01_1 kitchen)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
