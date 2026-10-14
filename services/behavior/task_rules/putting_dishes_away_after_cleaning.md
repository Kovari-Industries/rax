**Goal:** In the kitchen, gather all eight plates from the two countertops, place them all inside a single cabinet (either one), and make sure all cabinets are closed when you're done.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (exists 
                (?cabinet.n.01 - cabinet.n.01) 
                (forall 
                    (?plate.n.04 - plate.n.04) 
                    (inside ?plate.n.04 ?cabinet.n.01)
                )
            )
            (forall
                (?cabinet.n.01 - cabinet.n.01)
                (not
                    (open ?cabinet.n.01)
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop plate.n.04_1 countertop.n.01_1) 
        (ontop plate.n.04_2 countertop.n.01_1) 
        (ontop plate.n.04_3 countertop.n.01_1) 
        (ontop plate.n.04_4 countertop.n.01_1) 
        (ontop plate.n.04_5 countertop.n.01_2) 
        (ontop plate.n.04_6 countertop.n.01_2) 
        (ontop plate.n.04_7 countertop.n.01_2) 
        (ontop plate.n.04_8 countertop.n.01_2) 
        (inroom countertop.n.01_1 kitchen) 
        (inroom countertop.n.01_2 kitchen) 
        (inroom cabinet.n.01_1 kitchen) 
        (inroom cabinet.n.01_* kitchen)
        (inroom floor.n.01_1 kitchen) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
