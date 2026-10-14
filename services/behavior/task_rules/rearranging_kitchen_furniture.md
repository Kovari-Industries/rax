**Goal:** Move the toaster, food processor, and French press from the kitchen countertop into the same kitchen cabinet, and make sure that cabinet is closed at the end.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (exists
                (?cabinet.n.01 - cabinet.n.01)
                (and
                    (inside toaster.n.02_1 ?cabinet.n.01)
                    (inside food_processor.n.01_1 ?cabinet.n.01)
                    (inside french_press.n.01_1 ?cabinet.n.01)
                    (not
                        (open ?cabinet.n.01)
                    )
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop toaster.n.02_1 countertop.n.01_1)
        (ontop food_processor.n.01_1 countertop.n.01_1)
        (ontop french_press.n.01_1 countertop.n.01_1)
        (inroom floor.n.01_1 kitchen) 
        (inroom countertop.n.01_1 kitchen) 
        (inroom cabinet.n.01_1 kitchen) 
        (inroom cabinet.n.01_* kitchen)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
