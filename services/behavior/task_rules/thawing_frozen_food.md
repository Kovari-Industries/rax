**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal
        (and
            (not
                (frozen chicken_breast.n.02_1)
            )
            (exists
                (?plate.n.04 - plate.n.04)
                (and
                    (ontop chicken_breast.n.02_1 ?plate.n.04)
                    (inside ?plate.n.04 microwave.n.02_1)
                )
            )
            (toggled_on microwave.n.02_1)
            (not
                (open microwave.n.02_1)
            )
            (not
                (open electric_refrigerator.n.01_1)
            )
            (not
                (frozen bread_slice.n.01_1)
            )
            (exists
                (?plate.n.04 - plate.n.04)
                (and
                    (ontop bread_slice.n.01_1 ?plate.n.04)
                    (ontop ?plate.n.04 countertop.n.01_1)
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init
        (ontop chicken_breast.n.02_1 plate.n.04_1)
        (ontop bread_slice.n.01_1 plate.n.04_2)
        (inside plate.n.04_1 electric_refrigerator.n.01_1)
        (inside plate.n.04_2 electric_refrigerator.n.01_1)
        (frozen chicken_breast.n.02_1)
        (frozen bread_slice.n.01_1)
        (inroom microwave.n.02_1 kitchen)
        (inroom countertop.n.01_1 kitchen)
        (inroom electric_refrigerator.n.01_1 kitchen)
        (inroom floor.n.01_1 kitchen)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
