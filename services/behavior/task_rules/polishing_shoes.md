**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal
        (and
            (ontop scrub_brush.n.01_1 stand.n.04_1)
            (not
                (covered ?shoe.n.01_1 ?dust.n.01_1)
            )
            (not
                (covered ?shoe.n.01_2 ?dust.n.01_1)
            )
            (nextto ?shoe.n.01_1 ?shoe.n.01_2)
            (exists
                (?footstool.n.01 - footstool.n.01)
                (and
                    (nextto ?shoe.n.01_1 ?footstool.n.01)
                    (nextto ?shoe.n.01_2 ?footstool.n.01)
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init
        (covered shoe.n.01_1 dust.n.01_1)
        (covered shoe.n.01_2 dust.n.01_1)
        (ontop scrub_brush.n.01_1 stand.n.04_1)
        (ontop shoe.n.01_1 floor.n.01_1)
        (ontop shoe.n.01_2 floor.n.01_1)
        (inroom floor.n.01_1 bedroom)
        (inroom stand.n.04_1 bedroom)
        (inroom footstool.n.01_1 bedroom)
        (inroom footstool.n.01_2 bedroom)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
