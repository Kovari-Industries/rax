**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal
        (and
            (forall
                (?bowl.n.01 - bowl.n.01)
                (inside ?bowl.n.01 ?sink.n.01_1)
            )
            (forall
                (?plate.n.04 - plate.n.04)
                (inside ?plate.n.04 ?sink.n.01_1)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init
        (covered bowl.n.01_1 dirt.n.02_1)
        (covered bowl.n.01_2 dirt.n.02_1)
        (covered plate.n.04_1 stain.n.01_1)
        (covered plate.n.04_2 stain.n.01_1)
        (ontop bowl.n.01_1 booth.n.01_1)
        (ontop bowl.n.01_2 booth.n.01_1)
        (ontop plate.n.04_1 booth.n.01_1)
        (ontop plate.n.04_2 booth.n.01_1)
        (inroom floor.n.01_1 dining_room)
        (inroom sink.n.01_1 kitchen)
        (inroom booth.n.01_1 dining_room)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
