**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal
        (and
            (forall
                (?branch.n.02 - branch.n.02)
                (inside ?branch.n.02 ?recycling_bin.n.01_1)
            )
            (ontop ?recycling_bin.n.01_1 ?floor.n.01_1)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init
        (ontop branch.n.02_1 floor.n.01_1)
        (ontop branch.n.02_2 floor.n.01_1)
        (ontop branch.n.02_3 floor.n.01_1)
        (ontop branch.n.02_4 floor.n.01_1)
        (ontop recycling_bin.n.01_1 floor.n.01_1)
        (ontop agent.n.01_1 floor.n.01_1)
        (inroom floor.n.01_1 garden)
    )
```
