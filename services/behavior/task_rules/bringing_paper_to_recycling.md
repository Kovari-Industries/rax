**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal
        (and
       	    (inside sack.n.01_1 ?recycling_bin.n.01_1)
       	    (inside newspaper.n.03_1 ?recycling_bin.n.01_1)
            (not
                (open ?recycling_bin.n.01_1)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init
        (ontop sack.n.01_1 floor.n.01_1)
        (ontop newspaper.n.03_1 countertop.n.01_1)
        (ontop recycling_bin.n.01_1 floor.n.01_2)
        (inroom countertop.n.01_1 kitchen)
        (inroom floor.n.01_1 kitchen)
        (inroom floor.n.01_2 garden)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
