**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (exists
                (?stand.n.04 - stand.n.04)
                (ontop tissue_dispenser.n.01_1 ?stand.n.04)
            )
            (forpairs
                (?pillow.n.01 - pillow.n.01)
                (?bed.n.01 - bed.n.01)
                (ontop ?pillow.n.01 ?bed.n.01)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop pillow.n.01_1 floor.n.01_1) 
        (ontop pillow.n.01_2 floor.n.01_1) 
        (inroom floor.n.01_1 bedroom) 
        (inroom floor.n.01_2 bathroom)
        (inroom bed.n.01_1 bedroom)
        (inroom bed.n.01_2 bedroom)
        (inroom stand.n.04_* bedroom)
        (inroom armchair.n.01_1 bedroom)
        (ontop tissue_dispenser.n.01_1 armchair.n.01_1)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
