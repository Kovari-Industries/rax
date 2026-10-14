**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and
            (forall
                (?clove.n.03 - clove.n.03)
                (and
                    (ontop ?clove.n.03 ?frying_pan.n.01_1)
                    (cooked ?clove.n.03)
                )
            )
            (forall
                (?broccolini.n.01 - broccolini.n.01)
                (and
                    (ontop ?broccolini.n.01 ?frying_pan.n.01_1)
                    (cooked ?broccolini.n.01)
                )
            )
            (ontop ?frying_pan.n.01_1 ?stove.n.01_1)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop broccolini.n.01_1 plate.n.04_1)
        (ontop broccolini.n.01_2 plate.n.04_1) 
        (ontop broccolini.n.01_3 plate.n.04_1) 
        (ontop clove.n.03_1 plate.n.04_1)
        (ontop clove.n.03_2 plate.n.04_1)
        (ontop plate.n.04_1 countertop.n.01_1)
        (ontop frying_pan.n.01_1 countertop.n.01_1) 
        (not  
            (cooked broccolini.n.01_1)
        ) 
        (inroom stove.n.01_1 kitchen) 
        (inroom countertop.n.01_1 kitchen) 
        (inroom floor.n.01_1 kitchen) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
