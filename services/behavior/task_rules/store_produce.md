**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (exists
                (?electric_refrigerator.n.01 - electric_refrigerator.n.01)
                (and
                    (forall 
                        (?pomegranate.n.02 - pomegranate.n.02)
                        (inside ?pomegranate.n.02 ?electric_refrigerator.n.01) 
                    )
                    (forall 
                        (?mango.n.02 - mango.n.02)
                        (inside ?mango.n.02 ?electric_refrigerator.n.01) 
                    )
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (inside mango.n.02_1 wicker_basket.n.01_1) 
        (inside mango.n.02_2 wicker_basket.n.01_1) 
        (inside pomegranate.n.02_1 wicker_basket.n.01_1) 
        (inside pomegranate.n.02_2 wicker_basket.n.01_1) 
        (ontop wicker_basket.n.01_1 floor.n.01_1) 
        (inroom electric_refrigerator.n.01_1 kitchen) 
        (inroom electric_refrigerator.n.01_2 kitchen) 
        (inroom floor.n.01_1 kitchen) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
