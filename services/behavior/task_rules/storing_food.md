**Goal:** Put away all the food on the kitchen countertop by storing it inside the kitchen cabinets: move the two boxes of oatmeal, two bags of chips, two bottles of olive oil, and two jars of sugar from the countertop into the kitchen cabinets (each item can go into either cabinet).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall
                (?box__of__oatmeal.n.01 - box__of__oatmeal.n.01)
                (exists 
                    (?cabinet.n.01 - cabinet.n.01) 
                    (inside ?box__of__oatmeal.n.01 ?cabinet.n.01)
                )
            )
            (forall
                (?bag__of__chips.n.01 - bag__of__chips.n.01)
                (exists 
                    (?cabinet.n.01 - cabinet.n.01) 
                    (inside ?bag__of__chips.n.01 ?cabinet.n.01)
                )
            )
            (forall
                (?bottle__of__olive_oil.n.01 - bottle__of__olive_oil.n.01)
                (exists 
                    (?cabinet.n.01 - cabinet.n.01) 
                    (inside ?bottle__of__olive_oil.n.01 ?cabinet.n.01)
                )
            )
            (forall
                (?jar__of__sugar.n.01 - jar__of__sugar.n.01)
                (exists 
                    (?cabinet.n.01 - cabinet.n.01) 
                    (inside ?jar__of__sugar.n.01 ?cabinet.n.01)
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop box__of__oatmeal.n.01_1 countertop.n.01_1) 
        (ontop box__of__oatmeal.n.01_2 countertop.n.01_1) 
        (ontop bag__of__chips.n.01_1 countertop.n.01_1) 
        (ontop bag__of__chips.n.01_2 countertop.n.01_1) 
        (ontop bottle__of__olive_oil.n.01_1 countertop.n.01_1) 
        (ontop bottle__of__olive_oil.n.01_2 countertop.n.01_1) 
        (ontop jar__of__sugar.n.01_1 countertop.n.01_1) 
        (ontop jar__of__sugar.n.01_2 countertop.n.01_1) 
        (inroom countertop.n.01_1 kitchen) 
        (inroom cabinet.n.01_1 kitchen) 
        (inroom cabinet.n.01_* kitchen)
        (inroom floor.n.01_1 kitchen) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
