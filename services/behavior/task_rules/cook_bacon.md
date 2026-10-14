**Goal:** Take the tray with six slices of bacon out of the refrigerator in the kitchen, cook all six slices in the frying pan on the stove until they're cooked, and make sure the refrigerator is closed when you're done.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall 
                (?bacon.n.01 - bacon.n.01)
                (cooked ?bacon.n.01)
            )
            (not
                (open ?electric_refrigerator.n.01_1)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop bacon.n.01_1 tray.n.01_1) 
        (ontop bacon.n.01_2 tray.n.01_1) 
        (ontop bacon.n.01_3 tray.n.01_1) 
        (ontop bacon.n.01_4 tray.n.01_1) 
        (ontop bacon.n.01_5 tray.n.01_1) 
        (ontop bacon.n.01_6 tray.n.01_1) 
        (not 
            (cooked bacon.n.01_1)
        ) 
        (not 
            (cooked bacon.n.01_2)
        ) 
        (not 
            (cooked bacon.n.01_3)
        ) 
        (not 
            (cooked bacon.n.01_4)
        ) 
        (not 
            (cooked bacon.n.01_5)
        ) 
        (not 
            (cooked bacon.n.01_6)
        ) 
        (inside tray.n.01_1 electric_refrigerator.n.01_1)
        (ontop frying_pan.n.01_1 stove.n.01_1)
        (inroom stove.n.01_1 kitchen) 
        (inroom electric_refrigerator.n.01_1 kitchen) 
        (inroom floor.n.01_1 kitchen) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
