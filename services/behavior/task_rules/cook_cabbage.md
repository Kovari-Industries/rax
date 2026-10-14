**Goal:** From the kitchen refrigerator, take the cabbage and the chili, dice them on the chopping board with the knife, cook the diced cabbage and diced chili in the frying pan on the stove, and leave the cooked, diced cabbage and cooked, diced chili in the frying pan.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (real ?cooked__diced__head_cabbage.n.01_1)
            (real ?cooked__diced__chili.n.01_1)
            (contains ?frying_pan.n.01_1 ?cooked__diced__head_cabbage.n.01_1)
            (contains ?frying_pan.n.01_1 ?cooked__diced__chili.n.01_1) 
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (inside head_cabbage.n.02_1 electric_refrigerator.n.01_1)
        (ontop frying_pan.n.01_1 countertop.n.01_1) 
        (ontop plate.n.04_1 countertop.n.01_1) 
        (inside chili.n.02_1 electric_refrigerator.n.01_1) 
        (not 
            (cooked head_cabbage.n.02_1)
        ) 
        (ontop chopping_board.n.01_1 countertop.n.01_1)
        (ontop carving_knife.n.01_1 countertop.n.01_1)
        (future cooked__diced__chili.n.01_1)
        (inroom electric_refrigerator.n.01_1 kitchen) 
        (inroom stove.n.01_1 kitchen) 
        (inroom floor.n.01_1 kitchen) 
        (inroom countertop.n.01_1 kitchen)
        (future cooked__diced__head_cabbage.n.01_1)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
