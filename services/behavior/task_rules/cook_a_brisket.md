**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (cooked ?brisket.n.01_1) 
            (ontop ?brisket.n.01_1 ?chopping_board.n.01_1)
            (ontop ?frying_pan.n.01_1 ?countertop.n.01_1)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop chopping_board.n.01_1 countertop.n.01_1) 
        (inside brisket.n.01_1 electric_refrigerator.n.01_1) 
        (not 
            (cooked brisket.n.01_1)
        )
        (ontop frying_pan.n.01_1 countertop.n.01_1)
        (inroom electric_refrigerator.n.01_1 kitchen) 
        (inroom oven.n.01_1 kitchen) 
        (inroom floor.n.01_1 kitchen) 
        (inroom countertop.n.01_1 kitchen)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
