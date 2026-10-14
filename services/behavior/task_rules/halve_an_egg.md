**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (real ?half__hard-boiled_egg.n.01_1)
            (real ?half__hard-boiled_egg.n.01_2)
            (ontop ?half__hard-boiled_egg.n.01_1 ?plate.n.04_1)
            (ontop ?half__hard-boiled_egg.n.01_2 ?plate.n.04_1)
            (inside ?carving_knife.n.01_1 ?sink.n.01_1)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop chopping_board.n.01_1 countertop.n.01_1) 
        (ontop plate.n.04_1 countertop.n.01_1) 
        (ontop carving_knife.n.01_1 chopping_board.n.01_1) 
        (inside hard-boiled_egg.n.01_1 electric_refrigerator.n.01_1) 
        (cooked hard-boiled_egg.n.01_1)
        (inroom electric_refrigerator.n.01_1 kitchen) 
        (inroom countertop.n.01_1 kitchen) 
        (inroom sink.n.01_1 kitchen) 
        (inroom floor.n.01_1 kitchen) 
        (future half__hard-boiled_egg.n.01_1)
        (future half__hard-boiled_egg.n.01_2)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
