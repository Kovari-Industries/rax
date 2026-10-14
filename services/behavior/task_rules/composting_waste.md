**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (inside ?half__banana.n.01_1 ?ashcan.n.01_1) 
            (inside ?half__pomegranate.n.01_1 ?ashcan.n.01_1) 
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop half__banana.n.01_1 countertop.n.01_1) 
        (ontop half__pomegranate.n.01_1 countertop.n.01_1) 
        (ontop ashcan.n.01_1 floor.n.01_1) 
        (inroom countertop.n.01_1 kitchen) 
        (inroom floor.n.01_1 kitchen) 
        (inroom floor.n.01_2 living_room)
        (inroom floor.n.01_3 entryway)
        (ontop agent.n.01_1 floor.n.01_2)
    )
```
