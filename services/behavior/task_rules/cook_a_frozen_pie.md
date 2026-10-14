**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (ontop apple_pie.n.01_1 tray.n.01_1)
            (cooked ?apple_pie.n.01_1)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (inside apple_pie.n.01_1 electric_refrigerator.n.01_1) 
        (frozen apple_pie.n.01_1)
        (inroom countertop.n.01_1 kitchen)
        (inroom oven.n.01_1 kitchen) 
        (inroom floor.n.01_1 kitchen) 
        (inroom electric_refrigerator.n.01_1 kitchen)
        (ontop agent.n.01_1 floor.n.01_1)
        (ontop tray.n.01_1 countertop.n.01_1)
    )
```
