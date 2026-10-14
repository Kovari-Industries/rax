**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (exists 
                (?cabinet.n.01 - cabinet.n.01)
                (forall 
                    (?battery.n.02 - battery.n.02)
                    (inside ?battery.n.02 ?cabinet.n.01)
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop battery.n.02_1 coffee_table.n.01_1)
        (ontop battery.n.02_2 coffee_table.n.01_1)
        (ontop battery.n.02_3 countertop.n.01_1)
        (inroom coffee_table.n.01_1 living_room) 
        (inroom cabinet.n.01_1 living_room) 
        (inroom cabinet.n.01_2 living_room) 
        (inroom countertop.n.01_1 kitchen) 
        (inroom floor.n.01_1 living_room) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
