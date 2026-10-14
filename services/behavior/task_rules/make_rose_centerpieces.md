**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (inside ?rose.n.01_1 ?vase.n.01_1) 
            (inside ?rose.n.01_2 ?vase.n.01_1) 
            (inside ?rose.n.01_3 ?vase.n.01_1) 
            (ontop ?vase.n.01_1 ?coffee_table.n.01_1)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop vase.n.01_1 cabinet.n.01_1) 
        (ontop rose.n.01_1 cabinet.n.01_1) 
        (ontop rose.n.01_2 cabinet.n.01_1) 
        (ontop rose.n.01_3 cabinet.n.01_1) 
        (inroom cabinet.n.01_1 living_room) 
        (inroom coffee_table.n.01_1 living_room) 
        (inroom floor.n.01_1 living_room) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
