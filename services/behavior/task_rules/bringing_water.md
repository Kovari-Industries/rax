**Goal:** Retrieve the two bottles from the refrigerator in the kitchen, bring them to the living room, and place both on the coffee table. Make sure the refrigerator is closed when you finish.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall
                (?bottle.n.01 - bottle.n.01)
                (ontop ?bottle.n.01 ?coffee_table.n.01_1)
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
        (inside bottle.n.01_1 electric_refrigerator.n.01_1) 
        (inside bottle.n.01_2 electric_refrigerator.n.01_1)
        (inroom electric_refrigerator.n.01_1 kitchen) 
        (inroom coffee_table.n.01_1 living_room)
        (inroom floor.n.01_1 kitchen)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
