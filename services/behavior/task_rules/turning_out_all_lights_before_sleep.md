**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall 
                (?switch.n.01 - switch.n.01)
                (not 
                    (toggled_on ?switch.n.01)
                )
            )
            (forall
                (?table_lamp.n.01 - table_lamp.n.01)
                (not
                    (toggled_on ?table_lamp.n.01)
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (toggled_on switch.n.01_1)
        (toggled_on switch.n.01_2)
        (toggled_on switch.n.01_3)
        (toggled_on table_lamp.n.01_1)
        (toggled_on table_lamp.n.01_2)
        (inroom switch.n.01_1 kitchen)
        (inroom switch.n.01_2 utility_room)
        (inroom switch.n.01_3 dining_room)
        (inroom coffee_table.n.01_1 living_room)
        (inroom countertop.n.01_1 utility_room)
        (inroom floor.n.01_1 corridor)
        (ontop table_lamp.n.01_1 coffee_table.n.01_1)
        (ontop table_lamp.n.01_2 countertop.n.01_1)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
