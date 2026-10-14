**Goal:** Place the modem and the television receiver on the same living-room cabinet, with the modem next to the receiver, and switch the modem on. The modem starts switched off on the coffee table.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (toggled_on ?modem.n.01_1) 
            (nextto ?modem.n.01_1 ?television_receiver.n.01_1)
            (exists 
                (?cabinet.n.01 - cabinet.n.01)
                (and 
                    (ontop ?television_receiver.n.01_1 ?cabinet.n.01)
                    (ontop ?modem.n.01_1 ?cabinet.n.01)
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (not 
            (toggled_on modem.n.01_1)
        ) 
        (ontop modem.n.01_1 coffee_table.n.01_1) 
        (inroom coffee_table.n.01_1 living_room) 
        (inroom cabinet.n.01_1 living_room) 
        (inroom cabinet.n.01_2 living_room) 
        (inroom television_receiver.n.01_1 living_room) 
        (inroom floor.n.01_1 living_room) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
