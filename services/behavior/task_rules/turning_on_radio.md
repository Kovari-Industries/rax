**Goal:** Switch the radio receiver on. It starts switched off on a table in the living room.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (toggled_on ?radio_receiver.n.01_1)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (not 
            (toggled_on radio_receiver.n.01_1)
        ) 
        (ontop radio_receiver.n.01_1 table.n.02_1)
        (inroom table.n.02_1 living_room)
        (inroom floor.n.01_1 living_room)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
