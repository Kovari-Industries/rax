**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (attached fire_alarm.n.02_1 wall_nail.n.01_1)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop fire_alarm.n.02_1 table.n.02_1) 
        (inroom wall_nail.n.01_1 living_room)
        (inroom table.n.02_1 living_room)
        (inroom floor.n.01_1 living_room)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
