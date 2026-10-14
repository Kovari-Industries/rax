**Goal:** Put every can of soda inside the trash can. The three cans start on the living-room floor; the trash can is on the kitchen floor.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall 
                (?can__of__soda.n.01 - can__of__soda.n.01) 
                (inside ?can__of__soda.n.01 ?ashcan.n.01_1)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop ashcan.n.01_1 floor.n.01_2) 
        (ontop can__of__soda.n.01_1 floor.n.01_1) 
        (ontop can__of__soda.n.01_2 floor.n.01_1) 
        (ontop can__of__soda.n.01_3 floor.n.01_1) 
        (inroom floor.n.01_2 kitchen) 
        (inroom floor.n.01_1 living_room) 
        (ontop agent.n.01_1 floor.n.01_2)
    )
```
