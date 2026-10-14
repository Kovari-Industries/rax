**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal
        (and
            (ontop ?lawn_chair.n.01_1 ?floor.n.01_2)
            (exists
                (?lawn.n.01 - lawn.n.01)
                (ontop ?barrow.n.03_1 ?lawn.n.01)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop lawn_chair.n.01_1 floor.n.01_1)
        (ontop barrow.n.03_1 floor.n.01_1) 
        (inroom lawn.n.01_1 garden) 
        (inroom floor.n.01_1 living_room)
        (inroom floor.n.01_2 garden) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
