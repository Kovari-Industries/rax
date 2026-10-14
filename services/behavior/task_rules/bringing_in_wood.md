**Goal:** Bring the three plywood sheets from the garden into the corridor and place them on the floor there.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall 
                (?plywood.n.01 - plywood.n.01) 
                (ontop ?plywood.n.01 ?floor.n.01_2)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop plywood.n.01_1 floor.n.01_1) 
        (ontop plywood.n.01_2 floor.n.01_1) 
        (ontop plywood.n.01_3 floor.n.01_1) 
        (inroom floor.n.01_1 garden) 
        (inroom floor.n.01_2 corridor) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
