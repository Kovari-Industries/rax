**Goal:** Take the two hot dogs out of the refrigerator in the kitchen and cook them in the microwave until both are cooked.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall 
                (?hotdog.n.02 - hotdog.n.02) 
                (cooked ?hotdog.n.02)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (inside hotdog.n.02_1 electric_refrigerator.n.01_1) 
        (inside hotdog.n.02_2 electric_refrigerator.n.01_1) 
        (not 
            (cooked hotdog.n.02_1)
        ) 
        (not 
            (cooked hotdog.n.02_2)
        ) 
        (inroom electric_refrigerator.n.01_1 kitchen) 
        (inroom microwave.n.02_1 kitchen) 
        (inroom floor.n.01_1 kitchen) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
