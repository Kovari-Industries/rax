**Goal:** Wash the two baseball caps on the countertop in the utility room using the washer until they are no longer dirty.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall
                (?baseball_cap.n.01 - baseball_cap.n.01)
                (not 
                    (covered ?baseball_cap.n.01 ?dirt.n.02_1)
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop baseball_cap.n.01_1 countertop.n.01_1)
        (covered baseball_cap.n.01_1 dirt.n.02_1) 
        (ontop baseball_cap.n.01_2 countertop.n.01_1)
        (covered baseball_cap.n.01_2 dirt.n.02_1) 
        (inroom floor.n.01_1 utility_room) 
        (inroom washer.n.03_1 utility_room) 
        (inroom countertop.n.01_1 utility_room)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
