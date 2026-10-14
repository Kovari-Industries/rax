**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall 
                (?tile.n.01 - tile.n.01) 
                (ontop ?tile.n.01 ?floor.n.01_2)
            )
            (forall
                (?tile.n.01 - tile.n.01)
                (or
                    (nextto ?tile.n.01 tile.n.01_1)
                    (nextto ?tile.n.01 tile.n.01_2)
                    (nextto ?tile.n.01 tile.n.01_3)
                    (nextto ?tile.n.01 tile.n.01_4)
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop tile.n.01_1 floor.n.01_1) 
        (ontop tile.n.01_2 floor.n.01_1) 
        (ontop tile.n.01_3 floor.n.01_1) 
        (ontop tile.n.01_4 floor.n.01_1) 
        (inroom floor.n.01_1 corridor) 
        (inroom floor.n.01_2 bathroom) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
