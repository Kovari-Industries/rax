**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal
        (and
            (forall
                (?book.n.02 - book.n.02)
                (exists
                    (?bookcase.n.01 - bookcase.n.01)
                    (inside ?book.n.02 ?bookcase.n.01)
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop book.n.02_1 table.n.02_1) 
        (ontop book.n.02_2 table.n.02_1) 
        (ontop book.n.02_3 table.n.02_1) 
        (inroom table.n.02_1 living_room)
        (inroom bookcase.n.01_* living_room)
        (inroom floor.n.01_1 living_room) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
