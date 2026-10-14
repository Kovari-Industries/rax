**Goal:** In the bedroom, move the book from the bed onto either nightstand, and place the two sandals side by side next to the bed.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (nextto ?sandal.n.01_1 ?bed.n.01_1) 
            (nextto ?sandal.n.01_2 ?sandal.n.01_1) 
            (exists 
                (?table.n.02 - table.n.02) 
                (ontop ?book.n.02_1 ?table.n.02)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop sandal.n.01_1 floor.n.01_1) 
        (ontop sandal.n.01_2 floor.n.01_1) 
        (ontop book.n.02_1 bed.n.01_1) 
        (inroom table.n.02_1 bedroom) 
        (inroom table.n.02_* bedroom)
        (inroom bed.n.01_1 bedroom) 
        (inroom floor.n.01_1 bedroom) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
