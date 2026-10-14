**Goal:** Put all six books from the bookcases in the living room into the box on the living room floor.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall 
                (?book.n.02 - book.n.02) 
                (inside ?book.n.02 ?box.n.01_1)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (inside book.n.02_1 bookcase.n.01_1)
        (inside book.n.02_2 bookcase.n.01_1)
        (inside book.n.02_3 bookcase.n.01_2)
        (inside book.n.02_4 bookcase.n.01_2)
        (inside book.n.02_5 bookcase.n.01_3)
        (inside book.n.02_6 bookcase.n.01_4) 
        (ontop box.n.01_1 floor.n.01_1) 
        (inroom floor.n.01_1 living_room) 
        (inroom bookcase.n.01_1 living_room) 
        (inroom bookcase.n.01_2 living_room)
        (inroom bookcase.n.01_3 living_room)
        (inroom bookcase.n.01_4 living_room)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
