**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall 
                (?comic_book.n.01 - comic_book.n.01)
                (inside ?comic_book.n.01 ?bookcase.n.01_1)
            )
            (forn
                (2)
                (?comic_book.n.01 - comic_book.n.01)
                (or 
                    (ontop ?comic_book.n.01 ?comic_book.n.01_1)
                    (ontop ?comic_book.n.01 ?comic_book.n.01_2)
                    (ontop ?comic_book.n.01 ?comic_book.n.01_3)
                )
            )
            (forall 
                (?notebook.n.01 - notebook.n.01)
                (inside ?notebook.n.01 ?bookcase.n.01_1)
            )
            (exists
                (?notebook.n.01 - notebook.n.01)
                (or 
                    (ontop ?notebook.n.01 ?notebook.n.01_1)
                    (ontop ?notebook.n.01 ?notebook.n.01_2)
                )
            )
            (forall 
                (?hardback.n.01 - hardback.n.01)
                (inside ?hardback.n.01 ?bookcase.n.01_1)
            )
            (exists
                (?hardback.n.01 - hardback.n.01)
                (or 
                    (ontop ?hardback.n.01 ?hardback.n.01_1)
                    (ontop ?hardback.n.01 ?hardback.n.01_2)
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (inside notebook.n.01_1 bookcase.n.01_1) 
        (inside comic_book.n.01_1 bookcase.n.01_1) 
        (ontop notebook.n.01_2 comic_book.n.01_1) 
        (ontop comic_book.n.01_2 notebook.n.01_1) 
        (inside hardback.n.01_1 bookcase.n.01_1) 
        (inside comic_book.n.01_3 bookcase.n.01_1) 
        (ontop hardback.n.01_2 comic_book.n.01_3) 
        (inroom bookcase.n.01_1 living_room) 
        (inroom floor.n.01_1 living_room) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
