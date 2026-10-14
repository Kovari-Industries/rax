**Goal:** From the refrigerator in the kitchen, take out the two bell peppers, the two beets, and the zucchini. Then, on either chopping board on the countertop, use the parer to dice all of them so that only diced bell pepper, diced beet, and diced zucchini remain. Make sure the refrigerator is closed when you finish.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (real ?diced__zucchini.n.01_1)
            (real ?diced__bell_pepper.n.01_1)
            (real ?diced__beet.n.01_1)
            (not
                (open ?electric_refrigerator.n.01_1)
            )
            (forall
                (?bell_pepper.n.02 - bell_pepper.n.02)
                (not
                    (real ?bell_pepper.n.02)
                )
            )
            (forall
                (?beet.n.02 - beet.n.02)
                (not
                    (real ?beet.n.02)
                )
            )
            (forall
                (?zucchini.n.02 - zucchini.n.02)
                (not
                    (real ?zucchini.n.02)
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (inside bell_pepper.n.02_1 electric_refrigerator.n.01_1)
        (inside bell_pepper.n.02_2 electric_refrigerator.n.01_1)
        (inside beet.n.02_1 electric_refrigerator.n.01_1)
        (inside beet.n.02_2 electric_refrigerator.n.01_1)
        (inside zucchini.n.02_1 electric_refrigerator.n.01_1) 
        (ontop parer.n.02_1 countertop.n.01_1)
        (ontop chopping_board.n.01_1 countertop.n.01_1)
        (ontop chopping_board.n.01_2 countertop.n.01_1) 
        (inroom electric_refrigerator.n.01_1 kitchen) 
        (inroom countertop.n.01_1 kitchen) 
        (inroom floor.n.01_1 kitchen) 
        (future diced__beet.n.01_1)
        (future diced__bell_pepper.n.01_1)
        (future diced__zucchini.n.01_1)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
