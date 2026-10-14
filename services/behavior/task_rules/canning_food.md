**Goal:** In the kitchen, open the refrigerator and the cabinet. Take the steak and the pineapple out of the refrigerator and take two bowls from the cabinet. On the chopping board on the countertop, use the carving knife to dice the steak and to dice the pineapple. Put only the diced steak into one bowl and only the diced pineapple into the other bowl - do not mix them. Place both bowls back inside the cabinet, then close the refrigerator and close the cabinet.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (real ?diced__steak.n.01_1)
            (real ?diced__pineapple.n.01_1)
            (exists
                (?bowl.n.01 - bowl.n.01)
                (and 
                    (filled ?bowl.n.01 ?diced__steak.n.01_1)
                    (not
                        (contains ?bowl.n.01 ?diced__pineapple.n.01_1)
                    )
                )
            )
            (exists
                (?bowl.n.01 - bowl.n.01)
                (and 
                    (filled ?bowl.n.01 ?diced__pineapple.n.01_1)
                    (not
                        (contains ?bowl.n.01 ?diced__steak.n.01_1)
                    )
                )
            )
            (forall 
                (?bowl.n.01 - bowl.n.01) 
                (and
                    (inside ?bowl.n.01 ?cabinet.n.01_1)
                )
            ) 
            (not 
                (open ?electric_refrigerator.n.01_1)
            )
            (not
                (open ?cabinet.n.01_1)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (inside steak.n.01_1 electric_refrigerator.n.01_1) 
        (inside pineapple.n.02_1 electric_refrigerator.n.01_1) 
        (inside bowl.n.01_1 cabinet.n.01_1) 
        (inside bowl.n.01_2 cabinet.n.01_1) 
        (ontop chopping_board.n.01_1 countertop.n.01_1) 
        (ontop carving_knife.n.01_1 chopping_board.n.01_1)
        (inroom cabinet.n.01_1 kitchen) 
        (inroom electric_refrigerator.n.01_1 kitchen) 
        (inroom countertop.n.01_1 kitchen) 
        (inroom floor.n.01_1 kitchen) 
        (future diced__pineapple.n.01_1)
        (future diced__steak.n.01_1)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
