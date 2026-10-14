**Goal:** From the breakfast table in the kitchen, move both pizzas - keeping each on its plate - into the same refrigerator, put both bowls into one sink, and make sure the refrigerator is closed.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forpairs 
                (?pizza.n.01 - pizza.n.01) 
                (?plate.n.04 - plate.n.04)
                (ontop ?pizza.n.01 ?plate.n.04)
            ) 
            (exists
                (?electric_refrigerator.n.01 - electric_refrigerator.n.01)
                (forall
                    (?pizza.n.01 - pizza.n.01)
                    (inside ?pizza.n.01 ?electric_refrigerator.n.01)
                )
            )
            (exists
                (?sink.n.01 - sink.n.01)
                (forall 
                    (?bowl.n.01 - bowl.n.01)
                    (inside ?bowl.n.01 ?sink.n.01)
                )
            )
            (forall
                (?electric_refrigerator.n.01 - electric_refrigerator.n.01)
                (not
                    (open ?electric_refrigerator.n.01)
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop plate.n.04_1 breakfast_table.n.01_1) 
        (ontop plate.n.04_2 breakfast_table.n.01_1) 
        (ontop bowl.n.01_1 breakfast_table.n.01_1)
        (ontop bowl.n.01_2 breakfast_table.n.01_1)
        (ontop pizza.n.01_1 plate.n.04_1) 
        (ontop pizza.n.01_2 plate.n.04_2) 
        (ontop agent.n.01_1 floor.n.01_1) 
        (inroom breakfast_table.n.01_1 kitchen)
        (inroom floor.n.01_1 kitchen)
        (inroom electric_refrigerator.n.01_1 kitchen)
        (inroom electric_refrigerator.n.01_* kitchen)
        (inroom sink.n.01_1 kitchen)
        (inroom sink.n.01_* kitchen)
    )
```
