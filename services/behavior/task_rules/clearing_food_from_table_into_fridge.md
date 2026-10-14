**Goal:** Pack the half chicken and the half apple pie from the plates on the breakfast table into the two tupperware containers from the countertop, then put both tupperware containers inside the refrigerator in the kitchen and make sure the refrigerator is closed at the end.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (exists 
                (?tupperware.n.01 - tupperware.n.01)
                (inside ?half__apple_pie.n.01_1 ?tupperware.n.01)
            )
            (exists 
                (?tupperware.n.01 - tupperware.n.01)
                (inside ?half__chicken.n.01_1 ?tupperware.n.01)
            )
            (forall 
                (?tupperware.n.01 - tupperware.n.01) 
                (inside ?tupperware.n.01 ?electric_refrigerator.n.01_1)
            ) 
            (not
                (open ?electric_refrigerator.n.01_1)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop half__chicken.n.01_1 plate.n.04_1) 
        (ontop half__apple_pie.n.01_1 plate.n.04_2) 
        (ontop plate.n.04_1 breakfast_table.n.01_1) 
        (ontop plate.n.04_2 breakfast_table.n.01_1) 
        (ontop tupperware.n.01_1 countertop.n.01_1)
        (ontop tupperware.n.01_2 countertop.n.01_1)
        (inroom breakfast_table.n.01_1 kitchen) 
        (inroom electric_refrigerator.n.01_1 kitchen) 
        (inroom countertop.n.01_1 kitchen)
        (inroom floor.n.01_1 kitchen) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
