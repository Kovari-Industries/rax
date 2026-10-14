**Goal:** In the kitchen, take the two apple pies from the plates on the countertop, put each pie into a separate tupperware container taken from the cabinet, place both tupperwares inside the refrigerator, close the refrigerator, and leave them until the pies are frozen.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forpairs 
                (?apple_pie.n.01 - apple_pie.n.01) 
                (?tupperware.n.01 - tupperware.n.01) 
                (inside ?apple_pie.n.01 ?tupperware.n.01)
            ) 
            (forall 
                (?tupperware.n.01 - tupperware.n.01) 
                (inside ?tupperware.n.01 ?electric_refrigerator.n.01_1)
            ) 
            (forall 
                (?apple_pie.n.01 - apple_pie.n.01) 
                (frozen ?apple_pie.n.01)
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
        (ontop apple_pie.n.01_1 plate.n.04_1) 
        (ontop apple_pie.n.01_2 plate.n.04_2) 
        (ontop plate.n.04_1 countertop.n.01_1)
        (ontop plate.n.04_2 countertop.n.01_1)
        (inside tupperware.n.01_1 cabinet.n.01_1) 
        (inside tupperware.n.01_2 cabinet.n.01_1) 
        (not 
            (frozen apple_pie.n.01_1)
        ) 
        (not 
            (frozen apple_pie.n.01_2)
        )
        (inroom floor.n.01_1 kitchen) 
        (inroom cabinet.n.01_1 kitchen)
        (inroom countertop.n.01_1 kitchen)
        (inroom electric_refrigerator.n.01_1 kitchen) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
