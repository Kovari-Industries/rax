**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forpairs 
                (?apple.n.01 - apple.n.01) 
                (?tupperware.n.01 - tupperware.n.01) 
                (inside ?apple.n.01 ?tupperware.n.01)
            ) 
            (forpairs 
                (?strawberry.n.01 - strawberry.n.01) 
                (?tupperware.n.01 - tupperware.n.01) 
                (inside ?strawberry.n.01 ?tupperware.n.01)
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
        (inside tupperware.n.01_1 cabinet.n.01_1) 
        (inside tupperware.n.01_2 cabinet.n.01_1) 
        (ontop bowl.n.01_1 countertop.n.01_1) 
        (inside apple.n.01_1 bowl.n.01_1) 
        (inside apple.n.01_2 bowl.n.01_1) 
        (ontop tray.n.01_1 countertop.n.01_1) 
        (ontop strawberry.n.01_1 tray.n.01_1) 
        (ontop strawberry.n.01_2 tray.n.01_1)
        (inroom floor.n.01_1 kitchen)
        (inroom electric_refrigerator.n.01_1 kitchen) 
        (inroom countertop.n.01_1 kitchen) 
        (inroom cabinet.n.01_1 kitchen) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
