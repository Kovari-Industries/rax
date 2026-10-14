**Goal:** Open the kitchen cabinet, take out the two hinged jars, open them, place exactly two cooked bratwursts from the chopping board on the countertop into each jar, then close both jars, put them back inside the cabinet, and close the cabinet.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall 
                (?hinged_jar.n.01 - hinged_jar.n.01) 
                (inside ?hinged_jar.n.01 ?cabinet.n.01_1)
            ) 
            (forall 
                (?hinged_jar.n.01 - hinged_jar.n.01)
                (not
                    (open ?hinged_jar.n.01)
                )
            )
            (forall 
                (?hinged_jar.n.01 - hinged_jar.n.01) 
                (forn
                    (2)
                    (?bratwurst.n.01 - bratwurst.n.01)
                    (inside ?bratwurst.n.01 ?hinged_jar.n.01)
                )
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
        (inside hinged_jar.n.01_1 cabinet.n.01_1) 
        (inside hinged_jar.n.01_2 cabinet.n.01_1) 
        (ontop chopping_board.n.01_1 countertop.n.01_1) 
        (ontop bratwurst.n.01_1 chopping_board.n.01_1) 
        (ontop bratwurst.n.01_2 chopping_board.n.01_1) 
        (ontop bratwurst.n.01_3 chopping_board.n.01_1) 
        (ontop bratwurst.n.01_4 chopping_board.n.01_1) 
        (cooked bratwurst.n.01_1)
        (cooked bratwurst.n.01_2)
        (cooked bratwurst.n.01_3)
        (cooked bratwurst.n.01_4)
        (inroom cabinet.n.01_1 kitchen) 
        (inroom countertop.n.01_1 kitchen) 
        (inroom floor.n.01_1 kitchen) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
