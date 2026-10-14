**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal
        (and
            (forpairs
                (?wrapped_hamburger.n.01 - wrapped_hamburger.n.01)
                (?sack.n.01 - sack.n.01)
                (inside ?wrapped_hamburger.n.01 ?sack.n.01)
            )
            (forall
                (?sack.n.01 - sack.n.01)
                (ontop ?sack.n.01 ?storage_container.n.01_1)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init
        (ontop wrapped_hamburger.n.01_1 tray.n.01_1)
        (ontop wrapped_hamburger.n.01_2 tray.n.01_1)
        (ontop wrapped_hamburger.n.01_3 tray.n.01_1)
        (ontop tray.n.01_1 countertop.n.01_1)
        (ontop storage_container.n.01_1 countertop.n.01_1)
        (ontop sack.n.01_1 countertop.n.01_1)
        (ontop sack.n.01_2 countertop.n.01_1)
        (ontop sack.n.01_3 countertop.n.01_1)
        (inroom floor.n.01_1 kitchen)
        (inroom countertop.n.01_1 kitchen)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
