**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal
        (and
            (forall
                (?half__brussels_sprouts.n.01 - half__brussels_sprouts.n.01)
                (and
                    (real ?half__brussels_sprouts.n.01)
                    (cooked ?half__brussels_sprouts.n.01)
                    (inside ?half__brussels_sprouts.n.01 oven.n.01_1)
                )
            )
            (not
                (open ?electric_refrigerator.n.01_1)
            )
            (not
                (open ?oven.n.01_1)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init
        (inside brussels_sprouts.n.01_1 tupperware.n.01_1)
        (inside brussels_sprouts.n.01_2 tupperware.n.01_1)
        (inside brussels_sprouts.n.01_3 tupperware.n.01_1)
        (inside brussels_sprouts.n.01_4 tupperware.n.01_1)
        (inside tupperware.n.01_1 electric_refrigerator.n.01_1)
        (not (cooked brussels_sprouts.n.01_1))
        (not (cooked brussels_sprouts.n.01_2))
        (not (cooked brussels_sprouts.n.01_3))
        (not (cooked brussels_sprouts.n.01_4))
        (ontop chopping_board.n.01_1 countertop.n.01_1)
        (ontop carving_knife.n.01_1 chopping_board.n.01_1)
        (ontop cookie_sheet.n.01_1 countertop.n.01_1)
        (future half__brussels_sprouts.n.01_1)
        (future half__brussels_sprouts.n.01_2)
        (future half__brussels_sprouts.n.01_3)
        (future half__brussels_sprouts.n.01_4)
        (future half__brussels_sprouts.n.01_5)
        (future half__brussels_sprouts.n.01_6)
        (future half__brussels_sprouts.n.01_7)
        (future half__brussels_sprouts.n.01_8)
        (inroom electric_refrigerator.n.01_1 kitchen)
        (inroom oven.n.01_1 kitchen)
        (inroom countertop.n.01_1 kitchen)
        (inroom floor.n.01_1 kitchen)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
