**Goal:** Put both apple halves, the club sandwich, and the chocolate chip cookie from the chopping board on the kitchen countertop into the packing box on the countertop. Then take the bottle of tea out of the refrigerator, put it into the same box, and close the refrigerator when you're done.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall 
                (?half__apple.n.01 - half__apple.n.01)
                (inside ?half__apple.n.01 ?packing_box.n.02_1)
            )
            (inside ?club_sandwich.n.01_1 ?packing_box.n.02_1) 
            (inside ?chocolate_chip_cookie.n.01_1 ?packing_box.n.02_1) 
            (inside ?bottle__of__tea.n.01_1 ?packing_box.n.02_1)
            (not 
                (open ?electric_refrigerator.n.01_1)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop chopping_board.n.01_1 countertop.n.01_1) 
        (ontop packing_box.n.02_1 countertop.n.01_1) 
        (ontop half__apple.n.01_1 chopping_board.n.01_1) 
        (ontop half__apple.n.01_2 chopping_board.n.01_1) 
        (ontop club_sandwich.n.01_1 chopping_board.n.01_1) 
        (inside bottle__of__tea.n.01_1 electric_refrigerator.n.01_1) 
        (ontop chocolate_chip_cookie.n.01_1 chopping_board.n.01_1) 
        (inroom electric_refrigerator.n.01_1 kitchen) 
        (inroom countertop.n.01_1 kitchen) 
        (inroom floor.n.01_1 kitchen) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
