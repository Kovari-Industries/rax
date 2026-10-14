**Goal:** Make a pizza on the cookie sheet in the kitchen: take the grated cheese, the four pieces of pepperoni, and the two mushrooms from their tupperware containers in the refrigerator; chop the Vidalia onion on the chopping board with the knife, and also chop the two whole mushrooms in half; top the pizza dough that's already on the cookie sheet with the cheese, pepperoni, halved mushrooms, and chopped onion; bake it in the oven until it becomes a pizza, and leave the finished pizza on the cookie sheet.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (real ?pizza.n.01_1) 
            (ontop ?pizza.n.01_1 ?cookie_sheet.n.01_1)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop pizza_dough.n.01_1 cookie_sheet.n.01_1) 
        (filled tupperware.n.01_1 grated_cheese.n.01_1)
        (inside tupperware.n.01_1 electric_refrigerator.n.01_1) 
        (inside pepperoni.n.01_1 tupperware.n.01_3) 
        (inside pepperoni.n.01_2 tupperware.n.01_3) 
        (inside pepperoni.n.01_3 tupperware.n.01_3) 
        (inside pepperoni.n.01_4 tupperware.n.01_3)
        (inside tupperware.n.01_3 electric_refrigerator.n.01_1) 
        (ontop chopping_board.n.01_1 countertop.n.01_1) 
        (ontop carving_knife.n.01_1 countertop.n.01_1) 
        (ontop bowl.n.01_1 countertop.n.01_1) 
        (inside mushroom.n.05_1 tupperware.n.01_2) 
        (inside mushroom.n.05_2 tupperware.n.01_2) 
        (inside tupperware.n.01_2 electric_refrigerator.n.01_1)
        (ontop vidalia_onion.n.01_1 chopping_board.n.01_1) 
        (ontop cookie_sheet.n.01_1 countertop.n.01_1) 
        (future pizza.n.01_1) 
        (inroom oven.n.01_1 kitchen) 
        (inroom floor.n.01_1 kitchen) 
        (inroom cabinet.n.01_1 kitchen) 
        (inroom countertop.n.01_1 kitchen) 
        (inroom electric_refrigerator.n.01_1 kitchen) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
