**Goal:** Sort the vegetables from the two wicker baskets on the kitchen floor into the mixing bowls on the kitchen countertop: put all three bok choy and all three Vidalia onions together into one mixing bowl; put both leeks and both broccoli together into a second mixing bowl; and put all three sweet corn into a third mixing bowl.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (exists 
                (?mixing_bowl.n.01 - mixing_bowl.n.01)
                (and 
                    (forall 
                        (?bok_choy.n.02 - bok_choy.n.02)
                        (inside ?bok_choy.n.02 ?mixing_bowl.n.01)
                    )
                    (forall 
                        (?vidalia_onion.n.01 - vidalia_onion.n.01)
                        (inside ?vidalia_onion.n.01 ?mixing_bowl.n.01)
                    )
                )
            )
            (exists 
                (?mixing_bowl.n.01 - mixing_bowl.n.01)
                (and 
                    (forall 
                        (?leek.n.02 - leek.n.02)
                        (inside ?leek.n.02 ?mixing_bowl.n.01)
                    )
                    (forall 
                        (?broccoli.n.02 - broccoli.n.02)
                        (inside ?broccoli.n.02 ?mixing_bowl.n.01)
                    )
                )
            )
            (exists 
                (?mixing_bowl.n.01 - mixing_bowl.n.01)
                (and 
                    (forall 
                        (?sweet_corn.n.02 - sweet_corn.n.02)
                        (inside ?sweet_corn.n.02 ?mixing_bowl.n.01)
                    )
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop mixing_bowl.n.01_1 countertop.n.01_1) 
        (ontop mixing_bowl.n.01_2 countertop.n.01_1) 
        (ontop mixing_bowl.n.01_3 countertop.n.01_1) 
        (ontop wicker_basket.n.01_1 floor.n.01_1)
        (ontop wicker_basket.n.01_2 floor.n.01_1)
        (inside bok_choy.n.02_1 wicker_basket.n.01_1)
        (inside bok_choy.n.02_2 wicker_basket.n.01_1)
        (inside bok_choy.n.02_3 wicker_basket.n.01_1)
        (inside sweet_corn.n.02_1 wicker_basket.n.01_1)
        (inside sweet_corn.n.02_2 wicker_basket.n.01_1)
        (inside sweet_corn.n.02_3 wicker_basket.n.01_1)
        (inside broccoli.n.02_1 wicker_basket.n.01_2) 
        (inside broccoli.n.02_2 wicker_basket.n.01_2)
        (inside leek.n.02_1 wicker_basket.n.01_2)
        (inside leek.n.02_2 wicker_basket.n.01_2)
        (inside vidalia_onion.n.01_1 wicker_basket.n.01_2)
        (inside vidalia_onion.n.01_2 wicker_basket.n.01_2)
        (inside vidalia_onion.n.01_3 wicker_basket.n.01_2)
        (inroom countertop.n.01_1 kitchen) 
        (inroom floor.n.01_1 kitchen) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
