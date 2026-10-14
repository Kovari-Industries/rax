**Goal:** In the living room, take the wreath, three candy canes, and two pillar candles out of the wicker basket. Place the wreath and two of the candy canes on the same living-room sofa. Put the remaining candy cane on top of a dining-room table. Put both pillar candles together on top of one dining-room table (they can share the same table). Finally, place all three gift boxes under or right next to the Christmas tree in the living room.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall 
                (?gift_box.n.01 - gift_box.n.01)
                (or 
                    (nextto ?gift_box.n.01 ?christmas_tree.n.05_1)
                    (under ?gift_box.n.01 ?christmas_tree.n.05_1)
                    (touching ?gift_box.n.01 ?christmas_tree.n.05_1)
                )
            )
            (exists
                (?table.n.02 - table.n.02)
                (forall
                    (?pillar_candle.n.01 - pillar_candle.n.01)
                    (ontop ?pillar_candle.n.01 ?table.n.02)
                )
            )
            (exists
                (?table.n.02 - table.n.02)
                (forn
                    (1)
                    (?candy_cane.n.01 - candy_cane.n.01)
                    (ontop ?candy_cane.n.01 ?table.n.02)
                )
            )
            (exists
                (?sofa.n.01 - sofa.n.01)
                (forall
                    (?wreath.n.01 - wreath.n.01)
                    (ontop ?wreath.n.01 ?sofa.n.01)
                )
            )
            (exists
                (?sofa.n.01 - sofa.n.01)
                (forn
                    (2)
                    (?candy_cane.n.01 - candy_cane.n.01)
                    (ontop ?candy_cane.n.01 ?sofa.n.01)
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop christmas_tree.n.05_1 floor.n.01_1)
        (ontop wicker_basket.n.01_1 floor.n.01_1)
        (inside wreath.n.01_1 wicker_basket.n.01_1)
        (inside candy_cane.n.01_1 wicker_basket.n.01_1)
        (inside candy_cane.n.01_2 wicker_basket.n.01_1)
        (inside candy_cane.n.01_3 wicker_basket.n.01_1)
        (inside pillar_candle.n.01_1 wicker_basket.n.01_1)
        (inside pillar_candle.n.01_2 wicker_basket.n.01_1)
        (ontop gift_box.n.01_1 floor.n.01_1)
        (ontop gift_box.n.01_2 floor.n.01_1)
        (ontop gift_box.n.01_3 floor.n.01_1)
        (inroom floor.n.01_1 living_room)
        (inroom table.n.02_1 dining_room)
        (inroom table.n.02_* dining_room)
        (inroom sofa.n.01_1 living_room)
        (inroom sofa.n.01_* living_room)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
