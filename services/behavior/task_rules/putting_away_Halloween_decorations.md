**Goal:** Place each of the two pumpkins and all three candles from the living room inside a cabinet in the living room (use any cabinet), then make sure every cabinet is closed, and position the cauldron so it is next to a table in the living room.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall 
                (?pumpkin.n.02 - pumpkin.n.02)
                (exists 
                    (?cabinet.n.01 - cabinet.n.01) 
                    (inside ?pumpkin.n.02 ?cabinet.n.01)
                )
            ) 
            (forall 
                (?candle.n.01 - candle.n.01) 
                (exists 
                    (?cabinet.n.01 - cabinet.n.01) 
                    (inside ?candle.n.01 ?cabinet.n.01)
                )
            )
            (exists
                (?table.n.02 - table.n.02)
                (nextto ?caldron.n.01_1 ?table.n.02)
            )
            (forall
                (?cabinet.n.01 - cabinet.n.01)
                (not
                    (open ?cabinet.n.01)
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop pumpkin.n.02_1 floor.n.01_1) 
        (ontop pumpkin.n.02_2 floor.n.01_1) 
        (ontop caldron.n.01_1 floor.n.01_1) 
        (ontop candle.n.01_1 floor.n.01_1) 
        (ontop candle.n.01_2 floor.n.01_1) 
        (ontop candle.n.01_3 floor.n.01_1) 
        (inroom floor.n.01_1 living_room) 
        (inroom cabinet.n.01_1 living_room)
        (inroom cabinet.n.01_* living_room)
        (inroom table.n.02_1 living_room)
        (inroom table.n.02_* living_room)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
