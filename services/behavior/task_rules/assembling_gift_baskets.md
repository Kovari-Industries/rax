**Goal:** Place one candle, one butter cookie, one piece of Swiss cheese, and one bow from the table into each of the four wicker baskets on the floor in the living room.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forpairs 
                (?wicker_basket.n.01 - wicker_basket.n.01) 
                (?candle.n.01 - candle.n.01) 
                (inside ?candle.n.01 ?wicker_basket.n.01)
            ) 
            (forpairs 
                (?wicker_basket.n.01 - wicker_basket.n.01) 
                (?swiss_cheese.n.01 - swiss_cheese.n.01) 
                (inside ?swiss_cheese.n.01 ?wicker_basket.n.01)
            ) 
            (forpairs 
                (?wicker_basket.n.01 - wicker_basket.n.01) 
                (?butter_cookie.n.01 - butter_cookie.n.01) 
                (inside ?butter_cookie.n.01 ?wicker_basket.n.01)
            ) 
            (forpairs 
                (?wicker_basket.n.01 - wicker_basket.n.01) 
                (?bow.n.08 - bow.n.08) 
                (inside ?bow.n.08 ?wicker_basket.n.01)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop wicker_basket.n.01_1 floor.n.01_1) 
        (ontop wicker_basket.n.01_2 floor.n.01_1) 
        (ontop wicker_basket.n.01_3 floor.n.01_1) 
        (ontop wicker_basket.n.01_4 floor.n.01_1) 
        (ontop candle.n.01_1 table.n.02_1) 
        (ontop candle.n.01_2 table.n.02_1) 
        (ontop candle.n.01_3 table.n.02_1) 
        (ontop candle.n.01_4 table.n.02_1) 
        (ontop butter_cookie.n.01_1 table.n.02_1) 
        (ontop butter_cookie.n.01_2 table.n.02_1) 
        (ontop butter_cookie.n.01_3 table.n.02_1) 
        (ontop butter_cookie.n.01_4 table.n.02_1) 
        (ontop swiss_cheese.n.01_1 table.n.02_1) 
        (ontop swiss_cheese.n.01_2 table.n.02_1) 
        (ontop swiss_cheese.n.01_3 table.n.02_1) 
        (ontop swiss_cheese.n.01_4 table.n.02_1) 
        (ontop bow.n.08_1 table.n.02_1) 
        (ontop bow.n.08_2 table.n.02_1) 
        (ontop bow.n.08_3 table.n.02_1) 
        (ontop bow.n.08_4 table.n.02_1) 
        (inroom floor.n.01_1 living_room) 
        (inroom table.n.02_1 living_room) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
