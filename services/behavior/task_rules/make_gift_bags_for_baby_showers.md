**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall 
                (?sack.n.01 - sack.n.01) 
                (ontop ?sack.n.01 ?coffee_table.n.01_1)
            ) 
            (forpairs 
                (?toy_dice.n.01 - toy_dice.n.01) 
                (?sack.n.01 - sack.n.01) 
                (inside ?toy_dice.n.01 ?sack.n.01)
            ) 
            (forpairs 
                (?wafer.n.02 - wafer.n.02) 
                (?sack.n.01 - sack.n.01) 
                (inside ?wafer.n.02 ?sack.n.01)
            ) 
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop toy_dice.n.01_1 cabinet.n.01_1) 
        (ontop toy_dice.n.01_2 cabinet.n.01_1) 
        (inside wafer.n.02_1 cabinet.n.01_1) 
        (inside wafer.n.02_2 cabinet.n.01_1) 
        (ontop sack.n.01_1 breakfast_table.n.01_1) 
        (ontop sack.n.01_2 breakfast_table.n.01_1) 
        (inroom floor.n.01_1 living_room) 
        (inroom cabinet.n.01_1 living_room) 
        (inroom breakfast_table.n.01_1 living_room) 
        (inroom coffee_table.n.01_1 living_room)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
