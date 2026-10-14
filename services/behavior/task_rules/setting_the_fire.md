**Goal:** In the living room, place the newspaper from the table into the wood fireplace, then put both pieces of firewood from the floor on top of the newspaper. Use the cigar lighter to ignite any one of the items so that the whole pile catches fire, and then turn the lighter off.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall 
                (?firewood.n.01 - firewood.n.01) 
                (and
                    (on_fire ?firewood.n.01)
                    (ontop ?firewood.n.01 ?newspaper.n.03_1)
                    (inside ?firewood.n.01 ?wood_fireplace.n.01_1)
                )
            )
            (on_fire ?newspaper.n.03_1)
            (not
                (toggled_on cigar_lighter.n.01_1)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop firewood.n.01_1 floor.n.01_1) 
        (ontop firewood.n.01_2 floor.n.01_1) 
        (ontop newspaper.n.03_1 table.n.02_1)
        (ontop cigar_lighter.n.01_1 table.n.02_1)
        (inroom wood_fireplace.n.01_1 living_room)
        (inroom table.n.02_1 living_room)
        (inroom floor.n.01_1 living_room) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
