**Goal:** Dice the Vidalia onion and put the diced onion in the bowl, then put the paring knife and the chopping board in the sink. The onion starts in the kitchen sink; the bowl, chopping board and paring knife start on the countertop.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (real ?diced__vidalia_onion.n.01_1)
            (inside ?parer.n.02_1 ?sink.n.01_1)
            (inside ?chopping_board.n.01_1 ?sink.n.01_1)
            (contains ?bowl.n.01_1 ?diced__vidalia_onion.n.01_1)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init
        (ontop bowl.n.01_1 countertop.n.01_1)
        (ontop chopping_board.n.01_1 countertop.n.01_1) 
        (ontop parer.n.02_1 countertop.n.01_1)
        (inside vidalia_onion.n.01_1 sink.n.01_1)
        (inroom countertop.n.01_1 kitchen)
        (inroom sink.n.01_1 kitchen) 
        (inroom floor.n.01_1 kitchen) 
        (future diced__vidalia_onion.n.01_1)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
