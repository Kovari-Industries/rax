**Goal:** Take the three Easter eggs out of the wicker basket on the lawn in the garden, then place them on the lawn next to a single tree (choose any tree) so that all three eggs are next to the same tree and none are left in the basket.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (exists
                (?tree.n.01 - tree.n.01)
                (forall
                    (?easter_egg.n.01 - easter_egg.n.01)
                    (and
                        (nextto ?easter_egg.n.01 ?tree.n.01)
                    )
                )
            )
            (forall
                (?easter_egg.n.01 - easter_egg.n.01)
                (and
                    (not
                        (inside ?easter_egg.n.01 ?wicker_basket.n.01_1)
                    )
                    (ontop ?easter_egg.n.01 ?lawn.n.01_1)
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (inside easter_egg.n.01_1 wicker_basket.n.01_1) 
        (inside easter_egg.n.01_2 wicker_basket.n.01_1) 
        (inside easter_egg.n.01_3 wicker_basket.n.01_1) 
        (ontop wicker_basket.n.01_1 lawn.n.01_1) 
        (inroom lawn.n.01_1 garden) 
        (inroom tree.n.01_1 garden)
        (inroom tree.n.01_* garden)
        (ontop agent.n.01_1 lawn.n.01_1)
    )
```
