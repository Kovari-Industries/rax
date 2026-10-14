**Goal:** In the utility room, take the two teddy toys, the tennis ball, and the softball out of the cabinet and wash them in the washer so that both teddies are free of dirt and dust, the tennis ball has no debris, and the softball has no dirt.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall 
                (?teddy.n.01 - teddy.n.01)
                (and
                    (not 
                        (covered ?teddy.n.01 ?dirt.n.02_1)
                    )
                    (not 
                        (covered ?teddy.n.01 ?dust.n.01_1)
                    )
                )
            )
            (not 
                (covered ?tennis_ball.n.01_1 ?debris.n.01_1)
            ) 
            (not 
                (covered ?softball.n.01_1 ?dirt.n.02_1)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (inside teddy.n.01_1 cabinet.n.01_1) 
        (covered teddy.n.01_1 dirt.n.02_1) 
        (inside teddy.n.01_2 cabinet.n.01_1) 
        (covered teddy.n.01_2 dust.n.01_1) 
        (inside tennis_ball.n.01_1 cabinet.n.01_1) 
        (covered tennis_ball.n.01_1 debris.n.01_1) 
        (inside softball.n.01_1 cabinet.n.01_1) 
        (covered softball.n.01_1 dirt.n.02_1) 
        (inroom floor.n.01_1 utility_room) 
        (inroom cabinet.n.01_1 utility_room) 
        (inroom washer.n.03_1 utility_room) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
