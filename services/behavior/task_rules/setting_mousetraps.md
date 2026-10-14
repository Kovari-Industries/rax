**Goal:** Take the four mousetraps from the cabinet in the bathroom and place them on the bathroom floor. Make sure all four end up on the same floor surface, and ensure that at least two of them are either under or directly next to the same bathroom sink.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (exists
                (?floor.n.01 - floor.n.01)
                (forall
                    (?mousetrap.n.01 - mousetrap.n.01)
                    (ontop ?mousetrap.n.01 ?floor.n.01)
                )
            )
            (exists
                (?sink.n.01 - sink.n.01)
                (forn
                    (2)
                    (?mousetrap.n.01 - mousetrap.n.01)
                    (or
                        (under ?mousetrap.n.01 ?sink.n.01)
                        (nextto ?mousetrap.n.01 ?sink.n.01)
                    )
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop mousetrap.n.01_1 cabinet.n.01_1)
        (ontop mousetrap.n.01_2 cabinet.n.01_1)
        (ontop mousetrap.n.01_3 cabinet.n.01_1)
        (ontop mousetrap.n.01_4 cabinet.n.01_1)
        (inroom sink.n.01_1 bathroom)
        (inroom sink.n.01_* bathroom)
        (inroom floor.n.01_1 bathroom)
        (inroom floor.n.01_* bathroom)
        (inroom cabinet.n.01_1 bathroom)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
