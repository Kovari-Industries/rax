**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal
        (and
            (exists
                (?bucket.n.01 - bucket.n.01)
                (and
                    (forall
                        (?wine_bottle.n.01 - wine_bottle.n.01)
                        (inside ?wine_bottle.n.01 ?bucket.n.01)
                    )
                    (not
                        (inside ?can__of__soda.n.01_1 ?bucket.n.01)
                    )
                    (not
                        (inside ?newspaper.n.03_1 ?bucket.n.01)
                    )
                    (not
                        (inside ?magazine.n.02_1 ?bucket.n.01)
                    )
                )
            )
            (exists
                (?bucket.n.01 - bucket.n.01)
                (and
                    (forall
                        (?can__of__soda.n.01 - can__of__soda.n.01)
                        (inside ?can__of__soda.n.01 ?bucket.n.01)
                    )
                    (not
                        (inside ?wine_bottle.n.01_1 ?bucket.n.01)
                    )
                    (not
                        (inside ?newspaper.n.03_1 ?bucket.n.01)
                    )
                    (not
                        (inside ?magazine.n.02_1 ?bucket.n.01)
                    )
                )
            )
            (exists
                (?bucket.n.01 - bucket.n.01)
                (and
                    (forall
                        (?can__of__soda.n.01 - can__of__soda.n.01)
                        (not
                            (inside ?can__of__soda.n.01 ?bucket.n.01)
                        )
                    )
                    (forall
                        (?wine_bottle.n.01 - wine_bottle.n.01)
                        (not
                            (inside ?wine_bottle.n.01 ?bucket.n.01)
                        )
                    )
                    (inside ?newspaper.n.03_1 ?bucket.n.01)
                    (inside ?magazine.n.02_1 ?bucket.n.01)
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init
        (ontop bucket.n.01_1 floor.n.01_1)
        (ontop bucket.n.01_2 floor.n.01_1)
        (ontop bucket.n.01_3 floor.n.01_1)
        (ontop wine_bottle.n.01_1 countertop.n.01_1)
        (ontop wine_bottle.n.01_2 floor.n.01_1)
        (ontop can__of__soda.n.01_1 floor.n.01_1)
        (ontop can__of__soda.n.01_2 countertop.n.01_1)
        (ontop newspaper.n.03_1 countertop.n.01_1)
        (ontop magazine.n.02_1 countertop.n.01_1)
        (inroom floor.n.01_1 kitchen)
        (inroom countertop.n.01_1 kitchen)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
