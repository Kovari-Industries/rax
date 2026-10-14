**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal
        (and
            (ontop ?wine_bottle.n.01_1 ?countertop.n.01_1)
            (forall
                (?can__of__soda.n.01 - can__of__soda.n.01)
                (and
                    (or
                        (nextto ?can__of__soda.n.01 ?can__of__soda.n.01_1)
                        (nextto ?can__of__soda.n.01 ?can__of__soda.n.01_2)
                        (nextto ?can__of__soda.n.01 ?can__of__soda.n.01_3)
                    )
                    (ontop ?can__of__soda.n.01 ?countertop.n.01_1)
                )
            )
            (forall
                (?ice_cube.n.01 - ice_cube.n.01)
                (inside ?ice_cube.n.01 ?bucket.n.01_1)
            )
            (ontop ?bucket.n.01_1 ?countertop.n.01_1)
            (forall
                (?wineglass.n.01 - wineglass.n.01)
                (ontop ?wineglass.n.01 ?countertop.n.01_1)
            )
            (exists
                (?wineglass.n.01 - wineglass.n.01)
                (nextto ?wineglass.n.01 ?wine_bottle.n.01_1)
            )
            (nextto ?wineglass.n.01_1 ?wineglass.n.01_2)
            (ontop ?corkscrew.n.01_1 ?countertop.n.01_1)
            (nextto ?corkscrew.n.01_1 ?wine_bottle.n.01_1)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init
        (ontop can__of__soda.n.01_1 cabinet.n.01_1)
        (ontop can__of__soda.n.01_2 cabinet.n.01_1)
        (ontop can__of__soda.n.01_3 cabinet.n.01_1)
        (ontop ice_cube.n.01_1 tray.n.01_1)
        (ontop ice_cube.n.01_2 tray.n.01_1)
        (ontop ice_cube.n.01_3 tray.n.01_1)
        (ontop ice_cube.n.01_4 tray.n.01_1)
        (ontop ice_cube.n.01_5 tray.n.01_1)
        (ontop ice_cube.n.01_6 tray.n.01_1)
        (ontop tray.n.01_1 cabinet.n.01_1)
        (ontop bucket.n.01_1 cabinet.n.01_1)
        (ontop corkscrew.n.01_1 cabinet.n.01_1)
        (ontop wine_bottle.n.01_1 cabinet.n.01_1)
        (ontop wineglass.n.01_1 cabinet.n.01_1)
        (ontop wineglass.n.01_2 cabinet.n.01_1)
        (inroom cabinet.n.01_1 bar)
        (inroom countertop.n.01_1 bar)
        (inroom electric_refrigerator.n.01_1 bar)
        (inroom floor.n.01_1 bar)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
