**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall
                (?water_glass.n.02 - water_glass.n.02)
                (inside ?water_glass.n.02 ?ashcan.n.01_1)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (inroom sink.n.01_1 bathroom) 
        (inroom floor.n.01_1 bathroom) 
        (inroom floor.n.01_2 bedroom) 
        (ontop water_glass.n.02_1 sink.n.01_1) 
        (ontop water_glass.n.02_2 sink.n.01_1)
        (ontop water_glass.n.02_3 sink.n.01_1)
        (ontop water_glass.n.02_4 sink.n.01_1)
        (ontop ashcan.n.01_1 floor.n.01_2) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
