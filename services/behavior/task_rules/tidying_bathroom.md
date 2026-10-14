**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal
        (and
            (ontop ?bar_soap.n.01_1 ?soap_dish.n.01_1)
            (ontop ?tissue_dispenser.n.01_1 ?sink.n.01_1)
            (ontop ?toilet_tissue.n.01_1 ?toilet.n.02_1)
            (inside ?cork.n.04_1 ?ashcan.n.01_1)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init
        (ontop bar_soap.n.01_1 sink.n.01_1)
        (ontop soap_dish.n.01_1 sink.n.01_1)
        (ontop tissue_dispenser.n.01_1 floor.n.01_1)
        (ontop cork.n.04_1 sink.n.01_1)
        (ontop toilet_tissue.n.01_1 floor.n.01_1)
        (ontop ashcan.n.01_1 floor.n.01_1)
        (inroom sink.n.01_1 bathroom)
        (inroom toilet.n.02_1 bathroom)
        (inroom shelf.n.01_1 bathroom)
        (inroom floor.n.01_1 bathroom)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
