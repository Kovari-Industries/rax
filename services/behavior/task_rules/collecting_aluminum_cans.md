**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall 
                (?can__of__soda.n.01 - can__of__soda.n.01) 
                (inside ?can__of__soda.n.01 ?bucket.n.01_1)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop can__of__soda.n.01_1 bed.n.01_1) 
        (ontop can__of__soda.n.01_2 bed.n.01_1) 
        (ontop can__of__soda.n.01_3 bed.n.01_1) 
        (ontop can__of__soda.n.01_4 floor.n.01_1) 
        (ontop can__of__soda.n.01_5 floor.n.01_1) 
        (ontop can__of__soda.n.01_6 floor.n.01_1) 
        (ontop bucket.n.01_1 floor.n.01_1) 
        (inroom bed.n.01_1 bedroom) 
        (inroom floor.n.01_1 bedroom) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
