**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal
        (forall
            (?log.n.01 - log.n.01)
            (or
                (touching ?log.n.01 ?log.n.01_1)
                (touching ?log.n.01 ?log.n.01_2)
                (touching ?log.n.01 ?log.n.01_3)
                (touching ?log.n.01 ?log.n.01_4)
                (touching ?log.n.01 ?log.n.01_5)
                (touching ?log.n.01 ?log.n.01_6)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init
        (ontop log.n.01_1 driveway.n.01_1)
        (ontop log.n.01_2 driveway.n.01_1)
        (ontop log.n.01_3 driveway.n.01_1)
        (ontop log.n.01_4 driveway.n.01_1)
        (ontop log.n.01_5 driveway.n.01_1)
        (ontop log.n.01_6 driveway.n.01_1)
        (inroom driveway.n.01_1 garden)
        (ontop agent.n.01_1 driveway.n.01_1)
    )
```
