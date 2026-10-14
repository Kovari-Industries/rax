**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal
        (and
            (not
                (covered ?trowel.n.01_1 ?rust.n.01_1)
            )
            (not
                (covered ?scraper.n.01_1 ?rust.n.01_1)
            )
            (inside ?trowel.n.01_1 ?toolbox.n.01_1)
            (inside ?scraper.n.01_1 ?toolbox.n.01_1)
            (not
                (open ?toolbox.n.01_1)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init
        (covered trowel.n.01_1 rust.n.01_1)
        (covered scraper.n.01_1 rust.n.01_1)
        (ontop toolbox.n.01_1 driveway.n.01_1)
        (ontop trowel.n.01_1 driveway.n.01_1)
        (ontop scraper.n.01_1 driveway.n.01_1)
        (inside steel_wool.n.01_1 toolbox.n.01_1)
        (inroom driveway.n.01_1 garden)
        (ontop agent.n.01_1 driveway.n.01_1)
    )
```
