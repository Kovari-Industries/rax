**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal
        (and
            (forall
                (?battery.n.02 - battery.n.02)
                (inside ?battery.n.02 ?ashcan.n.01_1)
            )
            (ontop ?ashcan.n.01_1 ?floor.n.01_1)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init
        (ontop battery.n.02_1 desk.n.01_1)
        (ontop battery.n.02_2 desk.n.01_1)
        (ontop battery.n.02_3 cabinet.n.01_1)
        (ontop ashcan.n.01_1 floor.n.01_1)
        (inroom desk.n.01_1 private_office)
        (inroom cabinet.n.01_1 copy_room)
        (inroom floor.n.01_1 copy_room)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
