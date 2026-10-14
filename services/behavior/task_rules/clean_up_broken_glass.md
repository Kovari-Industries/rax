**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal
        (and
            (forall
                (?broken__glass.n.01 - broken__glass.n.01)
                (inside ?broken__glass.n.01 ?ashcan.n.01_1)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init
        (ontop broken__glass.n.01_1 floor.n.01_1)
        (ontop broken__glass.n.01_2 floor.n.01_1)
        (ontop broken__glass.n.01_3 floor.n.01_1)
        (ontop agent.n.01_1 floor.n.01_1)
        (ontop ashcan.n.01_1 floor.n.01_2)
        (inroom floor.n.01_1 dining_room)
        (inroom floor.n.01_2 corridor)
    )
```
