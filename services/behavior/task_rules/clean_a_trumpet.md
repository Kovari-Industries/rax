**Goal:** Remove the dust from the trumpet (cornet): it must no longer be covered with dust. The trumpet and a scrub brush start on the bedroom desk.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (not 
                (covered ?cornet.n.01_1 ?dust.n.01_1)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (covered cornet.n.01_1 dust.n.01_1) 
        (ontop scrub_brush.n.01_1 desk.n.01_1) 
        (ontop cornet.n.01_1 desk.n.01_1) 
        (inroom desk.n.01_1 bedroom) 
        (inroom floor.n.01_1 bedroom) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
