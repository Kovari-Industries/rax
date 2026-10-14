**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (inside glue_stick.n.01_1 carryall.n.01_1)
            (inside rubber_eraser.n.01_1 carryall.n.01_1)
            (inside paintbrush.n.01_1 carryall.n.01_1)
            (inside marker.n.03_1 carryall.n.01_1)
            (ontop carryall.n.01_1 desk.n.01_1)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop carryall.n.01_1 floor.n.01_1)
        (ontop glue_stick.n.01_1 desk.n.01_1)
        (ontop rubber_eraser.n.01_1 desk.n.01_1)
        (ontop paintbrush.n.01_1 desk.n.01_1)
        (ontop marker.n.03_1 desk.n.01_1)
        (inroom floor.n.01_1 bedroom)
        (inroom desk.n.01_1 bedroom)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
