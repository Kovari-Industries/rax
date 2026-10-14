**Goal:** In the utility room, put the drill, pliers, flashlight, Allen wrench, and screwdriver from the tabletop into the toolbox, keep the toolbox on the tabletop, and close the toolbox.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (inside ?drill.n.01_1 ?toolbox.n.01_1)
            (inside ?pliers.n.01_1 ?toolbox.n.01_1)
            (inside ?flashlight.n.01_1 ?toolbox.n.01_1)
            (inside ?allen_wrench.n.01_1 ?toolbox.n.01_1)
            (inside ?screwdriver.n.01_1 ?toolbox.n.01_1)
            (ontop ?toolbox.n.01_1 ?tabletop.n.01_1)
            (not
                (open ?toolbox.n.01_1)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop drill.n.01_1 tabletop.n.01_1) 
        (ontop pliers.n.01_1 tabletop.n.01_1) 
        (ontop toolbox.n.01_1 tabletop.n.01_1) 
        (ontop flashlight.n.01_1 tabletop.n.01_1) 
        (ontop allen_wrench.n.01_1 tabletop.n.01_1) 
        (ontop screwdriver.n.01_1 tabletop.n.01_1) 
        (inroom tabletop.n.01_1 utility_room)
        (inroom floor.n.01_1 utility_room)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
