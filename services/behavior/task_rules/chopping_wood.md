**Goal:** Chop the four logs on the driveway in the garden into eight half logs using the axe and the chopping block.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (real ?half__log.n.01_1)
            (real ?half__log.n.01_2)
            (real ?half__log.n.01_3)
            (real ?half__log.n.01_4)
            (real ?half__log.n.01_5)
            (real ?half__log.n.01_6)
            (real ?half__log.n.01_7)
            (real ?half__log.n.01_8)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init
        (ontop agent.n.01_1 driveway.n.01_1)
        (ontop log.n.01_1 driveway.n.01_1) 
        (ontop log.n.01_2 driveway.n.01_1) 
        (ontop log.n.01_3 driveway.n.01_1) 
        (ontop log.n.01_4 driveway.n.01_1) 
        (ontop ax.n.01_1 driveway.n.01_1)
        (ontop chopping_block.n.01_1 driveway.n.01_1) 
        (inroom driveway.n.01_1 garden)
        (future half__log.n.01_1)
        (future half__log.n.01_2)
        (future half__log.n.01_3)
        (future half__log.n.01_4)
        (future half__log.n.01_5)
        (future half__log.n.01_6)
        (future half__log.n.01_7)
        (future half__log.n.01_8)
    )
```
