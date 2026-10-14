**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (ontop ?toolbox.n.01_1 ?floor.n.01_1) 
            (inside ?pruner.n.02_1 ?toolbox.n.01_1) 
            (inside ?shears.n.01_1 ?toolbox.n.01_1)
            (not
                (open ?toolbox.n.01_1)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop pruner.n.02_1 floor.n.01_1) 
        (ontop shears.n.01_1 floor.n.01_1) 
        (ontop toolbox.n.01_1 floor.n.01_1) 
        (inroom floor.n.01_1 garage) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
