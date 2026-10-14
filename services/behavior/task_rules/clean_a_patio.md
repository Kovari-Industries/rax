**Goal:** Pick up the broom in the garden and sweep the mud off the patio floor until the floor is no longer covered in mud.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (not 
                (covered ?floor.n.01_1 ?mud.n.03_1)
            ) 
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (covered floor.n.01_1 mud.n.03_1) 
        (ontop broom.n.01_1 floor.n.01_1) 
        (inroom floor.n.01_1 garden) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
