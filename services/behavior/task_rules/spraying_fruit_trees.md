**Goal:** In the garden, pick up the pesticide atomizer on the floor and spray pesticide onto both trees until each tree trunk is fully covered.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (covered ?tree.n.01_1 ?pesticide.n.01_1)
            (covered ?tree.n.01_2 ?pesticide.n.01_1)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (insource pesticide__atomizer.n.01_1 pesticide.n.01_1) 
        (ontop pesticide__atomizer.n.01_1 floor.n.01_1)
        (inroom tree.n.01_1 garden)
        (inroom tree.n.01_2 garden)
        (inroom floor.n.01_1 garden) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
