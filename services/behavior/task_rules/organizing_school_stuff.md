**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (inside ?folder.n.02_1 ?carryall.n.01_1) 
            (inside ?book.n.02_1 ?carryall.n.01_1)
            (inside ?pencil.n.01_1 ?carryall.n.01_1) 
            (inside ?pen.n.01_1 ?carryall.n.01_1) 
            (inside ?calculator.n.02_1 ?carryall.n.01_1) 
            (ontop ?carryall.n.01_1 ?bed.n.01_1)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop pencil.n.01_1 bed.n.01_1) 
        (ontop pen.n.01_1 floor.n.01_1) 
        (ontop calculator.n.02_1 floor.n.01_1) 
        (ontop book.n.02_1 bed.n.01_1) 
        (ontop folder.n.02_1 bed.n.01_1) 
        (ontop carryall.n.01_1 floor.n.01_1) 
        (inroom floor.n.01_1 bedroom) 
        (inroom bed.n.01_1 bedroom) 
        (inroom table.n.02_1 bedroom) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
