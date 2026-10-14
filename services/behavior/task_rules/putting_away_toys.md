**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall 
                (?toy_figure.n.01 - toy_figure.n.01) 
                (exists 
                    (?toy_box.n.01 - toy_box.n.01) 
                    (inside ?toy_figure.n.01 ?toy_box.n.01)
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop toy_figure.n.01_1 floor.n.01_1) 
        (ontop toy_figure.n.01_2 floor.n.01_1) 
        (ontop toy_figure.n.01_3 floor.n.01_1) 
        (ontop toy_figure.n.01_4 floor.n.01_1) 
        (ontop toy_figure.n.01_5 floor.n.01_2) 
        (ontop toy_figure.n.01_6 floor.n.01_2) 
        (ontop toy_figure.n.01_7 floor.n.01_2) 
        (ontop toy_figure.n.01_8 floor.n.01_2) 
        (ontop toy_box.n.01_1 floor.n.01_1) 
        (ontop toy_box.n.01_2 table.n.02_1) 
        (inroom floor.n.01_1 living_room) 
        (inroom floor.n.01_2 dining_room) 
        (inroom table.n.02_1 dining_room) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
