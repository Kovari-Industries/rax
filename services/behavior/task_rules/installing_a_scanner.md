**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (toggled_on ?scanner.n.02_1) 
            (nextto ?scanner.n.02_1 ?laptop.n.01_1)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop scanner.n.02_1 table.n.02_1)
        (not 
            (toggled_on scanner.n.02_1)
        )
        (ontop laptop.n.01_1 table.n.02_1)
        (inroom table.n.02_1 private_office) 
        (inroom floor.n.01_1 private_office) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
