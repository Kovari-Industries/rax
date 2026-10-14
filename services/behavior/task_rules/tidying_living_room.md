**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal
        (and
            (ontop ?pot_plant.n.01_1 ?coffee_table.n.01_1)
            (exists
                (?bookcase.n.01 - bookcase.n.01)
                (inside ?hardback.n.01_1 ?bookcase.n.01)
            )
            (ontop ?newspaper.n.03_1 ?coffee_table.n.01_1)
            (ontop ?notebook.n.01_1 ?desk.n.01_1)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init
        (ontop pot_plant.n.01_1 floor.n.01_1)
        (ontop newspaper.n.03_1 floor.n.01_1)
        (ontop hardback.n.01_1 coffee_table.n.01_1)
        (ontop notebook.n.01_1 coffee_table.n.01_1)
        (inroom floor.n.01_1 living_room)
        (inroom coffee_table.n.01_1 living_room)
        (inroom desk.n.01_1 living_room)
        (inroom bookcase.n.01_* living_room)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
