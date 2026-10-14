**Goal:** Move the two storage containers from the living room to the garage. In the garage, place one container on the floor and stack the other container on top of it (either order is fine).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (or
                (and
                    (ontop ?storage_container.n.01_1 ?floor.n.01_2)
                    (ontop ?storage_container.n.01_2 ?storage_container.n.01_1)
                )
                (and
                    (ontop ?storage_container.n.01_2 ?floor.n.01_2)
                    (ontop ?storage_container.n.01_1 ?storage_container.n.01_2)
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop storage_container.n.01_1 floor.n.01_1)
        (ontop storage_container.n.01_2 floor.n.01_1)
        (inroom floor.n.01_1 living_room) 
        (inroom floor.n.01_2 garage) 
        (ontop agent.n.01_1 floor.n.01_2)
    )
```
