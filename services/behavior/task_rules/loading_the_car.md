**Goal:** Put the digital camera from the living room table into the container on the living room floor. Then take the container and the tennis racket to the garage, place both in the car trunk, and close it.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (inside ?container.n.01_1 ?car.n.01_1) 
            (inside ?digital_camera.n.01_1 ?container.n.01_1)
            (inside ?tennis_racket.n.01_1 ?car.n.01_1)
        )
        (not 
            (open ?car.n.01_1)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (inroom car.n.01_1 garage) 
        (ontop container.n.01_1 floor.n.01_2) 
        (ontop tennis_racket.n.01_1 table.n.02_1) 
        (ontop digital_camera.n.01_1 table.n.02_1)
        (inroom table.n.02_1 living_room)
        (inroom floor.n.01_1 garage) 
        (inroom floor.n.01_2 living_room) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
