**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall 
                (?briefcase.n.01 - briefcase.n.01)
                (nextto ?briefcase.n.01 ?sofa.n.01_1)
            )
            (forall
                (?satchel.n.01 - satchel.n.01)
                (nextto ?satchel.n.01 ?sofa.n.01_1)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (inroom car.n.01_1 garage)
        (inside briefcase.n.01_1 car.n.01_1) 
        (inside satchel.n.01_1 car.n.01_1) 
        (inroom sofa.n.01_1 living_room)
        (inroom floor.n.01_1 garage) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
