**Goal:** Pick up the two gym shoes and the two sandals from the corridor floor and place them onto the hallstand (shoe rack) in the corridor, making sure they are on the rack and not on the floor. Arrange them so the two gym shoes are next to each other and the two sandals are next to each other.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall 
                (?sandal.n.01 - sandal.n.01)
                (and
                    (touching ?sandal.n.01 ?hallstand.n.01_1)
                    (not
                        (touching ?sandal.n.01 floor.n.01_1)    
                    )
                )
            )
            (forall 
                (?gym_shoe.n.01 - gym_shoe.n.01)
                (and
                    (touching ?gym_shoe.n.01 ?hallstand.n.01_1)
                    (not
                        (touching ?gym_shoe.n.01 floor.n.01_1)    
                    )
                )
            )
            (nextto gym_shoe.n.01_1 gym_shoe.n.01_2)
            (nextto sandal.n.01_1 sandal.n.01_2)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop gym_shoe.n.01_1 floor.n.01_1) 
        (ontop gym_shoe.n.01_2 floor.n.01_1) 
        (ontop sandal.n.01_1 floor.n.01_1) 
        (ontop sandal.n.01_2 floor.n.01_1) 
        (inroom floor.n.01_1 corridor) 
        (inroom hallstand.n.01_1 corridor) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
