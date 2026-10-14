**Goal:** Take the sack of groceries out of the car trunk in the garage, bring it to the kitchen, and put both the tomato and the carton of milk into the refrigerator in the kitchen. When you're done, close the car trunk and make sure the refrigerator in the kitchen is closed.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (exists
                (?electric_refrigerator.n.01 - electric_refrigerator.n.01)
                (and
                    (inside ?beefsteak_tomato.n.01_1 ?electric_refrigerator.n.01)
                    (inside ?carton__of__milk.n.01_1 ?electric_refrigerator.n.01) 
                )
            )
            (not
                (open car.n.01_1)
            )
            (not
                (open electric_refrigerator.n.01_1)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (inroom car.n.01_1 garage) 
        (open car.n.01_1)
        (inside sack.n.01_1 car.n.01_1)
        (inside beefsteak_tomato.n.01_1 sack.n.01_1)
        (inside carton__of__milk.n.01_1 sack.n.01_1)
        (inroom electric_refrigerator.n.01_1 kitchen)
        (inroom electric_refrigerator.n.01_* kitchen)
        (inroom floor.n.01_1 garage) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
