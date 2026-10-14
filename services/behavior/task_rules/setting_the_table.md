**Goal:** satisfy every condition of the BDDL goal below (the official task definition).

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall 
                (?plate.n.04 - plate.n.04) 
                (ontop ?plate.n.04 ?breakfast_table.n.01_1)
            )
            (forpairs 
                (?plate.n.04 - plate.n.04) 
                (?cupcake.n.01 - cupcake.n.01) 
                (ontop ?cupcake.n.01 ?plate.n.04)
            ) 
            (forpairs
                (?tablefork.n.01 - tablefork.n.01)
                (?plate.n.04 - plate.n.04)
                (nextto ?tablefork.n.01 ?plate.n.04)
            ) 
            (forpairs
                (?table_knife.n.01 - table_knife.n.01)
                (?plate.n.04 - plate.n.04)
                (nextto ?table_knife.n.01 ?plate.n.04)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (inside cupcake.n.01_1 electric_refrigerator.n.01_1) 
        (inside cupcake.n.01_2 electric_refrigerator.n.01_1) 
        (ontop tablefork.n.01_1 sink.n.01_1) 
        (ontop tablefork.n.01_2 sink.n.01_1) 
        (ontop table_knife.n.01_1 sink.n.01_1) 
        (ontop table_knife.n.01_2 sink.n.01_1) 
        (inside plate.n.04_1 cabinet.n.01_1)
        (inside plate.n.04_2 cabinet.n.01_1)
        (inroom breakfast_table.n.01_1 dining_room) 
        (inroom floor.n.01_1 kitchen) 
        (inroom cabinet.n.01_1 kitchen)
        (inroom sink.n.01_1 kitchen) 
        (inroom electric_refrigerator.n.01_1 kitchen) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
