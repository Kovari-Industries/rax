**Goal:** Pick up the two dice from the bed, the two teddy bears from the floor, and the two board games (one from the desk and one from the bed), and place them all inside the same bookcase in the child's room.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (exists
                (?bookcase.n.01 - bookcase.n.01)
                (and 
                    (forall
                        (?die.n.01 - die.n.01)
                        (inside ?die.n.01 ?bookcase.n.01)
                    )
                    (forall
                        (?teddy.n.01 - teddy.n.01)
                        (inside ?teddy.n.01 ?bookcase.n.01)         
                    )
                    (forall
                        (?board_game.n.01 - board_game.n.01)
                        (inside ?board_game.n.01 ?bookcase.n.01)
                    )
                    (forall
                        (?train_set.n.01 - train_set.n.01)
                        (inside ?train_set.n.01 ?bookcase.n.01)
                    )
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop die.n.01_1 bed.n.01_1) 
        (ontop die.n.01_2 bed.n.01_1) 
        (ontop teddy.n.01_1 floor.n.01_1) 
        (ontop train_set.n.01_1 desk.n.01_1) 
        (ontop teddy.n.01_2 floor.n.01_1) 
        (ontop board_game.n.01_1 desk.n.01_1) 
        (ontop board_game.n.01_2 bed.n.01_1) 
        (inroom bookcase.n.01_1 childs_room) 
        (inroom bookcase.n.01_* childs_room)
        (inroom floor.n.01_1 childs_room) 
        (inroom desk.n.01_1 childs_room) 
        (inroom bed.n.01_1 childs_room)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
