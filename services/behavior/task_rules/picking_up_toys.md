**Goal:** Put all the toys in the child's room - the three board games (two on the bed and one on the table), the two jigsaw puzzles on the table, and the tennis ball on the table - inside the toy box on the table in the child's room.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (exists
                (?toy_box.n.01 - toy_box.n.01)
                (forall 
                    (?jigsaw_puzzle.n.01 - jigsaw_puzzle.n.01) 
                    (inside ?jigsaw_puzzle.n.01 ?toy_box.n.01)
                ) 
            )
            (exists
                (?toy_box.n.01 - toy_box.n.01)
                (forall 
                    (?board_game.n.01 - board_game.n.01) 
                    (inside ?board_game.n.01 ?toy_box.n.01)
                ) 
            )
            (exists
                (?toy_box.n.01 - toy_box.n.01)
                (forall
                    (?tennis_ball.n.01 - tennis_ball.n.01)
                    (inside ?tennis_ball.n.01 ?toy_box.n.01)
                )
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop board_game.n.01_1 bed.n.01_1)
        (ontop board_game.n.01_2 bed.n.01_1)
        (ontop board_game.n.01_3 table.n.02_1)
        (ontop jigsaw_puzzle.n.01_1 table.n.02_1) 
        (ontop jigsaw_puzzle.n.01_2 table.n.02_1) 
        (ontop tennis_ball.n.01_1 table.n.02_1)
        (inroom floor.n.01_1 childs_room)
        (inroom bed.n.01_1 childs_room)
        (inroom table.n.02_1 childs_room)
        (ontop toy_box.n.01_1 table.n.02_1)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
