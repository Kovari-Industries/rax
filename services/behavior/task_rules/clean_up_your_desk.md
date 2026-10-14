**Goal:** In the child's room, clean up the desk: put both folders and both paperback books into the bookcase; put the pencil and both pens into the pencil case and leave the case on the desk; take the stapler out of the bookcase and place it on the desk; move the laptop from the bed onto the desk and close it.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and  
            (forall 
                (?folder.n.02 - folder.n.02) 
                (inside ?folder.n.02 ?bookcase.n.01_1)
            ) 
            (forall 
                (?pen.n.01 - pen.n.01) 
                (inside ?pen.n.01 ?pencil_box.n.01_1)
            ) 
            (forall 
                (?paperback_book.n.01 - paperback_book.n.01) 
                (inside ?paperback_book.n.01 ?bookcase.n.01_1)
            ) 
            (inside ?pencil.n.01_1 ?pencil_box.n.01_1) 
            (ontop ?stapler.n.01_1 ?desk.n.01_1) 
            (ontop ?pencil_box.n.01_1 ?desk.n.01_1) 
            (ontop ?laptop.n.01_1 ?desk.n.01_1)
            (not
                (open ?laptop.n.01_1)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (open laptop.n.01_1) 
        (ontop folder.n.02_1 desk.n.01_1) 
        (ontop folder.n.02_2 chair.n.01_1)
        (ontop pencil.n.01_1 desk.n.01_1) 
        (ontop pen.n.01_1 desk.n.01_1)
        (ontop pen.n.01_2 desk.n.01_1) 
        (inside stapler.n.01_1 bookcase.n.01_1) 
        (ontop laptop.n.01_1 bed.n.01_1) 
        (ontop paperback_book.n.01_1 desk.n.01_1) 
        (ontop paperback_book.n.01_2 desk.n.01_1)
        (ontop pencil_box.n.01_1 desk.n.01_1)
        (inroom chair.n.01_1 childs_room)
        (inroom bed.n.01_1 childs_room)
        (inroom bookcase.n.01_1 childs_room) 
        (inroom desk.n.01_1 childs_room) 
        (inroom floor.n.01_1 childs_room)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
