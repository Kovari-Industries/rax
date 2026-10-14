**Goal:** From the two baskets on the bedroom floor, take out the items and organize them in the bathroom: place both detergent bottles under the bathroom sink next to each other; put the box of sanitary napkins on the bathroom shelf; set the soap dispenser on the sink; make sure the cup remains on the sink; put both the toothpaste tube and the toothbrush inside the cup.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (forall 
                (?bottle__of__detergent.n.01 - bottle__of__detergent.n.01)
                (under ?bottle__of__detergent.n.01 ?sink.n.01_1)
            )
            (nextto ?bottle__of__detergent.n.01_1 ?bottle__of__detergent.n.01_2)
            (ontop ?box__of__sanitary_napkin.n.01_1 ?shelf.n.01_1)
            (ontop ?soap_dispenser.n.01_1 ?sink.n.01_1)
            (ontop cup.n.01_1 sink.n.01_1)
            (nextto ?tube__of__toothpaste.n.01_1 cup.n.01_1)
            (inside ?toothbrush.n.01_1 cup.n.01_1)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (inside bottle__of__detergent.n.01_1 basket.n.01_1) 
        (inside bottle__of__detergent.n.01_2 basket.n.01_1) 
        (ontop basket.n.01_1 floor.n.01_1) 
        (ontop basket.n.01_2 floor.n.01_1) 
        (inside box__of__sanitary_napkin.n.01_1 basket.n.01_1) 
        (inside soap_dispenser.n.01_1 basket.n.01_2) 
        (inside tube__of__toothpaste.n.01_1 basket.n.01_2) 
        (inside toothbrush.n.01_1 basket.n.01_2) 
        (ontop cup.n.01_1 sink.n.01_1)
        (inroom floor.n.01_1 bedroom) 
        (inroom shelf.n.01_1 bathroom) 
        (inroom sink.n.01_1 bathroom)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
