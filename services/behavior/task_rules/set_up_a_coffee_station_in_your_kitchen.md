**Goal:** Set up a coffee station on the kitchen countertop: keep the coffee maker on the countertop, move the bottle of coffee from the kitchen shelf to the counter next to the coffee maker, place a paper coffee filter on top of the coffee maker, put the saucer next to the coffee maker with the coffee cup on the saucer, and place the electric kettle next to the coffee maker.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (ontop ?coffee_maker.n.01_1 ?countertop.n.01_1) 
            (nextto ?bottle__of__coffee.n.01_1 ?coffee_maker.n.01_1) 
            (ontop ?paper_coffee_filter.n.01_1 ?coffee_maker.n.01_1) 
            (nextto ?saucer.n.02_1 ?coffee_maker.n.01_1) 
            (ontop ?coffee_cup.n.01_1 ?saucer.n.02_1) 
            (nextto ?electric_kettle.n.01_1 ?coffee_maker.n.01_1)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop coffee_maker.n.01_1 countertop.n.01_1) 
        (ontop bottle__of__coffee.n.01_1 shelf.n.01_1) 
        (ontop paper_coffee_filter.n.01_1 countertop.n.01_1) 
        (ontop coffee_cup.n.01_1 countertop.n.01_1) 
        (ontop saucer.n.02_1 countertop.n.01_1) 
        (ontop electric_kettle.n.01_1 countertop.n.01_1) 
        (inroom countertop.n.01_1 kitchen) 
        (inroom shelf.n.01_1 kitchen) 
        (inroom dishwasher.n.01_1 kitchen) 
        (inroom floor.n.01_1 kitchen) 
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
