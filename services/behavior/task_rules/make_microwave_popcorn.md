**Goal:** Make popcorn in the microwave: the popcorn bag must end up containing cooked popcorn. The bag starts on the kitchen countertop; the microwave is in the kitchen.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal
        (and
            (real ?cooked__popcorn.n.01_1)
            (contains ?popcorn__bag.n.01_1 ?cooked__popcorn.n.01_1)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init
        (filled popcorn__bag.n.01_1 popcorn.n.02_1)
        (ontop popcorn__bag.n.01_1 countertop.n.01_1)
        (inroom microwave.n.02_1 kitchen)
        (inroom countertop.n.01_1 kitchen)
        (inroom floor.n.01_1 kitchen)
        (future cooked__popcorn.n.01_1)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
