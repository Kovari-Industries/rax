**Goal:** Pick up the pesticide atomizer in the garden and spray insectifuge to fully cover both potted plants in the garden.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal
        (forall
            (?pot_plant.n.01 - pot_plant.n.01)
            (and
                (covered ?pot_plant.n.01 ?insectifuge.n.01_1)
            )
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop insectifuge__atomizer.n.01_1 floor.n.01_1)
        (insource insectifuge__atomizer.n.01_1 insectifuge.n.01_1) 
        (inroom floor.n.01_1 garden)
        (ontop agent.n.01_1 floor.n.01_1)
        (ontop pot_plant.n.01_1 floor.n.01_1)
        (ontop pot_plant.n.01_2 floor.n.01_1)
    )
```
