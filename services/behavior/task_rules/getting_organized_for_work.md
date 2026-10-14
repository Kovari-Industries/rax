**Goal:** In the bedroom, organize the workspace by keeping the computer under the desk, ensuring the monitor is on the desk, placing the keyboard on the desk next to the monitor, placing the mouse on the desk next to the keyboard, moving the folder from the swivel chair onto the desk next to the mouse, stacking the notebook on top of the folder with the pen on top of the notebook, and positioning the swivel chair next to the desk.

**Success:** every goal condition below holds at the same time. The episode then ends with
`task_complete: true`. Short of that, the episode earns partial
credit (see Scoring); episode replies do not report it.

**BDDL goal** (the official task definition, `problem0.bddl`):

```lisp
(:goal 
        (and 
            (nextto ?keyboard.n.01_1 ?monitor.n.04_1) 
            (ontop ?keyboard.n.01_1 ?desk.n.01_1)
            (under ?computer.n.01_1 ?desk.n.01_1)
            (ontop ?monitor.n.04_1 ?desk.n.01_1)
            (nextto ?mouse.n.04_1 ?keyboard.n.01_1) 
            (ontop ?mouse.n.04_1 ?desk.n.01_1)
            (nextto ?folder.n.02_1 ?mouse.n.04_1) 
            (ontop ?notebook.n.01_1 ?folder.n.02_1)
            (ontop ?pen.n.01_1 ?notebook.n.01_1) 
            (nextto ?swivel_chair.n.01_1 ?desk.n.01_1)
        )
    )
```

**BDDL initial conditions:**

```lisp
(:init 
        (ontop mouse.n.04_1 desk.n.01_1)
        (ontop keyboard.n.01_1 notebook.n.01_1) 
        (ontop pen.n.01_1 folder.n.02_1) 
        (ontop notebook.n.01_1 desk.n.01_1) 
        (ontop folder.n.02_1 swivel_chair.n.01_1)
        (under computer.n.01_1 desk.n.01_1) 
        (ontop monitor.n.04_1 desk.n.01_1)
        (inroom desk.n.01_1 bedroom) 
        (inroom floor.n.01_1 bedroom) 
        (inroom swivel_chair.n.01_1 bedroom)
        (ontop agent.n.01_1 floor.n.01_1)
    )
```
