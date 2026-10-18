**Goal:** Move the mouse onto the purple mouse pad and click its left button.

**Success requires, held for a number of consecutive steps:**
- The mouse lies on the mouse pad (inside the pad's circle, close to its surface).
- The display has turned blue. It turns blue, and stays blue, once the left button is pressed down far enough while the mouse is on the pad.

**Episode start:** before you act, the evaluation runs a fixed number of steps that bring the hand to a fixed pose above the table; they count toward the environment's step cap, which the step limit already accounts for.

**Scene variation:** the mouse's position and heading (the pad follows the mouse) and the table height change between layouts; the display does not move.
