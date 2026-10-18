**Goal:** Finish the three-disk Tower of Hanoi in its last two moves: move the medium disk from the middle peg to the right peg with the right hand, then the small disk from the left peg onto it with the left hand.

**Success requires, held for a number of consecutive steps:**
- All three disks are stacked on the right peg, large at the bottom and small on top, each centred on the peg and resting at its stacking height, and all disks are still.

**Fails the episode if, held for a number of consecutive steps while the disks are still:**
- A larger disk rests on a smaller one, or a disk lies on the table off the pegs.

**Scene variation:** the base's position along the table changes between layouts; the starting arrangement (small disk on the left peg, medium in the middle, large on the right) does not.
