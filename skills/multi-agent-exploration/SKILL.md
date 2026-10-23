---
name: multi-agent-exploration
description: Divide exploration among subagents with explicit robot or environment ownership and complementary motion or subtask goals, and combine their results. Use before spawning any subagent.
---

# Multi-agent exploration

Follow AGENTS.md for what to do at each point of the task; this skill covers how to
split work between agents. With parallel environments, also read
[multi-env-exploration](../multi-env-exploration/SKILL.md).

## Decide whether to parallelize

Parallel workers save wall time only on independent questions; coordination costs
context, tokens and episodes. Keep work in one agent when each step depends on the
previous outcome or the question is short. Delegate a later subtask only if its worker
can physically create the required starting state; otherwise it waits for the earlier
result. Respect the subagent cap AGENTS.md gives, and size allowances so the episodes
needed later remain.

## Ownership of the robot

- **One environment per robot worker** when there are several: start no more robot
  workers than environments, and keep one for yourself if you will also act.
- **One shared robot:** you hold it by default and grant it to one holder at a time, in
  that worker's brief ("you hold the robot", its allowance and stopping condition).
  Everyone else works off the robot: analyzing saved frames and logs, writing and
  testing code offline, planning. The holder hands back by stopping its control loop,
  waiting for outstanding calls, describing the scene and episode state, and reporting;
  confirm the handoff before granting the robot again.
- **Perception and analysis subagents** never call motion tools. On a real robot, only
  one agent commands the robot.

Interrupting an agent does not cancel a dispatched action: before handing over, stop the
worker's loop, wait for outstanding calls and inspect the scene.

## Divide work before spawning

- Give different workers different motions, subtasks or strategy questions, chosen to
  combine into a solution. Duplicate work only to compare alternatives or test
  robustness, and then give each worker a genuinely different approach with the same
  success check.
- Brief each robot worker: its question, the starting state and how to create it, an
  observable end-state check, the output format, its environment or holding status,
  whether to start or continue an episode, episode and step allowances, stopping
  conditions and result paths. Pass the `TASK.md`, skill and memory paths; observations
  you pass along are not the worker's current scene.
- Launch independent workers without waiting for each one, and share useful findings as
  they arrive.

## Share decisions and findings

Record decisions every part must share (which arm acts, how an object is held, which
frame a handoff uses) in a ledger only you edit. Keep one append-only findings log:
workers claim an experiment before starting it, append results while working, and read
recent entries before choosing the next experiment. Both live in `memory/exploration/`;
see [references/coordination.md](references/coordination.md).

Give each worker separate note, code, data and log paths (`experiments/<agent>/`,
`output/<agent>/`). Only accepted code moves into `code/<project>/`, and only you merge
it there.

## Budget and progress

Allowances are agreements; only the shared budget is a hard limit. Workers stop and
report on success, on their give-up condition (for example the same failure twice
without a new hypothesis) or when their allowance runs out. At regular checkpoints,
move environments and allowances from stalled lines of work to promising ones. The
VLM budget is shared by all agents and by later evaluation: include model calls in each
allowance.

## Combine results

Workers report the tested motion or subtask, environment and episode ids, parameters
and observable preconditions, failures, evidence paths, remaining steps, whether the
episode is still active, and their cost. Distinguish local progress from confirmed task
success. Transfer procedures, not layout-specific coordinates.

A result enters the shared strategy only after you check its evidence. Test the
combined procedure end to end, preferably in a scene not used to develop it. Keep
rejected alternatives with the reason they lost. Record reusable results as
[experience-memory](../experience-memory/SKILL.md) describes. Only you decide when
exploration ends, call the harness's freeze or evaluation tools, and summarize accepted
findings and allocations in `memory/PROGRESS.md`.
