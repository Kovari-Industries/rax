---
name: multi-env-exploration
description: Select and operate parallel exploration environments, keep their independent scenes apart, and share one episode budget. Use when status lists more than one exploration environment.
---

# Multi-env exploration

Exploration offers several independent environments. `status` lists them and their
episode state; a top-level active episode is not the pool inventory. Continue an active
scene instead of starting again after a resume.

## Calls

- Pass the selected `env_id` on every environment-scoped call: starting, checking and
  finishing an episode, and every robot tool that
  [robot-interface](../robot-interface/SKILL.md) marks as scoped. Session-wide tools
  (`status`, the VLM, pure math) take none.
- In development Python, bind an environment once and call tools without repeating
  `env_id`; keep the binding in the development wrapper, so the controller's `main(ctx)`
  stays the same everywhere ([autonomous-control](../autonomous-control/SKILL.md)).
  Give each concurrent Python worker its own `Context()`: requests on one connection run
  one at a time.
- Harness calls that freeze or evaluate the controller act on the whole session; finish
  outstanding control in every environment first.

## Environment rules

- Each environment has its own scene, step counter and episode. Starting one does not
  affect the others, and every start, including a failed one, uses an episode from the
  shared budget. An idle environment is capacity, not a free episode.
- Keep multi-call sequences (a plan and its execution, a preview and its replay) in the
  environment that produced them, and pass an observation step only to the environment
  that produced it. Step ids count separately per environment, so a wrong one can still
  be accepted: track the `env_id` yourself.
- Calls within an environment run one at a time; different environments run
  concurrently. This is not a lock around a whole script.
- Tie observations, pixels, plans and evidence paths to the `env_id` and episode, not
  just a step number. Other environments have other layouts: they are not copies for
  same-state comparisons.

For delegated work, read [multi-agent-exploration](../multi-agent-exploration/SKILL.md).
