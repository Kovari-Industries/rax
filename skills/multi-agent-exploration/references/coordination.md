# Coordination files

Shared coordination state lives in `memory/exploration/`. Workers' own notes, code,
data and logs stay in their assigned paths; these files hold only what others need.

| File | Writer | Purpose |
|---|---|---|
| `memory/exploration/decisions.json` | main agent only | decisions every part must follow |
| `memory/exploration/findings.jsonl` | all agents, append only | claims, results, proposals and releases |

## `decisions.json`

A JSON list of `{"id", "decision", "reason", "evidence", "status", "supersedes"}`.
Replace an obsolete decision by marking it `superseded` and adding a new entry, so
earlier findings stay interpretable.

## `findings.jsonl`

One JSON object per line, appended in a single write; never edit or reorder earlier
lines. Read the recent lines before claiming new work.

Every record has `id` (unique, e.g. `<agent>-<n>`), `time` (UTC ISO-8601), `agent`,
`kind` and `claim` (the claim `id` it belongs to); with parallel environments, claims,
results and releases also carry the `env_id`. By kind:

- `claim`: `question` (one testable hypothesis), `allowance` (`episodes`, `steps`),
  `stop_when` (success and give-up conditions), `duplicates` (a claim `id` when the
  repetition is deliberate).
- `result`: `episode_id`, `steps` (first and last), `preconditions`, `procedure`,
  `reuse` (below), `outcome` (`supported`, `refuted` or `inconclusive`),
  `confirmation` (`observed` or `task-confirmed`), `evidence` (paths), `cost`
  (`episodes`, `steps`, `wall_seconds`, `model_calls`), optional `next`.
- `proposal`: `change` (to a decision `id`, or a new decision), `reason`, `evidence`.
  Only the main agent turns a proposal into a decision.
- `release`: `reason` (`done`, `gave_up`, `reassigned` or `allowance_spent`),
  `episode_active`, `steps_remaining`, `scene` (the state left behind). On a shared
  robot it hands the robot back to the main agent.

## Record results in reusable form

`reuse` holds what the final solution will contain, so the main agent can adopt it
without re-deriving it:

- `parameters`: each value the result depends on, with `name`, `value`, `unit`,
  `anchor` (what it is measured from), `tested` (values tried and which worked) and
  `source` (measured, tuned or assumed).
- `rules`: each decision as `when` (observable condition), `choose` and `otherwise`.
- `code`: `path`, `function` and `signature` of a tested function written against a
  supplied context, like `main(ctx)` in
  [autonomous-control](../../autonomous-control/SKILL.md), that opens no context of its
  own.

When merging, the main agent keeps parameters as named constants and cites the finding
`id` beside each. `confirmation` follows
[experience memory](../../experience-memory/references/format.md). The log is
coordination state, not a substitute for notes or experience memory, and the controller
must not depend on it.
