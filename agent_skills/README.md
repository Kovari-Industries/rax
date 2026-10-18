# Agent skills

Every skill an agent can receive lives here, once. The workspace template
(`auto_research_agent/`) holds the instructions and the in-container API; the workspace
generator (`services/workspace/`) builds each session's `.agents/skills/` from these
sources. Claude Code reads the same skills through a `.claude/skills` link.

## Layers

Skills are grouped by what they are about. A workspace copies the layers of its setup in
this order (`SKILL_LAYERS` in `services/workspace/recipes.py`). A later layer adds files
to a skill of the same name or replaces them, and a layer with no directory is skipped.

| Layer | Contents |
| --- | --- |
| `task/` | `manual-robot-control`, `in-context-action-learning`, `experience-memory` |
| `coordination/` | `multi-env-exploration`, `multi-agent-exploration` |
| `perception/<mode>-<robot>/` | `object-localization` for that mode and robot, and mode-specific files of other skills |
| `control/<robot>/` | the robot's `robot-interface` and control skills |
| `auto-research/` | `autonomous-control`, `gemini`, `transfer-handoff`, and the formal-copy variants of `autonomous-control` and `experience-memory` |
| `method/` | the research-method skills, for backends whose registry entry sets `method_skills` |

`<mode>` is `depth` (the `rgbd` profile) or `rgb-only`, and `<robot>` is `humanoid` (Unitree
G1 with Dex3 hands under SONIC, HumanoidArena). The agent sees one flat `.agents/skills/` with
robot-neutral names, so skills link to `robot-interface`, `free-space-move` or
`object-localization` without naming the robot:

| Skill | Contents |
| --- | --- |
| `robot-interface` | semantic40 reference frames, observations, feedback, saved frames, body facts and tool choice (depth adds its pixel-to-position helper) |
| `free-space-move` | cuRobo moves of one hand, optionally with the waist |
| `controller-tracking` | how SONIC tracks references only approximately, and how to measure and correct for it |
| `controller-field-guide` | measured facts about the controller: interface, body signals, falls, perception timing, with references on locomotion, upper body and hands |
| `learn-from-experts` | how to use the expert episodes in `runtime/experts/` (installed only with them) |
| `object-localization` | depth: `humanoid_pixel_to_position`; RGB-only: images and measured robot state, without depth or camera geometry |

The `method/` layer holds robot-neutral research practice: `measure-success`,
`evidence-and-regression`, `change-discipline`, `engineer-the-controller`,
`rehearse-the-controller`, `avoid-overfitting`, `learn-from-incidents`,
`match-precision-to-error`, `verify-committal-actions`, `prioritize-by-funnel`,
`spend-the-budget` and `keep-trying`. Skills never name the task's objects; those belong in
`TASK.md` (the registry's `task_nouns`).

## How a workspace gets its skills

1. **Copy** the setup's layers in the order above.
2. **Apply the role and leave out what the session cannot use:**
   - one environment drops `multi-env-exploration`; no subagents drops
     `multi-agent-exploration`; no demonstration drops `in-context-action-learning`; no
     expert episodes drops `learn-from-experts`;
   - `transfer-handoff` is installed for an adaptation source (`--write-handoff on`) and
     for a live-copy explorer whose copies carry `handoff/`, never for a formal copy;
   - a live-copy explorer drops `rehearse-the-controller`, and a formal copy drops the
     method skills about later episodes, bundles and changelogs;
   - a formal copy takes each `<file>.formal-episode.md` in place of `<file>.md` (other
     workspaces drop these variants) and drops the memory-format and pose-record
     references; its `AGENTS.md` comes from the template's `FORMAL_AGENTS.md`;
   - other CLIs than Codex drop `agents/openai.yaml` (Codex skill metadata).
3. **Render:** every Markdown file is a Jinja template (`services/workspace/templates.py`)
   rendered against the session's facts. Statement lines disappear from the output:

   ```text
   {% if formal_episode %}
   text for an agent formal copy
   {% else %}
   text for an exploring agent
   {% endif %}
   ```

   | Name | Value |
   | --- | --- |
   | `agent_cli` | `codex` or `claude` |
   | `phase` | `exploration`, or `isolated` for a formal copy |
   | `observation_profile`, `camera_geometry` | the profile; whether it has depth and camera geometry |
   | `environments`, `multi_env` | exploration environments; two or more |
   | `subagent_cap`, `shared_env_subagents` | the subagent cap; one environment shared by subagents |
   | `agent_formal` | an exploring agent in live-copy mode: no bundle tools, practice framing |
   | `formal_episode` | an agent formal copy during its one episode |
   | `handoff`, `carries_handoff` | the transfer-handoff skill is installed; the live copies will carry `handoff/` |
   | `demonstrations`, `demonstration_kind`, `one_demonstration_image` | the demonstration shown, its kind, and whether it is one image |
   | `expert_demonstrations`, `expert_recordings`, `expert_camera_only` | expert episodes mounted; complete recordings; camera-only episodes |
   | `partial_credit`, `graded_score`, `wall_limit` | episodes earn partial credit; formal scores are graded rather than success or failure; episodes have a wall-clock limit |
   | `method_skills` | the research-method skills are installed |
   | `benchmark`, `humanoid`, `robot.*` | the backend, a flag per backend, and its robot facts (tool names, cameras, control rate) from the registry (`services/benchmarks.py`) |

   Shared text blocks in `agent_blocks/` are included with `{{ block("<id>") }}`.

Prefer a layer over a condition: robot- or mode-specific text belongs in
`control/<robot>/` or `perception/<mode>-<robot>/`.

## Checking a change

`local_tests/unit/test_workspace_goldens.py` compares every rendered setup with
`local_tests/goldens/workspaces.json`, and `test_workspace_validation.py` checks each
workspace's links and vocabulary. See [tests](../local_tests/README.md#workspace-goldens)
for reviewing and refreshing the renderings.
