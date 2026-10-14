# Agent skills

Every skill an agent can receive lives here, once. The workspace template
(`auto_research_agent/`) holds the instructions and the agent's Python API; the workspace
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
| `perception/common/` | `wrist-camera-inspection` |
| `perception/<mode>-<robot>/` | `object-localization` for that mode and robot |
| `control/<robot>/` | the robot's `robot-interface` and motion skills |
| `auto-research/` | `autonomous-control`, `gemini`, `transfer-handoff`, and the formal-copy variants of `autonomous-control` and `experience-memory` |

`<mode>` is `depth` (the `rgbd` profile) or `rgb-only`, and `<robot>` is `robodojo` (dual
ARX X5, table-top) or `behavior` (R1Pro, BEHAVIOR-1K). The agent sees one flat
`.agents/skills/` with robot-neutral names, so skills link to `robot-interface`,
`free-space-move` or `object-localization` without naming the robot:

| Skill | RoboDojo | BEHAVIOR |
| --- | --- | --- |
| `robot-interface` | motion tools, bounded EEF steps, environment selection, pose records | action rows, frames, odometry, tool choice, environment selection, pose records |
| `free-space-move` | cuRobo single-arm moves | cuRobo arm and trunk moves, local IK |
| `object-localization` | depth: `robodojo_pixel_to_position`; RGB-only: images and measured robot poses, without depth or camera geometry | depth: `behavior_pixel_to_position` (always RGB-D) |
| `pose-math`, `action-preview`, `action-search` | yes | - |
| `base-navigation`, `motion-preview` | - | yes |

## How a workspace gets its skills

1. **Copy** the setup's layers in the order above.
2. **Apply the role and leave out what the session cannot use:**
   - one environment drops `multi-env-exploration`; no subagents drops
     `multi-agent-exploration`; no demonstration drops `in-context-action-learning`;
   - `transfer-handoff` is installed for an adaptation source (`--write-handoff on`) and
     for a live-copy explorer whose copies carry `handoff/`, never for a formal copy;
   - a formal copy takes each `<file>.formal-episode.md` in place of `<file>.md` (other
     workspaces drop these variants) and drops the memory-format and pose-record
     references; its `AGENTS.md` comes from the template's `FORMAL_AGENTS.md`;
   - other CLIs than Codex drop `agents/openai.yaml` (Codex skill metadata).
3. **Render:** every Markdown file is a Jinja template (`services/workspace/templates.py`)
   rendered against the session's facts (`Setup.context()` in
   `services/workspace/setup.py`). Statement lines disappear from the output:

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
   | `camera_geometry` | whether the observation profile has depth and camera geometry (`rgbd`) |
   | `environments`, `multi_env` | exploration environments; two or more |
   | `subagent_cap`, `shared_env_subagents` | the subagent cap; one environment shared by subagents |
   | `agent_formal` | an exploring agent in live-copy mode: no bundle tools, practice framing |
   | `formal_episode` | an agent formal copy during its one episode |
   | `handoff`, `carries_handoff` | the transfer-handoff skill is installed; the live copies will carry `handoff/` |
   | `demonstrations`, `demonstration_kind`, `one_demonstration_image` | the demonstration shown, its kind, and whether it is one image |
   | `partial_credit`, `wall_limit` | episodes earn partial credit; episodes have a wall-clock limit |
   | `robodojo`, `behavior`, `robot.*` | a flag per backend, and the backend's robot facts (tool names, cameras, control rate) from the registry (`services/benchmarks.py`) |

   Shared text blocks in `agent_blocks/` are included with `{{ block("<id>") }}`.

Prefer a layer over a condition: robot- or mode-specific text belongs in
`control/<robot>/` or `perception/<mode>-<robot>/`.

## Checking a change

`local_tests/unit/test_workspace_goldens.py` compares every rendered setup with
`local_tests/goldens/workspaces.json`, and `test_workspace_validation.py` checks each
workspace's links and vocabulary. See [tests](../local_tests/README.md#workspace-goldens)
for reviewing and refreshing the renderings.
