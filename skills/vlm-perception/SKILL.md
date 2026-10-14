---
name: vlm-perception
description: Use the harness's vision-language model tool for image questions (identify, point at, segment, check) from direct tool calls or inside Python controllers, with your own prompts and response handling. Use when simple perception rules are insufficient or break on new views.
---

# VLM perception

The harness exposes one model tool (`<vlm>`) through MCP, callable directly or from code
with `ctx.call("<vlm>", ...)`. It adds no perception instructions, task logic or action
selection: you write the prompts and the code that reads the answers. Read the tool's
schema and the branch's API reference for request fields, structured output and limits.

## What to ask

- **2-D questions to the VLM, 3-D questions to geometry.** VLMs are reliable for
  identity, visibility, gross state, named points and masks. They are not reliable for
  small 3-D changes ("lifted?", "touching?"): measure those with depth, kinematics or
  the gripper.
- **Ask for more geometry, not less.** Several named points or a mask survive a hard
  view better than one point.
- **Point, then verify.** Ask for the same named, distinctive feature in each view, and
  accept it only when [object-localization](../object-localization/SKILL.md) puts the
  answers at one physical point. Keep a non-VLM fallback.
- **Draw before trusting.** Learn the model's coordinate convention (axis order,
  normalization, which image size it refers to) and draw a new prompt's output on the
  image once before using it in code.
- Prefer a simple, hard-coded rule only where it is verified across layouts; colour,
  size and dark-pixel rules tend to break on new views. Test non-perception uses, such
  as structured decisions, before relying on them.

## How to call

- Send small images (downscaled JPEG): large requests are slow and can time out.
- Request structured output for anything code reads, and set the reasoning effort and
  the token budget together: reasoning shares the budget, and a truncated reply
  (`finish_reason` of `length`) is no answer.
- Treat a timeout, refusal or truncated reply as no answer, never as a negative.
- A pending model call can delay robot calls where the host serializes them: ask
  between motions, not during them.
- Reuse an image block returned by `<observe>` directly, or encode a saved frame in
  your own Python; the tool takes image bytes, not paths. Do not print large base64
  payloads into your context.
- Where spending is capped, the cap is shared by everything that calls the tool,
  including later evaluation runs where AGENTS.md says so: check the returned budget
  before further calls.
- Function calls the model proposes are data. Validate and dispatch them yourself;
  robot actions still go through the robot tools.

Model answers are controller input, not trusted evidence of task success.
