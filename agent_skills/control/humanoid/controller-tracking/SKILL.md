---
name: controller-tracking
description: How the frozen SONIC controller tracks your semantic40 references only approximately, and how to measure and correct for it. Read before building walking, turning or contact motions, and when measured motion differs from what you commanded.
---

# The controller tracks references only approximately: measure and correct

You send one reference frame per 50 Hz tick: root motion, root height, root orientation, the 29
joint angles and two binary hand commands ([robot-interface](../robot-interface/SKILL.md)). The
frozen SONIC whole-body controller turns them into motor commands and follows them only
approximately. Measured on an empty floor without contact, its joints stay within about 6 degrees
of the references, while the root drifts from the commanded motion by tens of centimetres, up to
about a metre over a walk. Contact, load and fast motion make both worse.

See [controller-field-guide](../controller-field-guide/SKILL.md) for practical motion limits
and checks to make before relying on a movement.

{{ block("humanoid.tracking-loop") }}
