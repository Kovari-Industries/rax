# Regression tests

Run from the repository root in the robot service's Python environment:

```bash
env -u PYTHONPATH "$RAX_PYTHON" -m pytest -q
```

`env -u PYTHONPATH` keeps packages a ROS installation puts on `PYTHONPATH` (often from
`~/.bashrc`) out of the run; their pytest plugins fail to import in this environment.

The suite uses temporary artifact roots, the `mock` robot backend, a simulated RB-Y1
behind the `SdkRobot` surface with the vendor URDF, and fake model providers. It never
moves hardware, opens cameras, calls a model or needs credentials. It needs `rby1-sdk`
(for the vendor kinematics) but no robot. Tests marked `hardware` are excluded by
default.

The Evo complete-path check requires the dedicated mock runtime described in
`EVO_V3.md`. It checks the runtime's simulation flag before it arms a test session.
Run it with the isolated socket and session root:

```bash
EVO_AGENT_TEST_SOCKET=/tmp/evo-agent-simulation/agent/agent.sock \
EVO_AGENT_TEST_SESSION_ROOT=/tmp/evo-agent-simulation-rax/sessions \
python -m pytest local_tests/integration/test_evo_agent_runtime.py -q
```

The complete-path check also sends cancel and hold through the RAX frontend
socket while an observation is pending. The action must reach a measured stop
before that observation is released. Unit socket tests cover blocked model work
and image writes; ordinary backend calls remain serialized.

The check saves measured results in the test session's `verification.json`.
Without these environment variables, pytest skips this check.

The remaining Evo integration cases require an explicit operator selection:

| Test | Extra setting |
|---|---|
| Recorder loss during motion | `EVO_AGENT_TEST_RECORDER_PID` names this mock runtime's recorder |
| Request replay after restart | `EVO_AGENT_TEST_PREVIOUS_SESSION` names a completed test session |
| Shutdown during motion | `EVO_AGENT_TEST_ALLOW_SHUTDOWN=1` |

Run each selected case by its pytest function name. The recorder and shutdown
checks stop the selected mock process. They check the runtime identity before
sending a signal. Restart Evo before the replay check and before any later motion
check. Each case writes its measured result to the test session directory.
