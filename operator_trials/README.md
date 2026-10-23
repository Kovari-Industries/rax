# Historical Evo operator trials

These files preserve the earlier bounded-step and J4 correction diagnostics from
the target branch. They use the former motion socket and direct ROS diagnostic
interfaces. They are not part of the current Evo backend, tool registry, or agent
workspace. There is no fallback from the current backend to these interfaces.

The [trial record](../docs/evo_trials_2026_10_20.md) gives the recorded outcomes and
physical limitations. Unit tests retain the finite bounds, target identity,
arming, driver-profile, and stop-result checks. They do not enable a physical trial.
