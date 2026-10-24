# Isaac elevator validation

The ML `packages/evo_isaac/scripts/navigation_trial.py` runner loads this checkout
and calls the existing elevator routines. `SimulationElevator` accepts only a
context with `simulated: true` and `scene.observation_source:
simulator_ground_truth`. Each button observation must repeat that source.
A live robot context cannot use this class.

```mermaid
flowchart LR
  Task[Rax elevator routine] --> Model[Evo native model and base controller]
  Model --> Physics[Isaac force-limited joints]
  Physics --> Measured[Measured joints, base pose and button travel]
  Measured --> Task
```

The simulation adapter supplies a clock which advances physics during door waits
and press dwell. Observation age still uses the capture wall clock. This prevents
a slow simulation from using a wall-time wait as a substitute for simulated dwell.
The default live clock is unchanged.

A press is successful only when activation verification reports success. The
simulation verifier requires one event for the requested button and release.
The live verifier still reports unknown until a qualified activation detector is
available. Native action failure continues to stop the routine.

Press preparation uses a bound on the actual orientation change of the tool.
An unchanged orientation requires no rotational clearance. A nonzero rotation
still requires clearance for the tool offset along the shortest orientation arc.

The ML pilot uses an assumed palm stylus and fixture geometry. Each run records
its fixed, planar or free-base constraint and model-gravity option. These observations
do not establish camera perception, a real tool calibration, tipping stability,
or transfer to hardware. See the ML campaign report for measured outcomes and
immutable evidence links. Failed runs remain failures.

The [ML Hilton reference](https://github.com/Kovari-Industries/ml/blob/benson/rax-isaac-elevator-navigation/packages/evo_isaac/docs/HILTON-REFERENCE.md)
uses the user-supplied UH and Texas Lodging maps. It selects the north-wing guest
elevator and keeps the adjacent garage shaft distinct. The first- and second-floor
profiles retain source hashes and map assumptions. Metric sizes and the test panel
remain assumed. The same added walls feed physical collision and the privileged
navigation geometry. A plotted lobby route is not a completed robot run.

The campaign saves raw state, USD scenes, phase images, sampled GIF/MP4 replays,
and verified S3 archive hashes. A local straight-motion control passes with the
native travel and measured-stop checks. Lateral steering startup and the combined
button/entry/exit task remain under test. This is simulation evidence only.

Run focused task checks from the Rax root:

```sh
uv run --with pytest --with jinja2 --with numpy --with pillow python -m pytest -q \
  local_tests/unit/test_elevator.py local_tests/unit/test_elevator_sim.py \
  local_tests/unit/test_evo_motion.py
```
