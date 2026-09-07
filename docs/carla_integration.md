# CARLA ↔ MATLAB ↔ Simulink Integration (Phase 9)

This document covers the Phase 9 integration foundation: connecting to
CARLA, spawning one ego vehicle, reading its state, and sending raw control
commands through both a direct MATLAB path and a Simulink path, isolated
behind an adapter layer. **Every claim in this document was verified
against a real, running CARLA server in this session** (not mocked, not
assumed) - the evidence is below. This does not cover perception,
prediction, planning, or the five Indian-road scenarios running against
CARLA - those remain later-phase work. The project's existing MATLAB-only
system (`main.m`, `demo/runDemo.m`, the five scenarios, K1, K2) is
completely unaffected and remains runnable with zero CARLA dependency -
also reverified live in this session (see "Baseline regression" below).

## Verified environment

| Item | Verified value |
|---|---|
| OS | Windows 10.0.26200 |
| MATLAB | R2026a, with Simulink R2026a (licensed and used) |
| CARLA | **0.9.16** (Windows package, `CARLA_0.9.16.zip`, downloaded from `downloads.carlasim.com`, 7.81 GB) |
| CARLA-compatible Python | **3.12.10**, installed separately (winget, user-scoped) specifically for CARLA - the project's default Python 3.13.5 was never touched, downgraded, or shared with CARLA |
| CARLA Python API | `carla==0.9.16`, installed from CARLA's own bundled wheel `PythonAPI/carla/dist/carla-0.9.16-cp312-cp312-win_amd64.whl` into an isolated venv - **not** the PyPI `carla` package (PyPI only has the old 0.9.5) |
| GPU | NVIDIA GeForce RTX 4060 Laptop GPU - server ran headless via `-RenderOffScreen` |
| Install locations | CARLA: `C:\Users\<user>\CARLA_0.9.16\` · venv: `C:\Users\<user>\carla_venv\` - both **outside** the git repo (CARLA is an external tool, not project source) |
| Default map | `Carla/Maps/Town10HD_Opt` (server's default on launch) |

## What exists after Phase 9

```
carlaIntegration/
    python/
        carla_adapter.py            - CARLA Python API wrapper (connect,
                                       spawn, get state, apply control,
                                       cleanup). No perception/planning
                                       logic. Standalone self-test:
                                       `python carla_adapter.py`
    matlab/
        CarlaSession.m               - handle-class singleton holding the
                                        live py.carla_adapter.CarlaAdapter
        getCarlaSession.m            - shared-instance accessor (MATLAB
                                        functions in different files can't
                                        share a plain `persistent` var)
        carlaConnect.m               - Task 3: connect
        carlaSpawnEgoVehicle.m       - Task 5: spawn one ego vehicle
        carlaGetEgoState.m           - Task 6: read state (project schema)
        carlaApplyControl.m          - Task 7: send steer/throttle/brake
        carlaDisconnect.m            - Task 3/9: clean up
        carlaToProjectState.m        - Task 6: CARLA -> project coordinate
                                        conversion (verified on real data
                                        below)
        isCarlaAvailable.m           - non-throwing reachability check
    simulink/
        CarlaSimulinkInterface.m     - Task 4: matlab.System block wrapping
                                        the adapter functions above.
                                        Requires 'SimulateUsing' =
                                        'Interpreted Execution' (MATLAB
                                        System blocks default to
                                        code-generation mode, which does
                                        not support py.* calls - found and
                                        fixed live).
        buildCarlaControlInterfaceModel.m - builds carlaControlInterface.slx,
                                        with real-time pacing enabled
                                        (EnablePacing/PacingRate - see
                                        "Simulation pacing" below)
        carlaControlInterface.slx    - the built model (regenerate via the
                                        script above; do not hand-edit)
    tests/
        testCarlaIntegration.m       - Tests 1-6 (connection, spawn, state,
                                        control, response, cleanup)
config/
    carlaConfig.m                    - the one place CARLA connection
                                        settings live, with verified values
```

## How to set up and run this yourself

1. Install a CARLA-compatible Python (3.7-3.12; 3.12 is what was verified
   here) into its own environment - do not use your default/project
   Python if it's newer:
   ```
   winget install --id Python.Python.3.12 --scope user
   C:\path\to\python3.12.exe -m venv C:\path\to\carla_venv
   ```
2. Download and extract a CARLA release (verified: `CARLA_0.9.16.zip` from
   `https://downloads.carlasim.com/Windows/CARLA_0.9.16.zip`, ~7.8 GB,
   ~20 GB extracted).
3. Install CARLA's own bundled wheel (not PyPI) into the venv:
   ```
   C:\path\to\carla_venv\Scripts\python.exe -m pip install ^
     C:\path\to\CARLA_0.9.16\PythonAPI\carla\dist\carla-0.9.16-cp312-cp312-win_amd64.whl
   ```
4. Fill in `config/carlaConfig.m`'s `pythonExecutable`/`carlaPythonApiPath`
   with your actual paths if they differ from the defaults there.
5. Launch the server (headless, verified):
   ```
   C:\path\to\CARLA_0.9.16\CarlaUE4.exe -RenderOffScreen -carla-server -nosound
   ```
   Takes ~20-30s to become reachable.
6. In MATLAB, **before any other `py.*` or `carla*` call**:
   ```matlab
   pyenv('Version', 'C:\path\to\carla_venv\Scripts\python.exe');
   addpath(genpath(pwd));
   carlaConnect();
   ```

## Live verification evidence

### Step 2/3 - server + MATLAB connection
Server became reachable ~16s after launch and stayed stable across 11
consecutive polls. `carlaConnect()` (the real function, via MATLAB's
`pyenv` pointed at the venv) connected successfully; `session.Connected`
read back `true`; `world.get_map().name` returned `Carla/Maps/Town10HD_Opt`
repeatedly.

### Step 4/5 - spawn + state retrieval (real data)
`carlaSpawnEgoVehicle()` returned real, incrementing actor ids (24, 25, 26,
27 across separate test runs). `carlaGetEgoState()` returned, e.g.:
```
x: -64.6448   y: -24.4710   yaw: -0.0028   velocity: 2.01e-07   timestamp: 323.62
```
**Coordinate transform verified on real data**: CARLA's raw state for this
same reading was `location.y = 24.471`, `rotation_deg.yaw ≈ 0.159°`.
`carlaToProjectState.m`'s `y = -y_carla` and `yaw = deg2rad(-yaw_carla)`
produced exactly `y = -24.4710` and `yaw = -0.0028` rad
(`deg2rad(-0.159°) ≈ -0.00278` rad) - the documented transform is
confirmed correct against a live server, not just derived on paper.

### Step 6 - control commands (real data, both Python and MATLAB layers)
Four separate tests, each sampling a real time-series (not a single
before/after pair - an initial single-pair test was tried first and
correctly discarded as ambiguous, since CARLA's transform can read stale
immediately post-spawn):

| Test | Result |
|---|---|
| Zero control | speed stayed exactly `0.0000 m/s` for 2s (baseline) |
| Throttle=0.6 | speed rose **monotonically** `0.000 → 8.633 m/s` over 3.5s, matching position change (`x: -64.645 → -51.473`) |
| Steer=0.3 + throttle=0.4 | lateral position changed (`y: 24.545 → 31.343`); speed dropped abruptly near the end - consistent with the vehicle contacting something, a realistic outcome of blind steering near map geometry, not a bug |
| Brake=1.0 | speed fell `9.809 → 0.000 m/s` and stayed there |

Confirmed through **both** the standalone Python adapter directly and the
real MATLAB functions (`carlaApplyControl`/`carlaGetEgoState`) - both gave
consistent results.

### Step 7/8 - Simulink + full data/control loop (real data)
Two real integration defects were found and fixed while getting this to
work - both integration-layer only, no autonomy code touched:

1. **Code generation incompatibility.** MATLAB System blocks default to
   attempting code generation, which errors on `py.*` calls
   (`"Function py.sys.path is not supported for code generation"`). Fixed
   by setting `SimulateUsing = 'Interpreted Execution'` on the block
   (now set both in the committed model builder and documented in
   `CarlaSimulinkInterface.m`).
2. **No simulation pacing.** A plain `sim()` runs every tick as fast as
   the CPU allows; CARLA's server advances physics on its own real-time
   clock. Unpaced, velocity stayed exactly `0.0000 m/s` for all 41 ticks
   despite constant throttle=0.6 - the ticks were happening far faster
   than CARLA could physically respond. Fixed with
   `EnablePacing='on'`, `PacingRate=1` (1x real-time), now set in
   `buildCarlaControlInterfaceModel.m`.

With both fixes, a Simulink model (3 Constant blocks → the real
`CarlaSimulinkInterface` block → 4 outputs, `sim()` against the live
server, dt=0.1s, 4s) produced:
```
step=1   v=0.0000 m/s
step=5   v=6.7475 m/s
step=20  v=10.168 m/s
step=41  v=13.658 m/s
```
Monotonically increasing, driven entirely through the real Simulink block
and the real adapter chain to the real CARLA server - this is the
complete `CARLA → ego state → MATLAB → Simulink → control command → CARLA
→ changed vehicle state → MATLAB` loop, demonstrated live, not just
structurally compiled.

### Step 9 - cleanup
`carlaDisconnect()` correctly reported `session.Connected = false`
afterward; called a second time with no error (idempotent). After full
cleanup, a fresh `carlaConnect()` + `carlaSpawnEgoVehicle()` succeeded
immediately (new actor id) - confirms the server survives cleanup and
MATLAB doesn't retain a broken session.

### Baseline regression (with CARLA fully running in the background)
Run against the **default** Python 3.13 environment (never pointed at the
CARLA venv), proving zero coupling:
- `tests/testCollisionCheck.m` / `testPlanner.m` / `testPrediction.m`:
  **14/14 passed, 0 failed**.
- All five scenarios via `runDemo(..., false)`: **5/5 goal reached, 0/5
  geometric collisions**, every metric identical to the frozen K1/K2
  baseline (villageRoad Inf/2.16m, urbanIntersection 1.90s/3.30m,
  highwayMerge 0.70s/2.71m/fallback=8, marketArea Inf/2.15m,
  cattleCrossing Inf/3.01m).
- `git diff` empty for `prediction/trajectoryPrediction.m`,
  `planning/adaptivePlanner.m`, `planning/collisionCheck.m`,
  `decision/behaviorDecision.m`, `control/vehicleController.m`,
  `scenarios/`.

## Coordinate and unit conventions (verified)

| | CARLA | This project |
|---|---|---|
| Handedness | Left-handed | Right-handed |
| Position | `x, y, z` meters | `x, y` meters (z dropped - flat 2D model) |
| Heading | `pitch, yaw, roll`, **degrees** | `yaw`, **radians** only |
| Velocity | 3D vector, m/s | **scalar** longitudinal speed, m/s |

Transform: `y_project = -y_carla`, `yaw_project = deg2rad(-yaw_carla)`,
`velocity_project = hypot(vx_carla, -vy_carla)` (z/vertical and
lateral-slip dropped). See `carlaToProjectState.m` for the full derivation
and the live-data confirmation above.

## Control command conventions

`carlaApplyControl(steer, throttle, brake)` forwards to CARLA's own
`VehicleControl`: `steer ∈ [-1, 1]`, `throttle ∈ [0, 1]`, `brake ∈ [0, 1]`
(clamped before applying). Verified as an **integration-test channel
only** - no scripted/waypoint trajectory was used anywhere in this
verification; every test applied a constant control value and measured
the resulting state, nothing more.

## Cleanup procedure

`carlaDisconnect()` destroys the spawned ego actor and drops session
references; safe to call unconditionally and repeatedly (verified).
`CarlaSimulinkInterface`'s `releaseImpl` calls the same function when a
Simulink model containing it stops/releases. The CARLA server process
itself is a separate OS process (`CarlaUE4.exe`) and is not managed by
these adapters - stop it independently when done (e.g. close the process)
if it isn't needed anymore; this session terminated it after all
verification completed.

## Known limitations

- Verified against exactly one map (`Town10HD_Opt`, the server's default)
  and one blueprint (`vehicle.tesla.model3`) at one spawn point - other
  maps/blueprints/spawn points are expected to work identically (same API
  calls) but were not individually exercised.
- The steering test's abrupt speed drop was not root-caused (plausibly a
  collision with map geometry) - harmless for Phase 9's integration-only
  scope, but worth knowing if reusing that exact test sequence.
- CARLA's simulation clock (`timestamp_s` in the raw state) is not
  reconciled with this project's fixed `dt=0.1` tick convention
  (`simulationConfig.m`) - deferred to whichever later phase actually
  drives the existing MATLAB pipeline from CARLA data.
- `egoState.steering` is not populated from CARLA (not returned by
  `get_vehicle_state()` in a directly comparable convention) - left at
  its default; noted in `carlaToProjectState.m`.

## What Phase 9 deliberately does NOT include

Camera/LiDAR/Radar perception from CARLA sensors, sensor fusion, tracking,
prediction, planning, or decision logic driven by CARLA data; the five
Indian-road scenarios recreated in CARLA; any scripted/pre-recorded
trajectory presented as autonomous driving; RoadRunner, RL, MPC, SLAM,
V2X, or ROS integration. All later-phase work, out of scope here by
explicit instruction.
