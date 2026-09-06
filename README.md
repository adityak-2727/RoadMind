# SIH 2026 — Problem Statement 26037
## Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles on Unstructured Indian Roads

Status: **Phase 5 — full perception-to-control loop, no more ground-truth
shortcut.** `perception/` is implemented: synthetic camera/lidar/radar
detection (each degrading ground truth to mimic that sensor's real
strengths/weaknesses - see `docs/architecture.md`'s perception section for
why and how this is honestly labeled, never presented as real AI output),
spatial-proximity sensor fusion, and nearest-neighbor object tracking with
persistent IDs. This feeds the Phase 3/4 prediction, decision
(`cruise`/`yield`/`emergency_stop`), adaptive planning, and speed-aware
control that were already in place - the vehicle now runs the entire
documented pipeline every step, not a `groundTruthAgents` passthrough.

Two more real bugs turned up validating this and are fixed:
- `decisionState` chattered between `yield`/`emergency_stop` every 0.5-0.9s
  in `highwayMerge` - `calculateTTC` depends on `egoState.velocity`, and the
  decision layer sets that same velocity, so braking hard enough could make
  the next instant's TTC estimate look briefly safe, flip back to `yield`,
  re-accelerate, and repeat. Fixed with a minimum dwell time before
  de-escalating (escalation stays immediate) - see architecture.md's "Known
  behavior" note.
- Added `rng(simCfg.randomSeed)` so runs stay reproducible despite the new
  sensor-noise/missed-detection randomness.

Re-verified all five scenarios end-to-end with real (noisy, imperfect)
perception: **all five still reach goal.** Clearance held at 2.0m+ for four
of five; **`urbanIntersection` tightened to 1.72m** (down from 2.75m under
perfect ground truth) - not a bug, but a real, honest cost of sensor noise
and occasional missed detections, and tight enough to flag rather than call
comfortably safe.

Every function still keeps the fixed signature from `docs/architecture.md`
(see its "Interface change log" for every signature that changed once a
function actually had to do its documented job) so later phases can fill in
real logic without breaking callers.

## Pipeline

```
sensors (camera/lidar/radar) -> perception -> tracking -> prediction
    -> decision -> planning -> control -> vehicle model -> [ego state feeds back to sensors]
```

See [docs/architecture.md](docs/architecture.md) for full function signatures and
the closed-loop data-flow diagram.

## Folder guide

| Folder | Purpose |
|---|---|
| `config/` | Struct schemas / config builders: simulation, vehicle, planner, agent, ego state |
| `perception/` | Per-sensor detection stubs + fusion + tracking |
| `prediction/` | Motion models that project tracked agents forward in time |
| `planning/` | Global route, local candidate generation, adaptive selection, collision check, smoothing |
| `decision/` | Behavior-level state machine (yield, overtake, stop, crawl, etc.) |
| `control/` | Trajectory-tracking controllers and the vehicle (bicycle) model |
| `scenarios/` | The 5 target Indian-road scenarios (village, intersection, highway merge, market, cattle crossing) |
| `evaluation/` | Metrics: TTC, path smoothness, replanning latency, completion rate |
| `visualization/` | Plotting helpers for objects, trajectories, paths, demo figure |
| `tests/` | Unit test stubs for collision check, planner, prediction |
| `results/` | Generated `figures/`, `tables/`, `logs/` (gitkept, populated at runtime) |
| `docs/` | Architecture, methodology, final report |
| `demo/` | End-to-end demo runner |

## Build budget

30-hour hackathon build. This phase (Hour 0-1) only sets up structure and
interfaces so Phase 1 onward is pure implementation, not design churn.
