# SIH 2026 — Problem Statement 26037
## Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles on Unstructured Indian Roads

Status: **Phase 4 — reactive planning + speed-aware decision layer.**
Prediction, collision checking, and a cost-based adaptive local planner
(Phase 3) are joined by a real decision layer: `decision/behaviorDecision.m`
classifies risk (worst-case TTC + proximity across tracked agents) into
`cruise`/`yield`/`emergency_stop`, and `decisionStateMachine.m` adds
hysteresis so that classification doesn't flicker. `main.m` now uses the
resulting state to modulate `vehicleController`'s target speed (plus a
scenario-appropriate base cruise speed - e.g. market traffic never cruises
as fast as a highway) - the vehicle can finally slow or stop, not just
steer. `villageRoad` and `urbanIntersection` also got their missing agent
types filled in (parked vehicle, pedestrian, motorcycle, auto-rickshaw,
pothole; a diagonally-merging auto-rickshaw) per the original scenario spec.

Two real bugs turned up building this and are fixed: `irregularMotionModel`
was growing a fast uncertainty radius for *any* irregular-class agent even
when stationary (a parked pushcart, a pothole), which made static obstacles
effectively unavoidable once that radius outgrew the planner's corridor -
now radius growth is keyed off actual agent speed. And the original
`PROXIMITY_YIELD_DIST` (5m) was wider than these roads are, so a stopped
ego next to any roadside static agent could never out-distance the trigger
and deadlocked permanently - tightened to 2m (a real near-graze distance).

Verified in MATLAB across all five scenarios after both fixes: **all five
reach goal with no critical TTC events and 2.0m+ clearance throughout**, and
`urbanIntersection`'s previous near-miss (TTC 0.80s/clearance 0.75m under
lateral-only avoidance) is resolved (TTC 3.80s/clearance 2.75m) now that the
vehicle can yield to crossing traffic instead of only swerving.

Camera/LiDAR/radar perception and tracking are still stubs; every stub
keeps the fixed signature from `docs/architecture.md` (see its "Interface
change log" for every signature that changed once a function actually had
to do its documented job) so later phases can fill in real logic without
breaking callers.

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
