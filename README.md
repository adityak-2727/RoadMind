# SIH 2026 — Problem Statement 26037
## Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles on Unstructured Indian Roads

Status: **Phase 2 — five scenarios with real geometry.** The kinematic bicycle
model, pure pursuit + speed controller (Phase 1), and all five scenarios
(`villageRoad`, `urbanIntersection`, `highwayMerge`, `marketArea`,
`cattleCrossing`) now have real road geometry and ground-truth agents. Each
scenario has been verified in MATLAB: the ego vehicle drives its fixed
reference path end-to-end and reaches the goal (11.6s-20.2s depending on
route length). Obstacle avoidance is not wired in yet — the vehicle currently
drives through agents rather than around them. Perception, prediction,
decision, and the adaptive/cost-based planner are still stubs (Phase 3+);
every stub keeps the fixed signature from `docs/architecture.md` so later
phases (and later swapping in MATLAB Automated Driving Toolbox objects) can
fill in real logic without breaking callers.

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
