# Stage 8: independent multi-vehicle simulation

Implemented on `stage8-fleet-simulation` after reviewing PR #7, confirming its successful CI, and merging it into `dev` (`8638479`). This is a civilian, software-only survey simulator. It adds no aircraft/motor control, network transport, website, terrain, collision response or automatic avoidance.

## Architecture

`drone_sim/fleet/` composes the unchanged Stage 6 engine and Stage 7 planner:

| Module | Responsibility |
| --- | --- |
| `config.py` | Versioned fleet/member/action records, unique IDs, shared timestep validation, embedded existing mission and vehicle configuration. |
| `engine.py` | Authoritative fixed-step clock, independent engines, broadcast/targeted commands, atomic publication of updates, derived fleet summaries. |
| `diagnostics.py` | Advisory spatial route separation and synchronized swept proximity. |
| `export.py` | Fleet JSONL, separate vehicle CSV/JSONL and replay configurations, optional static plots. |
| `__main__.py` | Local reproducible configuration runner. |

Planning remains independent of physics. Every `Member` contains an existing Stage 7 `Mission` and Stage 6 `VehicleConfig`; `planning.execution.to_scenario()` validates and converts these into existing waypoint records and reduced mission movement limits. The fleet does not regenerate routes, assign survey areas, choose homes, duplicate the PD controller, integrate physics, or implement another per-vehicle mission state machine. Areas, launch positions and homes are explicit inputs. The initial vehicle position must match the mission start; an arbitrary repositioning is rejected. Initially critical batteries, invalid missions, excessive capabilities and mismatched timesteps are rejected before running.

All **76 previously tracked files** under `existing-tracker/` and `drone_sim/` remain unchanged relative to the merge base. This includes the complete vision system and Stage 6–7 sources. [Preservation and replay evidence](diagnostics/stage8/verification.json).

## Clock and lifecycle

The coordinator advances one immutable `FixedStepClock` exactly once per update and passes the same clock object to every vehicle's `step(next_clock)`. Vehicles cannot independently select dt or advance through the public fleet API. Every member emits the same tick, dt and simulation time, including idle, paused, completed and aborted members. There is no wall-clock timing or randomness.

A step previews each existing engine on a copy. It retains Stage 7 polygon supervision: a predicted motion segment outside the member's flight boundary is rejected and invokes the existing virtual emergency stop at its previous position. Stage 6 continues to enforce the box geofence and battery rules. Updated engines are published together after all candidates are evaluated; an unexpected programming exception propagates without publishing a partly stepped fleet. Known virtual battery/geofence/polygon failures remain isolated to their member. No blanket exception handler converts bugs into successful missions.

The fleet's `start()`, `pause()` and `resume()` are broadcasts. `command(name, vehicle_id=None)` also supports `abort` and `emergency_stop`:

| Command | Broadcast recipients |
| --- | --- |
| start | Idle members only. |
| pause | Running members only. |
| resume | Paused members only, including ones paused independently. |
| abort / emergency_stop | All non-aborted members, including completed members. |

Broadcasts with no eligible recipients are recorded no-ops. Targeted commands require a known ID and a valid Stage 6 transition; invalid commands raise `ValueError`. Targeted repeated stop commands retain Stage 6's idempotent stop latch. Restarting a completed or aborted mission is unsupported. Commands at the same tick execute in declared order. A failed scheduled command fails the run visibly; it is not silently skipped. No mission replacement or dynamic member addition/removal is provided.

Pause retains Stage 6 semantics: bounded position holding, continuing dynamics, inertia and battery drain. It is not a frozen simulation clock. Aborted vehicles freeze physical state and battery but continue receiving ticks. Completed vehicles continue station keeping and battery drain until the configured run ends; they can subsequently exhaust the battery. Fleet simulation always emits an initial sample plus exactly `steps` updates.

Fleet status is derived from existing member states, not stored as another lifecycle. Priority is: all completed → `completed`; all terminal with any abort → `finished_with_failures`; otherwise any running → `running`, any paused → `paused`, otherwise `idle`. State counts and a separate `degraded` flag expose mixed states. Thus `paused` may coexist with idle or aborted members; it does not imply every member is paused. `completed_fraction` counts currently completed vehicles, while waypoint totals report progress. `first_all_completed_time_s` preserves a historical completion even if a vehicle later aborts; `all_completed` describes the final sample. An unfinished time budget remains unfinished and has no manufactured completion time.

## Advisory conflicts

Two intentionally different diagnostics are provided:

1. **Static route conflict:** minimum three-dimensional Euclidean separation between nominal closed route segments, including ascent/descent and launch-to-first-waypoint legs. One advisory per pair records the closest leg indices and distance when it is at or below `route_conflict_m`. It disregards timing, pauses, dynamics, holding after the final waypoint and battery failures. Overlap is a possible conflict, not a predicted simultaneous collision.
2. **Simulated proximity:** minimum separation of two synchronized linearly interpolated positions between accepted adjacent samples. Relative-motion minimization detects a crossing between ticks even if endpoints are separated. The record includes the minimum distance, endpoint distance and fractional tick of closest approach. Initial overlapping positions are checked too. Every member, including stationary aborted or idle vehicles, participates. `proximity_m` uses inclusive thresholds.

Both operate in ENU meters and full 3D, not image pixels or just horizontal distance. Alerts never change dynamics, mission commands, launch times or routes. A crossing can reach nearly zero simulated separation while both missions continue: there is no contact physics or collision avoidance. Swept checks describe a linear interpolation of sampled point-mass motion, not continuous curved flight or finite vehicle bodies. Near-parallel segment calculations use a floating-point tolerance. Neither diagnostic certifies separation or real aircraft safety.

`proximity_pair_samples` counts pair/record alerts; `proximity_episodes` counts entries into consecutive alert runs for each pair. These are not collision counts. The same pair may produce multiple episodes. `minimum_alert_separation_m` is null if no threshold alert occurred, rather than pretending to be the global closest fleet distance. Route advisories are in the summary and `FleetSimulator.route_advisories`; proximity records are in each fleet sample.

## Configuration and replay

Schema version 1 records `fleet_id`, `members`, `steps`, ordered tick-scheduled `actions`, and both diagnostic thresholds. Every member embeds the complete unchanged mission schema and vehicle configuration, preserving survey area, home, launch, battery, limits and metadata. Vehicle IDs are case-insensitively unique, 1–64 ASCII letters/digits/underscores/hyphens, beginning with a letter or digit; this prevents ambiguous output directories on case-insensitive systems. Members are canonicalized by ID, so input ordering does not change replay. Actions preserve their supplied order within each tick.

Limits are 1–16 vehicles, 512 total nominal route legs, and 1–100,000 updates. These bound configuration complexity, not guaranteed memory/throughput. The CLI collects complete runs in memory and should use modest scenarios. `simulate(config)` is a generator suitable for consuming samples incrementally; the current export helper requires the complete configured run. Summaries and static conflict computation can be expensive for larger fleets.

```sh
# No optional packages needed:
python3 -m drone_sim.fleet --config scenarios/stage8/independent-surveys.json \
  --output-dir /tmp/drone-fleet-surveys

# Replay exactly using the exported expanded configuration:
python3 -m drone_sim.fleet --config /tmp/drone-fleet-surveys/config.json \
  --output-dir /tmp/drone-fleet-replay

# Optional plots with the existing environment; no dependency installation:
MPLCONFIGDIR=/tmp/drone-stage8-mpl .venv-vision/bin/python -m drone_sim.fleet \
  --config scenarios/stage8/crossing.json --plots --output-dir /tmp/drone-fleet-crossing

python3 -m scenarios.stage8.generate
python3 -m unittest discover -q
MPLCONFIGDIR=/tmp/drone-stage8-mpl .venv-vision/bin/python -m unittest discover -q
```

Output directories must be new. CLI exit 0 means every member is currently completed; exit 2 means a valid run is failed or unfinished. Advisory conflicts alone do not change this code. Malformed configuration or invalid scheduled transitions raise explicit errors. Runtime errors do not manufacture successful reports.

Each run exports `config.json`, `summary.json`, `fleet.jsonl` and separate `vehicles/<ID>/` directories. Each vehicle directory has its mission, effective Stage 6 replay configuration, CSV, JSONL and summary. Vehicle JSONL adds `vehicle_id`, `fleet_id` and `fleet_failure` to existing Stage 6 records; CSV uses the unchanged Stage 6 columns with identity established by its directory. Fleet JSONL also records effective command recipients and advisory proximity. Read-only monitoring returns detached snapshots and does not consume events.

Independent vehicle replay records effective broadcast/targeted commands plus any polygon-triggered virtual stop. That recorded stop reproduces the trace; it does not implement new polygon supervision in Stage 6. Re-run the fleet configuration for active supervision on a new scenario. Comparisons against standalone Stage 6 telemetry remove only the three fleet annotation fields. JSON normalization accounts for tuples versus arrays.

Replay is exact within the tested Python/runtime and configuration, without nondeterministic inputs. Cross-version byte identity is **not guaranteed**: Python 3.9 versus 3.12 produced small floating-point differences in the generated traces. Both runtimes pass their own reproducibility tests. PNG byte identity is not guaranteed.

## Measured scenario results

All fixtures use dt=0.02 s and two vehicles. Expanded [scenario configurations](../scenarios/stage8/) and their generator are committed. [Machine-readable summaries and diagnostic plots](diagnostics/stage8/) are committed; full local exports are ignored under `artifacts/stage8/`. Results below were generated on Python 3.12.14.

| Scenario | Run duration | Completion / failure | Static route advisories | Proximity episodes |
| --- | ---: | --- | ---: | ---: |
| Independent surveys | 100 s | Both complete at 75.34 s with separate areas, homes and launches. | 0 | 0 |
| Simultaneous crossing | 14 s | Both complete at 8.50 s; motion unchanged by alerts. | 1 | 1 |
| Staggered crossing | 26 s | Complete at 8.50 / 20.50 s; manually configured second launch at 12 s. | 1 | 0 |
| Fleet lifecycle | 18 s | Fleet pause at 1 s; independent resume at 2 s, remaining resume at 3 s; completion 9.94 / 11.18 s. | 0 | 0 |
| One battery failure | 14 s | Artificial rapid drain aborts alpha; bravo completes at 8.50 s. | 0 | 0 |
| One boundary failure | 54 s | Concave survey aborts before a crossing; alpha completes at 8.50 s. | 0 | 0 |

The simultaneous crossing produces 52 pair/sample alerts and a minimum interpolated separation of approximately `9.81e-18 m` (numerically zero). This demonstrates detection of an unsafe simulated overlap, **not avoidance or a safe fleet mission**. The near-zero value is ideal point-mass geometry, not localization accuracy. The staggered schedule is a supplied fixture, not an automatically generated deconfliction plan. Survey completion means existing waypoints settled; it does not establish actual camera coverage.

## Validation and limitations

Complete suite: **198 tests**. Python 3.9.6 core: **194 pass, four optional tests skip**. Existing Python 3.12.14 optional environment: **all 198 pass**. The optional Stage 5 codec tests still emit existing PyAV/OpenCV duplicate Objective-C class warnings without failing. No new dependencies were installed.

The 27 new tests cover identity/config validation, mixed-dt rejection, mission/launch/capability rejection, canonical ordering and JSON round trip, survey/home preservation, shared object/tick synchronization across states, exact standalone dynamics compatibility, targeted and fleet lifecycle, read-only snapshots, battery/polygon failure isolation, completion history, scheduled replay, 3D segment geometry, between-tick crossing, synchronized versus asynchronous geometry, initial overlaps, vertical separation, stationary aborted members, advisory invariance, deterministic separate exports, CLI/partial-export behavior, all six fixtures and optional plotting. All prior 171 tests are retained.

All six exported fleets and all twelve separate vehicle traces were independently replayed exactly on Python 3.12.14. Numeric output byte reproducibility is also tested. Existing source preservation is checked against the merge base with Git's line-ending normalization.

Limitations: no automatic avoidance, contact dynamics, route optimization, survey-area assignment, fleet energy optimization, transport failures, dynamic membership, heterogeneous dt, real geodesy, terrain or perception coupling. Stage 6's simplified gravity-compensated point mass, exact-state PD feedback, heuristic battery and instantaneous virtual stop remain unchanged. Stage 7's boundary clearance/turn smoothing and live return-home handoff limitations remain. Diagnostic thresholds are user assumptions, not vehicle dimensions or certified separation minima. A future avoidance design must explicitly define prediction, authority, priorities, deadlocks and validation before it can alter trajectories.
