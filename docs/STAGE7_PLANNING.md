# Stage 7: independent civilian survey mission planning

Stage 7 was developed on `stage7-mission-planning` after review, passing tests/CI and the authorized merge of PR #6 into `dev` (`c57ba93`). Planning is independent of vehicle physics and computer vision. It produces versioned local-ENU missions, validates nominal routes, and optionally hands them to the **unchanged Stage 6 waypoint executor, PD controller, vehicle model and clock**.

This is a civilian survey simulation. No real aircraft control, hardware, website, terrain, geographic projection, camera functionality or new mission-state machine is included. No physical aircraft safety is inferred from the tests.

## Architecture

| Module | Responsibility |
| --- | --- |
| `planning/geometry.py` | Local planar polygons, containment, segment validation and visibility-graph connectors. |
| `planning/mission.py` | Mission records, capability/geofence validation, distance and duration estimates. |
| `planning/routes.py` | Alternating survey lanes, required perimeter pass, virtual return-home routes and coverage diagnostics. |
| `planning/execution.py` | Thin optional Stage 6 adapter and polygon-boundary supervision. No dynamics/state-machine reimplementation. |
| `planning/diagnostics.py` | Optional standalone route plots. |
| `planning/__main__.py` | Request/mission import, validation, export and optional simulation. |

The geometry and mission-planning modules use only the standard library and do not import vehicle or vision modules. The execution adapter translates explicit plan constraints into Stage 6 configuration and waypoints. Matplotlib remains an optional plotting dependency already available locally; no dependencies were installed for Stage 7.

## Mission and area definitions

Mission JSON schema version 1 contains:

- `mission_id`, `schema_version`, `coordinate_frame: "ENU"`, and finite JSON-object `metadata`.
- `home_enu_m`, `start_enu_m`: three-dimensional local coordinates in meters. A survey starts at home; a return-home plan starts at an explicitly supplied current position.
- `altitude_m`: survey/cruise **Up coordinate relative to the local origin**, not height above terrain or sea level.
- `speed_limit_mps`, `acceleration_limit_mps2`: route-wide three-dimensional norm limits.
- `flight_boundary`: a simple East/North polygon defining the permitted horizontal route boundary.
- `survey_area`: a separate simple East/North polygon, or null for a non-survey mission. Home may lie outside the survey area but must lie within the flight boundary and vehicle geofence.
- `waypoints`: positions and incoming `leg_kind` (`transit`, `survey_lane`, `survey_boundary`, `return_home`). These are configuration records, not another mission lifecycle.
- Survey spacing, assumed footprint width and lane orientation; Stage 6 settling/tolerance settings; an explicit duration-estimation settling allowance.

There is no latitude/longitude or Earth-curvature conversion. Polygons have no holes, islands, obstacle volumes or terrain heights. Survey and flight boundary may coincide, but that can leave insufficient room for the simulated controller to turn at concave corners. Providing a larger permitted flight boundary is explicit configuration, never a silently expanded fence.

The parser accepts clockwise/counterclockwise rings and an optional repeated closing vertex. It canonicalizes winding/start vertex and removes redundant straight vertices. It rejects too few/many vertices, duplicate points, nonfinite values, zero-area/collinear rings, backtracking, self-intersections and self-touching boundaries. Input is bounded to 64 vertices, coordinates within ±1,000,000 local meters, up to 1,000 lane bands and 2,000 waypoints. Numeric predicates use a 1e-8 m boundary tolerance; this is a computational tolerance, **not surveyed accuracy**. Near-degenerate geometry and huge/small coordinate scales are not a substitute for robust GIS preprocessing.

Whole segments are validated by splitting at boundary intersections and checking every interval. Endpoints—or endpoints plus one midpoint—alone are insufficient for concave polygons. The vehicle's three-dimensional axis-aligned geofence is checked independently, with the Stage 6 limits unchanged.

## Coverage-route algorithm and its meaning

Orientation is degrees clockwise from north toward east, matching Stage 6's heading reference: 0° means north/south lanes, 90° east/west. The planner projects polygon vertices onto along-lane and cross-lane axes. It divides the cross-lane extent into equal bands no wider than the requested maximum spacing and clips each band's centerline to the polygon using half-open edge intersections. It alternates traversal direction between bands. Concave bands can contain multiple disjoint intervals.

Connectors follow a straight line when it stays in the flight boundary; otherwise a deterministic visibility-graph shortest path uses boundary vertices. This is a shortest connector, not a globally optimized survey route. No automatic orientation optimizer or realistic turning-radius planner is implemented.

After the lanes, a **required perimeter pass** visits the survey polygon edges. This matters: parallel centerlines alone can leave sloped corners or narrow concave features beyond the assumed footprint. The perimeter provides coverage near those edges. The route then returns horizontally toward home at survey altitude and vertically to the configured home position. Vertical movement is an ideal Stage 6 waypoint leg, not a takeoff/landing procedure.

`swath_width_m` is an explicit idealized **disk-sweep diameter** around the nominal survey centerline; it is not derived from a real camera, altitude, lens, overlap specification or image quality. Lane spacing greater than swath width is rejected. For the ideal planar model, every interior point is either within half a band width of its clipped lane, or lies within that distance of the polygon boundary encountered before that lane; the perimeter pass covers the latter case. Floating-point tolerances still apply.

Validation requires all expected clipped lanes **and** complete polygon-edge legs to be present in imported survey missions. Removing a lane/perimeter leg is rejected as `incomplete_survey_route`, even if the remaining waypoints are inside the fence. This deliberately conservative structural check is not a general validator for arbitrary user-designed coverage patterns; splitting an expected leg into multiple separately imported legs may fail it.

An additional regular-grid-plus-vertices diagnostic independently samples nominal coverage, reporting sample count, coverage fraction and maximum distance to survey legs. Default sample spacing is 0.5 m; tests also use 0.25 m. Transit and return legs do not inflate survey coverage. Sampling is a finite diagnostic, not a proof about real sensor coverage. The 100% values in committed reports describe the **planned ideal footprint**, never flown or imaged coverage. A mission that aborts has not completed its survey regardless of nominal coverage.

## Validation and execution

Validation checks home/start/waypoints and survey extent against the vehicle geofence, route segments against the flight polygon, survey containment, survey-leg altitude, nonzero legs, declared speed/acceleration capabilities and coverage structure. It returns explicit reasons, including `speed_exceeds_vehicle`, `acceleration_exceeds_vehicle`, `point_outside_vehicle_geofence`, `leg_outside_flight_boundary`, `survey_leg_wrong_altitude` and `incomplete_survey_route`.

The adapter requires a stationary start, a vehicle initial position matching the plan and battery above Stage 6's critical threshold. It never silently relocates an explicitly supplied vehicle configuration. The CLI's default config intentionally initializes a **new virtual vehicle** at the declared plan start; this is written into the exported configuration. Mission limits can reduce vehicle speed/acceleration but cannot exceed configured capabilities. All control, dwell, battery and lifecycle transitions remain Stage 6 behavior.

Nominal geometric validity does not guarantee that a PD trajectory will stay inside a concave flight polygon. For optional execution, the adapter previews the **existing engine's next step on a copy**. If the previous-to-predicted position segment leaves the polygon, it discards the preview and invokes Stage 6 `emergency_stop()` at the last accepted state, then advances the existing clock. No physics is duplicated and no outside position is published. This adds a virtual polygon-boundary check alongside Stage 6's box geofence. It remains the same intentionally nonphysical instantaneous virtual stop.

Execution stops when the existing engine completes or aborts, or the requested step budget is exhausted. A timeout reports `step_budget_exhausted`, a null completion time and a still-unfinished mission; it is not marked successful. Other failures retain the engine's stop reason or report `predicted_flight_boundary_crossing`. The exported Stage 6 replay scenario includes the accepted duration and any recorded virtual-stop action; its telemetry matches the supervised run exactly. Replaying that recorded action is not autonomous polygon supervision; use `planning.execute` for new supervised runs.

## Virtual return-to-home

`return_home(current, home, flight_boundary, altitude_m, ...)` plans:

1. A vertical leg to a supplied cruise Up coordinate at least as high as both endpoints.
2. A visibility-graph horizontal route within the flight boundary.
3. A vertical leg to the home Up coordinate.

Current/home outside the boundary, an insufficient return altitude, excessive vehicle capabilities or an out-of-geofence route are rejected. `already_at_home` explicitly reports the no-route case; it does not manufacture an empty running mission. Critical initial battery refuses execution. Battery exhaustion, predicted boundary crossing and time budget exhaustion during execution are reported as failures, not successful arrival.

This is standalone **planning from an explicitly supplied stationary state**, not automatic battery-triggered RTH or in-flight mission replacement. No existing simulator is passed into the planner, mutated, reset or resumed. There is no mechanism here to revive an aborted Stage 6 engine. Live handoff, braking into a return route, energy-reserve prediction, landing and obstacle avoidance remain unsupported. A valid nominal RTH route is not a safety guarantee for a virtual or real aircraft.

## Distance and duration estimates

Distance sums every three-dimensional leg from the declared start, including vertical movement, connectors, survey edges and the return home. Reports separate survey and transit distance. `cruise_only_time_s = total_distance / speed_limit` is diagnostic, not an achievable duration prediction.

For each stop-to-stop leg of length `d`, speed limit `v` and acceleration limit `a`, approximate transit time is `2*sqrt(d/a)` when `d < v²/a`, otherwise `d/v + v/a`. The estimate adds the configured waypoint dwell (default 0.5 s) and an explicit heuristic settling allowance (default 2 s) for each waypoint. It ignores drag, exact PD response, tracking tolerance, battery degradation and physical aircraft effects. It is not a battery feasibility or endurance calculation.

The estimator uses no simulated completion result to tune its allowance. The benchmark reports actual Stage 6 completion time and signed `actual - estimated` error, retaining the observed underestimate. There is no duration comparison value when execution fails.

## Commands and artifacts

Run from the repository root:

```sh
python3 -m unittest discover -v

# Generate, validate and execute a survey; writes a new directory.
python3 -m drone_sim.planning --request scenarios/stage7/rectangle.json \
  --simulate --output-dir /tmp/drone-stage7-survey

# Import the complete exported mission, without regenerating its route.
python3 -m drone_sim.planning --mission /tmp/drone-stage7-survey/mission.json \
  --output-dir /tmp/drone-stage7-import

# Optional plots, using the existing local optional environment.
.venv-vision/bin/python -m drone_sim.planning \
  --request scenarios/stage7/return_home.json --simulate --plots \
  --output-dir /tmp/drone-stage7-return
```

Planner request files declare `operation: survey` or `return_home` and the explicit inputs. Mission exports expand all route and configuration fields and round-trip through `Mission.from_dict()`. Metadata remains descriptive JSON; it does not execute commands. Unknown constructor fields and malformed JSON/geometry fail explicitly.

`--vehicle-config` accepts a Stage 6 vehicle configuration object; its initial position must match the plan. `--max-steps` defaults to 30,000 and must be positive. Existing output directories are refused. `mission.json` and `report.json` contain the plan, validation, nominal coverage and estimates. With simulation, `execution/` contains the reusable Stage 6 configuration, complete CSV/JSONL telemetry and summary. With plotting, `route.png` distinguishes survey area, flight boundary, nominal legs, home, actual trajectory and virtual stop.

Capability rejection or unsuccessful execution exits with code 2 and a report; invalid geometry/input fails before a valid mission can be exported. No invalid route is silently replaced by a direct line home. Core operation needs no third-party packages. Optional route plotting reuses the Stage 6 Matplotlib requirement.

## Reproducible results

The four committed [requests](../scenarios/stage7/) were run with the unchanged default Stage 6 controller, dt=0.02 s, mission speed 3 m/s, mission acceleration 2 m/s² and the default settling settings. Expanded missions, reports and plots are committed in [diagnostics/stage7](diagnostics/stage7/). Full local exports are in ignored `artifacts/stage7/`.

| Request | Nominal route distance | Estimated duration | Simulated completion | Outcome |
| --- | ---: | ---: | ---: | --- |
| Rectangle survey | 180.982 m | 116.217 s | 132.34 s | Completed 14 waypoints; estimate short by 16.123 s. |
| Rotated triangle | 118.064 m | 106.748 s | 125.34 s | Completed 17 waypoints; estimate short by 18.592 s. |
| Return home | 21.422 m | 19.085 s | 22.44 s | Completed all three legs; estimate short by 3.355 s. |
| Tight concave boundary | 103.348 m | 108.766 s | unavailable | Aborted after 48.58 s: predicted flight-boundary crossing after eight completed waypoints. |

Nominal survey coverage diagnostics: rectangle **1,025/1,025**, triangle **353/353**, concave **369/369** sampled points within the configured ideal swath. These are plan-geometry diagnostics, not measured execution coverage. The concave survey remains **incomplete** despite its geometrically complete nominal route. It demonstrates why controller tolerance and turning clearance must be considered before relying on a boundary-following route. The planner does not silently relax that boundary or claim the failure resolved.

The [rectangle route plot](diagnostics/stage7/rectangle-route.png) shows successful execution. The [concave plot](diagnostics/stage7/concave_boundary-route.png) marks the virtual stop and leaves the unexecuted nominal route visible.

## Tests and limitations

`python3 -m unittest discover -q`: 171 tests, 168 passed and three optional-dependency tests skipped. `.venv-vision/bin/python -m unittest discover -q`: all 171 tests passed. The optional environment emits an existing PyAV/OpenCV duplicate Objective-C class warning; it did not fail the suite. [Verification evidence](diagnostics/stage7/verification.json) records source preservation and JSON-normalized replay comparisons. New tests cover polygon normalization/invalid geometry, boundary inclusion, concave excursions with valid endpoints and midpoint, visibility paths, exact rectangular lane spacing/count, multi-interval concave scanlines, sloping/rotated coverage, missing-lane rejection, bounded work, mission JSON round trips, capability/altitude/geofence rejection, distance/time formulas, stationary starts, RTH rejection/completion/battery failure, supervised boundary stopping, timeout reporting, exact Stage 6 replay, CLI import/rejection and plots.

All previous tracker and Stage 1–6 source files are unchanged. No new mission-state machine, PD controller, integrator, clock or vision logic was introduced. Independent verification also reproduces all four execution traces using exported Stage 6 scenarios.

Limits: single vehicle, finite simple polygons without holes, planar ENU, no geodetic/terrain model, no clearance buffer or turn smoothing, fixed altitude for survey legs, route-wide rather than per-leg speed, heuristic duration, no battery-reserve validation, no camera-footprint calibration or real coverage verification, no live RTH handoff, no real aircraft safety claim. Visibility-graph connectors may touch boundaries and the whole survey is not globally distance-optimal. Tight concave geometry can fail dynamic execution and is deliberately stopped. Floating-point predicates and diagnostic grids are intended for bounded local experiments, not certified computational geometry or flight planning.
