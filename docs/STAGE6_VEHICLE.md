# Stage 6: independent virtual UAV dynamics and waypoints

Stage 6 was implemented on `stage6-virtual-uav`, based on `dev` after review and the authorized merge of PR #5 (`a23f80c`). It is a **software-only point-mass simulator**, not an autopilot or a validated quadcopter model. It has no motor, aircraft, network, website, terrain or multi-vehicle integration.

The engine and exports use only Python 3.9+ standard-library modules. Diagnostic plotting is optional and uses Matplotlib already available in the Stage 5 local environment; no additional dependencies were installed for this stage.

## Components and boundaries

| Component | Responsibility |
| --- | --- |
| `drone_sim/clock.py` | Immutable fixed-step clock, integer tick and derived simulation seconds. |
| `drone_sim/vehicle/model.py` | Validated ENU state/configuration, bounded PD request, drag and numerical integration. |
| `drone_sim/vehicle/mission.py` | Waypoint settling, mission lifecycle, battery/geofence monitoring and virtual stop latch. |
| `drone_sim/vehicle/scenario.py` | Versioned reproducible scenarios, tick-scheduled commands, summaries. |
| `drone_sim/vehicle/export.py` | CSV/JSONL/config/summary export and optional diagnostic plots. |
| `drone_sim/vehicle/__main__.py` | Local scenario CLI. |

The vehicle package does not consume camera pixels, tracking IDs or triangulation results. It does not duplicate vision functionality. Its controller observes exact simulated vehicle state; state-estimation error and actuator delay are not modeled. Vehicle state is simulation ground truth, not a perception estimate.

The common clock computes `time_s = tick * dt_s`, without sleeping or accumulating elapsed floating-point seconds. Stage 1 now uses this same `FixedStepClock` primitive. That small refactor is the only change to existing source files; its public contracts and exact generated frames are unchanged. Stage 2–5 source and `existing-tracker/` are unchanged. A future orchestrator can advance one clock and call `vehicle.step(next_clock)` while publishing the resulting truth to separate camera/observation components. The engine rejects clock skips, duplicates and different dt values. The old camera simulator and new engine do not secretly advance one another's clocks.

A future mission-control service can translate commands into the explicit mission methods and consume telemetry. No transport, website, camera coupling or physical command conversion is added here.

## Coordinates, model and control

State includes position in local ENU meters `[east,north,up]`, velocity in m/s, realized acceleration in m/s², heading in radians and battery fraction in `[0,1]`. Heading is north-zero, positive toward east, wrapped into `[-π,π)`. This heading convention is explicit and independent of the camera optical-axis convention.

The model is a unit-mass translational point with ideal gravity compensation. Default initial position is `(0,0,2)` m, at rest. There are no rotors, roll/pitch dynamics, attitude/thrust allocation, aerodynamic lift, mass/inertia estimation, wind, ground contact, collision physics, actuator lag or electrical battery model. These omissions make it useful for mission/state-machine experiments, not aircraft performance or safety claims.

For a stationary target `r` and current position/velocity `p,v`:

```text
u = norm_limit(kp * (r - p) - kd * v, acceleration_limit)
a = norm_limit(u - drag * v, acceleration_limit)
v_next = norm_limit(v + a * dt, speed_limit)
p_next = p + v_next * dt
acceleration_reported = (v_next - v) / dt
```

Integration is **semi-implicit Euler**. Limits apply to the three-dimensional vector norm, not independently to each axis; diagonal motion cannot exceed the overall limit. The controller request and net acceleration are explicitly bounded. Velocity projection respects the speed limit; realized finite-difference acceleration is reported after that projection. The independent integrator rejects an initial velocity above the configured limit. No integral term or derivative of the target is used, so waypoint changes do not introduce a derivative kick.

Default configuration:

| Parameter | Default |
| --- | ---: |
| Fixed step | 0.02 s |
| Maximum speed | 5 m/s |
| Maximum acceleration | 3 m/s² |
| Maximum heading rate | 1.5 rad/s |
| Linear drag coefficient | 0.15 s⁻¹ |
| PD kp / kd | 1.2 s⁻² / 2.2 s⁻¹ |
| Position / speed arrival tolerance | 0.1 m / 0.1 m/s |
| Continuous settling time | 0.5 s |
| Geofence minimum / maximum | (-50,-50,0) / (50,50,30) m |
| Battery low / critical threshold | 20% / 10% |

Heading turns toward horizontal velocity using the shortest wrapped angular difference and the configured rate limit; at negligible horizontal speed it holds. It does not affect translational acceleration. A heading change here is not simulated quadcopter yaw torque.

The supported vehicle timestep is `(0,0.1]` s. Configuration rejects nonfinite/negative parameters and combinations violating `kp*dt² + 2*(kd+drag)*dt < 4`, the local unsaturated semi-implicit PD stability condition. This is a numerical guard, not a global proof for all saturated trajectories or arbitrary initial conditions. Default settling is tested at 0.01, 0.02, 0.05 and 0.1 s. No jerk limit or realistic braking-distance planner is implemented.

## Mission lifecycle

| State | Behavior and permitted commands |
| --- | --- |
| `idle` | Holds the initial position; a nonempty mission can `start()`. `abort()` / `emergency_stop()` are permitted. |
| `running` | Controls toward the current waypoint. Can pause, abort or emergency stop. |
| `paused` | Holds the position captured at pause using the same bounded PD controller; inertia can cause an excursion before settling. Mission progress freezes. Can resume, abort or emergency stop. |
| `completed` | All waypoints reached; holds the final waypoint. Start/resume are rejected. Abort or safety conditions can still stop the station-keeping simulation. |
| `aborted` | Latched virtual freeze. No restart/reset API; construct a new simulator for another run. Repeated stops preserve the first reason. |

Pause is **not a frozen clock or teleport**. Time, dynamics and battery drain continue. Resume returns to the unfinished waypoint. Invalid transitions raise `ValueError` instead of silently ignoring a command. Empty missions may remain idle but cannot start.

A waypoint is reached only after **both** distance and speed are within tolerance for `ceil(settle_s/dt)` consecutive updates (at least one update, even for zero settling time). An out-of-tolerance sample or pause resets the count. Only one waypoint can complete per update. No position is snapped to the waypoint. Completion holds the final target and keeps integrating, so battery drain continues until a stop or scenario end.

Scenario commands are applied at the stated integer tick boundary before the next integration step. Multiple commands at one tick run in their declared order. Initial telemetry is emitted before tick-zero commands; their events appear in the following sample but retain their actual command tick/time. Waypoint and automatic-safety events use the updated state tick. Runs produce the initial sample plus exactly `steps` updates, even after mission completion or abort.

## Simulation-only safety and battery

The geofence is an inclusive, axis-aligned convex ENU box. Initial positions and all mission waypoints must lie inside it. If a proposed numerical step leaves the box, the engine rejects the entire attempted movement, keeps the previous safe position, zeros velocity and acceleration, and aborts with `geofence_predicted_crossing`. It does not clip/teleport the position to the boundary or promise that a real aircraft could stop there. There are no obstacles or path planning.

Battery drain is a configurable heuristic:

```text
fraction_used = dt * (idle_drain_per_s
                   + speed_drain_per_m * |v_next|
                   + acceleration_drain_per_mps * |acceleration_reported|)
```

Defaults are 0.0002, 0.00005 and 0.00002 respectively. Battery is clamped to zero, never recharged, and low-battery crossing emits one warning event. Reaching the critical threshold aborts at the end of that update; the value can cross slightly below the threshold by one timestep. Initially critical battery starts in aborted state and cannot start a mission. These percentages do not model voltage, capacity, discharge curves, temperature, mass, flight endurance or an automatic landing.

`abort()`, `emergency_stop()`, geofence rejection and critical battery all use a **deliberately nonphysical instantaneous virtual freeze**. Telemetry emits `virtual_stop` with its reason and explicit `velocity_reset_enu_mps`. That velocity discontinuity overrides ordinary acceleration limits; the reported post-stop acceleration is zero. Peak ordinary acceleration summaries do not describe the instantaneous reset. After abort, position, velocity, heading and battery are frozen while the shared clock continues. There is no simulated emergency descent, landing or physical safety guarantee. A rejected geofence step does not consume that step's battery.

## Run and reproduce

Five complete, expanded JSON configurations are committed under [scenarios/stage6](../scenarios/stage6/): waypoint tour, pause/resume, emergency stop, low battery and geofence crossing. They include physics, controller, battery, initial state, waypoint tolerances, actions and duration. There is no randomness or wall-clock input to state updates.

```sh
python3 -m unittest discover -v
python3 -m drone_sim.vehicle \
  --config scenarios/stage6/waypoints.json --output-dir /tmp/drone-stage6-waypoints

# Optional plotting, using the already available Stage 5 local environment:
.venv-vision/bin/python -m drone_sim.vehicle \
  --config scenarios/stage6/waypoints.json --output-dir /tmp/drone-stage6-plots --plots
```

Select a new output directory each time; exports refuse to overwrite an existing one. If a separate plotting environment is needed, [requirements-vehicle-plots.txt](../requirements-vehicle-plots.txt) pins the tested Matplotlib 3.11.2 (tested on Python 3.12.14). Plotting imports are lazy; core simulation and CSV/JSONL need no optional dependencies. Plotting failure does not undo already completed numeric exports. Replay guarantees apply to numeric output on the tested runtime, not PNG byte identity or arbitrary hardware/platform floating-point results.

Python API example with an externally advanced shared clock:

```python
from drone_sim.clock import FixedStepClock
from drone_sim.vehicle import VehicleSimulator, Waypoint

clock = FixedStepClock(dt_s=0.02)
vehicle = VehicleSimulator([Waypoint((10, 0, 5))])
vehicle.start()
for _ in range(1000):
    clock = clock.advance()
    telemetry = vehicle.step(clock)
    # Publish telemetry; separate future consumers can use this same clock/time.
```

## Telemetry and diagnostics

Each output directory contains `config.json`, `telemetry.jsonl`, `telemetry.csv`, `summary.json` and, with `--plots`, `diagnostics.png`. Telemetry schema version 1 is a **vehicle** record type, separate from camera observation schemas. Records include ENU/time-domain identifiers, tick/dt/time, all state variables, mission status, active index/completed count, waypoint and last applied control target, error, settling count, battery status, stop reason and timestamped events.

`target_enu_m` / `waypoint_error_m` refer to the current mission waypoint, or the last waypoint after completion; they remain meaningful residuals after pause/abort. `control_target_enu_m` is the target used for the most recent integration and can differ while holding. When a waypoint completes, the reported mission target advances immediately, so its error plot jumps to the next waypoint's distance. The vehicle position does not jump. Idle missions without waypoints report null target/error. The CSV leaves unavailable scalar values blank and stores structured events in a quoted JSON column; JSONL retains the complete nested record.

Plots show all position components, all velocity components and speed, active-waypoint error and battery percentage, against simulation time. Vertical markers indicate waypoint completion or virtual stop. Battery axes are zoomed to the observed range; percentages should not be interpreted as calibrated endurance. [Waypoint diagnostics](diagnostics/stage6/waypoints-diagnostics.png) and plots for every failure scenario are committed alongside their machine-readable summaries. Full local numeric exports are under `artifacts/stage6/` and ignored by Git.

## Results

Local numerical execution: Python 3.9.6 and Python 3.12.14; optional plots use Matplotlib 3.11.2. All scenarios use dt=0.02 s.

| Scenario | Duration | Outcome | Key measured result |
| --- | ---: | --- | --- |
| Three-waypoint tour | 60 s | completed | Waypoints at 8.12, 16.16, 25.06 s; maximum speed 4.724 m/s, maximum ordinary acceleration 3 m/s². |
| Pause/resume | 36 s | completed | Pause at 2 s, resume at 7 s, completed at 15.02 s; hold/resume without stopping clock. |
| Emergency stop | 10 s | aborted | Command at 2 s; position held and velocity reset immediately; subsequent state/battery frozen. |
| Low battery | 20 s | aborted | Artificially high drain fixture: warning at 0.34 s, critical stop at 3.66 s with 9.941% remaining. |
| Geofence | 2 s | aborted | Outward initial velocity near boundary; first crossing rejected at 0.02 s, position remains (49.99,0,2) m. |

The tour ends within `4.831e-13` m of its final target after the extra holding period, with 98.579% battery remaining. This near-zero ideal numerical error uses exact simulated state and must not be presented as real localization or flight accuracy. Completion itself uses the declared tolerances and dwell, not the final tiny error. The fixture values are reproducible software results, not real UAV measurements.

## Verification and limitations

- Complete suite: **138 tests discovered; 136 pass and 2 optional tests skip** in the standard-library Python 3.9.6 environment. **All 138 pass** in the Python 3.12.14 environment with video/plot dependencies available. All 108 prior tests remain present.
- Thirty new tests cover analytic semi-implicit updates, analytic discrete drag, diagonal speed/acceleration limits, PD derivative direction, heading wrapping/rate, battery accounting, invalid configurations, convergence across supported timesteps, stationary settling, speed-sensitive arrival, ordered waypoint completion/no teleport, pause/resume and dwell reset, lifecycle rejection, latched stop/discontinuity, geofence/battery safety, idle behavior, shared-clock synchronization/rejection, scenario replay, CSV/JSONL equivalence and plotting.
- Independent preservation checks compare all legacy and prior source files to merged PR #5: **61 files unchanged**. The sole existing source change is Stage 1's clock primitive; eight old/new Stage 1 configurations produce exactly equal frames. [Preservation evidence](diagnostics/stage6/preservation.json).
- Repeated CLI runs produce identical configuration, CSV, JSONL and summary bytes. All five committed scenarios reproduce exactly and stay inside the geofence. Numeric tests run in the existing GitHub Actions Python 3.9/3.12 matrix; optional codec/plot tests also ran locally.

Remaining limitations: simplified gravity-compensated translation, exact-state feedback, no sensor errors/delays, no six-degree-of-freedom attitude/rotors, simplistic drag/energy, instantaneous virtual stops, axis-aligned geofence only, no obstacle/path planning, no real geodesy or flight limits, no multi-UAV interaction, no perception coupling or mission-control transport. Scenario export currently collects bounded runs in memory. Configurable gains passing the local numerical guard can still have poor saturation/settling behavior and need scenario testing. The existing Stage 5 macOS duplicate video-library warnings remain confined to its optional codec tests; vehicle simulation does not load those libraries.
