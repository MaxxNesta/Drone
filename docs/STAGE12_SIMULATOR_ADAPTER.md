# Stage 12 — Simulator adapter and optional PX4/Gazebo environment

## Status and audited baseline

PR #11 passed its GitHub Actions workflow (run 36721857884), was reviewed for coherent backend reads and authenticated geometry recovery, and was merged into `dev` at `5118b6b`. Stage 12 is developed on `stage12-simulator-adapter` from that merge. This is an architecture and executable contract stage, **not an executed PX4 integration**.

The existing numerical simulator remains the working default. No existing tracker, perception, vehicle, planning, fleet, backend or frontend code is changed. The new `simulator_adapter/` package is opt-in and is not imported by the server. It has no network, serial, MAVLink, motor, arming or person-following interfaces. No PX4/Gazebo dependency is required by core tests.

The audit covered the Stage 6–11 documentation, `FixedStepClock`, vehicle telemetry and lifecycle, fleet commands/stepping, backend publication/authentication/recordings, and the dashboard bridge. Relevant boundaries:

| Existing component | Finding and integration consequence |
| --- | --- |
| Stage 6 vehicle | Gravity-compensated point mass, bounded PD, approximate battery. Heading is north-zero clockwise, not conventional ENU yaw. Abort freezes state instantly: never translate it to autopilot termination. |
| Stage 7 planning | Local ENU survey geometry and mission validation. No geographic origin or terrain reference. PX4 mission upload is not interchangeable with this JSON. |
| Stage 8 fleet | Sole fixed-step clock; sorted stable IDs; per-vehicle state and advisory proximity. Numerical adapter delegates directly to this engine. |
| Stage 9/11 service | One serialized owner, tick-boundary command outcomes, bounded WebSocket queues and epoch/sequence resync. View/control Bearer auth and loopback restrictions remain untouched. |
| Recordings | Exact same-runtime numerical replay. A Gazebo/PX4 recording would need build/world/parameter/sensor/seed fingerprints and cannot promise bitwise replay. |
| Stage 10 dashboard | Consumes schema-1 exact truth and local ENU geometry. It cannot yet consume adapter records or autopilot estimates. Do not relabel estimates as truth to make it compatible. |

## Implemented boundary

`SimulatorAdapter` is a Python structural protocol: `capabilities` and a read-only `snapshot(now_monotonic_s)` method. Snapshot is a detached JSON-compatible **adapter schema 1**, type `simulator_snapshot`; this is a different namespace from Stage 9 telemetry schema 1. Reads never step physics. Unsupported schema evolution will require a new explicit converter rather than changing the existing WebSocket envelope silently.

`NumericalAdapter(config, epoch, now_monotonic_s)` wraps the original `FleetSimulator`. Its optional `step(now_monotonic_s)` applies configured actions in their original order then calls the original engine step; it returns the unmodified fleet row. `command` delegates existing simulation-only commands. A caller must serialize access and must not also apply configured actions itself. The engine owns the only tick; the adapter owns no second clock. Run budget exhaustion and invalid receipt time reject before stepping. Engine errors propagate; no independent safety/controller behavior is introduced.

`MockAdapter({vehicle_id: source_id}, epoch)` accepts complete fleet samples through `ingest(epoch, sequence, simulation_time_s, samples)`. It implements no dynamics or commands. It rejects wrong epochs, duplicate/reordered sequence numbers, decreasing source/receipt/simulation time, duplicate IDs, missing members and source substitution before changing state. A restart requires a new instance and new epoch; packets cannot reset time automatically. Disconnect retains the last samples for inspection but marks availability unavailable. Successful new ingestion restores availability. Sequence gaps are allowed; the producer must supply a complete snapshot, not a delta.

Each snapshot includes backend, capabilities, epoch, sequence, coordinate/time domain, simulation seconds, availability/reason and vehicle samples. Capabilities distinguish clock authority, explicit stepping, exact replay and supported commands. Numerical exact replay means the existing same-runtime/config/command guarantee, excluding injected wall receipt timestamps. Mock advertises no exact-physics replay or commands.

Each `VehicleSample` preserves:

- Stable vehicle ID and explicit source ID; future map could bind `uav-1` to a configured Gazebo model/PX4 instance. Discovery order or a mutable MAVLink system ID alone is insufficient identity.
- Source simulation seconds and local monotonic receipt seconds, never mixed with Unix time.
- Explicit `simulation_truth` or `autopilot_estimate` provenance. Future Gazebo world pose is truth; PX4 EKF output is an estimate. Vision tracking remains a separate perception contract.
- Optional ENU position, velocity, acceleration, north-zero heading and battery fraction. Unknown fields are null, never zero-filled. Acceleration is world kinematic acceleration, not raw IMU specific force.
- Mission state `unknown/idle/running/paused/completed/aborted` and optional paired completed/total waypoint counts. No arming state, navigation mode or distance threshold implicitly proves mission completion.

Freshness is distinct from availability and mission status: `fresh` before 0.5 s, `stale` at 0.5 s, `lost` at 2 s, based on the greater of receipt age (monotonic seconds) and source age (simulation seconds). Both ages are reported separately. `snapshot_record` accepts configurable thresholds; adapters use these defaults. A newly received old sample cannot become fresh merely by changing receipt time. During simulator pause, simulation age stops but an unrefreshed receipt can become stale; the adapter does not invent heartbeats. A fresh retained sample may coexist with an unavailable simulator. These are display/diagnostic conditions, not control policies.

Samples are frozen and validate finite numbers, vector dimensions, IDs, battery bounds and progress counts. The mock is an in-process test tool, not an untrusted wire decoder. The initial whole-fleet contract supports one selected state source per vehicle per adapter instance; simultaneous Gazebo truth and EKF estimates require separate explicitly labeled instances and a future service converter, not duplicate vehicle IDs in one snapshot. A future transport must additionally enforce message sizes, schema validation, authentication, source authorization, bounded queues and per-field validity/covariance.

## Coordinates, units and time

All adapter translations assume **the same explicitly established origin and axis alignment**:

```
ENU [e,n,u] -> NED [n,e,-u]
NED [n,e,d] -> ENU [e,n,-d]
R = [[0,1,0], [1,0,0], [0,0,-1]]
```

The implemented helpers validate finite 3-vectors. The same rotation applies to world velocity and kinematic acceleration; a position with a different origin needs translation too. Covariance conversion would be `R C Rᵀ` and is not implemented. PX4 body FRD to FLU is `[x,-y,-z]`; body IMU data must first be rotated into the world frame and gravity handled explicitly. Full quaternion/attitude conversion is deferred, not approximated by swapping Euler angles. PX4 documents its [world/body frame conventions](https://docs.px4.io/v1.16/en/ros2/user_guide#ros-2-px4-frame-conventions).

For conventional ENU yaw (east zero, positive toward north), SKYVIEW heading is `wrap(pi/2-yaw)`. PX4 NED heading already uses north-zero/east-positive convention when the message actually specifies NED yaw. Helpers and cardinal-axis regression tests check signs. Meters, seconds, m/s, m/s² and radians are canonical; battery is a fraction in [0,1]. GPS latitude/longitude and altitude datum require an explicit origin/ellipsoid/geoid policy before any geographic mission adapter. Gazebo world pose is not automatically surveyed ENU.

Numerical authority remains `tick * dt_s`. Future Gazebo authority is its simulation clock: do not derive it from browser timestamps, packet count, receipt time, or a SKYVIEW timer. PX4's Gazebo bridge updates PX4 time from simulation; synchronization must be verified for the selected transport/version. This differs from Gazebo Classic lockstep; see [PX4 time synchronization](https://docs.px4.io/v1.16/en/ros2/user_guide#ros-gazebo-and-px4-time-synchronization). PX4 microsecond timestamps require explicit domain/epoch mapping and conversion by 1e-6, not comparison with host monotonic seconds. Reboot/world reset/origin reset must create a new epoch and clear associations. Physics step, telemetry publication rate and wall playback speed remain separate. Future Gazebo stepping/pause support requires an integration test before advertising the capability.

## Selected optional environment

The checked-in [environment specification](../config/sitl/environment.json) selects Ubuntu 22.04 LTS amd64, PX4 **v1.16.0**, commit `6ea3539157ca358c70a515878b77077af7d4611d`, and Gazebo **Harmonic (gz-sim 8)**. This is a deliberately pinned baseline, not a claim to be the newest PX4 release. The tag's peeled commit was verified using upstream `git ls-remote`. Submodules are pinned by that commit. PX4's [versioned Gazebo documentation](https://docs.px4.io/v1.16/en/sim_gazebo_gz/) documents Harmonic, `gz_x500` and the lightweight default grey-plane world. [Gazebo binary installation](https://gazebosim.org/docs/harmonic/install_ubuntu/) documents supported Ubuntu packages.

One x500, no camera payload, no terrain, headless mode. As a provisional engineering budget, start with 4 CPU cores, 8 GB guest RAM and 30 GB free disk; these are **unmeasured planning allowances, not verified minimum requirements**. Limit build parallelism to two jobs and run the dashboard separately if memory is tight. Headless removes the GUI but does not guarantee a particular real-time factor. Do not add multiple realistic aircraft to compensate for low throughput.

Observed host: Darwin arm64, 8 CPU cores, 8 GB physical RAM. `gz`, `cmake`, `ninja`, Docker, Colima and Podman are absent from PATH; no PX4 checkout was supplied. See [actual environment diagnostic](diagnostics/stage12/environment.json). A full amd64 VM on this 8 GB machine is not the recommended first validation target. Use a separate local Linux workstation or separately qualified native ARM environment. Native macOS support evolves upstream; it has not been qualified against this pinned baseline. No heavyweight runtime was installed and **no SITL smoke test was executed**.

## Reproducible installation and smoke procedure

Run the following only inside the dedicated Ubuntu development environment, outside this repository. The setup script installs system packages and can request sudo. The selected script's [documented `--no-nuttx` option](https://docs.px4.io/v1.16/en/dev_setup/dev_env_linux_ubuntu) omits physical-flight-controller build tools. No ROS, MAVSDK, QGroundControl or hardware dependencies are needed for the first environment check.

```sh
git clone --no-checkout https://github.com/PX4/PX4-Autopilot.git skyview-px4
cd skyview-px4
git checkout --detach 6ea3539157ca358c70a515878b77077af7d4611d
git submodule update --init --recursive
bash Tools/setup/ubuntu.sh --no-nuttx
# Restart this environment if instructed by upstream setup.
mkdir -p ../sitl-evidence
git rev-parse HEAD > ../sitl-evidence/px4-commit.txt
git submodule status --recursive > ../sitl-evidence/submodules.txt
dpkg-query -W > ../sitl-evidence/packages.txt
python3 -m pip freeze > ../sitl-evidence/python-packages.txt
gz sim --versions > ../sitl-evidence/gazebo-version.txt
MAKEFLAGS=-j2 make px4_sitl
```

Source/submodules are pinned, but apt and upstream Python requirement resolution are **not hermetically locked**. Preserve the package manifest and VM image after a successful qualification. This recipe is reproducible at the source/configuration level; byte-identical build or physics outputs are not claimed. Do not substitute moving `main` branches or silently upgrade Gazebo major versions.

For runtime isolation, use a dedicated VM with no shared host devices, no USB/serial passthrough and no inbound port forwarding. After dependency/model retrieval, disconnect its external network for the smoke run. Keep console access through the VM application. PX4/Gazebo discovery/UDP services are not authenticated SKYVIEW services; do not expose them on a LAN or bind an adapter to them from the public network. No server setting or dashboard credential is changed by this procedure.

```sh
# In the pinned PX4 checkout; default world, exactly one unarmed x500.
HEADLESS=1 PX4_GZ_WORLD=default MAKEFLAGS=-j2 make px4_sitl gz_x500
```

In another terminal in that isolated environment:

```sh
gz topic -l
gz topic -e -t /world/default/clock
```

Use Ctrl-C after collecting several increasing clock samples. At the PX4 console, inspect `gz_bridge status`, `listener vehicle_status -n 1` and `listener vehicle_local_position -n 1`; capture output and logs. Do not issue arm, takeoff, offboard or hardware commands. Record world/model identity (exactly one x500), advancing simulation time, bridge connectivity, estimator validity flags and any errors. An unarmed vehicle at rest is sufficient for this **environment-only** smoke check. Startup logs alone are insufficient; advancing clock and live PX4 status must both be observed. Exit the owned PX4/Gazebo processes with their console controls; do not kill unrelated processes by port or name.

From SKYVIEW, a read-only preflight can be rerun without installing anything:

```sh
python3 -m scripts.sitl_diagnostics --px4 /path/to/skyview-px4 --report /tmp/sitl-preflight.json
python3 -m unittest tests.test_simulator_adapter -v
```

Preflight reports paths/versions and baseline blockers; it never claims success or starts a simulator. It does not certify Ubuntu patch level, plugin availability, resource sufficiency or transport isolation.

## Future backend integration gate

A later separately reviewed implementation should add a local SITL process owner and a read-only Gazebo/PX4 transport first. It must verify source identity, origin, timestamp resets, validity flags, rate/age and disconnect behavior against real packets. Keep Gazebo truth, autopilot estimate and perception estimate separate. Unknown mission progress remains null; an accepted autopilot command is not completed execution. A capability-negotiated command layer must define each supported operation and its applied outcome before enabling it. Numerical pause/abort/emergency-stop semantics must not be copied into PX4.

The existing FastAPI service remains on its numerical path. Future opt-in backend selection requires a new adapter-to-service converter and explicitly versioned telemetry extension with regressions for reconnect, auth, snapshots and recordings. No new endpoint or credentials are introduced here. The browser must never select arbitrary transport endpoints or write simulator state. Simulation-only isolation must be verified before any future command support.

## Diagnostics and limitations

| Symptom | Investigation |
| --- | --- |
| `gz` missing / unknown `gz_x500` target | Check pinned source, Ubuntu setup completion and Harmonic installation; use a clean dedicated build after changing simulator dependencies. |
| Process starts, clock does not advance | Inspect world pause state, Gazebo logs, bridge status and resource exhaustion; do not report smoke success. |
| Position rotated/mirrored | Check ENU/NED signs, world alignment, heading convention and local-origin reset; do not adjust frontend coordinates to hide it. |
| Fresh packet, old data | Compare both receipt and simulation ages and validity flags; transport receipt is not measurement freshness. |
| Memory pressure / low real-time factor | Use headless single x500, fewer build jobs, close unrelated workloads; measure actual factor before claiming performance. |
| Port/discovery trouble | Check isolated environment and owned processes. Do not open external firewall ports as a workaround. |

No real Gazebo packet decoding, quaternion conversion, PX4 mission upload, autopilot command mapping, aircraft safety behavior, geographic projection or cross-runtime deterministic physics replay is implemented. Mock tests validate contracts, not aerodynamics or PX4 behavior.

## Validation results

Results are recorded in [validation.json](diagnostics/stage12/validation.json). Full Python suites, dashboard checks and actual loopback transport/browser checks are run against the unchanged default simulator. The new contract suite checks frame/heading conversion, invalid data, identity, packet ordering, freshness, reconnect, detached reads, delegated commands and exact agreement with the original full crossing run. Real SITL remains explicitly unexecuted for the environment reasons above.

Reproduce the supported regression checks (loopback networking is required for transport/browser tests):

```sh
python3 -m unittest discover -v
.venv-backend/bin/python -m unittest discover -v
.venv-vision/bin/python -m unittest discover -v
.venv-backend/bin/python -m mission_control.smoke --report /tmp/stage12-smoke.json
.venv-backend/bin/python -m mission_control.release_check --report /tmp/stage12-release.json
(cd dashboard && npm run format:check && npm run build && npm run typecheck && npm test)
.venv-backend/bin/python -m scripts.check_startup --report /tmp/stage12-startup.json
(cd dashboard && npm run test:e2e)
```

[Mock snapshot examples](diagnostics/stage12/mock-snapshots.json) are labeled synthetic fixtures and are not PX4 telemetry. Existing CI discovers the new tests automatically; no optional SITL software is installed in CI.

Final local results (2026-10-01): 242 Python tests discovered per environment; core 227 passed/15 optional skips, backend 238 passed/4 skips, vision 231 passed/11 skips. The environment union exercises all 242 tests, including 11 new adapter regressions. Dashboard 13 unit tests and all 6 browser tests passed (54.0 s); production build, typecheck, formatting, launcher, 80-tick real WebSocket smoke and all 3 full transport/replay scenarios passed. Existing Starlette TestClient and macOS video-library warnings remain; no dependencies were upgraded to hide them. These results validate the unchanged numerical default and new contract, not PX4/Gazebo integration.
