# Stage 13 — Read-only PX4/Gazebo telemetry path

## Release status: real SITL milestone blocked

PR #12's two preflight review findings were fixed (Ubuntu release and Gazebo-major validation), its updated CI passed (run 36818795991), and it was merged into `dev` at `871efc7`. `stage13-read-only-sitl` starts from that merge.

**No PX4/Gazebo simulation was executed for this stage.** The current host is Darwin arm64 with 8 GB RAM, no compatible Ubuntu VM/container runtime, no Gazebo or PX4 checkout, and approximately 1.9 GiB free disk at the initial check. Provisioning the pinned amd64 environment is not feasible within that available disk/memory budget. No system image, PX4 tree, Gazebo package, model or aircraft connection was installed. [Environment diagnostics](diagnostics/stage13/environment.json) and [capacity/blocker evidence](diagnostics/stage13/runtime-blocker.json) record actual observations. There are no real SITL captures, advancing-clock observations, bridge-status logs, flight results, or simulator resource measurements to report.

The user-authorized fallback is implemented: optional receive-only transport code, conversion tests, clearly labeled synthetic protocol fixtures, authenticated local backend reads, and a separate dashboard display. **Successful code/fixture execution is not successful PX4 integration.**

## Implemented architecture

```text
Gazebo Transport 13: Clock + Pose_V subscriptions ─┐
                                                 ├─ optional local collector
PX4 MAVLink: passive IPv4 loopback UDP receive ───┘     ├─ bounded raw capture (optional)
                                                       └─ atomic local JSON snapshot
                                                              ↓
                                           existing authenticated FastAPI GET /v1/sitl
                                                              ↓
                                           existing session-protected bridge GET /api/sitl
                                                              ↓
                                           separate read-only SKYVIEW telemetry disclosure
```

The existing numerical fleet is still the default and its physics/planner/clock are untouched. The collector runs separately so Ubuntu's Gazebo Python ABI does not contaminate the backend environment. It does not create a second numerical simulation clock. No existing WebSocket schema is modified. Numerical WebSocket commands, recordings and playback remain strictly on the numerical path; none route to SITL.

`simulator_adapter/sitl.py` contains pure decoded-message assembly. `RealSitlAdapter.snapshot(now)` returns the existing Stage 12 `simulator_snapshot` contract for Gazebo truth. `telemetry(now)` adds a version-1 `sitl_telemetry` envelope with separate `simulation_truth` and nullable `autopilot_estimate` snapshots, estimator validity, diagnostic counters, evidence label and collector timestamp. Mission progress is always null and per-vehicle mission state is unknown.

`simulator_adapter/sitl_runtime.py` supplies optional transport subscriptions and passive UDP parsing. The Gazebo implementation follows its [Transport 13 Python subscription API](https://gazebosim.org/api/transport/13/python.html), using `gz.transport13.Node`, `gz.msgs10.Clock` and `Pose_V`. The runtime subscribes only to `/world/<world>/clock` and `/world/<world>/dynamic_pose/info`. The model name is selected explicitly, not by discovery order. No advertise/service-request API is used.

The MAVLink side uses `pymavlink==2.4.49`'s common dialect parser with **no writable MAVLink file**, rather than `mavlink_connection`. Its socket only binds and receives on `127.0.0.1`; there is no send, heartbeat transmission, stream-rate request, parameter write, command, serial support or remote endpoint. PX4 documents the [SITL ground-station UDP destination 14550](https://docs.px4.io/v1.16/en/simulation/). Do not run another listener on that port. Gazebo itself supplies PX4 physics directly; this passive MAVLink listener is not a physics bridge.

The first accepted configured source latches its local UDP endpoint. Other IPs/endpoints and system/component IDs are rejected. Heartbeat must identify a PX4 quadrotor. An armed heartbeat latches an unavailable state and requires collector restart; the collector cannot disarm or otherwise affect it. MAVLink is not authenticated by this local filter: an isolated SITL environment without hardware or forwarding is mandatory. A user flag is an explicit scope assertion, not technical proof that a source is simulated.

## Data mapping and validity

| Input | Mapping | Missing/invalid behavior |
| --- | --- | --- |
| Gazebo Clock `sim.sec/nsec` | Authoritative simulation seconds; no wall-time integration | Missing clock → unavailable. Backward time latches restart-required. |
| Pose_V header timestamp + exactly one configured model | World position transformed by configured yaw rotation and ENU translation; normalized quaternion → ENU yaw → north-zero heading | Missing/duplicate name, future/duplicate pose or invalid quaternion rejects. Changed entity ID latches restart-required. |
| PX4 HEARTBEAT | Configured system/component, PX4 autopilot and quadrotor, unarmed only | Missing/old heartbeat makes estimates unavailable/invalid; armed source latches a blocker. |
| LOCAL_POSITION_NED | Position `[n,e,d]` → `[e,n,-d]` plus explicit PX4 origin; velocity similarly rotated | Never labeled truth. Position/velocity stay null without fresh compatible estimator status. |
| ATTITUDE yaw | North-zero clockwise radians, wrapped; no ENU-yaw conversion applied twice | Heading null without compatible timestamp and valid attitude flag. Roll/pitch not exposed by Stage 12 contract. |
| ESTIMATOR_STATUS | Flags gate horizontal/vertical position, horizontal/vertical velocity and attitude independently | Unknown, invalid or stale status nulls affected measurements. This is not a full covariance-quality certification. |
| Battery, acceleration, mission items | Not derived from these streams | Null / unknown, never zero-filled or inferred from arming or stationary position. |

The [MAVLink common message definitions](https://mavlink.io/en/messages/common) define units and estimator flags. The [pinned PX4 ESTIMATOR_STATUS implementation](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/mavlink/streams/ESTIMATOR_STATUS.hpp) is the upstream source reference for the status stream. No claim is made that every selected message is present in the default x500 stream configuration; absent status deliberately prevents valid estimate display.

Gazebo world axes and PX4 local origin are **not automatically equal or geographically calibrated**. `config/sitl/telemetry.example.json` explicitly supplies world-to-ENU yaw, world origin, PX4 origin, model and IDs. Its zero offsets are starting configuration values requiring real validation, not measured calibration. Only yaw-aligned world frames are supported; arbitrary tilted world frames and full attitude conversion are not. No latitude/longitude or depth is fabricated.

PX4 boot milliseconds become seconds; estimator microseconds become seconds. The explicit `px4_boot_to_simulation_s` offset maps those clocks to Gazebo time. Unix-epoch or otherwise incompatible timestamps fail the clock window rather than being guessed. Position/status/attitude must be within 0.25 simulation seconds; status must be received within 0.5 wall seconds, heartbeat within 2 seconds. Future PX4 messages and messages more than 0.5 simulation seconds old reject. The current callback implementation drops poses arriving ahead of the latest clock rather than advancing time from pose data. Receipt and simulation-age freshness remain distinct, using Stage 12 thresholds.

Small out-of-order/duplicate PX4 timestamps reject; backward timestamps within the clock window latch restart-required. Larger boot resets outside the window reject as unmapped/old; retained state expires. Reconnect after a simple timeout can resume from increasing timestamps and the same peer/model. Clock/model replacement requires restarting the collector, which creates a new epoch. Local-position origin resets without a timestamp rollback are **not detectable from these selected messages**; stop and requalify offsets after EKF resets. The code does not claim complete estimator reset tracking.

## Backend and UI contract

Set `SKYVIEW_SITL_SNAPSHOT` in the **backend process environment** to an operator-chosen local snapshot path. The browser cannot choose paths or start collectors. With it unset, `GET /v1/sitl` returns `configured:false`, `status:disabled`, `telemetry:null`; the numerical default is unchanged. The additive endpoint accepts existing view/control Bearer authentication and all existing loopback/Host/Origin restrictions. POST is unsupported (405). The bridge permits only GET `/api/sitl`, using its server-held view token and existing browser session.

The reader caps files at 128 KiB, validates schema, finite fields, frame/provenance, identity and absence of invented progress, and recomputes ages on the same host monotonic clock. Files are written atomically with private temporary-file permissions and a single-writer lock. A missing file yields waiting, malformed data yields error, and an unchanged collector generation timestamp older than 2 seconds yields stale/unavailable. Estimator values are cleared when the handoff is at least 0.5 seconds old. A stale file cannot stay live because the browser polls it repeatedly. The file is a trusted operator-configured local handoff, not an authenticated external ingestion API; local filesystem attackers are outside the existing prototype threat model.

The dashboard disclosure **PX4 / Gazebo telemetry — Read only** polls once per second only while open. It shows separate Gazebo truth and PX4 estimate columns, source identity, source simulation time, ENU position/velocity, heading, nullable battery and freshness. It offers no flight or playback buttons. Errors clear the prior display. Every payload is labeled either **Synthetic fixture — not real SITL evidence** or **Live transport — real SITL qualification not established**. Receipt alone never changes that qualification label. Numerical map, missions and controls remain separate below it. This initial read-only path uses bounded REST snapshots, not a new lossless telemetry stream or SITL recording/replay system.

## Optional installation and real qualification procedure (not executed here)

First follow the pinned Ubuntu 22.04 amd64 / PX4 v1.16.0 commit / Gazebo Harmonic setup in [Stage 12](STAGE12_SIMULATOR_ADAPTER.md). Keep one unarmed x500 in the default plane world. No terrain, camera payload or real device passthrough. A separate adequately resourced local Ubuntu machine is preferable to provisioning on this nearly-full 8 GB host.

In that Ubuntu environment, use its distro Python for the Gazebo bindings, separate from the backend Python:

```sh
sudo apt install python3-gz-transport13 python3-venv
python3 -m venv --system-site-packages .venv-sitl
.venv-sitl/bin/python -m pip install -r requirements-sitl.txt
.venv-sitl/bin/python -c 'from gz.transport13 import Node; from gz.msgs10.clock_pb2 import Clock; from gz.msgs10.pose_v_pb2 import Pose_V'
python3 -m scripts.sitl_diagnostics --px4 /path/to/pinned/skyview-px4 --report /tmp/sitl-preflight.json
```

`--system-site-packages` is needed to expose Ubuntu's matching Gazebo bindings; a venv created from a different Python ABI may fail imports. The optional parser was tested separately on macOS; the combined Ubuntu bindings/runtime remain unexecuted. [Parser package versions](diagnostics/stage13/parser-packages.txt) are local test evidence, not a cross-platform lock.

Launch the pinned PX4 headless process from its checkout with discovery restricted to the same local environment and partition:

```sh
GZ_IP=127.0.0.1 GZ_PARTITION=skyview-sitl HEADLESS=1 PX4_GZ_WORLD=default MAKEFLAGS=-j2 make px4_sitl gz_x500
```

Perform the Stage 12 smoke observations before trusting any display: inspect advancing `/world/default/clock`, `gz_bridge status`, `listener vehicle_status -n 1`, `listener vehicle_local_position -n 1`, and exactly one `x500_0` model/entity. Prefix `gz topic` inspection commands with the same `GZ_IP`/`GZ_PARTITION`. Record actual world axes, pose origin, PX4 local origin, boot/simulation offset and estimator flags. Inspect emitted telemetry without sending stream requests. If a required stream is absent, record that limitation; do not mark null estimates valid.

Then, from SKYVIEW's root in the same isolated environment:

```sh
mkdir -p artifacts/stage13
.venv-sitl/bin/python -m simulator_adapter.sitl_runtime \
  --config config/sitl/telemetry.example.json \
  --output artifacts/stage13/live.json \
  --capture artifacts/stage13/raw-packets.jsonl \
  --duration 60 --confirmed-isolated-sitl
```

The explicit flag is required and never inferred. Capture files use exclusive creation and contain base64 original Gazebo protobuf / MAVLink UDP bytes with local monotonic receipt times and message type. Capture is bounded at 10 MiB (further packets are not recorded). This is evidence collection, not guaranteed lossless logging. Duration is bounded 1–3600 seconds. Shutdown unsubscribes, closes the owned socket, and writes an unavailable snapshot; a crash instead expires the file lease. Do not connect USB/serial devices, route network MAVLink into this process, or enable broadcast.

For viewing during collection, start the existing local backend/bridge on that **same host**, retaining private credentials and their established loopback bindings:

```sh
export SKYVIEW_SITL_SNAPSHOT="$PWD/artifacts/stage13/live.json"
python3 scripts/skyview.py --no-build
```

The existing launcher inherits this environment variable; it is not a browser variable or a replacement for `.skyview.env`. Build the updated dashboard first. If running inside a VM, use its local browser; this stage does not authorize exposing the backend or forwarding control ports.

For actual evidence, retain pinned source/submodule/package versions, world/model IDs, console and bridge logs, raw capture, snapshot, wall duration, and process RSS/CPU observations during the same run (for example Linux `ps`/`pidstat` in the isolated environment). Record whether Gazebo time advanced while unarmed and how freshness behaves when only the collector or simulator is stopped. Do not report throughput, clock accuracy, estimator error or resource requirements from the synthetic tests. After qualification, add sanitized **captured** fixtures with capture metadata alongside—not replacing—the current synthetic fixtures.

## Validation and remaining work

[Validation results](diagnostics/stage13/validation.json) record actual executed checks. New tests cover decoded ENU/NED and quaternion heading mapping, estimator validity/expiry, missing data, duplicate/future/identity rejection, armed-source rejection, clock/model replacement, stale handoffs, authenticated read-only API, parser checksum rejection, generated MAVLink v1/v2 bytes and actual loopback UDP receive using those generated bytes. Browser automation exercises a synthetic file through the real Python endpoint and Node bridge, renders provenance/unknown fields, ages it, replaces its epoch and reports malformed data. Screenshots are fixture UI evidence only.

The production Gazebo subscriptions are checked with test doubles; **actual Gazebo C++/Python ABI, protobuf callback shapes, topic rates, model publication, PX4 emission rates and combined runtime cleanup remain unverified**. Generated wire bytes are useful parser regressions but are not captured PX4 traffic. `tests/fixtures/stage13/` marks `captured_from_sitl:false`. No real flight/physics, calibrated-estimator accuracy, actual SITL lifecycle, multi-aircraft operation, SITL replay or hardware safety claim is made.

Run supported tests:

```sh
python3 -m unittest discover -v
.venv-backend/bin/python -m unittest discover -v
.venv-vision/bin/python -m unittest discover -v
# Optional decoder environment; no Gazebo required for these fixtures:
.venv-sitl/bin/python -m unittest tests.test_real_sitl -v
(cd dashboard && npm run format:check && npm run build && npm run typecheck && npm test && npm run test:e2e)
.venv-backend/bin/python -m mission_control.smoke --report /tmp/stage13-smoke.json
.venv-backend/bin/python -m mission_control.release_check --report /tmp/stage13-replay.json
.venv-backend/bin/python -m scripts.check_startup --report /tmp/stage13-startup.json
```

A separate CI parser job installs only `requirements-sitl.txt`; Gazebo/PX4 are still optional. The next milestone is an actual isolated Ubuntu single-x500 smoke and capture review, followed by qualification of frame/time/validity assumptions. No arming, takeoff, autopilot commands, physical aircraft, motors, person following or automatic avoidance is implemented.

Final local results (2026-10-01): 259 Python tests discovered per environment; core 241 passed/18 optional skips, backend 253/6, vision 245/14, isolated parser environment 243/16. The union executes all 259 tests, including 13 Stage 13 regressions. Dashboard build/typecheck/format and all 15 unit tests passed. All 7 browser tests passed (reported 1.1 min); the final handoff-consistency change was rechecked with its browser scenario (6.3 s including startup) and all Python suites. Numerical real-WebSocket smoke, three full recording/replay scenarios and owned-process launcher checks passed. Existing Starlette TestClient and macOS optional video-library warnings remain.

[Desktop fixture display](diagnostics/stage13/ui-fixture-1440.png) and [narrow fixture display](diagnostics/stage13/ui-fixture-390.png) were inspected. They intentionally show synthetic/stale/unavailable labels; they are not screenshots of an executed PX4 environment.
