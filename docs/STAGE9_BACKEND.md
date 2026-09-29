# Stage 9: local mission-control backend

Stage 9 was developed on `stage9-mission-control-backend` after reviewing PR #8, checking successful CI, and merging it into `dev` (`70c2d08`). It provides a Python FastAPI/WebSocket service for the **software-only** civilian survey fleet. There is no website, physical aircraft connection, MAVLink, motor control, terrain or automatic collision avoidance.

## Boundaries and ownership

The separate `mission_control/` package imports the existing Stage 8 coordinator and its complete Stage 7 mission/Stage 6 vehicle configuration records. It does not implement physics, planning, waypoint states, battery behavior, fleet controls or another simulation clock. All **82 previously tracked source/reference files** in `existing-tracker/` and `drone_sim/` remain unchanged. Existing APIs and all previous tests are preserved; the backend is an optional dependency group.

| Component | Responsibility |
| --- | --- |
| `app.py` | FastAPI REST/WebSocket boundary, loopback/host/origin restrictions, authentication and strict control request models. |
| `service.py` | Single-owner access to the fleet, deterministic command order, wall-time pacing, sequence envelopes and bounded viewer delivery. |
| `records.py` | Bounded local JSON recordings, checksums and same-runtime replay verification using Stage 8. |
| `__main__.py` | One-worker Uvicorn launcher, fixed loopback binding and bounded WebSocket input. |
| `smoke.py` | Actual Uvicorn subprocess, HTTP/WebSocket clients and reproducible local smoke test. |

One service owns one loaded fleet. Its asynchronous lock serializes load, controls, ticks and replay verification. The sole authoritative tick comes from `FleetSimulator.clock`; only `FleetSimulator.step()` advances it. Each fleet step still gives every vehicle the same clock. Browsers receive truth and cannot submit positions, velocities, dt overrides or physics calculations. A file lock prevents a second backend from owning the same recording directory. The launcher uses one worker with proxy headers disabled; multiple workers/distributed ownership are unsupported.

## Install and run locally

Tested transport runtime: Python **3.12.14**. Install the separate pinned [backend dependency lock](../requirements-backend-lock.txt) into an isolated environment. Core simulation continues to run on Python 3.9 without FastAPI. The existing vision environment does not need to be changed.

```sh
python3.12 -m venv .venv-backend
.venv-backend/bin/python -m pip install -r requirements-backend-lock.txt

# Generate separate credentials locally; do not commit or share them.
export DRONE_CONTROL_TOKEN="$(.venv-backend/bin/python -c 'import secrets; print(secrets.token_urlsafe(32))')"
export DRONE_VIEW_TOKEN="$(.venv-backend/bin/python -c 'import secrets; print(secrets.token_urlsafe(32))')"
.venv-backend/bin/python -m mission_control --port 8000 \
  --record-dir artifacts/stage9/records
```

Both tokens are required, distinct and at least 32 characters. Use randomly generated values; the length check alone cannot establish entropy. The view token authorizes reading, validation and telemetry; the control token additionally authorizes loading, playback, commands and replay. Tokens are constant-time compared and never intentionally logged. REST uses `Authorization: Bearer <token>`; WebSockets authenticate with their first JSON frame because native browser WebSockets cannot set that header. Do not put credentials in URLs.

The provided launcher binds **only `127.0.0.1`**, accepts ports 1024–65535, disables access logs and forwarded-header trust, and permits exact same-port localhost/127.0.0.1 HTTP origins. The ASGI boundary also rejects non-loopback peer addresses and unexpected Host headers. A factory caller may explicitly allow another local HTTP origin, for example a future localhost development client; arbitrary remote origins are rejected. No internet deployment is supported. HTTP rather than TLS is used only on loopback. Credentials are static local capabilities, not user accounts, OAuth, audit signatures, expiration or a production security model.

## REST API v1

All endpoints require authentication, including status and the OpenAPI document. There are no public Swagger pages. [Committed OpenAPI schema](diagnostics/stage9/openapi.json) describes typed controls; configuration wrapper details are below.

| Endpoint | Role | Meaning |
| --- | --- | --- |
| `POST /v1/configurations/validate` | View/control | Validate and normalize a full fleet or single-mission wrapper without changing the current run. |
| `POST /v1/runs` | Control | Load a validated configuration, allocate a run ID and publish initial truth. Starts with playback paused. |
| `GET /v1/status` | View/control | Current run/state, engine tick/dt, playback, sequence, viewer count, recording/error status. |
| `GET /v1/snapshot` | View/control | Latest full truth envelope with current playback status. |
| `POST /v1/playback` | Control | Set `run_id`, boolean `paused`, and finite `speed` between 0.1 and 20. |
| `POST /v1/commands` | Control | Queue an explicit simulation-only command; acknowledge its order and assigned target tick. |
| `GET /v1/commands` | View/control | Applied/rejected outcomes and pending count for the current run. |
| `GET /v1/results` | View/control | Recorded run IDs. |
| `GET /v1/results/{run_id}` | View/control | Configuration, summary, command outcomes, completion/error state, runtime and checksum. |
| `GET /v1/results/{run_id}/telemetry?offset=0&limit=100` | View/control | Recorded fleet samples; maximum page size 1000. |
| `POST /v1/results/{run_id}/verify-replay` | Control | Recompute with Stage 8 and compare the complete telemetry checksum. |
| `POST /v1/results/{run_id}/replay` | Control | Verify a finished recording, then load its effective command schedule as a new paused run. |
| `GET /v1/openapi.json` | View/control | Generated API schema. |

Full Stage 8 input uses `{"kind":"fleet","config": ...}` with a complete existing configuration. Each member includes `vehicle_id`, Stage 7 `mission` and Stage 6 `vehicle` configuration. This creates a runnable request from a committed fixture:

```sh
python3 - <<'PY'
import json
from pathlib import Path
config = json.loads(Path('scenarios/stage8/crossing.json').read_text())
Path('/tmp/drone-run-request.json').write_text(json.dumps({'kind':'fleet','config':config}))
PY
curl -H "Authorization: Bearer $DRONE_CONTROL_TOKEN" \
  -H 'Content-Type: application/json' --data-binary @/tmp/drone-run-request.json \
  http://127.0.0.1:8000/v1/runs
```

For a complete Stage 7 mission, use `kind: "mission"` with `config` containing `fleet_id`, `member`, `steps`, and `actions`. `member` has the same complete three fields as above. Vehicle capabilities and the launch state must be explicit; a bare mission alone cannot define them. It is wrapped in one existing Stage 8 member, not replanned. Stage 7 home/area/altitude fields are retained without inventing calibration or terrain data.

Unknown wrapper fields, malformed/invalid geometry, nonfinite data, wrong launch positions and excessive capabilities are rejected. Backend limits are four vehicles, 128 total route legs, 5000 ticks, 1000 configured actions and a 1 MiB HTTP body. Configuration validation delegates to existing records after inexpensive size checks. Invalid configuration returns 422; unauthenticated/insufficient-role requests return 401/403; body overflow returns 413. Stale run IDs, invalid control transitions/limits and unavailable run state return 409. Pydantic rejects unknown fields and coerced control types with 422.

## Playback versus mission control

Playback pause stops calling `step()`: tick, motion and battery do not advance. Resume continues at the next unchanged dt. Speed changes only the wait between steps (`dt / requested_speed`). Processing and scheduler overhead can make actual playback slower. There is no dt scaling, tick skipping or burst catch-up. Browser load and recording work must not change numerical integration results.

A **mission** pause is different. Queue `command: "pause"` to call the existing fleet or vehicle method at a tick boundary. It preserves Stage 6 holding, inertia and battery drain while playback continues. Accepted command names are only `start`, `pause`, `resume`, `abort`, and `emergency_stop`. Omitting `vehicle_id` uses Stage 8's eligible-member broadcast; supplying one preserves strict individual transitions. Commands are virtual-only; there is no generic method invocation, arbitrary code, vehicle state setter or external control bridge.

Example command body:

```json
{"run_id":"<returned run ID>","request_id":"operator-pause-001","command":"pause","vehicle_id":"eastbound","target_tick":100}
```

`target_tick` is optional; omission assigns the currently unexecuted tick while holding the service lock. Explicit ticks must be in the remaining run. Configuration actions execute first at each tick in declared order, then interactive commands in server acceptance order. Concurrent clients receive an explicit increasing order. Fixed target ticks are preferable for repeatable experiments; submitting at the same wall-clock instant at different playback speeds need not select the same tick.

An acknowledgement means **queued**, not successfully applied. Actual outcomes include tick, command, target, request ID/order, accepted flag, affected IDs or rejection reason. Invalid transitions are logged/reported without stopping other commands or vehicles. Unlike the batch Stage 8 CLI, the backend deliberately isolates an invalid scheduled command too; recordings retain its rejection and replay only successfully applied commands. No invalid action is silently presented as success.

During an active run, retries with the same request ID and identical parameters return the original acknowledgement without duplication. Reusing an ID for a different command is rejected. At most 128 commands are pending and 1000 interactive request IDs are retained per run. Request IDs are 1–64 characters. Stale run IDs cannot control replacement runs. Queued commands do not take effect while playback is paused. Commands near completion that target outside the configured run are rejected.

## WebSocket protocol and recovery

Connect to `ws://127.0.0.1:8000/v1/telemetry`, then send within five seconds:

```json
{"token":"<view token>","epoch":"<previous server epoch if reconnecting>","after_sequence":42,"vehicle_id":"eastbound"}
```

Only `token` is required. Omit `vehicle_id` for fleet telemetry. This stream is read-only: further client messages close it with policy code 1008. Use authorized REST for commands. Disallowed origins, bad credentials, unknown vehicle IDs or malformed cursors are refused. At most 16 connections, including pending authentication, are admitted. Uvicorn input frames are bounded to 16 KiB with a four-message protocol receive queue.

Telemetry envelopes have `schema_version: 1`, `type` (`telemetry` or `vehicle_telemetry`), `epoch`, `run_id`, `sequence`, `delivery`, `status`, `simulation_truth`, `estimated_tracking`, and `advisories`. Sequence increases once per published fleet sample, not per viewer or vehicle. Engine tick/time remains separate. `simulation_truth` carries existing vehicle position, velocity, acceleration, heading, battery, waypoint progress, mission state and timestamped events. Vehicle streams select the same-tick member record. `estimated_tracking` is explicitly null: no perception estimate is fabricated or mixed with exact simulated truth. Proximity and static route conflicts reside under `advisories` and cannot influence control.

A connection receives a full snapshot. A reconnect with the same server epoch and a retained sequence gets missing messages marked `delivery: "replay"`, if they fit the queue. Otherwise it receives the latest complete envelope marked `resync`, with a reason. At the current sequence it receives a snapshot. Run replacement clears history and queues; global sequence remains increasing within the server epoch. A vehicle-specific connection closes if its selected ID is absent from the new fleet.

History is bounded to 64 envelopes; each viewer queue holds eight. Overflow discards that viewer's queued older envelopes and supplies a complete latest `resync` envelope with the number discarded at that overflow. Sequence gaps are observable; this is **not lossless browser event delivery**. Recordings retain all samples and command outcomes independently. A slow or disconnected viewer never blocks simulation through an unbounded application queue. Socket sends have a two-second timeout; disconnect tasks unsubscribe and release capacity. Idle streams emit a status heartbeat every ten seconds without inventing new simulation ticks or sequence numbers. REST status/snapshot returns current playback status immediately.

## Recording, replay and failure behavior

Runs produce initial truth plus exactly the configured number of updates. Backend `state: "finished"` means this simulation budget completed and was recorded; it does **not** mean every mission succeeded. Consult the Stage 8 summary's `all_completed`, degraded status and per-vehicle stop reasons. A vehicle failure remains isolated by the existing engine. An unexpected engine error stops playback, records available partial truth as `failed`, and exposes the error. Graceful server shutdown saves an active run as `interrupted`; these partial runs cannot be verified as complete replay.

Recordings include expanded configuration, every accepted fleet sample, all processed command outcomes, runtime, Stage 8 summary and SHA-256 over canonical telemetry JSON. Replay reconstructs an effective tick schedule from accepted outcomes and runs the existing Stage 8 simulator. Interactive request metadata is not a physics input. Rejected commands remain in the audit record but are not applied during replay. Replaying through the service creates a fresh run ID while retaining the recorded numerical behavior. Exact verification requires the recording's Python version and matching recomputed checksum; cross-platform floating-point identity is not promised. Checksums detect accidental changes and replay mismatch; they are not signed tamper-proof evidence.

The store admits 20 recordings, each at most 32 MiB. Runs have a conservative half-limit budget for accumulated row bytes, leaving room for configuration/outcomes/summary. Overflow or disk errors stop playback and expose failure; no recording-success claim is made. Existing recordings are not automatically deleted. Archive them locally to regain capacity. Writes use flush/fsync and atomic same-directory rename; interrupted temporary files may require local cleanup after a process crash. There is no crash-resume/WAL guarantee or remote storage. Run-ID paths are restricted to generated hex IDs, not client file paths.

Full runs and recordings are held in memory for summaries and verification; large requests within limits can still consume noticeable memory/CPU. Replay verification serializes with controls and ticks and may briefly delay wall-time playback. REST reads are paginated but currently decode the bounded recording file before slicing. These limits suit local experiments, not an internet service or a hard real-time system.

## Tests and measured results

[Verification evidence](diagnostics/stage9/verification.json) records unchanged previous sources and complete suite execution:

| Environment | Discovered | Passed | Optional skips |
| --- | ---: | ---: | ---: |
| Core Python 3.9.6 | 226 | 212 | 14 (backend transport and video/plots) |
| Isolated backend Python 3.12.14 | 226 | 222 | 4 (video/plots) |
| Existing vision Python 3.12.14 | 226 | 216 | 10 (backend transport) |

All 226 test cases execute successfully across these environments; no single environment is claimed to contain all optional packages. All prior 198 tests remain. The 28 new tests cover existing-clock equivalence, playback pause/speed, real paced runner, command order/idempotency/concurrency, stale controls, bounds, snapshots and truth separation, multiple viewers, disconnect/reconnect, authorization/origins/remote peers, failure isolation, storage/checksum/disk failures, single-owner locking, interrupted shutdown, mission wrappers, recording and REST replay. A separate real Uvicorn smoke test exercises actual sockets rather than only ASGI fixtures. GitHub Actions adds a Python 3.12 backend job, full suite and loopback smoke run alongside the unchanged core matrix.

```sh
python3 -m unittest discover -q
.venv-backend/bin/python -m unittest discover -q
MPLCONFIGDIR=/tmp/drone-stage9-mpl .venv-vision/bin/python -m unittest discover -q
.venv-backend/bin/python -m mission_control.smoke \
  --report /tmp/drone-stage9-loopback.json
```

The [measured loopback smoke result](diagnostics/stage9/loopback-smoke.json) completed 80 ticks, dt=0.02 s (1.6 simulated seconds), with concurrent fleet/vehicle subscribers and all 80 post-initial fleet samples received. Requested playback was 2×; observed wall duration was **1.147 s**, including connection/request/recording overhead but excluding socket shutdown. This is about 1.395× achieved playback for this short local run, not a throughput guarantee. Reconnection received authoritative tick 80 and replay matched exactly. The run finished while its missions remained incomplete, which was explicitly reported. Random local authentication tokens were checked absent from captured server logs; the subprocess was shut down after testing.

Known environment warnings: pinned Starlette 1.7.0 still supports the installed httpx test client but emits a deprecation warning recommending httpx2; optional older video tests retain their PyAV/OpenCV duplicate Objective-C class warnings. Async test debug mode also reports long CPU sections in batch replay. None failed the suites. Backend requirements were installed only in `.venv-backend`; no vision/model weights were changed or downloaded.

Framework references used during implementation: [FastAPI WebSockets](https://fastapi.tiangolo.com/advanced/websockets/) and [Starlette TestClient](https://www.starlette.io/testclient/). The in-process and actual-socket tests verify the pinned versions' behavior.

## Remaining scope

This is a local single-process backend, not a production multi-user service. No automatic avoidance, physical interfaces, estimator integration, browser UI, dynamic fleet membership, live replanning or real terrain was added. Authentication authorizes simulation actions only. Existing simplified physics, advisory separation, battery and coverage limitations continue to apply; successful transport/replay tests do not validate real aircraft safety.
