# Stage 10 — local mission-control dashboard

PR #9 was reviewed, its successful CI checked, and merged into `dev` at `e63fc2d` before creating `stage10-mission-dashboard`. Stage 10 adds a separate Next.js/TypeScript/Tailwind application in `dashboard/`. The original tracker, all `drone_sim/` packages and the Stage 9 `mission_control/` implementation remain unchanged.

## Implemented interface

SKYVIEW provides a fleet sidebar, local ENU mission map, selected-vehicle telemetry, playback timeline, expandable timestamped events, validated configuration loading and completed-run replay. It uses the supplied dark palette and reference's restrained orange/blue/green indicators. Self-hosted Barlow/IBM Plex Mono, Phosphor icons and Radix dialogs keep presentation and interaction consistent. See [the design contract](../dashboard/DESIGN.md).

The final Stage 10 map/scope instructions take precedence over the earlier terrain request: this is a **2D ENU map in meters**, with no Mapbox, terrain, latitude/longitude, geographic origin or Gazebo connection. Independent vehicle positions come directly from Python. Route legs, survey polygons and homes come from normalized Stage 7/8 configurations; the browser does not generate missions or integrate physics. Up is a local coordinate, not measured height above terrain. Displayed speed is the magnitude of the 3D velocity vector.

Orange route-overlap advisories describe spatial plans; red proximity alerts describe the backend's synchronized swept-separation diagnostics. Neither triggers automatic avoidance. Exact simulation truth remains separate from the explicitly absent tracking estimate.

## Local setup

Use Node 24 and the existing Python 3.12 backend environment. From the repository root:

```sh
# If the backend environment is not already present:
python3.12 -m venv .venv-backend
.venv-backend/bin/python -m pip install -r requirements-backend-lock.txt

cd dashboard
npm ci
npm run build
cd ..

# Generate three distinct secrets; never put them in NEXT_PUBLIC variables.
export DRONE_CONTROL_TOKEN="$(.venv-backend/bin/python -c 'import secrets; print(secrets.token_urlsafe(32))')"
export DRONE_VIEW_TOKEN="$(.venv-backend/bin/python -c 'import secrets; print(secrets.token_urlsafe(32))')"
export DASHBOARD_ACCESS_KEY="$(.venv-backend/bin/python -c 'import secrets; print(secrets.token_urlsafe(32))')"

.venv-backend/bin/python -m mission_control --port 8000 \
  --record-dir artifacts/stage9/records &
BACKEND_PID=$!
(cd dashboard && npm start)
# After stopping the dashboard with Ctrl-C:
kill "$BACKEND_PID"
```

Open `http://127.0.0.1:3000` and enter the value of `DASHBOARD_ACCESS_KEY` generated in your own shell. Backend credentials stay in the Node/Python environment. Set `DASHBOARD_PORT` and `DRONE_BACKEND_URL` to change local ports; the backend URL must remain an `http://127.0.0.1:PORT` origin. An already-running backend and dashboard must use matching backend tokens. Three different values of at least 32 characters are mandatory; use random values, not repeated characters.

`npm run dev` starts the same protected local bridge with Next's development compiler. Hot WebSocket refresh is intentionally not forwarded; refresh the page after edits. The tested/recommended execution is `npm run build` followed by `npm start`. The custom launcher is required: bare `next start` does not provide the protected bridge. No public deployment configuration is provided.

## Architecture and transport

```text
Browser: React state + SVG ENU projection
  │ same-origin session cookie; HTTP + read-only WebSocket
  ▼
Node loopback bridge: dashboard/server/
  │ server-held view/control tokens; allowlisted Stage 9 requests
  ▼
Unchanged FastAPI service → unchanged Stage 8 fleet → Stage 6 engines/clock
```

| Code | Responsibility |
| --- | --- |
| `server/index.mjs` | Loopback-only Next server and WebSocket upgrade routing |
| `server/bridge.mjs` | Local sessions, request/origin checks, allowlisted REST, read-only telemetry relay, normalized plan cache |
| `lib/model.ts` | Version checks, sequence handling, bounded trails/events, display units and bounds |
| `lib/useMission.ts` | WebSocket lifecycle, cursor/reconnect, status/outcome polling and acknowledged requests |
| `components/MissionMap.tsx` | Local ENU presentation, layers, zoom, home/waypoint/vehicle selection |
| `components/Dashboard.tsx` | Mission workflow, controls, telemetry, recordings and dialogs |

Browser REST paths `/api/status`, `/snapshot`, `/validate`, `/runs`, `/playback`, `/commands`, `/results` and `/results/:id/{verify-replay,replay}` map to Stage 9's `/v1` endpoints. `/api/session` manages local access; `/api/presets` reads the three committed Stage 8 scenarios; `/api/plan` exposes a normalized configuration cached after an authorized load/replay. Backend validation runs again when loading. No arbitrary proxy URL is accepted.

The Stage 9 schema-version-1 envelope crosses `/telemetry` unchanged. Reconnect sends the last epoch and sequence; the backend supplies replay or a fresh snapshot. The client rejects older sequences within the same run/epoch and does not append duplicate samples. Sequence gaps and explicit resync break trajectory lines. Changing run/epoch resets histories. Retention is 600 received positions per vehicle, 100 events and 20 locally acknowledged commands. Historical trails are presentation samples, not a complete recording.

Status and command outcomes are polled every 1.5 seconds with no overlapping polls. Missing telemetry/heartbeats for 15 seconds marks the connection stale. Reconnect backoff ranges from 0.5 to 8 seconds. The bridge permits eight WebSocket viewers, rejects incoming browser telemetry commands, caps upstream frames at 2 MiB and disconnects a viewer when its outbound buffered bytes exceed 256 KiB before the next send. The threshold can be exceeded by one frame; this is a disconnect policy, not a durable message queue. Stage 9 retains its own bounded delivery/history and full recording rules.

## Authorization and command semantics

The bridge binds only to IPv4 loopback, validates the peer and Host, requires exact same-origin mutation/WebSocket requests, rejects cross-origin reads with an Origin header and uses a random HttpOnly SameSite=Strict session cookie. Sessions last eight hours, are capped at 16, and are revoked on Lock; login attempts are capped at ten per minute per loopback address. Request bodies are limited to 1 MiB. Loopback HTTP is deliberate; this is not an internet/TLS authentication design. Other processes or users with access to your local machine remain outside this isolation boundary.

The browser receives neither backend token. Its access key authorizes the local workspace and is cleared from React state after login. There is no localStorage credential storage. The bridge uses the read token for reads/validation and the control token for mutations. An unlocked workspace can control virtual simulations only.

Playback pause freezes time advancement. Vehicle/fleet Hold is the existing mission pause, which continues bounded position holding while playback runs. Start, Hold and Resume issue existing commands. Stop dialogs provide abort and emergency-stop actions and explain their irreversible-in-run virtual freeze. A successful HTTP acknowledgement is displayed as **QUEUED**, never as executed. The correlated backend outcome changes it to **APPLIED** or **REJECTED**; a paused clock requires resuming playback before application. Broadcast commands may affect no eligible vehicles under the existing backend contract.

A completed simulation budget is not necessarily a completed mission. The recordings dialog reports both run state and mission completion. Replay verification calls the Python verifier; replay creates a new paused run. The timeline is a read-only progress indicator, not a fabricated random-access seek control.

## Validation

Commands from the repository root:

```sh
python3 -m unittest discover -v
.venv-backend/bin/python -m unittest discover -v
.venv-vision/bin/python -m unittest discover -v  # existing optional codec/plot environment
.venv-backend/bin/python -m mission_control.smoke --report /tmp/stage10-loopback.json

cd dashboard
npm run format:check
npm run build
npm run typecheck
npm test
npx playwright install chromium
npm run test:e2e
```

Playwright launches actual loopback Node/FastAPI processes on ports 13000/18000 with test-only credentials and a temporary recording directory. `BACKEND_PYTHON` can select a different prepared interpreter; CI uses `python`. No real model weights, camera or aircraft are involved. Screenshots and failure traces are written beneath ignored `dashboard/test-results/`.

Results on 2026-09-29:

| Check | Result |
| --- | --- |
| Existing core Python suite | 226 discovered; 212 passed, 14 optional-dependency skips |
| Existing backend environment | 226 discovered; 222 passed, 4 optional codec/plot skips |
| Existing vision environment | 226 discovered; 216 passed, 10 optional backend skips |
| Existing Stage 9 real loopback smoke | Passed: 80 ticks, concurrent viewers, resync and exact replay |
| Dashboard build/typecheck/format | All passed; production Next.js build |
| Dashboard unit and transport tests | 12 passed (7 state/model, 5 local bridge/security) |
| Playwright integration tests | 3 passed against production Next.js and actual FastAPI; final run 13.1 s |

The three Python environments collectively execute every existing test. Browser tests cover real mission validation/load, selection, paused-clock consistency, queued/rejected/applied commands, concurrent viewers, playback speed, recorded results and exact deterministic replay. Controlled network fixtures cover explicit resync/older packets, API rejection and disconnection; these injected cases are labeled fixtures and are not measurements of network reliability. Unit tests cover finite coordinates/schema rejection, bounded histories, run/epoch reset, sequence gaps, prototype-like vehicle IDs, local access checks, server-side token routing and session revocation. Desktop and narrow screenshots are inspected for layout; reduced motion and keyboard focus are implemented, but no comprehensive assistive-technology audit or cross-browser certification is claimed.

Desktop and narrow-layout captures from the automated real-backend survey run: [desktop](diagnostics/stage10-dashboard-desktop.png), [narrow](diagnostics/stage10-dashboard-mobile.png). These show virtual simulation truth and explicitly tested command outcomes.

## Limitations

- Chromium desktop is the automated browser target. The narrow layout is a fallback; terrain and mobile-native interaction are out of scope.
- This is one local operator workspace, not a multi-user permission system. Concurrent viewers may issue competing commands; backend ordering remains authoritative.
- Stage 9 has no active-configuration GET endpoint. The bridge keeps four normalized plans in memory. Runs loaded elsewhere or surviving a bridge restart can display live positions but no route geometry; the UI says so. No configuration is reconstructed from telemetry.
- Bridge restart invalidates sessions and plan cache. Backend restart changes epoch. Client trails/events are bounded and reset on reload; full results remain in backend recordings.
- Replay uses Stage 9's same-runtime determinism contract. UI rendering cadence is not simulation time, and no frame-rate, network-latency or physical-safety guarantee is made.
- There is no interactive planner, geographic conversion, terrain, Gazebo integration, physical control or automatic avoidance. Previous camera/vision packages are untouched.
