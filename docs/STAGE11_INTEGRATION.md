# Stage 11 — SKYVIEW local prototype release candidate

Release candidate **0.1.0-rc.1**, identified by `SKYVIEW_VERSION`. PR #10 passed CI and was reviewed/merged into `dev` at `c292efc` before the `stage11-prototype-release` branch was created. This is a local software-only civilian mission-control prototype, not an aircraft controller or a certified safety system.

## What changed

- Added authenticated, read-only `GET /v1/configurations/active`; the dashboard reads it through the existing session-protected `/api/plan`. Removed the bridge's transient geometry cache. A bridge restart now requires re-unlocking but recovers the actual active mission, including replay configurations, from Python.
- Fixed a final-tick publication race found by full browser replay: while asynchronous recording finished, status could say `finished` while `/snapshot` still contained tick 4999 of 5000. Status, snapshots and heartbeat status now wait for the service lock, so a completed publication is read atomically. No vehicle dynamics, timestep, route or planning algorithms changed.
- Corrected centisecond formatting so floating-point tick times such as 2.44 s do not display as 2.43 s.
- Corrected the map's accessible group role, restored focus to dialog openers, retained keyboard access to file import, kept fleet controls visible in narrow layouts and prevented tablet command buttons stretching to the adjacent diagnostic panel's height.
- Added full-length real HTTP/WebSocket verification, browser release scenarios, launcher checks, a private environment initializer and local startup script. Previous public records/routes remain compatible.

All **82 tracked files under `existing-tracker/` and `drone_sim/`** remain unchanged from the merge base. Computer vision, dynamics, mission planning and fleet algorithms are preserved. Backend changes are confined to coherent reads and the additive configuration endpoint; other Stage 9 contracts remain intact.

## Reproduce the local release

Tested on macOS with Node 24 and Python 3.12.14 for the backend/vision environments; the dependency-free core suite also runs on Python 3.9. GitHub Actions includes Python 3.9/3.12 regression jobs and the Node 24 dashboard job. Windows is unsupported by the current backend's `fcntl` ownership lock; Linux CI is the second platform. Browser automation targets Chromium.

From the repository root or an extracted source release:

```sh
python3.12 -m venv .venv-backend
.venv-backend/bin/python -m pip install -r requirements-backend-lock.txt
(cd dashboard && npm ci)

# Creates .skyview.env with three different random secrets and mode 0600.
# Refuses to overwrite any existing file; never prints the secrets.
python3 scripts/skyview.py --init-env
python3 scripts/skyview.py --check
python3 scripts/skyview.py
```

Open **http://127.0.0.1:3000**. Read `DASHBOARD_ACCESS_KEY` from your private `.skyview.env` and enter it in the unlock form. Do not share that file. Backend tokens stay server-side. The checked-in [environment template](../config/skyview.env.example) documents every setting and contains no working credentials. Its placeholder values deliberately fail validation.

The launcher resolves interpreter/record paths relative to the repository root, checks dependencies and unused ports, builds the production dashboard, starts the authenticated backend before the dashboard, waits for readiness, and stops its owned processes on Ctrl-C, SIGTERM or a companion failure. It never kills a process merely because that process occupies a port. It does not install packages automatically, bind publicly or alter simulation parameters. `--no-build` uses an already verified production build; rerun without it after source changes. `--env /path/to/private.env` selects another private configuration. Settings are literal `KEY=value` text, not executable shell code or interpolated variables.

Default records are under ignored `artifacts/skyview/records/`. Graceful shutdown during a run produces the backend's existing `interrupted` recording; restarting the backend does **not** resume that in-memory run. Bridge restart recovery and backend process recovery are different capabilities. Use a completed recording for deterministic replay.

## Active-configuration API contract

`GET /v1/configurations/active` accepts the existing view or control Bearer token. All previous loopback, Host and Origin restrictions apply. No new mutation route or broader credential scope is introduced. Unauthorized reads return 401, disallowed Origins return 403 and POST is unsupported (405).

```json
{
  "schema_version": 1,
  "epoch": "backend-instance-id",
  "run_id": "current-run-id-or-null",
  "state": "loaded",
  "config": {"schema_version": 1, "fleet_id": "...", "members": [], "steps": 5000}
}
```

The configuration above is an abbreviated shape, not a loadable mission. Actual `config` is the complete normalized existing Stage 8 record, including Stage 7 mission geometry, vehicle limits, homes and scheduled actions. Before any load it is null, `run_id` is null and `state` is `empty`. Finished/failed runs retain their loaded configuration until replaced or the backend process ends. Reads are detached, serialized under the service lock and do not advance time or alter pending commands. HTTP clients must match the returned run ID before displaying geometry.

The bridge returns this response through `/api/plan` using its server-held view token. It stores no geometry cache. After a bridge restart invalidates sessions, a 401 returns the browser to the unlock form; reauthentication recovers the unchanged run and geometry. Routes loaded by another authorized client are recoverable too. Configuration is never inferred from trajectories.

## Verification and observed results

Run the complete checks from the repository root:

```sh
python3 -m unittest discover -v
.venv-backend/bin/python -m unittest discover -v
.venv-vision/bin/python -m unittest discover -v  # existing optional environment
.venv-backend/bin/python -m mission_control.smoke --report /tmp/stage9-smoke.json
.venv-backend/bin/python -m mission_control.release_check --report /tmp/stage11-transport.json

(cd dashboard && npm run format:check && npm run build && npm run typecheck && npm test)
.venv-backend/bin/python -m scripts.check_startup --report /tmp/stage11-startup.json
(cd dashboard && npx playwright install chromium && npm run test:e2e)
```

Do not run verification commands with Python `-O`: the two standalone release checks explicitly reject disabled assertions. Playwright uses ports 13000/18000, disposable records and test-only credentials. Its bridge-restart signal is an IPC hook in the test process owner; there is no production restart HTTP endpoint. CI runs the same release transport/launcher checks. Full reports and snapshots are linked in [diagnostic evidence](diagnostics/stage11/).

Final results are recorded in [validation.json](diagnostics/stage11/validation.json). Python: 231 discovered in each environment; core 216 passed/15 optional skips, backend 227 passed/4 skips, vision 220 passed/11 skips. Dashboard: 13 unit/transport tests passed, production build/typecheck/format passed, and all 6 Playwright scenarios passed in 50.8 s. The Python environments collectively execute all existing/new tests; optional dependency skips are explicit rather than counted as passes. Browser tests cover both the previous Stage 10 workflows and the new release workflows.

| Real transport scenario | Simulation duration | Recorded samples | Outcome |
| --- | --- | --- | --- |
| Independent surveys | 100 s / 5000 ticks | 5001 | Both vehicles complete; no proximity episodes |
| Crossing | 14 s / 700 ticks | 701 | Both complete; one advisory proximity episode |
| One battery failure | 14 s / 700 ticks | 701 | Alpha aborts at critical battery; bravo completes |

For each case, every downloaded recording row is checked against an independent execution of the original `drone_sim.fleet.simulate` on the same runtime. SHA-256 digests match, deterministic replay verifies, and all per-vehicle ticks/dt/times match the fleet clock. Two real concurrent WebSocket viewers receive truth checked against the reference at each received tick. Stale-cursor reconnects return authoritative resync snapshots. Recorded counts include the initial tick-zero sample. Measured wall durations and viewer sample counts are in `transport-results.json`; requested 20× playback is pacing, not a guaranteed throughput. These results do not promise lossless live delivery on slower clients.

The production browser workflow validates/loads the full survey, restarts only the bridge, re-unlocks, checks unchanged run/clock/geometry, completes 100 simulated seconds, verifies the recording and replays it to identical final truth. Separate browser tests verify battery failure isolation, mixed statuses and visible route-versus-proximity warnings. Existing browser tests retain targeted command acknowledgement/result checks, simultaneous viewers, API failures and controlled packet-gap/reordering fixtures.

The final-tick race has a deterministic regression that suspends the recording write and proves status/snapshot reads wait until publication completes. The earlier failed browser comparison was tick 4999 versus tick 5000; it was not fixed by relaxing equality or changing dynamics. Configuration tests cover empty/current/finished/replaced runs, detached reads, authorization and read-only behavior. The original recording-tamper regression remains in the complete suite.

## Usability and basic accessibility

Chromium checks exercise keyboard map selection, opening a dialog with Enter, focus remaining in the modal, Escape closing it, and focus returning to its opener. Native file input remains keyboard-reachable. The axe WCAG A/AA scan of the selected-vehicle dashboard found a nested-interactive SVG issue, fixed by treating the map as a labeled group rather than a flat image; the rerun reports zero violations in that tested state. This is **not** full WCAG conformance or a screen-reader certification.

[Desktop (1440 px)](diagnostics/stage11/layout-1440.png), [tablet (768 px)](diagnostics/stage11/layout-768.png) and [narrow (390 px)](diagnostics/stage11/layout-390.png) screenshots are inspected and tested for horizontal overflow. Fleet commands remain accessible on narrow screens. Touch hardware, mobile browser gestures, every modal/error combination and user studies remain untested. The compact map labels still favor desktop use; vehicle selection is also available through larger fleet cards.

Two existing runtime warnings remain: Starlette warns about its pinned `httpx` TestClient compatibility layer, and the optional macOS vision environment reports duplicate AVF video-library classes. Their tests pass; no dependency upgrade or computer-vision modification was made to conceal them. SKYVIEW startup does not import those video libraries.

## Troubleshooting

| Symptom | Resolution |
| --- | --- |
| Missing Python/Node dependencies | Install the pinned backend lock and run `npm ci` in `dashboard`; optional YOLO dependencies are not needed for SKYVIEW. |
| Port already in use | Stop the known owner or choose two other local ports in `.skyview.env`. The launcher will not kill unrelated processes. |
| Invalid environment or login key | Use `--init-env` for a new private file, preserve three distinct random values, and enter the dashboard key rather than a backend token. No `export`, quotes or shell substitutions belong in this file. |
| 401 after bridge restart | Unlock again. The backend's active configuration recovers geometry without reloading/replacing the run. |
| Plan unavailable after transient failure | Use Reconnect; the plan fetch is retried. If the backend restarted, it has no active run: load a configuration or replay a completed record. |
| Command remains QUEUED | Resume simulation playback. Commands apply at a tick boundary; vehicle Hold does not pause the global clock. |
| Command is REJECTED | Read the backend reason. A targeted Start/Resume may be invalid for the vehicle's current state. Abort/emergency stop cannot be undone within that run. |
| Run finished but missions incomplete | A run budget and mission completion are distinct. Inspect per-vehicle state, stop reasons and waypoint progress; do not label this a successful survey. |
| Recording directory already owned | Another backend owns the directory. Use one backend per directory; do not delete its lock to bypass ownership. |
| Recording store full | Archive completed files intentionally and select a fresh record directory. Existing limits are 20 records and 32 MiB per file, with conservative in-run budgeting. |
| Replay mismatch or checksum error | Keep the original recording for investigation. Use its exact Python runtime; modified/corrupt data must not be reported as a valid replay. |
| Stale build | Omit `--no-build`; build again after source/dependency changes. Bare `next start` omits the protected bridge. |

## Release checklist and boundaries

- [x] PR #10 reviewed, CI passed, merged; Stage 11 branch based on merged dev.
- [x] Existing tracker, computer vision, dynamics and planner preserved.
- [x] Authenticated active configuration and actual bridge-restart recovery tested.
- [x] Shared clock and original-simulation comparisons verified over full runs.
- [x] Independent commands, mixed states, simultaneous viewers, errors and resync exercised.
- [x] Battery failure and advisory proximity scenarios produce expected outcomes.
- [x] Full recordings, integrity checks and deterministic same-runtime replay tested.
- [x] Local startup, private environment generation and owned-process shutdown tested.
- [x] Production build, Python regression suites, dashboard tests and basic accessibility checks recorded.
- [x] Local source release candidate identified by version, commit and archive checksum.
- [ ] Maintainer review and explicit approval to merge this Stage 11 PR.
- [ ] Release qualification on additional browsers, assistive technologies and physical devices.

The local archive is `artifacts/stage11/skyview-0.1.0-rc.1.tar.gz`; its adjacent `release-manifest.json` records the commit and SHA-256. Recreate it from the intended committed revision using:

```sh
mkdir -p artifacts/stage11
git archive --format=tar.gz --prefix=skyview-0.1.0-rc.1/ \
  -o artifacts/stage11/skyview-0.1.0-rc.1.tar.gz HEAD
shasum -a 256 artifacts/stage11/skyview-0.1.0-rc.1.tar.gz
```

The local candidate is a **source archive**, not an offline binary installer. Node/Python dependencies must be installed separately. Secrets, `node_modules`, virtual environments, `.next`, runtime recordings and untracked personal files are excluded because packaging uses committed Git contents. The archive includes the preserved original reference files; they are not executed by the launcher.

Existing limits remain: one authoritative backend process/fleet; up to four backend vehicles, 5000 ticks and 128 route legs; eight bridge viewers; bounded client display history; same-runtime replay only. Checksums detect telemetry changes and replay inconsistencies, not malicious rewrites of all data by someone controlling the local filesystem. A backend crash is not durable in-flight recovery. Full-screen-reader auditing, general performance/load guarantees, arbitrary-rate lossless delivery and Windows support are not claimed. No aircraft hardware, external autopilot, collision avoidance, internet deployment, PX4 or terrain integration was added.
