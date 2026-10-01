# Stage 13 — MSI / Ubuntu 22.04 WSL2 handoff

Prepared 2026-10-01. **Ready to transfer development; real SITL qualification is still pending.** This handoff does not create a new Stage 13 implementation, start Stage 14, or authorize merging PR #13.

## Repository checkpoint

- Repository: `MaxxNesta/Drone`.
- Branch: `stage13-read-only-sitl`.
- Completed implementation commit: `0f2dae2609c6d0b3c0a0da52f84b24b5e2d116e3` (verified identical locally and on origin).
- Base: `dev`, after PR #12 merge `871efc7034c239b0a8b7d880f19ddd858db97f20`.
- [PR #13](https://github.com/MaxxNesta/Drone/pull/13) targets `dev`, remains open/unmerged, and follows this same branch. Fetch its latest head to include this handoff commit.
- This document records the implementation SHA rather than attempting to embed its own Git hash. Obtain the exact handoff revision with `git log -1 --format=%H -- docs/STAGE13_MSI_HANDOFF.md`; record `git rev-parse HEAD` with every MSI run.

All **five** implementation CI jobs passed in [run 36820402732](https://github.com/MaxxNesta/Drone/actions/runs/36820402732): `tests (3.9)`, `tests (3.12)`, `backend`, `dashboard`, and `sitl-parser`. No failed job required a software fix. The documentation push triggers a new run; check that run against the fetched head before subsequent changes. CI exercises synthetic fixtures and the numerical engine, not installed PX4/Gazebo.

Local evidence: 259 Python tests discovered per environment, collectively all executed; 15 dashboard unit tests; seven browser scenarios; production build/typecheck/format; numerical WebSocket/replay and launcher checks. See [Stage 13 results](STAGE13_REAL_SITL.md) and [validation record](diagnostics/stage13/validation.json) for exact passes/skips and scope.

## Ownership and completed implementation

| Machine | Responsibility from this handoff |
| --- | --- |
| MacBook | SKYVIEW frontend/backend development, PR reviews, existing automated regressions and future frontend integration. No further Stage 13 simulator installation or qualification work on this machine. |
| MSI / Ubuntu 22.04 WSL2 | PX4/Gazebo installation, actual unarmed single-x500 execution, real telemetry capture, frame/time/validity qualification and Linux/WSL troubleshooting. |

Completed code includes a receive-only Gazebo/PX4 collector, Stage 12 snapshot mapping, truth/estimate separation, freshness/identity/clock checks, bounded atomic local-file handoff, view-authenticated `GET /v1/sitl`, session-protected `GET /api/sitl`, and a separately labeled read-only SKYVIEW display. Optional parser dependencies remain separate. Numerical physics, planner, original tracker and computer vision are unchanged. No arming, takeoff, motor, serial, outbound MAVLink or autopilot command interface is present.

Read [Stage 12](STAGE12_SIMULATOR_ADAPTER.md), [Stage 13](STAGE13_REAL_SITL.md), `simulator_adapter/sitl.py`, `simulator_adapter/sitl_runtime.py`, `mission_control/sitl.py` and `config/sitl/telemetry.example.json` before qualification. Do not replace the current implementation to restart Stage 13.

## MSI prerequisites

Use **Ubuntu 22.04, x86-64, WSL version 2**, PX4 **v1.16.0** commit `6ea3539157ca358c70a515878b77077af7d4611d`, and **Gazebo Harmonic / gz-sim major 8**, with Transport 13 and msgs 10 Python bindings. One headless x500 in the default plane world; no camera payload or terrain. Use the committed [environment specification](../config/sitl/environment.json).

PX4 documents [WSL2 development](https://docs.px4.io/v1.16/en/dev_setup/dev_env_windows_wsl). The selected combination is a qualification target, not a claim that this MSI has already run it. The MSI's RAM, free disk, virtualization support, Windows/WSL versions and runtime package availability have not been inspected remotely. Stage 12's provisional allowance is 4 CPUs, 8 GB guest RAM and 30 GB free disk, not a measured minimum. Start with two build jobs and leave resources for Windows.

In Windows PowerShell, inspect the existing distro first:

```powershell
wsl --version
wsl --list --verbose
# Only if Ubuntu-22.04 is absent:
wsl --install -d Ubuntu-22.04
# Only if that existing distro is version 1:
wsl --set-version Ubuntu-22.04 2
wsl -d Ubuntu-22.04
```

Use the actual installed distro name if different. Do not unregister/reset an existing distro or overwrite its work. Microsoft documents these [WSL commands](https://learn.microsoft.com/en-us/windows/wsl/basic-commands). Keep Linux source/builds under the WSL Linux filesystem (for example `~/src`), not a Windows-mounted drive, following [Microsoft's filesystem guidance](https://learn.microsoft.com/en-us/windows/wsl/filesystems).

## Continue the existing branch

For a fresh clone inside WSL:

```sh
mkdir -p "$HOME/src"
cd "$HOME/src"
git clone --branch stage13-read-only-sitl https://github.com/MaxxNesta/Drone.git Drone
cd Drone
git fetch origin
git status --short
git rev-parse HEAD
git rev-parse origin/stage13-read-only-sitl
git merge-base --is-ancestor 0f2dae2609c6d0b3c0a0da52f84b24b5e2d116e3 HEAD
```

For an existing clone, inspect/save any local changes first, then `git fetch origin`, `git switch stage13-read-only-sitl` and `git pull --ff-only origin stage13-read-only-sitl`. Do not reset or force-push. Confirm both head SHAs match before starting. Use GitHub authentication locally when needed; never put a token in the remote URL or commit it.

Create one evidence folder per run; keep this shell's paths available in subsequent terminals:

```sh
SKYVIEW_REPO="$HOME/src/Drone"
SITL_SOURCE="$HOME/src/skyview-px4"
SITL_EVIDENCE="$SKYVIEW_REPO/artifacts/stage13/$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$SITL_EVIDENCE"
cd "$SKYVIEW_REPO"
git rev-parse HEAD > "$SITL_EVIDENCE/skyview-commit.txt"
cat /etc/os-release > "$SITL_EVIDENCE/os-release.txt"
uname -a > "$SITL_EVIDENCE/kernel.txt"
free -h > "$SITL_EVIDENCE/memory.txt"
df -h . > "$SITL_EVIDENCE/disk.txt"
nproc > "$SITL_EVIDENCE/cpu-count.txt"
```

These are user-relative example locations, not paths copied from the MacBook. Save PowerShell's WSL version/distro output alongside them. Do not dump the full environment, which may contain secrets.

## Install the pinned environment

Fresh PX4 checkout only; if the destination already exists, inspect it rather than cloning over or deleting it:

```sh
cd "$HOME/src"
git clone --no-checkout https://github.com/PX4/PX4-Autopilot.git skyview-px4
cd "$SITL_SOURCE"
git checkout --detach 6ea3539157ca358c70a515878b77077af7d4611d
git submodule update --init --recursive
bash Tools/setup/ubuntu.sh --no-nuttx
```

Upstream setup may request sudo/restart. Follow its output, then reopen the same WSL distro and restore the task path variables. Do not substitute moving `main`, Gazebo Classic or another Gazebo major. PX4 source/submodules are pinned; apt/pip dependency resolution is not a hermetic lock.

From the SKYVIEW checkout, create a new Linux environment rather than copying a Mac venv:

```sh
cd "$SKYVIEW_REPO"
sudo apt install python3-gz-transport13 python3-venv
python3 -m venv --system-site-packages .venv-sitl
.venv-sitl/bin/python -m pip install -r requirements-sitl.txt
.venv-sitl/bin/python -c 'from gz.transport13 import Node; from gz.msgs10.clock_pb2 import Clock; from gz.msgs10.pose_v_pb2 import Pose_V'
python3 -m scripts.sitl_diagnostics --px4 "$SITL_SOURCE" --report "$SITL_EVIDENCE/preflight.json"
.venv-sitl/bin/python -m unittest tests.test_real_sitl -v
```

Use Ubuntu's distro Python for these bindings; a different Python ABI may not import them. Read preflight `blockers`: writing a report or returning exit code zero does **not** mean compatibility passed. Confirm distro/release, pinned PX4 revision and a successful unambiguous major-8 version probe. Then preserve versions and build:

```sh
git -C "$SITL_SOURCE" rev-parse HEAD > "$SITL_EVIDENCE/px4-commit.txt"
git -C "$SITL_SOURCE" submodule status --recursive > "$SITL_EVIDENCE/submodules.txt"
gz sim --versions > "$SITL_EVIDENCE/gazebo-versions.txt"
dpkg-query -W > "$SITL_EVIDENCE/packages.txt"
"$SKYVIEW_REPO/.venv-sitl/bin/python" -m pip freeze > "$SITL_EVIDENCE/python-packages.txt"
cd "$SITL_SOURCE"
MAKEFLAGS=-j2 make px4_sitl
```

## First actual smoke and capture

Keep Gazebo, PX4, collector and any live backend inside **the same WSL distro**. No USB/serial passthrough, physical aircraft, forwarded MAVLink, broadcast or LAN exposure. Do not solve discovery issues by changing SKYVIEW's loopback/Host/Origin authentication boundaries. WSL networking differs by mode; consult [Microsoft's networking guidance](https://learn.microsoft.com/en-us/windows/wsl/networking) for diagnosis, not as authorization to expose services.

Terminal A, in the pinned PX4 checkout:

```sh
cd "$SITL_SOURCE"
GZ_IP=127.0.0.1 GZ_PARTITION=skyview-sitl HEADLESS=1 PX4_GZ_WORLD=default MAKEFLAGS=-j2 make px4_sitl gz_x500
```

Record this console session with a local terminal transcript. At the PX4 prompt inspect only:

```text
gz_bridge status
listener vehicle_status -n 1
listener vehicle_local_position -n 1
```

Terminal B in the same distro:

```sh
export GZ_IP=127.0.0.1 GZ_PARTITION=skyview-sitl
gz topic -l
gz topic -e -t /world/default/clock
```

Run the first two `gz` commands separately; stop the clock echo with Ctrl-C after enough samples to demonstrate advancing simulation time. Also inspect `/world/default/dynamic_pose/info` with `gz topic -e -t` and stop that echo after observing the configured model and entity ID. Confirm exactly one unarmed x500, expected world/model, live bridge and actual state messages. A successful process launch alone is not a passed smoke test.

Inspect `config/sitl/telemetry.example.json`. Its zero offsets and default IDs require validation against the real sources. Copy it into the ignored run directory and record any measured adjustments there:

```sh
cd "$SKYVIEW_REPO"
cp config/sitl/telemetry.example.json "$SITL_EVIDENCE/telemetry-config.json"
.venv-sitl/bin/python -m simulator_adapter.sitl_runtime \
  --config "$SITL_EVIDENCE/telemetry-config.json" \
  --output "$SITL_EVIDENCE/live.json" \
  --capture "$SITL_EVIDENCE/raw-packets.jsonl" \
  --duration 60 --confirmed-isolated-sitl
```

The scope flag is only appropriate after confirming isolation and unarmed SITL. Capture creation is exclusive; choose a new run path instead of overwriting evidence. Raw capture is capped at 10 MiB. Collector shutdown deliberately writes unavailable state. Preserve an additional copy of the live snapshot **while collecting** if needed for evidence; a final unavailable snapshot after shutdown is expected.

Verify actual frame signs, source timestamps/offset, estimator flags and freshness. If streams are absent or callbacks fail, retain logs and mark the case blocked/failed; do not alter fixtures or label unknown estimates valid. Keep arming/takeoff/autopilot commands outside this task.

## Live viewing and Mac/MSI boundary

Collector receipts use host monotonic time. **Do not copy the MSI's live JSON to the Mac and treat it as live telemetry.** Different hosts, WSL restarts and clock epochs are not interchangeable. Raw files can be transferred as offline evidence with their provenance, not fed into the Mac's live endpoint.

The Mac remains the frontend/backend development machine. If an end-to-end live UI qualification is needed on MSI, run the existing backend and dashboard builds temporarily in that same WSL distro using the supported Python 3.12 backend environment and Node 24. Follow [Stage 11 local setup](STAGE11_INTEGRATION.md), create fresh local `.skyview.env` credentials, and never copy the Mac's secrets or virtual environments. Ubuntu 22.04's distro Python used by Gazebo is separate from this backend interpreter.

After building the dashboard and preparing those prerequisites:

```sh
cd "$SKYVIEW_REPO"
export SKYVIEW_SITL_SNAPSHOT="$SITL_EVIDENCE/live.json"
python3 scripts/skyview.py --check
python3 scripts/skyview.py --no-build
```

Use a browser in the WSL environment (for example through an available WSLg setup) for the initial loopback-bound check. If Windows-host browser forwarding is rejected by the current security boundary, do not weaken it; file a concrete Linux integration finding for Mac-side review. No Mac-to-MSI live bridge or remote command endpoint has been authorized or implemented.

## Qualification evidence and known blockers

The Mac's absent runtime and insufficient disk blocked real qualification. Moving machines removes neither the need to verify WSL resources nor the following untested assumptions:

- Gazebo Python ABI, exact topic/message shapes, model/entity naming and callback behavior.
- PX4 default emission of HEARTBEAT, LOCAL_POSITION_NED, ATTITUDE and ESTIMATOR_STATUS; missing validity evidence must remain unavailable.
- ENU/NED signs, world/PX4 origin alignment, heading and boot-to-simulation offset. No measured calibration currently exists.
- Actual clock rollback, restart, identity/UDP endpoint replacement and disconnect/reconnect behavior. Some cases intentionally require collector restart/new epoch.
- Real rates, packet loss, WSL scheduling/resource usage and shutdown cleanup. No real throughput or physics performance is established by CI.
- EKF origin resets without timestamp rollback are not detectable with the selected streams. Stop and requalify offsets after an estimator reset.

For each real run retain: SKYVIEW/PX4 SHAs; submodule/package/WSL versions; configuration; bridge/status console output; multiple increasing Gazebo clock samples; model/entity and MAVLink IDs; original captured packets; labeled truth/estimate snapshots; validity/rejection reasons; start/stop times; observed CPU/RSS/disk usage; and an explicit pass/fail/blocked result for each case. Sample Linux `ps` or `pidstat` for the actual owned processes and label resource units. Preserve failures, not only successful screenshots.

After the first smoke passes, test collector stop/staleness, simulator restart/new epoch, identity changes and reconnect. Add a **small sanitized captured fixture** with version/capture metadata alongside the existing synthetic fixtures. Do not relabel generated bytes as captured. Write a measured-results addendum to Stage 13 and submit it for review on PR #13. Keep large captures/logs/builds local or in a separately approved evidence store.

## Repository hygiene and coordination

The handoff audit inspected the current tracked tree and Stage 13 diff: no tracked venvs, dependency caches, working secret files or newly generated large artifacts were found; credential-signature scans found no matches. Firmware credentials are placeholders and backend/browser test credentials are labeled test-only. No personal Mac/home paths were found in Stage 13 files. This is a current-tree audit, not a guarantee about all historical Git objects or every possible secret format.

Preserved legacy exceptions: the original ~97 MB tracker demo and ~6.5 MB YOLO checkpoint are intentional pre-existing assets. `existing-tracker/.claude/settings.local.json` contains an old Windows clone path; it is not used by the new runtime and remains unchanged under the preservation requirement. Standard diagnostic executable paths such as `/usr/bin/git` are observations, not workstation-dependent runtime configuration.

`.venv-sitl/` and `artifacts/stage12/` / `artifacts/stage13/` are ignored by the handoff update. Existing untracked Mac work, including `.DS_Store` and other `artifacts/` contents, is preserved and not pushed. Do not use blanket `git add .` for qualification outputs. Before each MSI commit:

```sh
git status --short
git diff --check
git diff --stat
git diff --cached --stat
git diff --name-only dev -- existing-tracker drone_sim
```

Use explicit file staging, inspect staged content for secrets/absolute paths and unexpected binary sizes, then commit and push to `stage13-read-only-sitl`. Pull with `--ff-only` before work on either machine. Coordinate branch ownership: MSI owns Stage 13 simulator qualification after this handoff; Mac work should use separate appropriately scoped branches so neither machine overwrites the other's work. No force-push, work deletion, numerical changes, automatic merge or Stage 14 start is part of this handoff.
