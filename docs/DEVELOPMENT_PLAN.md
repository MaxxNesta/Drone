# Development plan

> Publication update (2026-09-26): After the audit, the owner authorized replacing the Wi-Fi SSID/password in both firmware files with placeholders, deleting the local ZIP, and committing/pushing the project. Preservation and no-commit statements below describe the original audit snapshot. All 42 extracted files remain present; only firmware credential values were intentionally changed after that audit.

Status: Stages 1–11 are merged. Stage 12 adapter architecture and mock are implemented for review; see [Stage 11 results](STAGE11_INTEGRATION.md). Real-video accuracy/identity validation still requires independent manual labels.

## First algorithm to implement

Recommend a **fixed-step, three-dimensional constant-velocity point-target simulation with calibrated camera projection**.

At tick k, advance prescribed target truth using `p[k+1] = p[k] + v[k] Δt`. Begin with two stationary cameras and one target, all in a local ENU frame. A virtual drone is represented only by its position/velocity here; this is a kinematic observation testbed, not a flight controller or rigid-body flight model.

For each camera, transform target truth into optical coordinates `[X,Y,Z]`. If `Z > 0`, project `u = fx X/Z + cx`, `v = fy Y/Z + cy`; reject points outside image bounds. Emit timestamped synthetic detections. Start noise-free, then add seeded pixel noise and dropouts. Keep world truth separate from estimates.

This is the smallest useful first algorithm because it supplies independent, geometrically consistent camera observations and known truth. It directly addresses the supplied simulator's missing feedback geometry and enables objective tests of tracking and triangulation. It does not require YOLO, a renderer, an autopilot, reinforcement learning or six-degree-of-freedom physics.

## Stages and acceptance criteria

| Stage | Work in a future authorized task | Completion evidence |
| --- | --- | --- |
| 1: deterministic observation simulator | Separate Python package; fixed simulation clock; prescribed constant-velocity target; two calibrated cameras; ENU/optical transformations and pinhole projection; in-process records. | Analytical trajectories match after known durations; repeated runs reproduce outputs; image center and signed offsets are correct; behind-camera/out-of-view points are invalid; distinct camera origins yield expected parallax. No hardware or network needed. |
| 2: estimator baseline | New initialized constant-velocity image filter with monotonic/injected timing; clear centered/coasting/lost states; gated association; calibrated ray least squares with conditioning, forward-ray and residual checks. | Stationary and constant-velocity cases converge; compare raw versus filtered pixel error; noise-free rays recover known truth to numerical tolerance; parallel/near-parallel geometry returns invalid; measure 3D RMSE against truth. |
| 3: failure scenarios | Seeded noise, irregular observation rates, latency, stale/duplicate packets, dropped frames, occlusion, crossings and reacquisition. | Report identity switches, tracking availability, pixel/3D RMSE and latency. Stale observations never count as fresh detections; centered targets stay valid. Scenario-specific thresholds are declared before benchmarking. |
| 4: existing tracker integration | Isolated recorded-video YOLO adapter; versioned image observations shared with simulation; original timestamps, dimensions, confidence and camera identity; optional webcam; no motor/Unity bridge. | Offline fixtures verify conversion, lifecycle states, identity continuity, detection scoring and processing latency; legacy code unchanged. Real 3D remains unsupported without independent calibrated/synchronized cameras. |
| 5: real detector validation | Isolated environment with selected pinned versions only when needed; labeled recorded footage; explicit checkpoint path and verified class mapping; detector produces the same record contract as synthetic detections. | Precision/recall or relevant detection metrics, target continuity, measured latency and filter error on held-out footage. No unsupported drone-detection claim. |
| 6: independent virtual vehicle | Fixed-step ENU point-mass dynamics, bounded PD control, waypoint mission states, battery/geofence monitoring, explicit virtual stops and structured telemetry/plots. | Convergence, norm limits, deterministic replay, waypoint completion, pause/resume and safety tests; vehicle truth remains separate from perception; no physical aircraft or website integration. |
| 7: advanced mission planning | Independent ENU mission/polygon definitions, lawnmower/perimeter coverage, capability validation, approximate duration and virtual return-home plans. | Reproducible geometry/coverage tests, unchanged Stage 6 execution, explicit route rejection and duration comparison, structured JSON and route plots; no live hardware or website. |
| 8: multi-vehicle simulation | Independent vehicle missions and batteries on one authoritative clock; fleet/targeted controls; advisory route/proximity diagnostics and separate telemetry. | Independent execution, failure isolation, shared ticks, deterministic fleet and standalone replay, reproducible survey/crossing/lifecycle fixtures; no automatic avoidance or hardware. |
| 9: local mission-control backend | Optional authenticated FastAPI/WebSocket transport, bounded telemetry, authorized virtual commands, pacing, recordings and replay. | Shared-clock invariance, concurrent viewers, reconnect/resync, command ordering, failure isolation, actual loopback smoke test and complete regression suite; no website or physical integration. |
| 10: local mission-control dashboard | Separate Next.js/TypeScript interface, protected server-side bridge, ENU map, telemetry, commands, results and replay. | Browser reconnect/resync, validation/load, authoritative telemetry, acknowledged versus applied commands, error states, replay and complete regression checks; no terrain or physical integration. |
| 11: integration and local release | Authenticated active configuration, coherent final publication, complete release workflows and reproducible local startup. | Original-simulator recording equality, bridge restart recovery, full surveys/failures/replay, keyboard/accessibility checks and local source RC; no algorithm changes. |

## Fix order for future working copies

1. Resolve target-validity ambiguity and command-loss behavior; fix firmware buffer boundary and reject invalid numeric input before any hardware validation.
2. Seed/reset Kalman state; add capture freshness, stable association and bounded timing.
3. Align coordinate signs, origin, per-node camera configuration and endpoints. Include camera height/extrinsics and real image bearings.
4. Exclude stale/searching/unassociated rays and reject poor triangulation geometry.
5. Package the Unity project, correct MonoBehaviour filename, isolate configuration/credentials, document dependency/board versions, then address concurrency and diagnostics.

Do not retrofit all these changes into the original reference. Preserve the archive and extracted bytes for comparison. The initial simulator can be developed independently with synthetic observations while subsequent fixes remain planned.

## Required later evidence

- For hardware: board and stepper specifications, gearing, mounting signs, travel limits, sensor calibration, valid GPS fixes and surveyed node heights/positions.
- For Unity: editor/package versions, scene, node prefab, camera, shaders and Input System configuration.
- For detector benchmarking: target classes, representative labeled footage, actual model identity and chosen compute environment.
- For eventual flight simulation: whether the drone is the observed target or a tracking observer, desired behavior, fidelity and performance goals.

These do not block the proposed point-target observation simulator. They do block claims of verified physical tracking or flight performance.

## Current stopping point

Stage 11 supplies a local source release candidate with authenticated geometry recovery after bridge restart, coherent final-tick reads, a private-environment startup script and recorded end-to-end checks. Vehicle dynamics, planning, fleet algorithms and computer vision remain unchanged. The PR targets `dev` for review without automatic merge. See [Stage 11 setup, validation and known limitations](STAGE11_INTEGRATION.md).

Stage 12 adds an optional simulator contract and pinned single-x500 PX4/Gazebo setup instructions. Real SITL execution is blocked by absent runtime dependencies on the current host; no integration success is claimed. See [Stage 12](STAGE12_SIMULATOR_ADAPTER.md). The next integration gate is a qualified read-only SITL telemetry transport, not aircraft commands.
