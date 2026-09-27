# Development plan

> Publication update (2026-09-26): After the audit, the owner authorized replacing the Wi-Fi SSID/password in both firmware files with placeholders, deleting the local ZIP, and committing/pushing the project. Preservation and no-commit statements below describe the original audit snapshot. All 42 extracted files remain present; only firmware credential values were intentionally changed after that audit.

Status: Stages 1–3 are merged. Stage 4 recorded-video integration is implemented for review; see [Stage 4 results](STAGE4_INTEGRATION.md). Stages 5–6 remain planned. `existing-tracker/` remains unchanged by all simulation stages.

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
| 6: future vehicle behavior | Only after a separate instruction defines the drone's role, simulator fidelity, objectives and constraints: introduce virtual dynamics and then evaluate a suitable controller. | Requirements and evaluation criteria agreed before implementation; perception and flight-control interfaces remain distinct. |

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

Stage 4 implements the owner’s revised recorded-video integration scope, superseding the earlier optional UDP/Unity proposal. The complete 98-test suite passes without optional detector dependencies. A new PR targets `dev` for review without automatic merge. Real detector performance remains unmeasured; representative labeled footage and an explicitly selected runtime/model are needed for Stage 5. No legacy code, aircraft control or real stereo integration was changed.
