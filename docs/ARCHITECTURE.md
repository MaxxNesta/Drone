# Tracking architecture and future simulator boundary

> Publication update (2026-09-26): After the audit, the owner authorized replacing the Wi-Fi SSID/password in both firmware files with placeholders, deleting the local ZIP, and committing/pushing the project. Preservation and no-commit statements below describe the original audit snapshot. All 42 extracted files remain present; only firmware credential values were intentionally changed after that audit.

## Current system

```text
Webcam / video file (one shared capture)
  -> two serial YOLO detection passes
  -> highest-confidence box per node
  -> image-space constant-velocity Kalman filter
  -> 0.40 s predicted pixel center
  -> normalized image error over UDP (~62.5 Hz requested)
     -> Python virtual pan/tilt nodes (localhost 3333 / 3334)
     OR configured ESP32 pan/tilt firmware (each host port 3333)
  -> heading, pitch and GPS telemetry (~10 Hz) to Unity :4444
  -> node transforms -> line least-squares estimate -> visualization

Unity buttons -> mode commands directly to configured motor endpoints
Python buttons -> mode commands directly to Python-configured endpoints
```

The supplied defaults select the virtual nodes in Python, while Unity buttons still select physical-node addresses. These alternatives are not automatically reconciled. There is no return path from virtual node pose to video capture, and no Unity 3D estimate sent back to Python.

## Existing contracts

| Message | Producer → consumer | Meaning / limitations |
| --- | --- | --- |
| `active` / `disabled` | Python or Unity → motor node | ASCII mode command; no acknowledgment or sequence. |
| `x,y` | Python → motor node | `x=clip((px-320)/320)`, `y=clip((240-py)/240)`, three decimal places. Image-right/image-up are positive. Unitless image errors, not angles, positions or drone commands. |
| `ID:0,M:T,P:0.0,MAG:45.0,LAT:37.774929,LON:-122.419418` | Node → Unity | Mode D/S/T, pitch/heading degrees, geodetic coordinates degrees; lacks age, validity, altitude, target identity and covariance. |

Python's 640×480 frame uses x right, y down. Kalman velocities use pixels/second. Unity maps east to +X, north to +Z, up to +Y; GPS conversion currently forces up to zero. A positive Unity Euler X rotates forward down, hence default pitch inversion. Firmware logical sensor axes are forward/right/down, with fixed calibration and a 180° heading correction; true-north alignment is unspecified.

## Separate Python simulator: Stages 1–3

The target-motion and camera-observation layer is implemented in `drone_sim/`; see [Stage 1 usage and contracts](STAGE1_SIMULATOR.md). Stage 2 adds `drone_sim/tracking.py` and `drone_sim/triangulation.py`; see [tracking contracts and results](STAGE2_TRACKING.md). Stage 3 adds `drone_sim/robustness/` for fault injection, unlabeled association and truth-isolated scoring; see [robustness results and contracts](STAGE3_ROBUSTNESS.md). Stage 4 adds the isolated `drone_sim/vision/` adapter; see [recorded-video contracts and limitations](STAGE4_INTEGRATION.md). Stage 6 adds independent software-only vehicle dynamics; physical control remains outside scope.

Keep `existing-tracker/` as the immutable reference. A future simulator should own its clock, world state, cameras and generated measurements in a separate package. It should run without Unity, YOLO, ESP32 or network access for numerical tests. Unity can later become an optional viewer.

Separate responsibilities:

1. **World truth:** target position/velocity and surveyed camera positions/orientations. A virtual drone can initially be a prescribed moving point; this requires no flight control.
2. **Observation model:** transform world points into each camera frame, project through intrinsics, enforce field of view/occlusion, attach timestamps, and optionally add seeded noise/dropouts.
3. **Tracking:** accept synthetic detections first, then a future adapter around reusable detection/filter logic. Keep target identity, measurement age and uncertainty explicit.
4. **Localization:** convert calibrated image points into world rays, combine observations of the same target at compatible times, and reject invalid or ill-conditioned estimates.
5. **Telemetry/viewer:** publish world truth and estimated state separately so visualization cannot be mistaken for evidence of correctness.
6. **Future control boundary:** reserve a distinct interface for virtual vehicle commands only after a later approved task defines dynamics and desired behavior. Existing image errors must never be interpreted directly as thrust or flight commands.

A legacy UDP adapter can observe normalized errors and emulate telemetry for compatibility tests. However, existing UDP alone cannot convey detections, freshness, target identity or uncertainty; a richer adapter/working copy is necessary for reliable integration. Avoid importing the scripts into an application unchanged: both parse CLI arguments at module import, and the tracker couples UI, model loading, capture and command output.

## Proposed typed records

These are architectural contracts, not a finalized wire protocol. Stage 1 implements point detections and camera calibration; Stage 2 returns image-space track covariance and validated two-camera positions. See the stage documents for the exact implemented record fields and remaining limitations.

| Record | Minimum fields |
| --- | --- |
| Detection | schema version, source/camera ID, frame sequence, capture time, time domain, image dimensions, class ID, box, confidence, valid flag, optional association ID |
| Camera pose | camera ID, time, world position in meters, orientation, intrinsics/distortion, extrinsic calibration version, validity |
| Track estimate | track ID, time, coordinate frame, position/velocity, covariance, last measurement time, tracking/coasting/lost status |
| Simulator state | scenario/seed, tick, simulation time, truth target state, camera poses |

Use a local right-handed ENU world frame: `[east,north,up]` in meters, radians internally, and simulation seconds driven by fixed ticks. Unity display conversion is `(X,Y,Z)=(east,up,north)`; orientation conversion must explicitly account for handedness rather than blindly swapping Euler angles. Use a calibrated rotation to convert camera optical coordinates (right/down/forward) to world coordinates. Use actual focal lengths and principal point, not normalized error as an angular substitute.

Pixel ray construction is `d_camera = normalize(K^-1 [u,v,1])`, followed by `d_world = R_world_camera d_camera`. Camera origin is its surveyed position plus mounting offset. Synchronize poses to capture time. Never triangulate two copies of one camera feed as independent observations.

Start with in-process records for deterministic tests; add a versioned localhost transport only when separate processes are needed. Sequence numbers and timestamps should reject duplicates/out-of-order data, queues should be bounded, and mode commands should have acknowledgment if control is later introduced. Treat centered, lost, stale and disabled as distinct states. A single camera provides bearing, not general metric depth; use multiple calibrated views or an explicit depth/plane assumption.

## Validation boundary

The existing simulator can test legacy message formatting and display behavior. It cannot validate world geometry or drone behavior. The first new simulation should expose ground truth and independent per-camera observations so filter and localization error can be measured. See [development plan](DEVELOPMENT_PLAN.md) for staged acceptance criteria and [audit](TRACKER_AUDIT.md) for defects that must not be copied into the new design.

## Stage 4 image-observation boundary

`video_frames → optional YoloDetector → ObservationFrame v2 → ObservationTracker`

`Stage 1 Detection v1 → from_simulation → ObservationFrame v2 → ObservationTracker`

The common v2 frame preserves source/camera identity, original image coordinates, clock provenance and timestamps; it carries unlabeled detections, not truth object identities. Scorer-only annotation records remain separate. The tracker reuses Stage 2/3 image-plane estimation without inventing camera calibration. Real observations are not wired to the Stage 2 triangulator. Recorded video is the primary source; webcam receipt timestamps are explicitly weaker than exposure timestamps. Original Stage 1–3 contracts remain unchanged. See [Stage 4](STAGE4_INTEGRATION.md) for commands, scoring definitions and limitations.

## Stage 5 validation tooling

`vision.validate` composes the unchanged Stage 4 decoder, local YOLO detector and observation tracker with measured timings and a timestamp-preserving annotated video exporter. Raw detector evidence remains separate from explicitly injected observation suppression. `vision.annotations` exports original frames and unfilled review-required templates; labels enter only the scorer. Runtime and source/model fingerprints accompany reports. See [Stage 5 validation](STAGE5_VALIDATION.md) for actual execution results, the pinned optional environment and unvalidated accuracy/occlusion claims. No new real-world depth or control boundary is introduced.

## Stage 6 independent vehicle simulation

`drone_sim/vehicle/` owns simplified ENU vehicle truth, bounded PD motion, waypoint execution and virtual safety policy. It imports the common `FixedStepClock`, not perception/association/triangulation code. Stage 1 now uses the same clock primitive with unchanged frames and public contracts. A future orchestrator may advance one clock, step the vehicle, and pass its truth to separate camera projection code; observations and estimated state must remain separate from this truth.

Vehicle telemetry and mission commands form in-process boundaries for a future website/service. No transport or physical command mapping exists. Pause means bounded position holding; abort/emergency/geofence/critical-battery stops deliberately freeze the virtual state and are not aircraft safety algorithms. See [Stage 6 model, lifecycle and results](STAGE6_VEHICLE.md).

## Stage 7 independent planning boundary

`planning/geometry.py`, `mission.py` and `routes.py` produce versioned civilian-survey mission data from local ENU polygons. They do not import vehicle physics or perception. Explicit home/start, cruise altitude, speed/acceleration limits, footprint assumptions and route legs are validated before the optional execution adapter creates Stage 6 waypoints. The existing Stage 6 engine remains the sole owner of lifecycle, control, dynamics and clock behavior.

The adapter can preview an existing-engine step and invoke its virtual stop before a polygon-boundary crossing. A nominally valid route is not proof of executable coverage. Standalone virtual return-home planning does not replace a live mission or resume an aborted engine. See [Stage 7 planning, estimates and failures](STAGE7_PLANNING.md).

## Stage 8 fleet orchestration

`drone_sim/fleet/` composes complete Stage 7 missions with independent Stage 6 engines. One coordinator advances and passes an authoritative immutable clock to all members, including paused and failed vehicles. Derived fleet status exposes individual state counts and failures; commands remain in-process and virtual only. Polygon supervision is a thin check around existing engine steps.

Static route overlap and synchronized swept proximity are advisory records only; they cannot modify controls or mission plans. Per-vehicle telemetry and expanded fleet configurations support deterministic replay on a fixed runtime. Planning, physics and perception retain their independent contracts. See [Stage 8 lifecycle, replay and measured results](STAGE8_FLEET.md).

## Stage 9 local mission-control transport

`mission_control/` is an optional FastAPI/WebSocket package outside the unchanged simulation packages. A single owner serializes configuration, authorized tick-boundary commands and playback pacing around the existing fleet coordinator; only the fleet advances simulation time. Versioned envelopes separate exact simulation truth, absent tracking estimates and advisory warnings. Bounded viewer queues favor current snapshots while full recordings retain deterministic replay inputs/results.

The provided server is authenticated and loopback-only with one worker; clients cannot set physics or vehicle state. Playback pause differs from the existing virtual mission pause. See [Stage 9 API, authorization, results and limitations](STAGE9_BACKEND.md).

## Stage 10 local presentation boundary

`dashboard/` contains a separate Next.js/TypeScript client and loopback Node bridge. The browser renders the Stage 9 versioned truth/advisory envelope, tracks bounded sequence-aware display history and submits authorized REST requests through a local session. Backend control/view credentials never enter browser bundles. The unchanged Python service, fleet coordinator, vehicle engines and shared clock remain authoritative.

The initial map is an SVG projection of local east/north meters with up displayed separately. It consumes normalized existing mission geometry, not generated routes or invented latitude/longitude. Geographic integration would require an explicit surveyed origin and a separate projection adapter. No terrain or Gazebo contract exists. See [Stage 10 authentication, UI behavior and validation](STAGE10_DASHBOARD.md).

## Stage 11 coherent reads and release orchestration

The existing backend now exposes view-authenticated `GET /v1/configurations/active`, returning a detached normalized config and matching run/epoch under its service lock. The dashboard's session-protected `/api/plan` proxies this read; its old transient plan cache is removed. Status, snapshot and heartbeat reads share the publication lock so final asynchronous recording cannot expose a finished status with prior-tick truth. The authoritative clock and simulation algorithms are unchanged.

`scripts/skyview.py` owns only local process startup, credential configuration and shutdown. `mission_control.release_check` compares real transport/recording output against independent execution of the original simulator. See [Stage 11 release evidence and limitations](STAGE11_INTEGRATION.md).

## Stage 12 optional simulator adapter boundary

`simulator_adapter/` defines a separate version-1 read-only snapshot protocol, a delegating numerical wrapper and a packet-injection mock. The existing server still directly uses the numerical fleet engine. Adapter snapshots carry explicit provenance, ENU units, source/receipt times, identity, freshness and availability; they are not the existing WebSocket schema. A future PX4/Gazebo transport must qualify clock/origin mapping and capabilities before backend integration. See [Stage 12 environment, mappings and untested assumptions](STAGE12_SIMULATOR_ADAPTER.md).
