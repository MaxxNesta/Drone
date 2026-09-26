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

## Separate Python simulator: Stage 1 implemented

The target-motion and camera-observation layer is implemented in `drone_sim/`; see [Stage 1 usage and contracts](STAGE1_SIMULATOR.md). Tracking, localization, legacy integration and control below remain future design.

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

These are design contracts, not implemented files or a finalized wire protocol.

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
