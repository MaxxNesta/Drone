# Tracker audit

> Publication update (2026-09-26): After the audit, the owner authorized replacing the Wi-Fi SSID/password in both firmware files with placeholders, deleting the local ZIP, and committing/pushing the project. Preservation and no-commit statements below describe the original audit snapshot. All 42 extracted files remain present; only firmware credential values were intentionally changed after that audit.

Date: 2026-09-26. Scope: supplied archive, static source review and isolated standard-library checks. Original code remains unchanged.

## Provenance and coverage

The supplied file is `nodetracking.zip` (not `note tracking.zip`). Its outer `nodetracking/` directory was removed during extraction into `existing-tracker/`. All 42 extracted files were compared byte-for-byte with their ZIP entries and match. Archive SHA-256: `86879442f2fdfd34e901c2b6ba89b134dfda17bf5b2e9ceb51b23adacc4c010e`.

The workspace initially contained only the archive and no Git metadata. A repository was initialized on the unborn `dev` branch; no commit or push was made.

| Area | Inspected contents and role |
| --- | --- |
| Python | Both complete scripts: `YOLOProcessing.py` (capture, detection, image-space filter, UDP/UI) and `esp32_simulator.py` (two virtual pan/tilt nodes). |
| Firmware | Complete `ID0.txt` and `ID1.txt`; differ only in telemetry ID and tilt output sign. Arduino-style source, not ready-to-build sketches. |
| Unity | All eight C# scripts: camera navigation, ray visualization, GPS grid, tooltip/control UI, triangulation, node presentation, node state, UDP ingestion. |
| Setup | README, requirements, Windows launcher, `.gitattributes`, archived Claude settings. No AGENTS.md, test suite, dependency lockfile, firmware board configuration, Unity scenes/prefabs/package manifest or project settings. |
| Assets | One Fusion CAD file, ten STL files, ten JPGs, three MP4s, one model checkpoint inventoried and integrity checked. CAD fit, video performance, image content and model tensors were not evaluated; these are not executable verification evidence. |

The README is a brief demonstration overview, not a reproducible setup/calibration guide. Its media folder spelling differs from the archive. The launcher is Windows-only, starts the simulator separately, and leaves it running on tracker exit. The model path is relative to the working directory. No project license file is supplied; reuse/distribution terms need clarification before distribution.

## What the algorithms actually do

### Detection and image tracking

`Software/Python/YOLOProcessing.py:191` resizes a frame to 640×480, invokes `model.predict` with `imgsz=320` and confidence 0.40, optionally filters class IDs, then selects the highest-confidence bounding box. `xywh` supplies its pixel center. This matches the documented detection API. It does **not** invoke an identity-preserving tracker: no ByteTrack, SORT, BoT-SORT, appearance association, IoU matching or persistent object IDs exist. Each new winner updates the same filter, even when it is another object. See [Ultralytics detection](https://docs.ultralytics.com/tasks/detect/) and [tracking API](https://docs.ultralytics.com/modes/track/).

The nominal 50 ms inference interval is a throttle, not a measured 20 FPS guarantee. Both controllers run inference serially against one shared stream; frames can change between their reads. Resizing arbitrary aspect ratios to 4:3 distorts geometry. No drone-specific training data, metrics, calibration images or validation set is supplied. The checkpoint name suggests YOLOv8 nano, but its actual classes/provenance were not loaded or verified.

### Kalman filter

`YOLOProcessing.py:46` uses state `[x,y,vx,vy]`, position-only observations, a constant-velocity transition with elapsed wall time, `Q=0.01I`, `R=I`, and 0.40 s lookahead. The state dimension and transition are internally consistent for pixel positions and pixels/second. Prediction is followed by correction when detections exist; after 0.50 s without a detection, velocity is zeroed and the public prediction returns image center.

There is no measurement-based initialization or reset of covariance on reacquisition. OpenCV initializes state and covariance to zero. On an immediate first predict/correct, the position gain is `0.01/1.01`, so a first measurement `(320,240)` yields approximately `(3.17,2.38)`, a large false off-center command. More predictions before acquisition change this example. This is a mathematical consequence of the source, not an executed OpenCV result. The extra `statePost=statePre.copy()` is redundant: OpenCV prediction already propagates state and covariance; it is not a missing-covariance bug. See [OpenCV implementation](https://raw.githubusercontent.com/opencv/opencv/4.x/modules/video/src/kalman.cpp).

Fixed Q per UI iteration makes tuning depend on loop rate, despite dt-dependent motion. Wall time is not monotonic; there is no dt bound, innovation gate, confidence-dependent covariance, camera-motion compensation or uncertainty output. Target switching and reacquisition inherit the old state. Image velocity combines target motion and camera motion; it is not world velocity.

### Motor control and sensors

`Firmware/ID0.txt` and `ID1.txt`, `loop()` and `handleMotor()`, implement **PD**, despite PID comments: `Kp*error + Kd*delta_error/dt`, with no integral term. Errors inside ±0.1 stop tracking motion. Nonzero output selects direction and a truncated speed clamped to 5–17 RPM, followed by two blocking steps. Pan/tilt execute serially with sensor/network work, so configured RPM is not guaranteed sustained shaft speed. Derivative response to intermittent packet steps can saturate; no derivative filter exists.

Pitch comes from accelerometer gravity geometry; the gyro reading is unused. Heading applies hard-coded calibration, axis remapping, tilt compensation, a 180° correction, and a wrap-aware low-pass filter. Circular heading smoothing handles north crossing sensibly. Pitch is not filtered despite the `pitch_filt` name. Acceleration, motor magnetic interference, per-device mounting/calibration and magnetic-versus-true north alignment remain unverified. GPS fix validity/age and altitude are not transmitted. Motor direction and gear ratio require a physical calibration; opposite tilt signs in the two firmware files may be intentional mounting compensation.

### Unity localization

`MultiNodeTriangulation.CalculateLinearLeastSquares` correctly assembles `A = Σ(I-ddᵀ)`, `b = Σ(I-ddᵀ)o`, and solves `Ap=b` for unit forward directions. This minimizes perpendicular distances to **infinite lines**. A determinant rejection plus subsequent forward-ray/cone checks provide basic validation. This is a geometric position estimate, not temporal target tracking.

It cannot establish that cameras see the same object. It uses camera boresights, not actual calibrated target-bearing rays, and receives no image detections. Accuracy therefore depends on accurate centering, pose calibration and synchronized measurements. There are no residual/covariance estimates, robust outlier rejection or conditioning checks beyond a fixed determinant threshold. Range is checked only against the first node. A 30° acceptance cone is permissive and is not a measurement-noise model.

## Findings by priority

Paths below are relative to `existing-tracker/`; method names identify the relevant source when no line is given. Confirmed means visible in supplied code; physical consequences remain untested.

| ID / priority | Evidence | Finding and consequence |
| --- | --- | --- |
| A01 / High | `YOLOProcessing.py:75,178`; firmware incoming control; simulator `_physics_loop` | Centered and absent targets both produce zero error. Sustained center triggers search, moving away from a correctly centered target. Confirmed. Future protocol needs explicit target validity. |
| A02 / High | Both firmware files, timeout and search motor branch | Timeout zeroes errors but does not clear `isSearching` or disable. A searching node continues panning after communication loss. Python shutdown sends no disable. Simulator timeout independently leads to search. Confirmed. |
| A03 / High | Both firmware files, `char packetBuffer[255]`, `udp.read(...,255)` | When 255 bytes are read, `packetBuffer[len]=0` writes one byte out of bounds. Numeric input also lacks strict parsing, bounds and finite-value checks. Confirmed source defect; not exercised on hardware. |
| A04 / High | `YOLOProcessing.py:46–79` | Unseeded Kalman state biases initial tracking; stale position/covariance survives loss. See first-update calculation above. |
| A05 / High | `YOLOProcessing.py:213–217` | Highest-confidence selection switches identity with confidence ordering, including across classes by default. Two nodes cannot prove common-target association. |
| A06 / High | `VideoStream._update/read`, `DroneController.process` | Capture failure retains the last frame. Re-detection can continually refresh target age from stale imagery. If no frame exists, the detection-expiry branch is skipped. No capture timestamp or sequence exists. |
| A07 / High | `main`, shared stream; simulator physics | Both nodes view the same source; changing virtual heading never changes pixels. No camera projection/rendering feedback exists. This is a transport/visualization demo, not a closed-loop geometric simulation. |
| A08 / High | `MultiNodeTriangulation.Update`; `NodeManager_Ascii` | Searching nodes are explicitly included in triangulation. Received nodes never expire, so stale poses/modes remain eligible indefinitely. False 3D fixes can look valid. |
| A09 / High | `NodeManager_Ascii.GpsToUnity`; telemetry format | Every node gets y=0. No surveyed height, altitude, camera extrinsics or target pixel ray exists. This can substantially bias elevated-target localization. |
| A10 / Medium | Python config; `HoverTooltipUI.systemConfig`; firmware UDP output | Python sends localhost 3333/3334; Unity buttons default to two LAN IPs on 3333; firmware sends telemetry to a fixed LAN host. Defaults do not describe one coherent deployment. |
| A11 / Medium | Simulator initial GPS; Unity origin defaults | Simulated nodes are in San Francisco while Unity's origin is near Los Angeles: roughly 420 km of offset, outside the default 4 km grid and with poor float precision/local-map assumptions. |
| A12 / Medium | Simulator `_physics_loop`; Unity default `invertPitch=true` | Positive image-up error decreases simulated pitch. Unity then applies positive Euler X, pointing forward downward. This sign mismatch is confirmed for supplied defaults, independently of unknown physical mounting. |
| A13 / Medium | `CameraRigController.cs:4` | File declares MonoBehaviour `FlyMover`, not `CameraRigController`; Unity script attachment/import requires correction in a future working copy. No complete Unity project is provided to verify scene setup. |
| A14 / Medium | Python sender, shutdown and capture threads | Unsynchronized filter reads/writes, unjoined daemon threads, release/read races and swallowed socket errors impede consistency and diagnosis. Mode flags are optimistic local state; no ACK reconciles Unity and Python commands. |
| A15 / Medium | Both firmware files, setup/state transitions | No homing, limit-switch logic or travel bounds; Wi-Fi connection can wait indefinitely; brownout detection is disabled. Search state and derivative history are not consistently reset across modes. Physical operation needs separate verification. |
| A16 / Medium | Simulator receive/state loops | Accepts unclamped/non-finite floats and errors while disabled; three unsynchronized threads, wall-clock dt, 0.05 search deadband and 300 ms timeout differ from firmware (0.01 search, 0.1 motor deadband, 200 ms). It is not a fidelity model of firmware PD/stepping. |
| A17 / Medium | `NodeManager_Ascii.OnReceive/TryParseYourFormat` | Unbounded queue/node creation, no age/sequence/source validation, no finite/range checks; arbitrary mode strings become Searching. Malformed or excessive traffic can corrupt visualization or exhaust resources. |
| A18 / Low | `HoverTooltipUI`; `GpsGridDrawer`; visualization classes | Button has manual and EventSystem click paths (possible duplicate sends); missing main camera can dereference null; grid spacing ≤0 can hang. Per-frame object discovery allocates; generated tooltip/triangulation objects/materials lack complete lifecycle cleanup. |
| A19 / Medium | Firmware Wi-Fi config; requirements/setup | Credentials are embedded in both firmware files (values intentionally omitted here). Dependency lower bounds only, no environment/board/Unity versions or reproducible build instructions. Configuration should be external in future copies. |

## Reusable components

- Detection-to-box-center extraction and display overlays, once separated from process startup and configuration.
- Four-state constant-velocity filter structure, after explicit initialization, monotonic timing, gating and uncertainty handling.
- Latest-frame capture concept, after adding freshness metadata, resource ownership and paced video replay.
- Compact node telemetry and simulator as a **legacy protocol fixture**, not as a drone model.
- Wrap-aware heading smoothing and sensor-axis calibration structure; constants require per-device validation.
- Least-squares ray geometry as a baseline, with robust validity/conditioning checks and calibrated bearings.
- Unity GPS map, node status and ray visualization concepts after scene packaging and coordinate cleanup.

`DroneController` is a camera-node controller by behavior. No thrust, drone dynamics, flight state estimator, navigation or flight controller exists.

## Verification performed and limits

1. Parsed both Python files with `ast.parse`: pass, without importing them or writing bytecode.
2. Checked every extracted file against the archive: 42/42 exact matches.
3. Compared firmware variants: only telemetry ID and tilt sign differ.
4. Executed the original simulator class's physics method through AST extraction, bypassing constructor/socket creation, with a fake clock and 60 iterations of 20 ms. Active fresh zero commands entered search and advanced heading 10.2°. Stale commands also entered search and advanced 10.2°. A fresh positive unit vertical error produced pitch −29.5°. These isolate logic, not scheduling/network performance.
5. Reviewed detection APIs against official Ultralytics docs and filter initialization against OpenCV source. Derived first-update behavior algebraically; OpenCV was not executed.
6. Checked availability: `cv2`, `numpy`, and `ultralytics` are absent. Installed nothing; did not deserialize the model, start cameras, transmit motor commands, compile firmware or run Unity.

No detection accuracy, FPS, filter convergence, physical motor stability, sensor accuracy or end-to-end triangulation accuracy is claimed verified. Full validation requires a controlled environment, labeled footage, complete Unity scene, board/library versions and hardware or calibrated simulated observations.
