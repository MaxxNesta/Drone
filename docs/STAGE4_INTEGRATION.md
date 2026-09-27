# Stage 4: recorded-video integration

Stage 4 was built on `stage4-video-integration` from `dev` at `a093ad9`, after PR #3 was merged. Commits use **Kaung**. The adapter lives entirely in `drone_sim/vision/`. All preexisting Stage 1–3 Python files and `existing-tracker/` remain unchanged. No motor, UDP, aircraft, Unity or stereo connection is added.

## Original components and reuse

Inspection of `existing-tracker/Software/Python/YOLOProcessing.py` identified useful YOLO prediction, confidence/class extraction and bounding-box centroid logic. The new `YoloDetector` uses those concepts through an isolated backend, retaining all detections rather than the original highest-confidence-only selection. It does not import the original application: that script parses arguments, loads a model, manages capture/UI and starts motor-command networking.

The original four-state constant-velocity Kalman model is useful conceptually, but its zero initialization, wall-clock timing and fixed process noise are not reused. The adapter uses the measurement-initialized Stage 2 filter and Stage 3 observation association. Association is image-plane, class-partitioned, one-to-one motion matching; class IDs are detector categories, never ground-truth instance IDs.

The original forced 640×480 resize, repeated cached frames and unpaced looping at file EOF are not copied. Recorded-video decoding preserves original dimensions and presentation timestamps and terminates at EOF.

## Boundary and clocks

`ObservationFrame` is an additive **schema version 2** interface consumed by `ObservationTracker.consume()`. Stage 1's version 1 contracts remain unchanged. `from_simulation(detection, source_id)` converts valid synthetic points or empty observations into the same interface, strips truth target identity and leaves box/class/confidence unknown. No confidence or bounding box is invented for point simulations.

| Field | Meaning |
| --- | --- |
| `source_id` | Video content SHA-256; webcam device/session identity; explicitly supplied simulator source identity. Identical files renamed or assigned different camera IDs retain the same video source fingerprint. |
| `camera_id`, `frame_sequence` | Caller-supplied camera identity and zero-based decoded frame sequence. |
| `capture_time_s`, `time_domain` | Original video PTS seconds (`video_pts`), fixed simulation seconds (`simulation`), or host time immediately after webcam read (`monotonic_receive`). |
| `timestamp_provenance`, `pts`, `time_base` | Explain timestamp origin; preserve integer video PTS and rational time base exactly. |
| `image_size` | Original width and height; changing resolution within a tracker is rejected. |
| `observations` | Unlabeled pixel centroids, optional xyxy boxes, confidence and class IDs. Empty means no detections. |

Pixels are original-image coordinates, x right and y down. Confidence is the detector score, not calibrated correctness probability. Boxes must be finite, ordered, within the image and consistent with their centroid. Invalid records fail explicitly rather than being clipped or silently changed. Unsupported schema/clock, nonfinite timestamps and invalid dimensions are rejected.

Video timestamps are `pts × time_base`, not frame index divided by nominal FPS. Variable frame rates, negative starts and nonzero starts are retained. Missing, repeated or decreasing PTS stop decoding with an error. Container timestamps are media presentation times; they do **not** establish physical exposure time or cross-camera synchronization. Webcam support is explicit opt-in, bounded by a frame limit and labeled as receipt time, not exposure time. Failed reads are errors, never recycled frames.

Internally the bridge subtracts the first timestamp to create a nonnegative elapsed clock for the existing Stage 2/3 filter. Its internal version 1 records use that filter's simulation-clock convention only as an implementation detail; they are not exported as real-camera simulation measurements. Public records and track timestamps retain their actual source time. The image-plane bridge supplies image dimensions and center only: no focal length, world pose or calibration is invented.

Each tracker binds one source/camera/dimension/clock tuple and rejects changes. Duplicate or out-of-order observations are rejected without advancing state. Offline replay tracks at media time, independent of processing speed; slow processing does not invalidate a historical measurement. This is not a live freshness monitor.

## Tracking and diagnostics

Output JSONL separates `observation`, `tracking`, `timing_ms` and detector metadata. Raw detection state (`detected` / `no_detection`) is separate from per-track lifecycle:

- `tracking`: a measurement was associated at this timestamp.
- `coasting`: prediction without a fresh measurement, age at most 0.25 s.
- `stale`: measurement age above 0.25 s, at most 0.75 s.
- `lost`: age above 0.75 s; the track is retired and later detections receive new IDs.

`centered` is independent of lifecycle. Outputs include filtered pixel, pixel velocity, measurement age, observation-index-to-track-ID assignments, rejection reasons and state transitions. Lifecycle advances when records arrive, including empty frames; no artificial end-of-video timeout records are generated.

Association retains Stage 3's 30 px distance and squared Mahalanobis 16 gates, default 1 px² measurement variance and 32 active tracks **per class**. At most eight observations per class are tracked per frame, ranked by confidence with original-index tie breaking; excess observations remain in raw output and are explicitly rejected from tracking as `observation_capacity`. Detection scoring still includes them. These default thresholds are a baseline, not calibrated to a real detector. Category flicker, abrupt motion, occlusion, crowded crossings and immediate track births can fragment identity. There are no appearance features or persistent re-identification. Long recordings can grow bookkeeping memory; bounded-session replay is the supported use case.

## Running

Core, replay, scoring and tests require only Python 3.9+ and the standard library:

```sh
python3 -m unittest discover -v
python3 -m drone_sim.vision replay \
  --input tests/fixtures/vision/observations.jsonl --output /tmp/stage4-replay.jsonl
python3 -m drone_sim.vision evaluate \
  --input /tmp/stage4-replay.jsonl \
  --annotations tests/fixtures/vision/annotations.jsonl \
  --output /tmp/stage4-report.json
```

For actual video inference, install `requirements-vision.txt` in a separate environment only when needed. It contains optional Ultralytics and PyAV dependencies, not core requirements. The ranges are not a validated lockfile. No optional dependencies were installed for this implementation. Select a Python/runtime combination supported by the installed packages and retain its versions with evaluation results.

```sh
python -m pip install -r requirements-vision.txt
python -m drone_sim.vision run --video /path/to/recording.mp4 \
  --weights /path/to/local-detection-model.pt --camera-id camera-A \
  --confidence 0.4 --image-size 320 --output /tmp/video-observations.jsonl
# Optional webcam, only when explicitly selected; OpenCV is needed (also used by Ultralytics).
python -m drone_sim.vision run --webcam 0 --max-frames 300 \
  --weights /path/to/local-detection-model.pt --camera-id camera-A \
  --output /tmp/webcam-observations.jsonl
```

There is no default camera or model download: an existing local weights file is required. Detector metadata records weight SHA-256, class names, confidence threshold, inference size, Python and installed package versions. The original bundled checkpoint is not loaded or assumed suitable for drones. Only bounding-box detection models are supported; pose/segmentation-specific outputs are not adapted. First video stream only; rotation/display metadata, deinterlacing, rolling shutter and lens correction are not implemented. Confirm orientation against annotations before using real footage. Explicit output paths are overwritten; failures may leave a partial JSONL useful for diagnostics.

The backend follows [Ultralytics' documented box/confidence/class outputs](https://docs.ultralytics.com/modes/predict/) and [PyAV frame presentation timestamps](https://pyav.org/docs/stable/api/frame.html). Optional backend APIs are covered by injected fakes, not an installed decoder/model integration run.

## Evaluation and results

Annotations are a separate JSONL file. Each row has `observation` (the same versioned frame contract containing ground-truth boxes/classes) and `truth_ids`, a parallel array of unique object strings. They are read only by `evaluate`, never by the tracker. Source fingerprint, camera ID and sequence join frames; timestamps, dimensions and clock domain must agree. Duplicate annotations/results, out-of-order results and missing labeled frames fail. Unlabeled result frames are counted separately and excluded from accuracy metrics. A fully labeled recording, including empty frames, is required to claim whole-recording precision/recall.

Detection matching is confidence-ordered, one-to-one, same-class IoU matching at a configurable Python API threshold (default 0.5). Report TP/FP/FN, precision, recall and mean matched IoU. This is not COCO mAP. Identity switches count a matched true object changing assigned local ID, including after expiration. Reacquisitions count a matched object returning after at least one intervening frame sequence without an assigned true-positive track. This sequence-gap metric assumes contiguous decoded frames; it is not an exposure-duration or IDF1 metric. `tracked_true_positive_fraction` reports assigned tracks among correct detections, not total target visibility. Annotations cannot change association decisions.

Processing latency is measured with a monotonic clock around serial decode, inference/conversion and tracking. Output includes each component and total; scoring summarizes total mean and nearest-rank p95, including its sample count. First-frame decode includes file hashing/opening; model construction precedes measurement. Serialization, model-load time, physical exposure and camera transport latency are excluded. Media timestamps are never subtracted from the processing clock. Replay omits latency rather than fabricating a benchmark.

The six-frame **synthetic** fixture includes motion, three missing detections, a false positive and a long loss. [Committed machine-readable results](diagnostics/stage4_fixture.json):

| Metric | Fixture result |
| --- | ---: |
| Labeled frames | 6 |
| TP / FP / FN | 3 / 1 / 3 |
| Precision / recall | 0.75 / 0.50 |
| Mean matched IoU | 1.0 |
| Identity switches / reacquisitions | 1 / 1 |
| Tracked true-positive fraction | 1.0 |
| Measured latency samples | 0 (null summaries) |

These numbers test scoring and failure visibility. They are **not YOLO accuracy or real-camera performance measurements**. A fake-clock pipeline test independently verifies 5 ms total timing and original timestamp preservation without claiming hardware performance.

Local complete suite: **98 tests passed**, Python 3.9.6. This includes the existing 74 tests and 24 new tests for validation/round trips, simulator interchange, measurement initialization, moving/stationary/centered tracks, all lifecycle states, class partitioning/capacity, duplicates/order/source changes, VFR/nonzero/negative timestamps, missing/repeated PTS, YOLO extraction, missing local weights, source fingerprints, webcam failure/release, injected latency, annotation isolation, scoring errors, and deterministic CLI replay against the committed report. Optional dependencies, webcam access and weight downloads are not required. Git preservation checks confirm no changes to legacy files or existing Stage 1–3 source. CI runs the same suite on Python 3.9 and 3.12; hosted status is reported on the PR.

## Real 3D boundary

This adapter exports image-plane observations only. It does not call triangulation or fabricate depth. Duplicate footage under two camera names is still one viewpoint; the video content fingerprint makes exact copies detectable but cannot prove physical independence of edited footage. No multi-video stereo entry point exists in Stage 4.

Real metric localization requires independently positioned cameras, measured intrinsics and distortion, surveyed/extrinsic poses with uncertainty, verified image orientation, synchronized exposure times or validated clock mappings, and reliable cross-view target association. Moving mounts additionally require timestamped pose. Unknown baselines, timestamp offsets, rolling shutter and inaccurate calibration can produce plausible but wrong 3D results. A single camera generally gives a bearing, not metric depth. Existing ideal simulator calibration must never be attached to real video merely to obtain a numerical position. Labeled representative footage and independently measured 3D truth remain prerequisites for real accuracy claims.
