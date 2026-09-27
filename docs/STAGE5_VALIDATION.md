# Stage 5: local recorded-video validation

PR #4 was reviewed (implementation/contracts and all 98 tests), then merged into `dev` at `c147563` with the owner's authorization. Stage 5 uses the new `stage5-recorded-video-validation` branch. All original tracking files and Stage 1–4 source files remain unchanged; new tools reuse Stage 4's decoder, YOLO adapter, observation contract and tracker. No hardware, network control, 3D estimation or graphics application is connected.

**Execution succeeded on the existing demonstration video using the existing checkpoint. Detection accuracy and identity accuracy remain unvalidated because no manual ground truth was supplied.** No labels were invented. The user explicitly selected the bundled demonstration after its content was inspected.

## Input inspection

Source: `existing-tracker/Media/Demo Video/CameraTracker.mp4`.

- H.264 video, 1920×1080, 30 frames/s, 1,540 frames, 51.333333 s container duration; AAC audio is present in the input.
- PTS spans 0–51.3 s. Original image dimensions and timestamps are preserved in telemetry.
- Content includes hardware close-ups, code/application screens, two video panels showing people holding a drone, and simulator views. It is a composed demonstration/screen recording, not a clean independent camera validation dataset. Two panels within this single recording are never treated as stereo cameras.
- Source SHA-256: `c5d7b9e27a89d705bfbadf8aec469c3b99ec3e1a4df9ee13efcde73ab5b076b3`.
- Two small prototype clips were inspected at representative frames. They show hardware on a desk or held in a hand and offer little additional target-validation coverage; they were not benchmarked.

Checkpoint: `existing-tracker/Software/Python/yolov8n.pt`, SHA-256 `f59b3d833e2ff32e194b5bb8e08d211dc7c5bdf144b90d2c8412c47ccfc83b36`. It loads successfully as a YOLOv8 detection model with COCO's 80 classes. There is **no drone class**. A detection on or near a drone is not evidence that this checkpoint recognizes drones. No weights were downloaded, replaced or modified.

## Reproducible environment and commands

Actual environment: macOS 26.5.2, arm64, Python 3.12.14. CPU inference, FP32, PyTorch intra-op threads 1, seed 0, deterministic algorithms enabled. Key packages: Ultralytics 8.3.200, PyTorch 2.8.0, torchvision 0.23.0, PyAV 15.1.0, OpenCV 5.0.0.93 and NumPy 2.5.3. Every resolved Python dependency is pinned in [requirements-vision-lock.txt](../requirements-vision-lock.txt). `pip check` reported no broken requirements.

The local `.venv-vision/`, `.vision-cache/` and generated `artifacts/stage5/` are ignored by Git. No core dependency requirement changed. The lock records the tested macOS/arm64/Python environment; availability of the same wheels on other platforms is not guaranteed. OS codecs, CPU load and package build differences can change results or timing.

```sh
python3.12 -m venv .venv-vision
.venv-vision/bin/python -m pip install -r requirements-vision-lock.txt
.venv-vision/bin/python -m pip check

.venv-vision/bin/python -m drone_sim.vision.validate \
  --video 'existing-tracker/Media/Demo Video/CameraTracker.mp4' \
  --weights existing-tracker/Software/Python/yolov8n.pt \
  --camera-id demo-recording --output-dir artifacts/stage5/baseline

.venv-vision/bin/python -m drone_sim.vision.validate \
  --video 'existing-tracker/Media/Demo Video/CameraTracker.mp4' \
  --weights existing-tracker/Software/Python/yolov8n.pt \
  --camera-id demo-recording --suppress 21:21.2 --suppress 22:23 \
  --output-dir artifacts/stage5/observation-loss
```

Use a **new output directory** for each run; the tool refuses to overwrite an existing directory. No source/model is downloaded. `YOLO_OFFLINE=true` is set for the validation process, and model loading requires an existing path. Local runtime caches live outside `existing-tracker/`. Package installation is the only requested network-dependent preparation.

The new `vision.validate` runner calls the existing `video_frames`, `YoloDetector.detect` and `ObservationTracker.consume`. It adds timing, export and explicit interventions; it does not replace their detection or tracking algorithms. Model configuration uses explicit CPU/FP32 prediction overrides. [Ultralytics documents prediction options](https://docs.ultralytics.com/modes/predict/); [PyAV documents timestamp/time-base handling](https://pyav.org/docs/stable/api/time.html). Installed-version behavior was exercised by real decoding/inference/encoding, not inferred only from documentation.

## Exported artifacts

Local output under `artifacts/stage5/baseline/`:

- `annotated.mp4`: silent H.264 at original resolution and presentation intervals, with detection boxes, confidence/class names, assigned IDs, tracking states and predicted coasting/stale markers. The first output timestamp is normalized to zero. Source audio is deliberately omitted.
- `telemetry.jsonl`: one version-2 observation and tracking result per frame, raw detector evidence, intervention marker, per-stage timing and lifecycle transitions.
- `report.json`: exact configuration, software metadata, source/model hashes, measured summaries and artifact hashes.
- `report.md`: human-readable generated evaluation report.

A corresponding set exists in `artifacts/stage5/observation-loss/`. Compact reports and verification results are committed under [diagnostics/stage5/](diagnostics/stage5/); full video and telemetry remain local rather than adding generated media to Git. Reproduction commands regenerate them from the already tracked source/checkpoint.

Overlay IDs such as `c0/track-16` abbreviate `0:demo-recording:track-16` for this single camera. Green boxes are detections; yellow crosses are predictions without accepted observations. A status panel shows up to ten inactive tracks and reports overflow; full states remain in telemetry. Dense/overlapping detections can still clutter labels. Scene cuts are not recognized automatically, so old tracks can remain visible until their timeout. There is no claim that any assigned ID is a verified real identity.

## Measured baseline

The final baseline processed all 1,540 frames. These are single-run measurements on the stated computer, not a deployment guarantee or controlled hardware benchmark.

| Measurement | Result |
| --- | ---: |
| Export throughput (includes overlay and encoding) | 27.294 frames/s |
| Decode + inference + tracking latency, mean | 23.726 ms |
| Decode + inference + tracking latency, p95 | 31.847 ms |
| Decode + inference + tracking latency, maximum | 155.744 ms |
| Detection/conversion latency, mean | 14.733 ms |
| Tracking latency, mean | 0.256 ms |
| Per-frame pipeline including rendering/encoding, mean | 36.392 ms |
| Raw detections | 1,169 |
| Frames with multiple predictions | 352 |
| Frames with no predictions | 1,010 |
| Distinct assigned local IDs | 244 |
| Manual labeled frames | 0 |
| Precision, recall, identity accuracy | **Not measured** |

Throughput divides frame count by wall time around the complete serial run, including JSON serialization, overlay/encoding and encoder flush. It excludes imports/model construction and report generation. Separate `model_setup_s` measures model construction only, not imports. Per-frame `total` measures decode, inference/conversion and tracking; `pipeline_with_render` additionally includes overlay/encoder calls but excludes JSON writing and final encoder flush. First-frame costs are retained, with no discarded warm-up. Decode includes source hashing/opening on the first frame. CPU inference avoids asynchronous accelerator timing ambiguity. These are processing times, not physical exposure-to-result latency. The observed export rate is below the source's 30 frames/s; this offline runner does not drop frames to keep pace.

The model emitted predictions for person, suitcase, sports ball, chair, TV, laptop, keyboard, cell phone and book. Inspection shows predictions over hardware and screen content, as well as people in displayed footage; these are qualitative observations only. The 244 IDs expose substantial track creation, but cannot be converted into an identity-switch count without labels. A fixed 30 px association gate and the unchanged 1 px² measurement variance are not tuned for 1920×1080 footage, scene cuts or screen-recorded motion. No thresholds were fitted to obtain better-looking results.

The score file explicitly marks `blocked_no_manual_annotations`; accuracy/continuity values are null. Zero TP/FP/FN counters in the underlying Stage 4 scorer mean zero **scored** samples, not zero errors. The demonstration establishes successful pipeline execution, not validated detection quality, real-camera tracking reliability or drone recognition.

## Loss, multiple detections and occlusion boundary

The baseline naturally exercises multiple simultaneous predictions and intervals without detections. Both tracking and coasting/stale/lost states appear. The second run deliberately suppresses estimator observations for half-open elapsed intervals `[21,21.2)` and `[22,23)` seconds: 6 and 30 frames. YOLO still executes on the unchanged decoded frame, and its outputs remain in `raw_observation`; only the effective observation supplied to tracking is empty. Telemetry and video explicitly mark `synthetic_observation_suppression`. If labels are supplied later, scoring uses effective observations after the intervention; use the baseline for detector-accuracy evaluation.

In the short interruption, existing tracks coast and some resume tracking when measurements return at frame 636. During the long interruption, tracks become stale, expire, and new IDs are allocated when observations return at frame 690. This verifies lifecycle handling with real detector output under a controlled measurement-loss intervention. Fewer allocated IDs in the fault run does not indicate better tracking: detections were deliberately withheld.

This is **not** a physical or pixel-level occlusion experiment. No reliable natural-occlusion ground truth exists for the supplied montage, and a measurement gap cannot distinguish occlusion from model failure or target absence. Physical occlusion robustness and correctness of reacquired identities remain unvalidated. The original Stage 3 synthetic occlusion tests remain intact, and new tests verify the explicit Stage 5 suppression boundary.

## Manual labeling preparation

Fifteen original-resolution, unannotated PNGs and an unfilled template were generated locally:

```sh
.venv-vision/bin/python -m drone_sim.vision.annotations prepare \
  --telemetry artifacts/stage5/baseline/telemetry.jsonl \
  --video 'existing-tracker/Media/Demo Video/CameraTracker.mp4' \
  --frames 0,300,600,601,602,630,631,632,660,661,662,900,1200,1201,1202 \
  --output-dir artifacts/stage5/manual-labeling
```

The committed [template](diagnostics/stage5/manual_template.json) has `objects: null` and `reviewed: false` for every frame. It cannot be converted to ground truth until all selected frames are explicitly reviewed. [Labeling instructions](diagnostics/stage5/LABELING.md) cover independent box entry, COCO classes, screen-instance policy, empty frames, stable IDs and contiguous sequences for continuity evaluation. No detector boxes or invented identity labels were copied into it. Sparse representative frames alone cannot establish whole-video or occlusion/identity accuracy. The montage supports exploratory labeling, but a representative deployment dataset remains unavailable.

## Tests, warnings and limitations

Complete test discovery:

- Python 3.9.6, standard library environment: **108 discovered, 107 passed, 1 optional codec test skipped**.
- Python 3.12.14 in `.venv-vision`: **108 passed**, including real PyAV H.264 encoding/decoding with variable presentation intervals and visible overlays.
- All original 98 tests are preserved. New regressions cover measured time accounting, suppression lifecycle, retained raw evidence, multiple/empty observations, decoder cleanup on export failure, invalid intervals/statistics, unfilled-template rejection, manual-label conversion and actual codec round trips.
- Full real-video export has 1,540 frames, original resolution and 51.333333 s duration. Full per-frame timestamp verification and deterministic detector replay checks are recorded in the committed verification JSON.
- No legacy files or previous-stage source files were changed. No model download/replacement, webcam access or hardware integration occurred.

Observed runtime issues: the initial run warned that its not-yet-created YOLO cache directory was unwritable and fell back to `/tmp`; the runner now creates its isolated cache directories before imports. PyAV and OpenCV wheels emit macOS Objective-C duplicate `AVFFrameReceiver`/`AVFAudioReceiver` class warnings because both bundle FFmpeg device libraries. No crash occurred in the file-based runs or codec tests. The warnings remain an environment limitation; library binaries were not edited and webcam behavior is not validated by this stage.

Other limitations: no annotation accuracy metrics, no true 3D/calibration/clock validation, no interpretation of displayed two-panel footage as stereo, no audio export, no scene-cut reset, no appearance-based re-identification, no calibrated uncertainty/thresholds for this model, bounded-session in-memory telemetry aggregation, no guaranteed cross-platform bitwise or timing reproducibility. The current export codec requires compatible dimensions/formats; odd-sized inputs and changing resolution are not supported by the tested H.264/YUV420 path. Subsequent work should obtain independently reviewed labels and representative footage before changing algorithms or making detection/identity performance claims.
