# Stage 1: deterministic target and camera observations

The standalone `drone_sim` package implements Stage 1 of the development plan. It requires Python 3.9 or later and only the standard library. Run commands from the repository root; no installation, camera, Unity, network, or hardware is needed.

```sh
python3 -m unittest discover -v
python3 -m drone_sim --steps 100 --dt 0.02
python3 -m drone_sim --steps 100 --dt 0.02 --output /tmp/drone-stage1.jsonl
```

The CLI emits JSON Lines to stdout or the requested file. Invalid time steps or step counts exit with an error. Output files are overwritten when explicitly selected. Configuration can also be constructed through the Python API:

```python
from dataclasses import replace
from drone_sim import Target, default_scenario, simulate

scenario = replace(default_scenario(), target=Target((0., 20., 5.), (1., 0., 0.)))
for frame in simulate(scenario):
    print(frame.tick, frame.truth.position_enu_m, frame.detections)
```

## Motion and clock

`simulate` yields tick zero and then exactly `steps` updates, with `p_next = p + v*dt` and unchanged velocity. Timestamps are `tick*dt`, independent of execution speed, system clock and sleeping. Records are frozen dataclasses using tuple coordinates. A run does not change the scenario and can be replayed. Floating-point integration has small rounding error; diagnostics compare against the independent analytical trajectory `p0 + v*t`.

Stage 1 has no randomness, so no seed is needed. Seeded measurement noise/dropouts and other failure scenarios remain deferred to Stage 3. Numerical replay is tested on the same Python runtime; bit-for-bit equivalence across arbitrary Python/platform versions is not asserted.

## Calibration and independent cameras

World coordinates are local right-handed ENU meters. Optical coordinates are right/down/forward. `Camera` stores its ENU origin, intrinsics and three optical unit basis vectors expressed in ENU (rows of `R_camera_world`). Validation rejects nonorthonormal or reflected bases. `look_at` derives these vectors from an aim point and up hint; degenerate/parallel inputs are rejected. Direct basis construction supports arbitrary calibrated orientations, including roll.

The calibrated transform is `q = R_camera_world * (p_world - camera_origin)`. Projection uses `u=fx*q.x/q.z+cx`, `v=fy*q.y/q.z+cy`. Focal lengths/principal point are pixels. This is an ideal distortion-free pinhole camera; there is no lens distortion correction or occlusion model in Stage 1. Validity requires positive depth and `0 <= u < width`, `0 <= v < height`. Points behind/on the camera plane or outside the image yield `pixel=null`, `valid=false` and a reason. A centered point remains valid.

Default calibration: 640×480, fx=fy=400 px, principal point (320,240). Cameras sit at ENU (-5,0,2) and (5,0,2), both looking north. The baseline is 10 m. Target starts at (0,20,5) with velocity (0.5,0.25,0.1) m/s. At tick zero the detections are (420,180) and (220,180): disparity 200 px agrees with `focal_length * baseline / depth`. Each camera computes its own transform/projection and validity, so a point can be visible in one and absent in the other. Both observe the same truth at the same simulation time.

## Record and diagnostic contract

The first JSONL row is a `scenario` record containing schema version, coordinate/time conventions and the complete scenario: target initial state, camera positions/bases, intrinsics and calibration version, step count and dt. This supplies reproducible calibration and static camera poses without duplicating them per frame.

Each `frame` contains integer tick, simulation time, separate ground-truth target state, and two `Detection` records. Detection fields are camera ID, frame sequence, capture time, time domain, image dimensions, synthetic target ID, pixel center, validity, rejection reason and schema version. The known synthetic target ID is ground-truth association, not evidence of an implemented association algorithm. These are point observations; no artificial bounding box, COCO class or confidence score is fabricated. A detector adapter can extend the schema later.

The final `summary` reports frame count, valid observation counts per camera, final truth and maximum absolute coordinate error against analytical motion. There are no state estimates or estimator error metrics yet. No filter, triangulation, legacy UDP connection, aircraft control, rendering, website or terrain is implemented.

## Verification results

Local validation: Python 3.9.6, 2026-09-26.

- `python3 -m unittest discover -v`: **15 tests passed**, covering analytical 3D motion for 1,000 updates, stationary/zero-step runs, timestamps, center/sign conventions, distinct intrinsics, translated/rotated/tilted cameras, boundaries, behind-camera rejection, analytic stereo parallax, independent camera visibility, invalid inputs, diagnostics and CLI output/replay.
- CLI replay: separate subprocess runs produce identical JSONL bytes; file output matches stdout. No clock or random generator influences observations.
- Default diagnostic run: 101 frames, 101 valid observations for each camera; maximum motion coordinate error `9.947598300641403e-14` m. Final position approximately (1,20.5,5.2) m at t=2 s.
- Existing tracker: SHA-256 manifest of all 42 files matched the pre-Stage-1 commit after accounting for Git's existing CRLF normalization; Git diff against the baseline is empty. No original tracker files were edited.
- Both legacy Python scripts were syntax-parsed. No legacy automated test suite exists; inference, firmware compilation and Unity execution remain outside the available environment and Stage 1 scope.

GitHub Actions is configured to run the same standard-library test suite on Python 3.9 and 3.12 for pull requests targeting dev. Local results above do not claim a hosted CI result.
