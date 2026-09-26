# Stage 2: observation tracking and calibrated triangulation

Implemented on `stage2-tracking-triangulation`, branched from `dev` after PR #1 was merged (base `878ea65`). New commits use the repository author name **Kaung**. No changes were made to `existing-tracker/`; no third-party dependencies were installed.

## Run and reproduce

Python 3.9+ standard library only; run from the repository root:

```sh
python3 -m unittest discover -v
python3 -m drone_sim.evaluate --scenario moving --steps 500 --noise-std 1 --seed 7 --output /tmp/stage2-moving.jsonl
python3 -m drone_sim.evaluate --scenario stationary --steps 500 --noise-std 0
python3 -m drone_sim.evaluate --scenario dropout --steps 500 --noise-std 1 --seed 7
python3 -m drone_sim.evaluate --scenario degenerate --steps 500 --noise-std 0
```

The original `python3 -m drone_sim` Stage 1 command is unchanged. JSONL evaluation output has a complete configuration record, per-tick truth, input observations, filter states/covariances, triangulation acceptance/rejection diagnostics and Euclidean position errors, then a summary. `--output` overwrites the explicitly selected file. Machine-readable summaries and exact reproduction commands are committed in [diagnostics/stage2_summary.json](diagnostics/stage2_summary.json).

The public modules can be tested or called independently:

```python
from drone_sim import default_scenario, simulate
from drone_sim.tracking import CameraTracker
from drone_sim.triangulation import triangulate

scenario = default_scenario()
trackers = [CameraTracker(camera, scenario.target.target_id) for camera in scenario.cameras]
for frame in simulate(scenario):
    estimates = [tracker.step(frame.simulation_time_s, observation)
                 for tracker, observation in zip(trackers, frame.detections)]
    position = triangulate(scenario.cameras,
                           [estimate.observation for estimate in estimates],
                           frame.simulation_time_s)
    print(position.valid, position.reason, position.position_enu_m)
```

## Filter model, initialization and association

`CameraTracker` maintains a four-state image model `[u,v,du,dv]` as two independent 2×2 position/velocity blocks. This is equivalent to a separable constant-velocity Kalman filter with independent equal-variance pixel noise, not two independent object tracks. The full covariance is returned in `[u,v,du,dv]` order.

For each axis, `F=[[1,dt],[0,1]]`, `H=[1,0]`. Continuous white acceleration yields `Q=q*[[dt³/3,dt²/2],[dt²/2,dt]]`, so covariance propagation is tied to elapsed simulation time rather than loop count. Corrections use Joseph-form covariance updates. Defaults: measurement variance 1 px², acceleration spectral density 4 px²/s³, initial velocity variance 100 (px/s)².

The first valid, timely measurement sets the actual pixel position directly, velocity to zero (unknown), and the stated initial covariance. No correction pulls a newly acquired target toward image origin. After loss, a fresh observation seeds a new state, so old velocity/covariance is not carried into reacquisition.

Each tracker is bound to one camera ID and declared target ID. Wrong identity, mismatched dimensions/schema/time domain, invalid/nonfinite/out-of-frame pixels, duplicate sequence, clock reversal and incompatible timestamps are rejected. Association within this declared target is gated by joint squared Mahalanobis innovation distance, threshold 16. Rejected innovations never refresh measurement age. This is a gated single-target baseline, not appearance recognition or general multi-object association.

## State and freshness contract

The caller supplies a finite nondecreasing simulation time to `step`. One observation per camera per step is supported. Observations must be synchronous with that time (1e-9 s numerical tolerance). A delayed observation is rejected rather than incorrectly correcting a current state with a past measurement; out-of-sequence buffering/replay is not implemented. A repeated step at the same time cannot assimilate a second observation.

| Field / condition | Meaning |
| --- | --- |
| `tracking` | An observation was accepted this step. |
| `coasting` | No observation accepted, but the last accepted measurement is at most 0.25 s old; state is predicted. |
| `stale` | Last accepted measurement is older than 0.25 s but at most 0.75 s old. Prediction remains diagnostic only. |
| `lost` | Uninitialized or last accepted measurement older than 0.75 s; no pixel/velocity/covariance output. |
| `centered` | Separate boolean: predicted/updated pixel is within 3 px of principal point; None when lost. It can coexist with tracking, coasting or stale. |
| `accepted`, `reason` | Measurement outcome and explicit rejection reason, separate from lifecycle status. |
| `age_s`, `last_measurement_time_s` | Age and provenance of last accepted measurement; prediction does not refresh either. |

Each estimate also exposes an `observation` for triangulation. It is valid **only when a measurement was accepted at that step**. Coasting/stale/lost estimates never masquerade as fresh camera observations. Centering alone never means missing or triggers search behavior.

## Calibrated two-camera triangulation

`triangulate` accepts exactly two distinct calibrated cameras and detections of the same declared target. It matches by camera ID, independent of input order. `camera_ray` applies `normalize(R_world_camera K^-1 [u,v,1])`, using Stage 1's optical right/down/forward and ENU convention. Camera positions retain their height.

Two closest points on the bearing lines are computed analytically; their midpoint is the least-squares solution minimizing total squared perpendicular distance to both lines. This is equivalent to solving `A p=b`, with `A=sum(I-ddᵀ)`, without a general matrix inverse.

For unit rays d and e, A's eigenvalues are `2, 1+abs(d·e), 1-abs(d·e)`. The spectral condition number `2/(1-abs(d·e))` therefore rejects both parallel and antiparallel degeneracy. Default limits:

- Capture age ≤0.25 s; future observations rejected.
- Capture-time skew ≤0.01 s; output timestamp is their midpoint, not processing time.
- Baseline ≥0.01 m; condition number ≤10,000 (roughly a minimum 1.15° angle from parallel/antiparallel).
- Closest points lie forward of both cameras.
- Separation between closest points ≤0.25 m.
- Estimated point lies within 500 m of **both** cameras.

A rejected result has `valid=false`, an explicit reason and no position. Condition, separation and range diagnostics are retained when available. There is no arbitrary zero-position fallback. Stale/schema-invalid/different-target observations are rejected before geometric calculations. This two-camera solver is stateless: stream sequence deduplication is the tracker's responsibility; raw callers must manage stream ordering themselves.

## Measured diagnostics

Local run: Python 3.9.6. Each benchmark uses dt=0.02 s and 500 updates plus initial frame (501 frames, ten seconds); noisy benchmarks use independent per-camera Gaussian pixel noise with std=1 px and seed=7. The seed and camera IDs determine the streams. Truth is only used for observation generation and scoring, never passed into either estimator. Dropout removes observations at ticks 100–119 and 200–249 inclusive for both cameras; these small fixtures support Stage 2 validation, not a full Stage 3 failure framework.

| Scenario | Raw / filtered pixel RMSE (px) | Raw / filtered 3D RMSE (m) | Raw / filtered 3D availability |
| --- | --- | --- | --- |
| Stationary, noiseless | 0 / 0 | 7.16e-15 / 7.16e-15 | 100% / 100% |
| Moving, noiseless | 0 / 0.0397 | 3.00e-14 / 0.00227 | 100% / 100% |
| Moving, 1 px noise | 1.4193 / 0.4300 | 0.17226 / 0.05134 | 99.80% / 100% |
| Moving with dropouts, 1 px noise | 1.4055 / 0.4570 | 0.17286 / 0.05609 | 85.83% / 86.03% |
| Parallel geometry, noiseless | 0 / 0 | unavailable / unavailable | 0% / 0% |

Position RMSE uses Euclidean 3D error over **accepted positions**, including initialization. Availability is accepted positions divided by all 501 frames. Pixel RMSE uses Euclidean pixel error over valid raw observations or accepted filter updates, excluding prediction-only steps. Counts, mean and maximum errors are in the JSON artifact. Raw and filtered metrics can have different accepted sample sets; availability and rejection counts are reported alongside error to expose this difference.

One raw noisy position is rejected for excessive ray separation; all filtered noisy positions pass. The dropout case has 70 frames without either observation and never triangulates coasting predictions. Track status counts across both cameras: 862 tracking, 48 coasting, 66 stale, 26 lost. All 501 degenerate frames are rejected as ill-conditioned. `null` error with zero samples means unavailable, not zero error.

The noiseless moving filter introduces small lag: its initial velocity is unknown, and constant world velocity with changing depth does not project to exactly constant image velocity. It is not expected to outperform perfect noiseless detections.

## Tests and limitations

`python3 -m unittest discover -v`: **45 tests passed** (15 Stage 1 + 30 Stage 2). Coverage includes a hand-computed Kalman update, covariance consistency under split dt, stationary and moving targets, initialization, innovation gating, duplicate/invalid/stale/future observations, explicit lifecycle boundaries, reacquisition, calibrated rotated camera geometry, noise-free moving 3D recovery, timestamp compatibility, forward/range/residual rejection, parallel/antiparallel and nearly parallel rays, dropout availability and repeatable CLI diagnostics.

The moving-noise regression requires filtered pixel and position RMSE each below 80% of their raw counterparts and filtered availability above 95%. Noise-free moving raw 3D recovery is checked below 1e-9 m; noise-free stationary filtered recovery below 1e-10 m. These are deterministic scenario acceptance criteria, not claims about real-camera performance.

The existing GitHub Actions workflow runs all discovered tests on Python 3.9 and 3.12. Local legacy Python files were syntax-parsed; the legacy project has no automated test suite, and YOLO/firmware/Unity runtime validation remains unavailable. `git diff` against the merged Stage 1 baseline confirms no tracker changes.

Limitations: ideal fixed calibrated cameras, one known target identity, diagonal image noise model, tunable heuristic thresholds, no 3D uncertainty propagation, no temporal 3D filter, and no general multi-camera robust estimator. Accepted nonzero timestamp skew is an approximation; no motion compensation aligns rays to the midpoint. Seeded noise tests do not represent sensor calibration errors or every failure mode. Physical control, real-camera integration, UDP integration, website graphics and terrain are absent. Stage 3 and later work remain separate tasks.
