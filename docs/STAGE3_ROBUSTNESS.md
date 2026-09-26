# Stage 3: robustness and failure-scenario simulation

Stage 3 was built on `stage3-robustness` after the authorized merge of PR #2 into `dev` (base `e45a851`). New commits use **Kaung**. The implementation is a Python standard-library experiment harness; `existing-tracker/` and all Stage 1–2 Python files and public contracts are unchanged.

## Reproduce

Run from the repository root with Python 3.9 or later:

```sh
python3 -m unittest discover -v
python3 -m drone_sim.robustness --config scenarios/stage3/crossing.json --output /tmp/crossing.jsonl
python3 -m drone_sim.robustness --suite scenarios/stage3 --summary-only --output /tmp/stage3_benchmarks.jsonl
```

`--config` runs one experiment; `--suite` runs all JSON files in filename order. Default output is stdout. `--summary-only` retains configuration and summary records; otherwise each delivery tick includes packets, association decisions, track snapshots, transitions, stereo results and a separately named evaluation section. Explicit output paths are overwritten. The committed [machine-readable benchmark](diagnostics/stage3_benchmarks.jsonl) is produced by the suite command above, using all twelve [scenario configurations](../scenarios/stage3/).

The configuration record includes the complete experiment, camera calibration, filter and triangulation defaults, and association policy. Packet timestamps, sequence numbers and sample references permit tracing each accepted/error sample. There are no runtime clocks or network sockets.

## Components and estimator/truth separation

| Module | Responsibility |
| --- | --- |
| `robustness/config.py` | Validated JSON configuration for timing, objects, per-camera faults and association gates. |
| `robustness/world.py` | Ground-truth motion, Stage 1 camera projection, noise/occlusion/drop generation and deterministic packet delivery schedule. |
| `robustness/association.py` | Per-camera tracking and cross-camera matching using only calibration, unlabeled pixels, sequence numbers and reported timestamps. |
| `robustness/runner.py` | Orchestration and scoring; owns the private sample-to-truth mapping. |
| `robustness/__main__.py` | Single-scenario/suite JSONL command line. |

Estimator input `Packet` has only `camera_id`, `sequence`, `timestamp_s`, and `pixels`. Detections are shuffled separately for every camera frame. A sample key `(camera_id, sequence, shuffled_index)` identifies a measurement, not an object, and changes every frame. True object IDs, true capture times, positions and ideal pixels live in a separate `Evidence` mapping used only by the scorer. The estimator never receives this mapping or the configured truth objects.

Tracking IDs are allocated by the estimator from observed pixel geometry. Cross-camera candidate IDs are generated after observation matching; they are not simulator labels. A regression renames every truth object and proves that generated packets, association decisions, tracks and stereo results do not change. Scoring can identify a wrong match after estimation, but cannot veto it or feed corrected identity back into the estimator.

## Deterministic configuration and faults

Defaults are 300 updates plus tick zero at dt=0.02 s (six seconds), seed 7, and Stage 1's two stationary calibrated cameras. One to eight constant-velocity objects are supported. Truth motion is evaluated as `p0+v*t` on the fixed simulation grid. Gaussian noise, Bernoulli drops, delivery jitter and observation shuffling use separate seed streams keyed by camera/frame/effect. Switching on packet duplication does not perturb noise samples.

Each camera has the following configuration:

| Field | Semantics |
| --- | --- |
| `period_ticks`, `phase_ticks` | Capture at phase + n×period. Effective observation rate is 1/(period×dt); rates are integer multiples of the base tick. |
| `noise_std_px` | Independent Gaussian noise on each projected pixel coordinate. |
| `drop_probability`, `drop_ticks` | Drop the whole frame before transmission, randomly and/or at explicit capture ticks. |
| `occlusions` | Half-open `[start, stop)` capture-tick intervals; `object_id` selects a truth object or `*` in the sensor generator only. A visible empty frame is still transmitted. |
| `latency_ticks`, `jitter_ticks` | Delivery at capture tick + latency + a seeded uniform integer jitter in [0,jitter]. |
| `timestamp_offset_s` | Bias the reported capture clock, independent of actual delivery latency. No estimator clock correction uses this known simulator offset. |
| `duplicate_every`, `duplicate_delay_ticks` | Retransmit every Nth captured frame after its original delivery plus the specified delay. 0 disables duplication. |
| `reorder_every`, `reorder_delay_ticks` | Add extra delay to every Nth captured frame so later frames can arrive first. 0 disables injection. |

Sequence numbers count capture frames, including dropped ones. Fault patterns use sequence+1 for every-N rules. Packets arriving after the capture window are drained and scored; this does not extend the capture-opportunity denominator. The simulator does not silently drop end-of-run delayed observations. For noninteger rates, use a finer base tick; arbitrary asynchronous sub-tick schedules are not implemented.

## Estimation policy

### Temporal association and packet handling

Per-camera association performs maximum-cardinality, minimum-cost one-to-one assignment using dynamic programming over at most eight observations. Candidate edges require Euclidean distance ≤30 px and squared Mahalanobis innovation ≤16. Cost is the Gaussian negative log likelihood up to shared constants: squared Mahalanobis distance plus log innovation-covariance determinant. The determinant term avoids favoring old, highly uncertain tracks solely because they have small normalized residuals. Ties are deterministic.

Predicted copies of the Stage 2 filter provide association costs. Only the selected update changes a live filter; unmatched detections initialize new filters from their measured positions. Missing observations coast, become stale after 0.25 s and expire after 0.75 s. Expired tracks are retired; later detections get new IDs. At most 32 active tracks per camera are allowed. Unconfirmed immediate track birth is a deliberate baseline limitation: an isolated outlier can spawn an extra track and fragment identity.

The delivery adapter rejects duplicate sequences, older sequences/timestamps, future/invalid timestamps, and packets more than 0.25 s old according to their reported clock. Duplicates are recognized even if their original packet was rejected. A fresh delayed packet is processed at its reported capture timestamp, in increasing capture-time order. It is never retimestamped to arrival time. This preserves Stage 2's synchronous `CameraTracker.step()` contract while allowing bounded in-order latency in the adapter. Snapshot predictions to delivery time operate on copies, so inspecting a track cannot advance its live capture-time clock and block a subsequent delayed measurement.

Track snapshots report delivery-time coasting/stale/lost state and centering, separately from capture-time measurement acceptance. Thus a freshly delivered delayed measurement can yield a coasting present-time snapshot. An invalid or rejected packet never refreshes measurement age. There is no out-of-sequence filter replay or smoothing.

### Stereo association

The latest accepted packet's measurements from each camera form the stereo candidate sets. Freshness and timestamp compatibility use the unchanged Stage 2 gates, including ≤10 ms skew. No interpolation creates a synthetic partner at another camera's capture time. Empty received frames clear that camera's candidate set; dropped frames leave older data subject to the normal age/skew checks.

Candidate pairing uses closest-ray separation and the Stage 2 calibration/geometry checks. A 0.5 m separation gate permits candidate selection before the stricter 0.25 m final triangulation residual gate. If another candidate in a row or column is within 0.01 m of a candidate's cost (or is better), that edge is rejected as ambiguous. Global one-to-one assignment selects among the remaining edges. Raw and filtered triangulation use the **same selected measurement pair**; association is based on raw geometry, not on truth or whichever method scores better.

All packets at a delivery tick are processed before stereo matching, so only the latest accepted packet per camera at that tick is considered. Each selected sample-pair key is scored once, even if duplicate arrivals trigger another matching pass. No persistent world-track identity is inferred from a pair of local track IDs.

## Metric definitions and matched samples

- **Matched pixel errors:** raw and filtered Euclidean pixel errors from the same accepted measurement key against its ideal capture-time pixel. Rejected measurements enter neither error array.
- **Matched position errors:** the exact same pair keys must have both raw and filtered triangulation accepted and both observations must belong to the same truth object. Truth is evaluated at the midpoint of the two actual capture times. Both error arrays share this mask, count and ordering. This scores capture-time localization, not a prediction to delivery time.
- **False stereo associations:** pairs drawn from different true objects are counted explicitly as `cross_view_identity_mismatches`. They are excluded from both matched error arrays because there is no single corresponding true position; their existence remains visible in output counts and traces. Low matched RMSE alone is not sufficient evidence of reliability.
- **Observation availability:** unique accepted measurements divided by scheduled, geometrically in-view camera/object opportunities before frame drops or occlusion. Different camera rates change the schedule denominator; they are not counted as drops.
- **Position availability:** correctly associated accepted position coverage divided by all base-grid object ticks visible to both cameras, before faults. Coverage is a set keyed by truth object and nearest base tick to the actual capture midpoint. This intentionally exposes reduced coverage at slower or mismatched rates and prevents repeated pairs from inflating availability. Raw/filtered coverage is reported separately. `valid_position_outputs` also counts false associations, unlike coverage.
- **Identity switches:** for each camera/true object, a different assigned local track ID than its previous accepted observation. This includes a new ID after a long loss. `track_label_changes` separately counts a local track changing which true object it represents. These are camera-local metrics, not a world-track IDF1 score.
- **Reacquisition:** a previously observed camera/object gets an accepted observation after a true capture-time gap greater than 1.5×its scheduled period. This includes recovery from dropped or reordered frames, not only full `lost` state. Report total and same-ID recovery counts. Recovery delay is arrival time minus the next expected capture after the last accepted observation, so it includes the missing interval and transport delay.
- **Latency:** true transport latency and estimator-visible reported age are distinct. Statistics cover all delivered packets, including duplicates; matched position latency is delivery time minus actual capture midpoint. A timestamp-offset test can have zero transport latency but nonzero reported age.
- **Rejections and transitions:** packet and measurement rejection reasons are separate from stereo candidate/final-result reasons. Candidate rejection counts can exceed frame counts because every candidate pair is tested. Per-tick transition records expose births, tracking/coasting/stale/lost changes. State counts are track-tick counts, not object availability.

No accepted samples means RMSE/mean/max are `null`, not zero. Every error summary includes its sample count, mean, RMSE and maximum. Per-tick evaluation records provide the matched errors and sample keys for auditability.

## Benchmark results

Local execution used Python 3.9.6. All twelve committed scenarios use seed 7 and 301 capture ticks. Crossing experiments use two objects starting at (-3,20,4) and (3,20,6) m, moving at (1,0,1/3) and (-1,0,-1/3) m/s: they meet at (0,20,5) at t=3 s. Crossing noise is 0.5 px; the combined experiment uses 1 px noise plus rate mismatch, drops, occlusion, latency/jitter, clock offset, duplicates and reordering.

| Scenario | Matched 3D pairs | Raw / filtered 3D RMSE (m) | Filtered position availability | Identity switches | Observation-gap recoveries |
| --- | ---: | --- | ---: | ---: | ---: |
| Baseline | 301 | 2.68e-14 / 0.00283 | 100% | 0 | 0 |
| Noise (1 px) | 300 | 0.16140 / 0.04351 | 100% | 8 | 0 |
| Frame drops (20% each camera) | 186 | 2.73e-14 / 0.00328 | 61.79% | 0 | 98 |
| Temporary occlusion | 221 | 2.64e-14 / 0.00452 | 73.42% | 2 | 4 |
| Rates (50 Hz / 16.67 Hz) | 101 | 2.77e-14 / 0.00522 | 33.55% | 0 | 0 |
| Latency (80 ms both cameras) | 301 | 2.68e-14 / 0.00283 | 100% | 0 | 0 |
| Stale latency (400 ms) | 0 | unavailable / unavailable | 0% | 0 | 0 |
| Timestamp offset (-15 ms on camera 1) | 0 | unavailable / unavailable | 0% | 0 | 0 |
| Duplicates (every fourth frame) | 301 | 2.68e-14 / 0.00283 | 100% | 0 | 0 |
| Out-of-order (every fifth frame delayed) | 241 | 2.72e-14 / 0.00308 | 80.07% | 0 | 120 |
| Crossing objects | 580 | 0.07441 / 0.01972 | 96.35% | 4 | 0 |
| Combined failures | 93 | 0.14669 / 0.05280 | 15.61% | 4 | 150 |

Interpretation:

- Noise reduction works, but the noisy single-object run still has eight local identity switches from tentative track fragmentation. This should motivate future track-birth confirmation, not be hidden behind improved RMSE.
- The crossing case has four identity switches and four track-label changes. Ambiguous stereo pairs are rejected; there are no false accepted stereo associations in that particular run. Geometry and point motion alone cannot reliably resolve coincident targets.
- Combined faults produce **one false stereo association**, only 15.61% correctly associated filtered coverage, and many observation-gap recoveries. Low error on the surviving 93 matched pairs does not imply reliable continuous tracking.
- Duplicate injection creates 150 rejected duplicate packets without changing baseline accepted counts/errors. Reordering rejects 120 late packets rather than reversing filter time. Stale latency rejects all 602 packets.
- The occlusion fixture removes both views for ticks 80–99 and 150–209. The short gap preserves IDs; the long gap expires tracks and creates two new IDs. All four tracking lifecycle states appear.
- Bounded latency retains capture-time accuracy and exposes 80 ms delay; it does not provide a current-time position estimate. Clock-skew correction and interpolation are intentionally unsupported.

## Tests and scope limits

`python3 -m unittest discover -v`: **74 tests passed** locally, including all 45 Stage 1–2 tests and 29 Stage 3 tests. There is a regression for every committed failure scenario, as well as config round trips/validation, exact replay, seed changes, shuffled ordering, true-label renaming invariance, global assignment, snapshot clock isolation, explicit drop/phase settings, object-specific occlusion, duplicate accounting, matched denominators and CLI/suite behavior.

Scenario acceptance assertions include baseline full availability and zero switches; noise filtered matched errors below 80% of raw errors; zero positions under stale latency or incompatible skew; unchanged accepted counts/errors under duplicates; expected rate/dropout coverage; explicit crossing switches and ambiguity; and explicit false association under combined faults. Some tests deliberately preserve and expose known failure behavior instead of declaring every scenario successful tracking.

The existing GitHub Actions workflow runs the complete discovery suite on Python 3.9 and 3.12 for this PR. Local preservation checks compare the legacy tracker and all Stage 1–2 Python files against merged PR #2. No legacy tests exist beyond the supplied Python syntax checks; no real-camera, firmware or Unity execution is claimed.

Limitations: fixed two-camera calibration, linear point motion, at most eight observations per frame, integer-grid camera rates, ideal pinhole optics, no scene/terrain occlusion reasoning, appearance features, clutter/false-positive generator, track confirmation, general re-identification, global 3D identity tracking, smoothing/reorder buffers, clock synchronization, rate interpolation, moving-camera calibration or 3D uncertainty. Observation metadata and covariance gates are not a security protocol. JSON experiments are intended for bounded local simulations; very long runs or very large delays increase runtime/memory. Random replay is verified on the tested runtime, not promised bit-identical across every platform. No physical aircraft control, real-camera integration, website or terrain graphics were added.
