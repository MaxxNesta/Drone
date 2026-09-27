# Recorded-video validation report

Source: `existing-tracker/Media/Demo Video/CameraTracker.mp4`

Frames: 1540; export throughput: 27.857 frames/s.

Decode + detection + tracking latency (ms): {'samples': 1540, 'mean': 23.259277073908354, 'p50': 23.148749954998493, 'p95': 27.982749976217747, 'max': 181.01633386686444}.

Raw detections: 1169; frames with multiple predictions: 352; zero-detection frames: 1010.

Track state samples: `{'tracking': 1041, 'coasting': 2219, 'stale': 3626, 'lost': 229}`. Unique IDs: 229 (not an identity-switch metric).

Accuracy status: **blocked_no_manual_annotations**.

Accuracy/continuity metrics: `{"false_negatives": 0, "false_positives": 0, "identity_switches": null, "iou_threshold": 0.5, "labeled_frames": 0, "mean_matched_iou": null, "precision": null, "processing_latency_ms": {"mean": 23.259277073908354, "p95": 27.982749976217747, "samples": 1540}, "reacquisitions": null, "recall": null, "schema_version": 1, "tracked_true_positive_fraction": null, "true_positives": 0, "unlabeled_frames": 1540}`

Injected suppression frames: 36. Suppression is an artificial estimator stress test, not evidence of physical occlusion handling.

Throughput includes decode, inference, tracking, overlay, encoding, JSON serialization and encoder flush; model setup is reported separately. Per-frame latency excludes serialization and encoder flush. First inference is included; no warm-up samples are discarded. No exposure-to-result latency, accuracy without labels, stereo or 3D claim is made. See report.json for exact configuration, runtime, hashes and all measurements.
