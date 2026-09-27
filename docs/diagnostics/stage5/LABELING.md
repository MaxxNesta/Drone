# Manual labeling protocol (no annotations supplied)

`manual_template.json` is **not ground truth**. It contains only original frame metadata, `reviewed: false`, and `objects: null`. No detector predictions or invented identities are prefilled. Local unannotated frame PNGs were exported to `artifacts/stage5/manual-labeling/` at original resolution.

Selected frame sequences: 0, 300, 600–602, 630–632, 660–662, 900, 1200–1202. These cover hardware, screen/application imagery and several short consecutive snippets. They are a starting point for exploratory labeling, not a statistically representative or held-out dataset. For continuity validation, export and fully label longer contiguous intervals, including missing/occluded appearances. Do not interpret gaps between these snippets as measured reacquisition.

1. Inspect the **original PNG**, not the annotated prediction video. Define the evaluated class set before looking at predictions. The supplied model uses COCO's 80 classes and has no drone class. This demonstration includes objects shown on monitors; decide whether those displayed instances count, and apply that rule consistently.
2. For each visible instance, enter `{"bbox_xyxy": [left, top, right, bottom], "class_id": integer, "truth_id": "your-stable-instance-id"}` in `objects`. Coordinates use original 1920×1080 pixels, not a resized preview. Use a real independently inspected box and the model's documented class map in `report.json`. Do not copy predictions into ground truth.
3. Keep instance IDs stable through a continuous snippet only when correspondence can be verified. Different displayed views are different image instances, even if they show the same physical object. Do not infer stereo correspondence. Record the policy for occluded/partially visible objects and scene cuts outside the template in your labeling notes.
4. Use `objects: []` only after explicitly confirming no in-scope visible instances. `null` means unreviewed. Set `reviewed: true` only after manual inspection. Every selected frame must be reviewed before conversion.
5. The current scorer evaluates all predicted classes; for an all-class score, label all applicable COCO instances. A restricted-class benchmark requires applying the same declared class filter to both predictions and labels before scoring; that filter is not provided here. Unsupported targets must not be reassigned to an arbitrary COCO class just to obtain a score.
6. Have a second reviewer inspect labels if practical. Record reviewer, date, dataset selection and class/occlusion policies. Keep a held-out evaluation set for any later threshold tuning.

After real manual review:

```sh
python3 -m drone_sim.vision.annotations convert \
  --template /path/to/manually-reviewed-template.json \
  --output /tmp/manual-annotations.jsonl
python3 -m drone_sim.vision evaluate \
  --input artifacts/stage5/baseline/telemetry.jsonl \
  --annotations /tmp/manual-annotations.jsonl --output /tmp/labeled-subset-score.json
```

Conversion rejects unreviewed/null frames, invalid boxes and duplicate identities. It cannot verify that a person actually inspected a frame. Scoring reports labeled coverage and excludes unannotated frames. Sparse samples cannot establish whole-video precision/recall, identity accuracy or occlusion recovery; Stage 4's continuity counters assume a contiguous evaluated sequence. Precision/recall and identity metrics remain unavailable until suitable independent labels exist.
