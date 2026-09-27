"""Scorer-only annotations: never supplied to the detector or tracker."""
from math import ceil, isfinite
from .records import ObservationFrame


def iou(a, b):
    intersection = max(0, min(a[2],b[2])-max(a[0],b[0]))*max(0, min(a[3],b[3])-max(a[1],b[1]))
    union = (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-intersection
    return intersection/union if union else 0.


def evaluate(rows, annotations, threshold=.5):
    """Confidence-ordered class-aware IoU matching; precision/recall, not COCO AP.

    Labels use the same source/camera/sequence/clock/image contract plus scorer-only
    truth_ids parallel to observations. All labeled frames must occur exactly once.
    """
    if not 0 < threshold <= 1:
        raise ValueError('IoU threshold must be in (0, 1]')
    def key(f):
        return f.source_id, f.camera_id, f.frame_sequence
    labels = {}
    for label in annotations:
        f = ObservationFrame.from_dict(label['observation'])
        ids = label['truth_ids']
        if len(ids) != len(f.observations) or len(set(ids)) != len(ids) or any(not isinstance(x,str) or not x for x in ids):
            raise ValueError('Unique nonempty truth IDs required per labeled frame')
        if any(o.bbox_xyxy is None for o in f.observations):
            raise ValueError('Box annotations required')
        if key(f) in labels:
            raise ValueError('Duplicate annotation frame')
        labels[key(f)] = (f, ids)
    tp = fp = fn = switches = reacquisitions = tracked = eligible = 0
    overlaps, latencies, seen, previous = [], [], set(), {}
    last_frame = {}
    for row in rows:
        f = ObservationFrame.from_dict(row['observation'])
        k = key(f)
        if k in seen:
            raise ValueError('Duplicate result frame')
        seen.add(k)
        stream = (f.source_id, f.camera_id)
        if stream in last_frame and f.capture_time_s <= last_frame[stream]:
            raise ValueError('Results must be in timestamp order')
        last_frame[stream] = f.capture_time_s
        timing = row.get('timing_ms', {}).get('total')
        if timing is not None:
            if not isfinite(timing) or timing < 0:
                raise ValueError('Invalid processing latency')
            latencies.append(timing)
        if k not in labels:
            continue
        eligible += 1
        truth, ids = labels[k]
        if (f.image_size != truth.image_size or f.time_domain != truth.time_domain
                or abs(f.capture_time_s-truth.capture_time_s) > 1e-9):
            raise ValueError('Annotation clock/dimensions do not match results')
        if any(o.bbox_xyxy is None for o in f.observations):
            raise ValueError('Box predictions required for detection evaluation')
        available = set(range(len(ids)))
        assignments = {a['index']:a['track_id'] for a in row['tracking']['assignments']}
        for index in sorted(range(len(f.observations)), key=lambda i: (-(f.observations[i].confidence or 0),i)):
            predicted = f.observations[index]
            candidates = [(iou(predicted.bbox_xyxy,truth.observations[j].bbox_xyxy),j)
                          for j in available if predicted.class_id == truth.observations[j].class_id]
            score, j = max(candidates, default=(0,None), key=lambda pair:(pair[0],-pair[1]))
            if j is None or score < threshold:
                fp += 1
                continue
            available.remove(j)
            tp += 1
            overlaps.append(score)
            identity = assignments.get(index)
            if identity is not None:
                tracked += 1
                identity_key = stream+(ids[j],)
                old = previous.get(identity_key)
                if old is not None:
                    switches += old[0] != identity
                    reacquisitions += old[1] < f.frame_sequence-1
                previous[identity_key] = (identity, f.frame_sequence)
        fn += len(available)
    missing = set(labels)-seen
    if missing:
        raise ValueError('Annotated frames missing from results')
    precision = tp/(tp+fp) if tp+fp else None
    recall = tp/(tp+fn) if tp+fn else None
    latencies.sort()
    return {'schema_version':1, 'iou_threshold':threshold, 'labeled_frames':eligible,
            'unlabeled_frames':len(seen)-eligible, 'true_positives':tp, 'false_positives':fp, 'false_negatives':fn,
            'precision':precision, 'recall':recall,
            'mean_matched_iou':sum(overlaps)/len(overlaps) if overlaps else None,
            'identity_switches':switches if eligible else None,
            'reacquisitions':reacquisitions if eligible else None,
            'tracked_true_positive_fraction':tracked/tp if tp else None,
            'processing_latency_ms':{'samples':len(latencies),
                'mean':sum(latencies)/len(latencies) if latencies else None,
                'p95':latencies[ceil(.95*len(latencies))-1] if latencies else None}}
