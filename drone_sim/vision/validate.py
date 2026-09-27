"""Local recorded-video validation with explicit measurement and annotation boundaries."""
import argparse
from collections import Counter
from dataclasses import replace
from fractions import Fraction
import json
from math import ceil, isfinite
from pathlib import Path
import platform
from time import perf_counter
from .backends import YoloDetector, video_frames, fingerprint
from .evaluate import evaluate
from .tracker import ObservationTracker


def interval(value):
    start, end = map(float, value.split(':'))
    if not isfinite(start) or not isfinite(end) or not 0 <= start < end:
        raise ValueError('Suppression interval must be finite 0 <= start < end')
    return start, end


def measure(frames, detector, sink, suppress=(), max_frames=None, clock=perf_counter):
    """Yield Stage 4-compatible rows. Fault injection never alters detector evidence.

    sink(image, row) receives the actual decoded image for overlay/export. Latency
    separates decode, inference/conversion, tracking and rendering/encoding.
    """
    tracker, iterator = ObservationTracker(), iter(frames)
    first = None
    count = 0
    try:
        while max_frames is None or count < max_frames:
            start = clock()
            try:
                record, image = next(iterator)
            except StopIteration:
                break
            decoded = clock()
            raw = detector.detect(image)
            detected = clock()
            if first is None:
                first = record.capture_time_s
            elapsed = record.capture_time_s-first
            suppressed = any(a <= elapsed < b for a,b in suppress)
            raw_record = replace(record, observations=raw)
            effective = replace(raw_record, observations=()) if suppressed else raw_record
            tracking = tracker.consume(effective)
            tracked = clock()
            row = {'observation':effective.to_dict(), 'tracking':tracking,
                   'raw_observation':raw_record.to_dict(),
                   'intervention':'synthetic_observation_suppression' if suppressed else None,
                   'timing_ms':{'decode':1000*(decoded-start), 'detection':1000*(detected-decoded),
                                'tracking':1000*(tracked-detected), 'total':1000*(tracked-start)}}
            sink(image, row)
            rendered = clock()
            row['timing_ms']['render_encode'] = 1000*(rendered-tracked)
            row['timing_ms']['pipeline_with_render'] = 1000*(rendered-start)
            count += 1
            yield row
    finally:
        if hasattr(iterator, 'close'):
            iterator.close()


def distribution(values):
    values = sorted(values)
    if any(not isfinite(x) or x < 0 for x in values):
        raise ValueError('Invalid latency sample')
    return {'samples':len(values), 'mean':sum(values)/len(values) if values else None,
            'p50':values[ceil(.5*len(values))-1] if values else None,
            'p95':values[ceil(.95*len(values))-1] if values else None,
            'max':values[-1] if values else None}


def summarize(rows, wall_s, annotations=()):
    if not isfinite(wall_s) or wall_s <= 0:
        raise ValueError('Positive finite wall duration required')
    states, transitions, rejections, classes = Counter(), Counter(), Counter(), Counter()
    ids = set()
    for row in rows:
        t = row['tracking']
        states.update(s['status'] for s in t['tracks'])
        transitions.update(str(s['from'])+'->'+s['to'] for s in t['transitions'])
        rejections.update(s['reason'] for s in t.get('rejections', []))
        classes.update(str(o['class_id']) for o in row['raw_observation']['observations'])
        ids.update(s['track_id'] for s in t['tracks'])
    fields = ('decode','detection','tracking','total','render_encode','pipeline_with_render')
    n = len(rows)
    return {'frames':n, 'wall_s_including_export':wall_s,
            'export_throughput_fps':n/wall_s,
            'capture_span_s':rows[-1]['observation']['capture_time_s']-rows[0]['observation']['capture_time_s'] if n else None,
            'raw_detections':sum(len(r['raw_observation']['observations']) for r in rows),
            'raw_multiple_detection_frames':sum(len(r['raw_observation']['observations']) > 1 for r in rows),
            'raw_zero_detection_frames':sum(not r['raw_observation']['observations'] for r in rows),
            'suppressed_frames':sum(r['intervention'] is not None for r in rows),
            'class_detection_counts':dict(classes), 'unique_assigned_ids':len(ids),
            'track_state_samples':dict(states), 'state_transitions':dict(transitions),
            'tracking_rejections':dict(rejections),
            'latency_ms':{key:distribution([r['timing_ms'][key] for r in rows]) for key in fields},
            'accuracy':evaluate(rows, annotations),
            'accuracy_status':'labeled_subset_only' if annotations else 'blocked_no_manual_annotations'}


def overlay(image, row, names):
    import cv2
    image = image.copy()
    height,width = image.shape[:2]
    scale = max(.35, min(1., width/1200))
    line_height = max(14, int(28*scale))
    tracks = {t['track_id']:t for t in row['tracking']['tracks']}
    assignments = {a['index']:a['track_id'] for a in row['tracking']['assignments']}
    def text(label, x, y, color=(255,255,255)):
        cv2.putText(image,label,(max(0,int(x)),max(line_height,min(height-3,int(y)))),
                    cv2.FONT_HERSHEY_SIMPLEX,scale,color,max(1,int(2*scale)),cv2.LINE_AA)
    def display_id(identity):
        return 'c'+identity.split(':')[0]+'/'+identity.rsplit(':',1)[-1]
    for i,o in enumerate(row['observation']['observations']):
        b = o['bbox_xyxy']
        identity = assignments.get(i)
        status = tracks[identity]['status'] if identity in tracks else 'detected/unassigned'
        color = (40,220,40)
        if b:
            x1,y1,x2,y2 = map(round,b)
            cv2.rectangle(image,(x1,y1),(x2,y2),color,2)
            category = names.get(o['class_id'],str(o['class_id']))
            label = f"{category} {o['confidence']:.2f} {display_id(identity) if identity else '-'} {status}"
            text(label,x1,max(3*line_height+12,y1+line_height),color)
    for t in tracks.values():
        if t['status'] != 'tracking' and t['pixel'] is not None:
            x,y = map(round,t['pixel'])
            cv2.drawMarker(image,(x,y),(0,180,255),cv2.MARKER_CROSS,12,2)
    inactive = [t for t in tracks.values() if t['status'] != 'tracking']
    if inactive:
        panel_width = min(width,int(430*scale))
        x = width-panel_width
        cv2.rectangle(image,(x,3*line_height),(width,(4+min(10,len(inactive)))*line_height),(0,0,0),-1)
        for index,t in enumerate(inactive[:10]):
            text(display_id(t['track_id'])+' '+t['status'],x+3,(4+index)*line_height,(0,180,255))
        if len(inactive) > 10:
            text(f'+{len(inactive)-10} more (see telemetry)',x+3,14*line_height,(0,180,255))
    counts = dict(Counter(t['status'] for t in tracks.values()))
    cv2.rectangle(image,(0,0),(width,2*line_height+8),(0,0,0),-1)
    text(f"frame {row['observation']['frame_sequence']} PTS {row['observation']['capture_time_s']:.3f}s {counts}",3,line_height)
    text('INJECTED observation loss (not physical occlusion)' if row['intervention'] else
         'Detector predictions / local IDs - no ground truth',3,2*line_height)
    return image


class DemoWriter:
    """Silent H.264 demo; preserve presentation intervals, normalize first PTS to zero."""
    def __init__(self, path, names):
        import av
        self.av, self.container, self.names = av, av.open(str(path),'w'), names
        self.stream = None
        self.origin = None
        self.last = None

    def __call__(self, image, row):
        f = row['observation']
        stamp = f['pts']*Fraction(*f['time_base'])
        if self.origin is None:
            self.origin = stamp
            self.stream = self.container.add_stream('libx264',rate=30)
            self.stream.width,self.stream.height = f['image_size']
            self.stream.pix_fmt = 'yuv420p'
            self.stream.time_base = Fraction(*f['time_base'])
            self.stream.codec_context.time_base = self.stream.time_base
            self.stream.options = {'crf':'23','preset':'fast'}
        ticks = (stamp-self.origin)/self.stream.codec_context.time_base
        if ticks.denominator != 1 or (self.last is not None and ticks <= self.last):
            raise ValueError('Output time base cannot represent source timestamps exactly')
        self.last = ticks
        annotated = overlay(image,row,self.names)
        frame = self.av.VideoFrame.from_ndarray(annotated,format='bgr24')
        frame.pts,frame.time_base = int(ticks),self.stream.codec_context.time_base
        for packet in self.stream.encode(frame):
            self.container.mux(packet)

    def close(self):
        try:
            if self.stream is not None:
                for packet in self.stream.encode():
                    self.container.mux(packet)
        finally:
            self.container.close()


def report_markdown(report):
    s = report['summary']
    latency = s['latency_ms']['total']
    return ('# Recorded-video validation report\n\n'
        f"Source: `{report['config']['video']}`\n\n"
        f"Frames: {s['frames']}; export throughput: {s['export_throughput_fps']:.3f} frames/s.\n\n"
        f"Decode + detection + tracking latency (ms): {latency}.\n\n"
        f"Raw detections: {s['raw_detections']}; frames with multiple predictions: {s['raw_multiple_detection_frames']}; "
        f"zero-detection frames: {s['raw_zero_detection_frames']}.\n\n"
        f"Track state samples: `{s['track_state_samples']}`. Unique IDs: {s['unique_assigned_ids']} "
        '(not an identity-switch metric).\n\n'
        f"Accuracy status: **{s['accuracy_status']}**.\n\n"
        f"Accuracy/continuity metrics: `{json.dumps(s['accuracy'],sort_keys=True)}`\n\n"
        f"Injected suppression frames: {s['suppressed_frames']}. Suppression is an artificial estimator stress test, "
        'not evidence of physical occlusion handling.\n\n'
        'Throughput includes decode, inference, tracking, overlay, encoding, JSON serialization and encoder flush; '
        'model setup is reported separately. Per-frame latency excludes serialization and encoder flush. '
        'First inference is included; no warm-up samples are discarded. '
        'No exposure-to-result latency, accuracy without labels, stereo or 3D claim is made. '
        'See report.json for exact configuration, runtime, hashes and all measurements.\n')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--video',required=True)
    p.add_argument('--weights',required=True)
    p.add_argument('--camera-id',default='recorded-camera')
    p.add_argument('--output-dir',required=True)
    p.add_argument('--annotations')
    p.add_argument('--confidence',type=float,default=.4)
    p.add_argument('--image-size',type=int,default=320)
    p.add_argument('--max-frames',type=int)
    p.add_argument('--suppress',action='append',default=[],type=interval,metavar='START:END')
    args = p.parse_args()
    if args.max_frames is not None and args.max_frames <= 0:
        p.error('--max-frames must be positive')
    if not Path(args.video).is_file() or not Path(args.weights).is_file():
        p.error('Existing local video and weights required')
    output = Path(args.output_dir)
    output.mkdir(parents=True,exist_ok=False)
    # Keep optional runtime caches outside the immutable legacy tree.
    import os
    os.environ.setdefault('YOLO_CONFIG_DIR',str(Path('.vision-cache/yolo').resolve()))
    os.environ.setdefault('MPLCONFIGDIR',str(Path('.vision-cache/matplotlib').resolve()))
    os.environ.setdefault('YOLO_OFFLINE','true')
    Path(os.environ['YOLO_CONFIG_DIR']).mkdir(parents=True,exist_ok=True)
    Path(os.environ['MPLCONFIGDIR']).mkdir(parents=True,exist_ok=True)
    from ultralytics import YOLO
    import torch
    torch.set_num_threads(1)
    torch.manual_seed(0)
    torch.use_deterministic_algorithms(True)
    def factory(path):
        model = YOLO(path)
        # Explicit CPU/FP32 avoids asynchronous accelerator timing ambiguity.
        model.overrides.update(device='cpu',half=False,save=False,save_txt=False)
        return model
    setup = perf_counter()
    detector = YoloDetector(args.weights,args.confidence,args.image_size,factory=factory)
    setup_s = perf_counter()-setup
    annotations = []
    if args.annotations:
        with open(args.annotations) as stream:
            annotations = [json.loads(line) for line in stream if line.strip()]
    rows = []
    writer = DemoWriter(output/'annotated.mp4',detector.model.names)
    start = perf_counter()
    try:
        with open(output/'telemetry.jsonl','w') as stream:
            for row in measure(video_frames(args.video,args.camera_id),detector,writer,args.suppress,args.max_frames):
                stream.write(json.dumps(row,sort_keys=True,allow_nan=False)+'\n')
                rows.append(row)
    finally:
        writer.close()
    wall_s = perf_counter()-start
    if not rows:
        raise ValueError('Video produced no frames')
    report = {'schema_version':1,'config':vars(args),'runtime':{'python':platform.python_version(),
        'platform':platform.platform(),'machine':platform.machine(),'device':'cpu','torch_threads':1,
        'deterministic_algorithms':True,'seed':0,'model_setup_s':setup_s},
        'detector':detector.metadata,'source_id':rows[0]['observation']['source_id'],
        'summary':summarize(rows,wall_s,annotations),
        'artifacts':{name:fingerprint(output/name) for name in ('annotated.mp4','telemetry.jsonl')}}
    if args.annotations:
        report['annotations_sha256'] = fingerprint(args.annotations)
    (output/'report.json').write_text(json.dumps(report,indent=2,sort_keys=True,allow_nan=False)+'\n')
    (output/'report.md').write_text(report_markdown(report))
    print(json.dumps({'frames':len(rows),'output_dir':str(output),'throughput_fps':len(rows)/wall_s}))


if __name__ == '__main__':
    main()
