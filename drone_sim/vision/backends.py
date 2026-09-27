"""Lazy optional dependencies. Local files only; never import the legacy application."""
from hashlib import sha256
from pathlib import Path
from time import perf_counter
from .records import Observation, ObservationFrame


def fingerprint(path):
    digest = sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return 'sha256:'+digest.hexdigest()


class YoloDetector:
    def __init__(self, weights, confidence=.4, image_size=320, factory=None):
        path = Path(weights)
        if not path.is_file():
            raise ValueError('An existing local weights file is required; downloads are disabled')
        if not 0 <= confidence <= 1 or type(image_size) is not int or image_size <= 0:
            raise ValueError('Invalid inference configuration')
        if factory is None:
            from ultralytics import YOLO
            factory = YOLO
        self.model = factory(str(path.resolve()))
        from importlib.metadata import version, PackageNotFoundError
        import platform
        versions = {}
        for package in ('ultralytics', 'av', 'torch', 'opencv-python'):
            try:
                versions[package] = version(package)
            except PackageNotFoundError:
                versions[package] = None
        self.metadata = {'weights': fingerprint(path), 'confidence': confidence, 'image_size': image_size,
                         'class_names': getattr(self.model, 'names', None),
                         'python': platform.python_version(), 'packages': versions}

    def detect(self, image):
        results = self.model.predict(image, conf=self.metadata['confidence'],
                                     imgsz=self.metadata['image_size'], verbose=False)
        if len(results) != 1:
            raise ValueError('Expected one result for one frame')
        boxes = results[0].boxes
        if boxes is None:
            raise ValueError('A bounding-box detection model is required')
        xyxy, confidence, classes = (x.cpu().tolist() for x in (boxes.xyxy, boxes.conf, boxes.cls))
        if not len(xyxy) == len(confidence) == len(classes):
            raise ValueError('Inconsistent detector output')
        output = []
        for b,c,k in zip(xyxy, confidence, classes):
            if int(k) != k:
                raise ValueError('Invalid detector class')
            output.append(Observation(((b[0]+b[2])/2,(b[1]+b[3])/2),tuple(b),c,int(k)))
        return tuple(output)


def video_frames(path, camera_id, opener=None):
    """Yield original presentation timestamps, including VFR/nonzero start times."""
    source = fingerprint(path)
    if opener is None:
        import av
        opener = av.open
    with opener(str(path)) as container:
        previous = None
        for sequence, frame in enumerate(container.decode(video=0)):
            if frame.pts is None or frame.time_base is None:
                raise ValueError('Video frame lacks a presentation timestamp')
            stamp = float(frame.pts*frame.time_base)
            if previous is not None and stamp <= previous:
                raise ValueError('Non-increasing video presentation timestamps')
            previous = stamp
            record = ObservationFrame(source, camera_id, sequence, stamp,
                (frame.width,frame.height), (), 'video_pts', 'container_presentation_timestamp',
                frame.pts, (frame.time_base.numerator,frame.time_base.denominator))
            yield record, frame.to_ndarray(format='bgr24')


def process(frames, detector, clock=perf_counter):
    """Serial pipeline; media-time tracking and local compute-time latency stay separate."""
    from dataclasses import replace
    from .tracker import ObservationTracker
    tracker = ObservationTracker()
    iterator = iter(frames)
    try:
        while True:
            start = clock()
            try:
                record, image = next(iterator)
            except StopIteration:
                break
            decoded = clock()
            record = replace(record, observations=detector.detect(image))
            detected = clock()
            tracking = tracker.consume(record)
            finished = clock()
            yield {'observation': record.to_dict(), 'tracking': tracking,
                   'timing_ms': {'decode': (decoded-start)*1000, 'detection': (detected-decoded)*1000,
                                 'tracking': (finished-detected)*1000, 'total': (finished-start)*1000},
                   'detector': detector.metadata}
    finally:
        if hasattr(iterator, 'close'):
            iterator.close()


def webcam_frames(index, camera_id, max_frames, factory=None, clock=perf_counter):
    """Explicit opt-in. Receipt time is not an exposure timestamp or a stereo clock."""
    from uuid import uuid4
    if type(index) is not int or index < 0 or type(max_frames) is not int or max_frames <= 0:
        raise ValueError('Nonnegative webcam index and positive frame limit required')
    if factory is None:
        import cv2
        factory = cv2.VideoCapture
    capture = factory(index)
    source = 'webcam:'+str(index)+':'+str(uuid4())
    try:
        if not capture.isOpened():
            raise ValueError('Cannot open webcam')
        for sequence in range(max_frames):
            ok, image = capture.read()
            stamp = clock()
            if not ok:
                raise ValueError('Webcam read failed; cached frames are never replayed')
            h,w = image.shape[:2]
            yield ObservationFrame(source, camera_id, sequence, stamp, (w,h), (),
                'monotonic_receive', 'host_receipt_after_read_not_exposure'), image
    finally:
        capture.release()
