"""Version 2 image-plane boundary. No truth identities or fabricated calibration."""
from dataclasses import dataclass, asdict
from math import isfinite


@dataclass(frozen=True)
class Observation:
    pixel: tuple
    bbox_xyxy: tuple = None
    confidence: float = None
    class_id: int = None


@dataclass(frozen=True)
class ObservationFrame:
    source_id: str
    camera_id: str
    frame_sequence: int
    capture_time_s: float
    image_size: tuple
    observations: tuple
    time_domain: str
    timestamp_provenance: str
    pts: int = None
    time_base: tuple = None
    schema_version: int = 2

    def __post_init__(self):
        if self.schema_version != 2 or not self.source_id or not self.camera_id:
            raise ValueError('Unsupported schema or missing source/camera')
        if type(self.frame_sequence) is not int or self.frame_sequence < 0 or not isfinite(self.capture_time_s):
            raise ValueError('Invalid sequence or timestamp')
        if self.time_domain not in ('simulation', 'video_pts', 'monotonic_receive') or not self.timestamp_provenance:
            raise ValueError('Unsupported clock or missing provenance')
        if len(self.image_size) != 2 or any(type(x) is not int or x <= 0 for x in self.image_size):
            raise ValueError('Invalid image dimensions')
        if self.time_domain == 'video_pts':
            if (type(self.pts) is not int or self.time_base is None or len(self.time_base) != 2
                    or any(type(x) is not int or x <= 0 for x in self.time_base)
                    or abs(self.capture_time_s-self.pts*self.time_base[0]/self.time_base[1]) > 1e-9):
                raise ValueError('Video requires consistent original PTS and time base')
        w, h = self.image_size
        for o in self.observations:
            if len(o.pixel) != 2 or not all(isfinite(x) for x in o.pixel) or not (0 <= o.pixel[0] < w and 0 <= o.pixel[1] < h):
                raise ValueError('Invalid observation pixel')
            if o.confidence is not None and (not isfinite(o.confidence) or not 0 <= o.confidence <= 1):
                raise ValueError('Invalid confidence')
            if o.class_id is not None and (type(o.class_id) is not int or o.class_id < 0):
                raise ValueError('Invalid class')
            if o.bbox_xyxy is not None:
                b = o.bbox_xyxy
                if (len(b) != 4 or not all(isfinite(x) for x in b)
                        or not (0 <= b[0] < b[2] <= w and 0 <= b[1] < b[3] <= h)
                        or any(abs(a-c) > 1e-6 for a,c in zip(o.pixel, ((b[0]+b[2])/2, (b[1]+b[3])/2)))):
                    raise ValueError('Invalid box or inconsistent centroid')

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, value):
        value = dict(value)
        value['image_size'] = tuple(value['image_size'])
        if value.get('time_base') is not None:
            value['time_base'] = tuple(value['time_base'])
        value['observations'] = tuple(Observation(tuple(o['pixel']),
            tuple(o['bbox_xyxy']) if o.get('bbox_xyxy') is not None else None,
            o.get('confidence'), o.get('class_id')) for o in value['observations'])
        return cls(**value)


def from_simulation(detection, source_id):
    """Strip simulator truth IDs. Missing boxes/confidence remain unknown."""
    if detection.schema_version != 1 or detection.time_domain != 'simulation':
        raise ValueError('Expected Stage 1 simulation detection')
    observations = (Observation(detection.pixel),) if detection.valid else ()
    return ObservationFrame(source_id, detection.camera_id, detection.frame_sequence,
        detection.capture_time_s, detection.image_size, observations, 'simulation', 'fixed_step')
