"""JSON-serializable Stage 3 experiment configuration."""
from dataclasses import asdict, dataclass, field
from math import isfinite
from ..core import Target


@dataclass(frozen=True)
class Occlusion:
    start: int
    stop: int  # exclusive
    object_id: str = '*'  # generator-only selector; never exposed to estimator

    def __post_init__(self):
        if type(self.start) is not int or type(self.stop) is not int or not 0 <= self.start < self.stop:
            raise ValueError('Occlusion requires integer 0 <= start < stop')


@dataclass(frozen=True)
class CameraFaults:
    period_ticks: int = 1
    phase_ticks: int = 0
    noise_std_px: float = 0.0
    drop_probability: float = 0.0
    drop_ticks: tuple = ()
    occlusions: tuple = ()
    latency_ticks: int = 0
    jitter_ticks: int = 0
    timestamp_offset_s: float = 0.0
    duplicate_every: int = 0
    duplicate_delay_ticks: int = 1
    reorder_every: int = 0
    reorder_delay_ticks: int = 5

    def __post_init__(self):
        for name in ('period_ticks','phase_ticks','latency_ticks','jitter_ticks',
                     'duplicate_every','duplicate_delay_ticks','reorder_every','reorder_delay_ticks'):
            value = getattr(self,name)
            if type(value) is not int or value < 0:
                raise ValueError(f'{name} must be a nonnegative integer')
        if self.period_ticks < 1 or self.phase_ticks >= self.period_ticks:
            raise ValueError('Camera phase must be smaller than positive period')
        if not isfinite(self.noise_std_px) or self.noise_std_px < 0:
            raise ValueError('Noise must be finite and nonnegative')
        if not isfinite(self.drop_probability) or not 0 <= self.drop_probability <= 1:
            raise ValueError('Drop probability must be in [0,1]')
        if not isfinite(self.timestamp_offset_s):
            raise ValueError('Timestamp offset must be finite')
        if any(type(t) is not int or t < 0 for t in self.drop_ticks):
            raise ValueError('Drop ticks must be nonnegative integers')


@dataclass(frozen=True)
class RobustnessConfig:
    name: str = 'baseline'
    seed: int = 7
    dt_s: float = 0.02
    steps: int = 300
    objects: tuple = field(default_factory=lambda: (Target(),))
    cameras: tuple = field(default_factory=lambda: (CameraFaults(),CameraFaults()))
    association_gate_px: float = 30.0
    epipolar_gate_m: float = 0.5
    ambiguity_margin_m: float = 0.01

    def __post_init__(self):
        if type(self.steps) is not int or not 0 <= self.steps <= 100000:
            raise ValueError('steps must be an integer in [0,100000]')
        if type(self.seed) is not int or not isfinite(self.dt_s) or self.dt_s <= 0:
            raise ValueError('Integer seed and finite positive dt required')
        if not isfinite(self.steps*self.dt_s):
            raise ValueError('Duration must be finite')
        if not 1 <= len(self.objects) <= 8 or len({o.target_id for o in self.objects}) != len(self.objects):
            raise ValueError('One to eight uniquely labeled truth objects required')
        if len(self.cameras) != 2:
            raise ValueError('Exactly two camera fault configurations required')
        labels={o.target_id for o in self.objects}
        if any(o.object_id != '*' and o.object_id not in labels
               for camera in self.cameras for o in camera.occlusions):
            raise ValueError('Occlusion references an unknown object ID')
        for value in (self.association_gate_px,self.epipolar_gate_m,self.ambiguity_margin_m):
            if not isfinite(value) or value < 0:
                raise ValueError('Association gates must be finite and nonnegative')
        if not self.association_gate_px or not self.epipolar_gate_m:
            raise ValueError('Association gates must be positive')

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, data):
        data = dict(data)
        if 'objects' in data:
            objects=[]
            for item in data['objects']:
                item=dict(item)
                item['position_enu_m']=tuple(item['position_enu_m'])
                item['velocity_enu_mps']=tuple(item['velocity_enu_mps'])
                objects.append(Target(**item))
            data['objects']=tuple(objects)
        if 'cameras' in data:
            cameras = []
            for item in data['cameras']:
                item = dict(item)
                item['occlusions'] = tuple(Occlusion(**o) for o in item.get('occlusions',()))
                item['drop_ticks'] = tuple(item.get('drop_ticks',()))
                cameras.append(CameraFaults(**item))
            data['cameras'] = tuple(cameras)
        return cls(**data)
