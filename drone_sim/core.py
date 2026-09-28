"""ENU meters -> calibrated optical right/down/forward -> image pixels."""
from dataclasses import dataclass
from .clock import FixedStepClock
from math import isclose, isfinite, sqrt
from typing import Iterator, Optional, Tuple

Vec3 = Tuple[float, float, float]
Pixel = Tuple[float, float]


def vector(value: Vec3) -> None:
    if len(value) != 3 or not all(isfinite(x) for x in value):
        raise ValueError('Expected three finite coordinates')


def dot(a: Vec3, b: Vec3) -> float:
    return sum(x * y for x, y in zip(a, b))


def cross(a: Vec3, b: Vec3) -> Vec3:
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def unit(a: Vec3) -> Vec3:
    length = sqrt(dot(a, a))
    if length < 1e-12:
        raise ValueError('Camera direction and up hint must be nonzero and nonparallel')
    return tuple(x / length for x in a)


@dataclass(frozen=True)
class Intrinsics:
    width: int = 640
    height: int = 480
    fx: float = 400.0
    fy: float = 400.0
    cx: float = 320.0
    cy: float = 240.0

    def __post_init__(self):
        if (type(self.width) is not int or type(self.height) is not int
                or self.width <= 0 or self.height <= 0):
            raise ValueError('Image dimensions must be positive integers')
        if not all(isfinite(x) for x in (self.fx, self.fy, self.cx, self.cy)):
            raise ValueError('Intrinsics must be finite')
        if self.fx <= 0 or self.fy <= 0:
            raise ValueError('Focal lengths must be positive')


@dataclass(frozen=True)
class Camera:
    camera_id: str
    position_enu_m: Vec3
    # Rows of R_camera_world: optical basis vectors expressed in ENU.
    right: Vec3
    down: Vec3
    forward: Vec3
    intrinsics: Intrinsics = Intrinsics()
    calibration_version: str = 'ideal-pinhole-v1'

    def __post_init__(self):
        if not self.camera_id:
            raise ValueError('Camera ID is required')
        vector(self.position_enu_m)
        basis = (self.right, self.down, self.forward)
        for row in basis:
            vector(row)
        for i in range(3):
            for j in range(3):
                if not isclose(dot(basis[i], basis[j]), float(i == j), abs_tol=1e-9):
                    raise ValueError('Camera basis must be orthonormal')
        if not isclose(dot(cross(self.right, self.down), self.forward), 1., abs_tol=1e-9):
            raise ValueError('Camera basis must be right-handed')

    @classmethod
    def look_at(cls, camera_id: str, position: Vec3, aim: Vec3,
                intrinsics: Intrinsics = Intrinsics(), up: Vec3 = (0., 0., 1.)):
        vector(position)
        vector(aim)
        vector(up)
        forward = unit(tuple(a - p for a, p in zip(aim, position)))
        right = unit(cross(forward, up))
        down = cross(forward, right)
        return cls(camera_id, position, right, down, forward, intrinsics)

    def optical_coordinates(self, point: Vec3) -> Vec3:
        vector(point)
        offset = tuple(p - o for p, o in zip(point, self.position_enu_m))
        return tuple(dot(axis, offset) for axis in (self.right, self.down, self.forward))

    def project(self, point: Vec3):
        """Return (pixel or None, reason); visible domain is [0,w) × [0,h)."""
        x, y, z = self.optical_coordinates(point)
        if z <= 0:
            return None, 'behind_or_on_camera_plane'
        k = self.intrinsics
        u, v = k.fx*x/z+k.cx, k.fy*y/z+k.cy
        if not (isfinite(u) and isfinite(v) and 0 <= u < k.width and 0 <= v < k.height):
            return None, 'outside_image'
        return (u, v), 'visible'


@dataclass(frozen=True)
class Target:
    position_enu_m: Vec3 = (0., 20., 5.)
    velocity_enu_mps: Vec3 = (0.5, 0.25, 0.1)
    target_id: str = 'target-0'

    def __post_init__(self):
        vector(self.position_enu_m)
        vector(self.velocity_enu_mps)
        if not self.target_id:
            raise ValueError('Target ID is required')


@dataclass(frozen=True)
class Detection:
    camera_id: str
    frame_sequence: int
    capture_time_s: float
    image_size: Tuple[int, int]
    target_id: str
    pixel: Optional[Pixel]
    valid: bool
    reason: str
    schema_version: int = 1
    time_domain: str = 'simulation'
    # Stage 1 is a point observation, not an invented YOLO bounding box/class.


@dataclass(frozen=True)
class Frame:
    tick: int
    simulation_time_s: float
    truth: Target
    detections: Tuple[Detection, Detection]


@dataclass(frozen=True)
class Scenario:
    cameras: Tuple[Camera, Camera]
    target: Target = Target()
    dt_s: float = 0.02
    steps: int = 100
    scenario_id: str = 'two-camera-constant-velocity'

    def __post_init__(self):
        if not isfinite(self.dt_s) or self.dt_s <= 0:
            raise ValueError('dt_s must be finite and positive')
        if type(self.steps) is not int or self.steps < 0:
            raise ValueError('steps must be a nonnegative integer')
        if not isfinite(self.steps * self.dt_s):
            raise ValueError('Simulation duration must be finite')
        if len(self.cameras) != 2 or len({c.camera_id for c in self.cameras}) != 2:
            raise ValueError('Exactly two uniquely identified cameras are required')
        if self.cameras[0].position_enu_m == self.cameras[1].position_enu_m:
            raise ValueError('Independent camera origins are required')


def default_scenario(dt_s: float = 0.02, steps: int = 100) -> Scenario:
    # Parallel north-facing cameras, 10 m east-west baseline, 2 m height.
    cameras = tuple(Camera.look_at(f'camera-{i}', (east, 0., 2.), (east, 20., 2.))
                    for i, east in enumerate((-5., 5.)))
    return Scenario(cameras, dt_s=dt_s, steps=steps)


def simulate(scenario: Scenario) -> Iterator[Frame]:
    """Yield the initial frame plus `steps` updates, with no wall-clock access."""
    truth = scenario.target
    clock = FixedStepClock(scenario.dt_s)
    for tick in range(scenario.steps + 1):
        timestamp = clock.time_s
        observations = []
        for camera in scenario.cameras:
            pixel, reason = camera.project(truth.position_enu_m)
            observations.append(Detection(
                camera.camera_id, tick, timestamp,
                (camera.intrinsics.width, camera.intrinsics.height),
                truth.target_id, pixel, pixel is not None, reason))
        yield Frame(tick, timestamp, truth, tuple(observations))
        if tick < scenario.steps:
            position = tuple(p + v * scenario.dt_s for p, v in
                             zip(truth.position_enu_m, truth.velocity_enu_mps))
            truth = Target(position, truth.velocity_enu_mps, truth.target_id)
            clock = clock.advance()
