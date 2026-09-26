"""Calibrated two-ray least squares with explicit validity and geometry gates."""
from dataclasses import dataclass
from math import isfinite, sqrt
from typing import Optional, Sequence
from .core import Camera, Detection, Vec3, dot, unit
from .tracking import detection_error


@dataclass(frozen=True)
class TriangulationConfig:
    max_age_s: float = 0.25
    max_time_skew_s: float = 0.01
    max_condition_number: float = 10000.0
    min_baseline_m: float = 0.01
    max_ray_separation_m: float = 0.25
    max_range_m: float = 500.0

    def __post_init__(self):
        if not all(isfinite(x) and x >= 0 for x in vars(self).values()):
            raise ValueError('Triangulation limits must be finite and nonnegative')
        if self.max_condition_number < 2 or self.min_baseline_m <= 0 or self.max_range_m <= 0:
            raise ValueError('Condition limit must be >=2; baseline and range must be positive')


@dataclass(frozen=True)
class TriangulationResult:
    valid: bool
    reason: str
    position_enu_m: Optional[Vec3] = None
    timestamp_s: Optional[float] = None  # midpoint of capture times, not processing time
    target_id: Optional[str] = None
    condition_number: Optional[float] = None
    ray_separation_m: Optional[float] = None
    ranges_m: Optional[tuple] = None


def camera_ray(camera: Camera, pixel) -> Vec3:
    """R_world_camera K^-1 [u,v,1], normalized in ENU."""
    if len(pixel) != 2 or not all(isfinite(x) for x in pixel):
        raise ValueError('Pixel must contain two finite values')
    k = camera.intrinsics
    optical = ((pixel[0]-k.cx)/k.fx, (pixel[1]-k.cy)/k.fy, 1.)
    return unit(tuple(sum(axis[i]*weight for axis, weight in
                          zip((camera.right,camera.down,camera.forward),optical))
                      for i in range(3)))


def triangulate(cameras: Sequence[Camera], observations: Sequence[Detection], now_s: float,
                config: TriangulationConfig = TriangulationConfig()) -> TriangulationResult:
    if not isfinite(now_s) or now_s < 0:
        raise ValueError('Processing time must be finite and nonnegative')
    fail = lambda reason: TriangulationResult(False, reason)
    if len(cameras) != 2 or len(observations) != 2:
        return fail('requires_two_observations')
    by_id = {c.camera_id:c for c in cameras}
    if len(by_id) != 2 or len({o.camera_id for o in observations}) != 2:
        return fail('duplicate_camera')
    ordered = []
    for observation in observations:
        camera = by_id.get(observation.camera_id)
        if camera is None:
            return fail('unknown_camera')
        error = detection_error(observation, camera)
        if error:
            return fail(error)
        age = now_s-observation.capture_time_s
        if age < -1e-9:
            return fail('future_observation')
        if age > config.max_age_s:
            return fail('stale_observation')
        ordered.append(camera)
    first, second = observations
    if first.target_id != second.target_id:
        return fail('target_mismatch')
    if abs(first.capture_time_s-second.capture_time_s) > config.max_time_skew_s:
        return fail('timestamp_mismatch')
    a, b = ordered
    baseline = tuple(y-x for x,y in zip(a.position_enu_m,b.position_enu_m))
    if sqrt(dot(baseline,baseline)) < config.min_baseline_m:
        return fail('insufficient_baseline')
    d, e = camera_ray(a, first.pixel), camera_ray(b, second.pixel)
    cosine = max(-1.,min(1.,dot(d,e)))
    # A=(I-dd^T)+(I-ee^T) has eigenvalues 2, 1+|d.e|, 1-|d.e|.
    minimum = 1-abs(cosine)
    condition = None if minimum <= 0 else 2/minimum
    if condition is None or condition > config.max_condition_number:
        return TriangulationResult(False,'ill_conditioned',condition_number=condition)
    # Closest points on the two lines; their midpoint minimizes squared
    # perpendicular distance to both lines (equivalent to solving A p=b).
    bd, be = dot(baseline,d), dot(baseline,e)
    denominator = 1-cosine*cosine
    s = (bd-cosine*be)/denominator
    t = (cosine*bd-be)/denominator
    if s <= 0 or t <= 0:
        return TriangulationResult(False,'behind_camera',condition_number=condition)
    p = tuple(o+s*v for o,v in zip(a.position_enu_m,d))
    q = tuple(o+t*v for o,v in zip(b.position_enu_m,e))
    difference = tuple(x-y for x,y in zip(p,q))
    separation = sqrt(dot(difference,difference))
    position = tuple((x+y)/2 for x,y in zip(p,q))
    ranges = tuple(sqrt(sum((x-y)**2 for x,y in zip(position,c.position_enu_m))) for c in ordered)
    reason = ('excessive_residual' if separation > config.max_ray_separation_m else
              'out_of_range' if max(ranges) > config.max_range_m else 'accepted')
    return TriangulationResult(reason == 'accepted',reason,
                               position if reason == 'accepted' else None,
                               (first.capture_time_s+second.capture_time_s)/2,
                               first.target_id,condition,separation,ranges)
