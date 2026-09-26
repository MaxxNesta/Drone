"""Single-target image-space Kalman tracking with an injected observation clock."""
from dataclasses import dataclass
from math import hypot, isfinite
from typing import Optional, Tuple
from .core import Camera, Detection, Pixel


def detection_error(detection: Detection, camera: Camera) -> Optional[str]:
    """Validate observation payload independently of its age or target association."""
    if detection.camera_id != camera.camera_id:
        return 'wrong_camera'
    if detection.schema_version != 1 or detection.time_domain != 'simulation':
        return 'unsupported_schema_or_time_domain'
    if detection.image_size != (camera.intrinsics.width, camera.intrinsics.height):
        return 'wrong_image_size'
    if not detection.target_id:
        return 'missing_target_id'
    if (type(detection.frame_sequence) is not int or detection.frame_sequence < 0
            or not isfinite(detection.capture_time_s) or detection.capture_time_s < 0):
        return 'invalid_timestamp_or_sequence'
    if not detection.valid or detection.pixel is None:
        return 'invalid_observation'
    if len(detection.pixel) != 2 or not all(isfinite(x) for x in detection.pixel):
        return 'invalid_pixel'
    u, v = detection.pixel
    if not (0 <= u < camera.intrinsics.width and 0 <= v < camera.intrinsics.height):
        return 'outside_image'
    return None


@dataclass(frozen=True)
class FilterConfig:
    measurement_variance_px2: float = 1.0
    acceleration_spectral_density: float = 4.0  # px²/s³; continuous white acceleration
    initial_velocity_variance: float = 100.0   # (px/s)²
    gate_squared_mahalanobis: float = 16.0
    stale_after_s: float = 0.25
    lost_after_s: float = 0.75
    centered_radius_px: float = 3.0

    def __post_init__(self):
        values = tuple(vars(self).values())
        if not all(isfinite(x) and x >= 0 for x in values):
            raise ValueError('Filter parameters must be finite and nonnegative')
        if (self.measurement_variance_px2 == 0 or self.initial_velocity_variance == 0
                or self.gate_squared_mahalanobis == 0
                or not 0 < self.stale_after_s < self.lost_after_s):
            raise ValueError('Positive variances/gate and ordered freshness thresholds required')


@dataclass(frozen=True)
class TrackEstimate:
    camera_id: str
    target_id: str
    timestamp_s: float
    last_measurement_time_s: Optional[float]
    age_s: Optional[float]
    status: str  # tracking, coasting, stale, lost
    centered: Optional[bool]  # orthogonal to status; None when no usable state
    pixel: Optional[Pixel]
    velocity_px_s: Optional[Pixel]
    covariance: Optional[Tuple[Tuple[float, ...], ...]]  # [u,v,du,dv]
    accepted: bool
    reason: str
    observation: Detection  # invalid unless an observation was accepted at this timestamp


class _Axis:
    """One [position, velocity] block of a separable four-state Kalman filter."""
    def __init__(self, position, config):
        self.position, self.velocity = position, 0.0
        self.a = config.measurement_variance_px2
        self.b = 0.0
        self.c = config.initial_velocity_variance

    def predict(self, dt, q):
        self.position += dt * self.velocity
        self.a, self.b, self.c = (
            self.a + 2*dt*self.b + dt*dt*self.c + q*dt**3/3,
            self.b + dt*self.c + q*dt*dt/2,
            self.c + q*dt)

    def correct(self, measurement, variance):
        innovation = measurement - self.position
        s = self.a + variance
        k0, k1 = self.a/s, self.b/s
        self.position += k0*innovation
        self.velocity += k1*innovation
        # Joseph form: (I-KH) P (I-KH)^T + KRK^T.
        a, b, c = self.a, self.b, self.c
        self.a = (1-k0)**2*a + k0*k0*variance
        self.b = (1-k0)*(b-k1*a) + k0*k1*variance
        self.c = c-2*k1*b+k1*k1*a+k1*k1*variance


class CameraTracker:
    """Tracks one declared target/camera. No global clock, threads or external state.

    Measurements must be synchronous with step(now_s). Delayed observations are
    rejected, rather than applying past measurements to a current-time state.
    """
    def __init__(self, camera: Camera, target_id: str, config: FilterConfig = FilterConfig()):
        if not target_id:
            raise ValueError('Target identity is required')
        self.camera, self.target_id, self.config = camera, target_id, config
        self._axes = None
        self._time = None
        self._last_measurement = None
        self._last_sequence = -1

    def step(self, now_s: float, detection: Optional[Detection] = None) -> TrackEstimate:
        if not isfinite(now_s) or now_s < 0 or (self._time is not None and now_s < self._time):
            raise ValueError('Time must be finite, nonnegative and monotonic')
        cfg = self.config
        if self._last_measurement is not None and now_s-self._last_measurement > cfg.lost_after_s:
            self._axes = None  # reacquisition must seed from a new measurement
        if self._axes is not None:
            dt = now_s-self._time
            for axis in self._axes:
                axis.predict(dt, cfg.acceleration_spectral_density)
        previous_time = self._time
        self._time = now_s
        reason = 'missing_observation'
        accepted = False
        if detection is not None:
            reason = detection_error(detection, self.camera)
            if reason is None and detection.target_id != self.target_id:
                reason = 'wrong_target'
            if reason is None:
                age = now_s-detection.capture_time_s
                if age < -1e-9:
                    reason = 'future_observation'
                elif age > cfg.stale_after_s:
                    reason = 'stale_observation'
                elif age > 1e-9:
                    reason = 'delayed_observation'
                elif (detection.frame_sequence <= self._last_sequence
                      or (previous_time is not None and now_s == previous_time)):
                    reason = 'duplicate_or_out_of_order'
            if reason is None:
                if self._axes is None:
                    self._axes = [_Axis(p, cfg) for p in detection.pixel]
                else:
                    distance = sum((p-axis.position)**2/(axis.a+cfg.measurement_variance_px2)
                                   for p, axis in zip(detection.pixel, self._axes))
                    if distance > cfg.gate_squared_mahalanobis:
                        reason = 'innovation_rejected'
                    else:
                        for p, axis in zip(detection.pixel, self._axes):
                            axis.correct(p, cfg.measurement_variance_px2)
                if reason is None:
                    accepted = True
                    reason = 'accepted'
                    self._last_measurement = now_s
                    self._last_sequence = detection.frame_sequence
        age = None if self._last_measurement is None else now_s-self._last_measurement
        status = ('lost' if self._axes is None else
                  'stale' if age > cfg.stale_after_s else
                  'tracking' if accepted else 'coasting')
        pixel = velocity = covariance = centered = None
        if self._axes is not None:
            x, y = self._axes
            pixel, velocity = (x.position, y.position), (x.velocity, y.velocity)
            covariance = ((x.a,0.,x.b,0.), (0.,y.a,0.,y.b),
                          (x.b,0.,x.c,0.), (0.,y.b,0.,y.c))
            centered = hypot(pixel[0]-self.camera.intrinsics.cx,
                             pixel[1]-self.camera.intrinsics.cy) <= cfg.centered_radius_px
        observation = Detection(self.camera.camera_id,
                                detection.frame_sequence if accepted else max(0,self._last_sequence),
                                now_s, (self.camera.intrinsics.width,self.camera.intrinsics.height),
                                self.target_id, pixel if accepted else None, accepted, reason)
        return TrackEstimate(self.camera.camera_id,self.target_id,now_s,self._last_measurement,
                             age,status,centered,pixel,velocity,covariance,accepted,reason,observation)
