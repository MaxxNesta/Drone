"""Version 1 in-process, read-only simulator boundary (not the Stage 9 wire schema)."""
from dataclasses import asdict, dataclass
from math import isfinite, pi
from typing import Optional, Protocol, Tuple
from drone_sim.fleet.config import identifier

Vector = Tuple[float, float, float]


def number(value, name, minimum=None):
    if type(value) not in (int, float) or not isfinite(value) or (minimum is not None and value < minimum):
        raise ValueError('Invalid '+name)


def vector(value):
    if not isinstance(value, (tuple, list)) or len(value) != 3:
        raise ValueError('Expected three finite coordinates')
    for component in value:
        number(component, 'coordinate')
    return tuple(value)


def enu_to_ned(value):
    east, north, up = vector(value)
    return north, east, -up


def ned_to_enu(value):
    north, east, down = vector(value)
    return east, north, -down


def wrap(angle):
    number(angle, 'angle')
    return (angle+pi) % (2*pi)-pi


def enu_yaw_to_heading(yaw):
    """ENU east-zero counterclockwise yaw -> north-zero clockwise heading."""
    return wrap(pi/2-yaw)


@dataclass(frozen=True)
class VehicleSample:
    vehicle_id: str
    source_id: str
    source_time_s: float
    received_monotonic_s: float
    provenance: str  # simulation_truth or autopilot_estimate; never inferred
    position_enu_m: Optional[Vector] = None
    velocity_enu_mps: Optional[Vector] = None
    acceleration_enu_mps2: Optional[Vector] = None
    heading_rad: Optional[float] = None
    battery_fraction: Optional[float] = None
    mission_state: str = 'unknown'
    completed_waypoints: Optional[int] = None
    waypoint_count: Optional[int] = None

    def __post_init__(self):
        identifier(self.vehicle_id)
        identifier(self.source_id)
        number(self.source_time_s, 'source time', 0)
        number(self.received_monotonic_s, 'receipt time', 0)
        if self.provenance not in ('simulation_truth', 'autopilot_estimate'):
            raise ValueError('Explicit supported provenance required')
        for name in ('position_enu_m', 'velocity_enu_mps', 'acceleration_enu_mps2'):
            if getattr(self, name) is not None:
                object.__setattr__(self, name, vector(getattr(self, name)))
        if self.heading_rad is not None:
            object.__setattr__(self, 'heading_rad', wrap(self.heading_rad))
        if self.battery_fraction is not None:
            number(self.battery_fraction, 'battery', 0)
            if self.battery_fraction > 1:
                raise ValueError('Battery must be a fraction')
        if self.mission_state not in ('unknown', 'idle', 'running', 'paused', 'completed', 'aborted'):
            raise ValueError('Unsupported mission state')
        counts = self.completed_waypoints, self.waypoint_count
        if counts != (None, None):
            if any(type(v) is not int or v < 0 for v in counts) or counts[0] > counts[1]:
                raise ValueError('Invalid mission progress')


@dataclass(frozen=True)
class Capabilities:
    backend: str
    clock_authority: str
    exact_replay: bool
    externally_stepped: bool
    commands: tuple = ()
    schema_version: int = 1


class SimulatorAdapter(Protocol):
    capabilities: Capabilities

    def snapshot(self, now_monotonic_s: float) -> dict:
        """Detached versioned snapshot; never advances time or executes commands."""
        ...


def snapshot_record(capabilities, epoch, sequence, time_s, samples, now, *,
                    available=True, reason=None, stale_after_s=.5, lost_after_s=2.):
    identifier(epoch)
    if type(sequence) is not int or sequence < 0:
        raise ValueError('Invalid sequence')
    number(time_s, 'simulation time', 0)
    number(now, 'monotonic time', 0)
    number(stale_after_s, 'stale threshold', 0)
    number(lost_after_s, 'lost threshold', 0)
    if lost_after_s <= stale_after_s:
        raise ValueError('Lost threshold must exceed stale threshold')
    ids = [sample.vehicle_id.casefold() for sample in samples]
    if len(ids) != len(set(ids)):
        raise ValueError('Duplicate vehicle identity')
    vehicles = []
    for sample in sorted(samples, key=lambda s: s.vehicle_id):
        if sample.received_monotonic_s > now or sample.source_time_s > time_s:
            raise ValueError('Sample is ahead of its clock')
        age = now-sample.received_monotonic_s
        source_age = time_s-sample.source_time_s
        oldest = max(age, source_age)
        freshness = 'lost' if oldest >= lost_after_s else 'stale' if oldest >= stale_after_s else 'fresh'
        vehicles.append(dict(asdict(sample), receipt_age_s=age, simulation_age_s=source_age, freshness=freshness))
    return {'schema_version': 1, 'type': 'simulator_snapshot', 'backend': capabilities.backend,
            'capabilities': asdict(capabilities), 'epoch': epoch, 'sequence': sequence,
            'coordinate_frame': 'ENU', 'time_domain': 'simulation', 'simulation_time_s': time_s,
            'availability': 'available' if available else 'unavailable', 'reason': reason,
            'vehicles': vehicles}
