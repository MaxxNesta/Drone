"""Software-only ENU point mass. No actuator, autopilot or perception dependencies."""
from dataclasses import dataclass, field
from math import atan2, hypot, isfinite, pi

ZERO = (0.,0.,0.)


def vector(value):
    if len(value) != 3 or not all(isfinite(x) for x in value):
        raise ValueError('Expected three finite ENU coordinates')
    return tuple(value)


def norm(value):
    return hypot(*value)


def limit(value, maximum):
    vector(value)
    size = norm(value)
    if not isfinite(size):
        raise ValueError('Vector magnitude overflow')
    return tuple(x*min(1.,maximum/size) for x in value) if size else ZERO


def wrap(angle):
    return (angle+pi)%(2*pi)-pi


@dataclass(frozen=True)
class VehicleState:
    position_enu_m: tuple = (0.,0.,2.)
    velocity_enu_mps: tuple = ZERO
    acceleration_enu_mps2: tuple = ZERO
    heading_rad: float = 0.  # north=0; positive towards east
    battery_fraction: float = 1.

    def __post_init__(self):
        for name in ('position_enu_m','velocity_enu_mps','acceleration_enu_mps2'):
            object.__setattr__(self,name,vector(getattr(self,name)))
        if not isfinite(self.heading_rad) or not isfinite(self.battery_fraction) or not 0 <= self.battery_fraction <= 1:
            raise ValueError('Finite heading and battery in [0,1] required')
        object.__setattr__(self,'heading_rad',wrap(self.heading_rad))


@dataclass(frozen=True)
class MovementLimits:
    max_speed_mps: float = 5.
    max_acceleration_mps2: float = 3.
    max_heading_rate_radps: float = 1.5

    def __post_init__(self):
        if any(not isfinite(x) or x <= 0 for x in vars(self).values()):
            raise ValueError('Movement limits must be finite and positive')


@dataclass(frozen=True)
class PDConfig:
    kp: float = 1.2  # 1/s²
    kd: float = 2.2  # 1/s

    def __post_init__(self):
        if any(not isfinite(x) or x <= 0 for x in (self.kp,self.kd)):
            raise ValueError('PD gains must be finite and positive')


def pd_acceleration(state, target, controller, limits):
    """Bounded acceleration request, derivative on measured velocity, zero target velocity."""
    vector(target)
    return limit(tuple(controller.kp*(p-x)-controller.kd*v for p,x,v in
        zip(target,state.position_enu_m,state.velocity_enu_mps)),limits.max_acceleration_mps2)


@dataclass(frozen=True)
class BatteryConfig:
    idle_drain_per_s: float = .0002
    speed_drain_per_m: float = .00005
    acceleration_drain_per_mps: float = .00002
    low_fraction: float = .2
    critical_fraction: float = .1

    def __post_init__(self):
        if any(not isfinite(x) or x < 0 for x in vars(self).values()):
            raise ValueError('Battery parameters must be finite and nonnegative')
        if not 0 <= self.critical_fraction < self.low_fraction <= 1:
            raise ValueError('Ordered battery thresholds required')


@dataclass(frozen=True)
class Geofence:
    minimum_enu_m: tuple = (-50.,-50.,0.)
    maximum_enu_m: tuple = (50.,50.,30.)

    def __post_init__(self):
        object.__setattr__(self,'minimum_enu_m',vector(self.minimum_enu_m))
        object.__setattr__(self,'maximum_enu_m',vector(self.maximum_enu_m))
        if any(a >= b for a,b in zip(self.minimum_enu_m,self.maximum_enu_m)):
            raise ValueError('Fence minimum must be below maximum on all axes')

    def contains(self, position):
        vector(position)
        return all(a <= p <= b for a,p,b in zip(self.minimum_enu_m,position,self.maximum_enu_m))


@dataclass(frozen=True)
class VehicleConfig:
    dt_s: float = .02
    drag_per_s: float = .15
    limits: MovementLimits = field(default_factory=MovementLimits)
    controller: PDConfig = field(default_factory=PDConfig)
    battery: BatteryConfig = field(default_factory=BatteryConfig)
    geofence: Geofence = field(default_factory=Geofence)
    initial_state: VehicleState = field(default_factory=VehicleState)

    def __post_init__(self):
        if not isfinite(self.dt_s) or not 0 < self.dt_s <= .1:
            raise ValueError('Vehicle dt must be in (0,0.1] seconds')
        if not isfinite(self.drag_per_s) or self.drag_per_s < 0:
            raise ValueError('Drag must be finite and nonnegative')
        # Jury condition for the unsaturated semi-implicit PD linearization.
        if self.controller.kp*self.dt_s**2+2*(self.controller.kd+self.drag_per_s)*self.dt_s >= 4:
            raise ValueError('PD gains/drag exceed the fixed-step local stability bound')
        if not self.geofence.contains(self.initial_state.position_enu_m):
            raise ValueError('Initial position is outside the geofence')
        if norm(self.initial_state.velocity_enu_mps) > self.limits.max_speed_mps:
            raise ValueError('Initial velocity exceeds movement limits')
        if norm(self.initial_state.acceleration_enu_mps2) > self.limits.max_acceleration_mps2:
            raise ValueError('Initial acceleration exceeds movement limits')


def integrate(state, requested_acceleration, config):
    """Semi-implicit Euler, speed projection, and realized finite-difference acceleration.

    Geofence and emergency policy belong to the mission engine, not this integrator.
    """
    dt,limits = config.dt_s,config.limits
    if norm(state.velocity_enu_mps) > limits.max_speed_mps+1e-12:
        raise ValueError('Input velocity exceeds configured movement limits')
    command = limit(requested_acceleration,limits.max_acceleration_mps2)
    acceleration = limit(tuple(a-config.drag_per_s*v for a,v in zip(command,state.velocity_enu_mps)),
                         limits.max_acceleration_mps2)
    velocity = limit(tuple(v+a*dt for v,a in zip(state.velocity_enu_mps,acceleration)),limits.max_speed_mps)
    actual = tuple((v-u)/dt for v,u in zip(velocity,state.velocity_enu_mps))
    position = tuple(p+v*dt for p,v in zip(state.position_enu_m,velocity))
    heading = state.heading_rad
    if hypot(*velocity[:2]) > 1e-8:
        difference = wrap(atan2(velocity[0],velocity[1])-heading)
        heading += max(-limits.max_heading_rate_radps*dt,min(limits.max_heading_rate_radps*dt,difference))
    b = config.battery
    drain = dt*(b.idle_drain_per_s+b.speed_drain_per_m*norm(velocity)+b.acceleration_drain_per_mps*norm(actual))
    return VehicleState(position,velocity,actual,heading,max(0.,state.battery_fraction-drain))
