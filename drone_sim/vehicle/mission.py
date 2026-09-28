"""Waypoint lifecycle and virtual-only safety policy on the shared simulation clock."""
from dataclasses import asdict, dataclass, replace
from math import ceil, isfinite
from ..clock import FixedStepClock
from .model import ZERO, VehicleConfig, vector, norm, pd_acceleration, integrate


@dataclass(frozen=True)
class Waypoint:
    position_enu_m: tuple
    position_tolerance_m: float = .1
    speed_tolerance_mps: float = .1
    settle_s: float = .5

    def __post_init__(self):
        object.__setattr__(self,'position_enu_m',vector(self.position_enu_m))
        if (any(not isfinite(x) or x <= 0 for x in (self.position_tolerance_m,self.speed_tolerance_mps))
                or not isfinite(self.settle_s) or self.settle_s < 0):
            raise ValueError('Positive tolerances and nonnegative finite settling time required')


class VehicleSimulator:
    def __init__(self, waypoints=(), config=None):
        self.config = config or VehicleConfig()
        self.waypoints = tuple(waypoints)
        if any(not self.config.geofence.contains(w.position_enu_m) for w in self.waypoints):
            raise ValueError('All waypoints must be inside the geofence')
        self.state = self.config.initial_state
        self.clock = FixedStepClock(self.config.dt_s)
        self.status = 'idle'
        self.completed_waypoints = 0
        self.settle_ticks = 0
        self.hold_position = self.state.position_enu_m
        self.control_target = self.hold_position
        self.stop_reason = None
        self.events = []
        self.low_battery = self.state.battery_fraction <= self.config.battery.low_fraction
        if self.state.battery_fraction <= self.config.battery.critical_fraction:
            self._stop('critical_battery')

    def _event(self, kind, **details):
        self.events.append({'kind':kind,'tick':self.clock.tick,'time_s':self.clock.time_s,**details})

    def start(self):
        if self.status != 'idle' or not self.waypoints:
            raise ValueError('Start requires an idle simulator with a nonempty mission')
        self.status = 'running'
        self._event('mission_started')

    def pause(self):
        if self.status != 'running':
            raise ValueError('Only a running mission can pause')
        self.status = 'paused'
        self.settle_ticks = 0
        self.hold_position = self.state.position_enu_m
        self._event('mission_paused')

    def resume(self):
        if self.status != 'paused':
            raise ValueError('Only a paused mission can resume')
        self.status = 'running'
        self._event('mission_resumed')

    def abort(self):
        self._stop('mission_abort')

    def emergency_stop(self):
        self._stop('emergency_stop')

    def _stop(self, reason):
        if self.status == 'aborted':
            return  # latch the first reason; no restart/reset API
        self._event('virtual_stop',reason=reason,velocity_reset_enu_mps=tuple(-v for v in self.state.velocity_enu_mps))
        self.state = replace(self.state,velocity_enu_mps=ZERO,acceleration_enu_mps2=ZERO)
        self.status,self.stop_reason = 'aborted',reason
        self.settle_ticks = 0
        self.hold_position = self.state.position_enu_m

    def telemetry(self, consume_events=False):
        target = None
        if self.waypoints:
            target = self.waypoints[min(self.completed_waypoints,len(self.waypoints)-1)].position_enu_m
        row = {'schema_version':1,'type':'vehicle_state','coordinate_frame':'ENU','time_domain':'simulation',
                'tick':self.clock.tick,'simulation_time_s':self.clock.time_s,'dt_s':self.clock.dt_s,
                'vehicle':asdict(self.state),'mission_state':self.status,
                'completed_waypoints':self.completed_waypoints,'waypoint_count':len(self.waypoints),
                'active_waypoint_index':self.completed_waypoints if self.status in ('running','paused') else None,
                'target_enu_m':target,'control_target_enu_m':self.control_target,
                'waypoint_error_m':norm(tuple(t-p for t,p in zip(target,self.state.position_enu_m))) if target is not None else None,
                'settle_ticks':self.settle_ticks,'low_battery':self.low_battery,
                'emergency_stopped':self.status == 'aborted','stop_reason':self.stop_reason,
                'events':list(self.events)}
        if consume_events:
            self.events = []
        return row

    def step(self, next_clock=None):
        clock = next_clock or self.clock.advance()
        if clock.dt_s != self.clock.dt_s or clock.tick != self.clock.tick+1:
            raise ValueError('Expected the next tick of the same fixed-step clock')
        self.clock = clock
        if self.status != 'aborted':
            if self.status == 'running':
                self.control_target = self.waypoints[self.completed_waypoints].position_enu_m
            else:
                self.control_target = self.hold_position
            command = pd_acceleration(self.state,self.control_target,self.config.controller,self.config.limits)
            candidate = integrate(self.state,command,self.config)
            if not self.config.geofence.contains(candidate.position_enu_m):
                # Reject the entire attempted movement: never teleport to a clipped boundary.
                self._stop('geofence_predicted_crossing')
            else:
                self.state = candidate
                if not self.low_battery and candidate.battery_fraction <= self.config.battery.low_fraction:
                    self.low_battery = True
                    self._event('battery_low')
                if candidate.battery_fraction <= self.config.battery.critical_fraction:
                    self._stop('critical_battery')
                elif self.status == 'running':
                    w = self.waypoints[self.completed_waypoints]
                    settled = norm(tuple(t-p for t,p in zip(w.position_enu_m,self.state.position_enu_m))) <= w.position_tolerance_m
                    settled = settled and norm(self.state.velocity_enu_mps) <= w.speed_tolerance_mps
                    self.settle_ticks = self.settle_ticks+1 if settled else 0
                    if settled and self.settle_ticks >= max(1,ceil(w.settle_s/self.config.dt_s)):
                        self._event('waypoint_reached',waypoint_index=self.completed_waypoints)
                        self.completed_waypoints += 1
                        self.settle_ticks = 0
                        if self.completed_waypoints == len(self.waypoints):
                            self.status = 'completed'
                            self.hold_position = w.position_enu_m
                            self._event('mission_completed')
        return self.telemetry(consume_events=True)
