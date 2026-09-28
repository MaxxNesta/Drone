"""Validated reproducible configurations and a simulation-clock-only scheduler."""
from dataclasses import asdict, dataclass, field
from .model import VehicleConfig, VehicleState, MovementLimits, PDConfig, BatteryConfig, Geofence, norm
from .mission import VehicleSimulator, Waypoint


@dataclass(frozen=True)
class Action:
    tick: int
    command: str

    def __post_init__(self):
        if type(self.tick) is not int or self.tick < 0:
            raise ValueError('Action tick must be a nonnegative integer')
        if self.command not in ('start','pause','resume','abort','emergency_stop'):
            raise ValueError('Unsupported virtual mission command')


@dataclass(frozen=True)
class Scenario:
    scenario_id: str = 'waypoint-tour'
    vehicle: VehicleConfig = field(default_factory=VehicleConfig)
    waypoints: tuple = field(default_factory=lambda: (Waypoint((10.,0.,5.)),Waypoint((10.,10.,5.)),Waypoint((0.,0.,2.))))
    actions: tuple = field(default_factory=lambda: (Action(0,'start'),))
    steps: int = 3000
    schema_version: int = 1

    def __post_init__(self):
        if self.schema_version != 1 or not isinstance(self.scenario_id,str) or not self.scenario_id:
            raise ValueError('Supported schema and scenario ID required')
        if type(self.steps) is not int or self.steps < 0:
            raise ValueError('Steps must be a nonnegative integer')
        if any(a.tick >= self.steps for a in self.actions):
            raise ValueError('Action must precede an executed update')
        if any(not self.vehicle.geofence.contains(w.position_enu_m) for w in self.waypoints):
            raise ValueError('Waypoint outside geofence')

    @classmethod
    def from_dict(cls, data):
        data = dict(data)
        vehicle = dict(data.pop('vehicle',{}))
        for key,kind in (('initial_state',VehicleState),('limits',MovementLimits),('controller',PDConfig),
                         ('battery',BatteryConfig),('geofence',Geofence)):
            if key in vehicle:
                vehicle[key] = kind(**vehicle[key])
        if 'waypoints' in data:
            data['waypoints'] = tuple(Waypoint(**w) for w in data['waypoints'])
        if 'actions' in data:
            data['actions'] = tuple(Action(**a) for a in data['actions'])
        return cls(vehicle=VehicleConfig(**vehicle),**data)

    def to_dict(self):
        return asdict(self)


def simulate(scenario):
    """Initial sample plus N updates; actions occur at the preceding tick boundary.

    Multiple actions at one tick execute in declared order. Invalid lifecycle
    transitions raise rather than silently ignoring an operator command.
    """
    sim = VehicleSimulator(scenario.waypoints,scenario.vehicle)
    schedule = {}
    for action in scenario.actions:
        schedule.setdefault(action.tick,[]).append(action.command)
    yield sim.telemetry(consume_events=True)
    for _ in range(scenario.steps):
        for command in schedule.get(sim.clock.tick,()):
            getattr(sim,command)()
        yield sim.step(sim.clock.advance())


def summarize(rows):
    if not rows:
        raise ValueError('At least the initial state is required')
    final = rows[-1]
    events = [event for row in rows for event in row['events']]
    return {'schema_version':1,'samples':len(rows),'duration_s':final['simulation_time_s'],
            'final_mission_state':final['mission_state'],'completed_waypoints':final['completed_waypoints'],
            'completion_time_s':next((e['time_s'] for e in events if e['kind']=='mission_completed'),None),
            'final_position_enu_m':final['vehicle']['position_enu_m'],
            'final_waypoint_error_m':final['waypoint_error_m'],
            'maximum_speed_mps':max(norm(r['vehicle']['velocity_enu_mps']) for r in rows),
            'maximum_acceleration_mps2':max(norm(r['vehicle']['acceleration_enu_mps2']) for r in rows),
            'battery_final_fraction':final['vehicle']['battery_fraction'],
            'stop_reason':final['stop_reason'],'events':events}
