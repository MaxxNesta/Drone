"""Versioned inputs composed from Stage 6 vehicle and Stage 7 mission records."""
from dataclasses import dataclass, asdict
import re
from math import isfinite
from ..planning import Mission
from ..planning.execution import to_scenario
from ..vehicle import VehicleConfig
from ..vehicle.scenario import Scenario


def identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', value):
        raise ValueError('IDs require 1–64 ASCII letters, digits, underscores or hyphens')


@dataclass(frozen=True)
class Member:
    vehicle_id: str
    mission: Mission
    vehicle: VehicleConfig

    def __post_init__(self):
        identifier(self.vehicle_id)
        to_scenario(self.mission, self.vehicle, 1)

    def to_dict(self):
        return {'vehicle_id': self.vehicle_id, 'mission': self.mission.to_dict(), 'vehicle': asdict(self.vehicle)}

    @classmethod
    def from_dict(cls, data):
        data = dict(data)
        data['mission'] = Mission.from_dict(data['mission'])
        data['vehicle'] = Scenario.from_dict({'vehicle': data['vehicle'], 'waypoints': [], 'actions': [], 'steps': 1}).vehicle
        return cls(**data)


@dataclass(frozen=True)
class FleetAction:
    tick: int
    command: str
    vehicle_id: str = None

    def __post_init__(self):
        if type(self.tick) is not int or self.tick < 0:
            raise ValueError('Action tick must be a nonnegative integer')
        if self.command not in ('start', 'pause', 'resume', 'abort', 'emergency_stop'):
            raise ValueError('Unknown virtual command')
        if self.vehicle_id is not None:
            identifier(self.vehicle_id)


@dataclass(frozen=True)
class FleetConfig:
    fleet_id: str
    members: tuple
    steps: int = 3000
    actions: tuple = (FleetAction(0, 'start'),)
    proximity_m: float = 2.
    route_conflict_m: float = 2.
    schema_version: int = 1

    def __post_init__(self):
        identifier(self.fleet_id)
        if type(self.schema_version) is not int or self.schema_version != 1:
            raise ValueError('Unsupported fleet schema')
        if type(self.steps) is not int or not 1 <= self.steps <= 100000:
            raise ValueError('Steps must be in [1,100000]')
        members = tuple(sorted(self.members, key=lambda m: m.vehicle_id))
        if not 1 <= len(members) <= 16 or len({m.vehicle_id.casefold() for m in members}) != len(members):
            raise ValueError('Require 1–16 unique vehicle IDs')
        if len({m.vehicle.dt_s for m in members}) != 1:
            raise ValueError('All vehicles must share exactly the same dt')
        if sum(len(m.mission.waypoints) for m in members) > 512:
            raise ValueError('Fleet routes are limited to 512 total legs')
        for value in (self.proximity_m, self.route_conflict_m):
            if type(value) not in (int, float) or not isfinite(value) or value <= 0:
                raise ValueError('Diagnostic thresholds must be positive and finite')
        actions = tuple(self.actions)
        ids = {m.vehicle_id for m in members}
        if any(a.tick >= self.steps or (a.vehicle_id is not None and a.vehicle_id not in ids) for a in actions):
            raise ValueError('Action outside run or unknown vehicle')
        object.__setattr__(self, 'members', members)
        object.__setattr__(self, 'actions', actions)

    @property
    def dt_s(self):
        return self.members[0].vehicle.dt_s

    def to_dict(self):
        return {'schema_version': self.schema_version, 'fleet_id': self.fleet_id,
                'members': [m.to_dict() for m in self.members], 'steps': self.steps,
                'actions': [asdict(a) for a in self.actions], 'proximity_m': self.proximity_m,
                'route_conflict_m': self.route_conflict_m}

    @classmethod
    def from_dict(cls, data):
        data = dict(data)
        data['members'] = tuple(Member.from_dict(m) for m in data['members'])
        if 'actions' in data:
            data['actions'] = tuple(FleetAction(**a) for a in data['actions'])
        return cls(**data)
