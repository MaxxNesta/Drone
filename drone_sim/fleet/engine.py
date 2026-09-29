"""One authoritative clock; existing engines retain individual mission states."""
from collections import Counter
from copy import deepcopy
from ..clock import FixedStepClock
from ..planning.execution import to_scenario
from ..vehicle import VehicleSimulator
from ..vehicle.scenario import summarize as vehicle_summary
from .diagnostics import proximity, route_conflicts


def status(rows):
    counts = Counter(r['mission_state'] for r in rows.values())
    total = len(rows)
    terminal = counts['completed']+counts['aborted'] == total
    if counts['completed'] == total:
        label = 'completed'
    elif terminal:
        label = 'finished_with_failures'
    else:
        label = next(s for s in ('running', 'paused', 'idle') if counts[s])
    return {'status': label, 'state_counts': dict(sorted(counts.items())),
            'all_terminal': terminal, 'all_completed': counts['completed'] == total,
            'degraded': bool(counts['aborted']), 'completed_fraction': counts['completed']/total,
            'completed_waypoints': sum(r['completed_waypoints'] for r in rows.values()),
            'waypoint_count': sum(r['waypoint_count'] for r in rows.values())}


class FleetSimulator:
    def __init__(self, config):
        self.config = config
        self.clock = FixedStepClock(config.dt_s)
        self._engines, self._missions, self._failures = {}, {}, {}
        self._commands = []
        for member in config.members:
            scenario = to_scenario(member.mission, member.vehicle, config.steps)
            engine = VehicleSimulator(scenario.waypoints, scenario.vehicle)
            engine.clock = self.clock
            self._engines[member.vehicle_id] = engine
            self._missions[member.vehicle_id] = member.mission
        self.route_advisories = route_conflicts(config)

    def command(self, command, vehicle_id=None):
        """Broadcasts affect eligible states only; targeted commands are strict."""
        eligible = {'start': ('idle',), 'pause': ('running',), 'resume': ('paused',),
                    'abort': ('idle', 'running', 'paused', 'completed'),
                    'emergency_stop': ('idle', 'running', 'paused', 'completed')}
        if command not in eligible:
            raise ValueError('Unknown virtual command')
        if vehicle_id is not None and vehicle_id not in self._engines:
            raise ValueError('Unknown vehicle ID')
        ids = [vehicle_id] if vehicle_id is not None else list(self._engines)
        selected = [key for key in ids if self._engines[key].status in eligible[command]]
        if vehicle_id is not None and not selected:
            if command in ('abort', 'emergency_stop'):
                selected = ids  # Existing stop commands are idempotent.
            else:
                raise ValueError('Command invalid for vehicle mission state')
        for key in selected:
            getattr(self._engines[key], command)()
        self._commands.append({'tick': self.clock.tick, 'command': command, 'vehicle_id': vehicle_id,
                               'affected_vehicle_ids': selected})
        return tuple(selected)

    def start(self): return self.command('start')
    def pause(self): return self.command('pause')
    def resume(self): return self.command('resume')

    def _record(self, rows, previous, consume=False):
        positions = {key: r['vehicle']['position_enu_m'] for key, r in rows.items()}
        record = {'schema_version': 1, 'type': 'fleet_state', 'fleet_id': self.config.fleet_id,
                  'coordinate_frame': 'ENU', 'time_domain': 'simulation', 'tick': self.clock.tick,
                  'simulation_time_s': self.clock.time_s, 'dt_s': self.clock.dt_s,
                  'vehicles': {key: dict(r, vehicle_id=key, fleet_id=self.config.fleet_id,
                                        fleet_failure=self._failures.get(key)) for key, r in rows.items()},
                  'summary': status(rows), 'commands': deepcopy(self._commands),
                  'proximity': proximity(previous or positions, positions, self.config.proximity_m)}
        if consume:
            self._commands = []
        return record

    def telemetry(self):
        """Read-only snapshot; never drains command or engine events."""
        return deepcopy(self._record({key: engine.telemetry() for key, engine in self._engines.items()}, None))

    def step(self):
        next_clock = self.clock.advance()  # Only one clock advance for the fleet.
        previous = {key: engine.state.position_enu_m for key, engine in self._engines.items()}
        candidates, rows, failures = {}, {}, dict(self._failures)
        for key, engine in self._engines.items():
            candidate = deepcopy(engine)
            row = candidate.step(next_clock)
            # Stage 7 polygon supervision also applies to holding motion.
            if not self._missions[key].flight_boundary.contains_segment(previous[key][:2], candidate.state.position_enu_m[:2]):
                candidate = deepcopy(engine)
                candidate.emergency_stop()
                row = candidate.step(next_clock)
                failures[key] = 'predicted_flight_boundary_crossing'
            candidates[key], rows[key] = candidate, row
        self._engines, self._failures, self.clock = candidates, failures, next_clock
        return self._record(rows, previous, consume=True)


def simulate(config):
    fleet = FleetSimulator(config)
    yield fleet.telemetry()
    schedule = {}
    for action in config.actions:
        schedule.setdefault(action.tick, []).append(action)
    for _ in range(config.steps):
        for action in schedule.get(fleet.clock.tick, ()):
            fleet.command(action.command, action.vehicle_id)
        yield fleet.step()


def summarize(config, rows):
    if not rows:
        raise ValueError('At least one fleet sample required')
    final = rows[-1]
    per_vehicle = {m.vehicle_id: vehicle_summary([r['vehicles'][m.vehicle_id] for r in rows]) for m in config.members}
    for key in per_vehicle:
        per_vehicle[key]['fleet_failure'] = final['vehicles'][key]['fleet_failure']
    pairs, alert_samples, episodes = set(), 0, 0
    active = set()
    for row in rows:
        current = {tuple(a['vehicle_ids']) for a in row['proximity']}
        episodes += len(current-active)
        alert_samples += len(current)
        pairs.update(current)
        active = current
    return {'schema_version': 1, 'fleet_id': config.fleet_id, 'samples': len(rows),
            'duration_s': final['simulation_time_s'], **final['summary'], 'vehicles': per_vehicle,
            'first_all_completed_time_s': next((r['simulation_time_s'] for r in rows if r['summary']['all_completed']), None),
            'route_advisories': route_conflicts(config), 'proximity_pair_samples': alert_samples,
            'proximity_episodes': episodes, 'proximity_pairs': [list(p) for p in sorted(pairs)],
            'minimum_alert_separation_m': min((a['minimum_separation_m'] for r in rows for a in r['proximity']), default=None)}
