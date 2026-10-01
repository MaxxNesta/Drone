"""Optional wrappers; neither is selected by the running mission-control service."""
from .contracts import Capabilities, VehicleSample, number, snapshot_record
from drone_sim.fleet import FleetSimulator
from drone_sim.fleet.config import identifier


class NumericalAdapter:
    """Wrap the existing engine; retain its sole clock and command semantics.

    Explicit step() calls apply configured actions before the engine's next tick,
    just like fleet.simulate. An external owner supplies monotonic receipt time.
    """
    capabilities = Capabilities('numerical', 'fleet_fixed_step', True, True,
                               ('start', 'pause', 'resume', 'abort', 'emergency_stop'))

    def __init__(self, config, epoch='numerical-run', now_monotonic_s=0.):
        identifier(epoch)
        number(now_monotonic_s, 'receipt time', 0)
        self._engine = FleetSimulator(config)
        self._config, self._epoch = config, epoch
        self._received = now_monotonic_s

    def command(self, command, vehicle_id=None):
        return self._engine.command(command, vehicle_id)

    def step(self, now_monotonic_s):
        number(now_monotonic_s, 'receipt time', self._received)
        if self._engine.clock.tick >= self._config.steps:
            raise ValueError('Run budget exhausted')
        for action in self._config.actions:
            if action.tick == self._engine.clock.tick:
                self._engine.command(action.command, action.vehicle_id)
        row = self._engine.step()
        self._received = now_monotonic_s
        return row

    def snapshot(self, now_monotonic_s):
        row = self._engine.telemetry()
        samples = tuple(VehicleSample(vehicle_id=key, source_id=key,
                        source_time_s=row['simulation_time_s'], received_monotonic_s=self._received,
                        provenance='simulation_truth', **value['vehicle'],
                        mission_state=value['mission_state'], completed_waypoints=value['completed_waypoints'],
                        waypoint_count=value['waypoint_count']) for key, value in row['vehicles'].items())
        return snapshot_record(self.capabilities, self._epoch, row['tick'], row['simulation_time_s'],
                               samples, now_monotonic_s)


class MockAdapter:
    """Explicit whole-fleet packet injection, with no network or physics.

    Fixed identities per epoch. Atomic acceptance rejects duplicate/reordered
    packets, clock rollback and source substitution. Restart needs a new instance
    and epoch; old packets cannot silently reset a run.
    """
    capabilities = Capabilities('mock', 'external_simulation', False, False)

    def __init__(self, vehicle_sources, epoch='mock-run'):
        identifier(epoch)
        if not vehicle_sources or len({k.casefold() for k in vehicle_sources}) != len(vehicle_sources):
            raise ValueError('Unique vehicle identities required')
        if len(set(vehicle_sources.values())) != len(vehicle_sources):
            raise ValueError('Source identities must be unique')
        for key, source in vehicle_sources.items():
            identifier(key)
            identifier(source)
        self._sources = dict(vehicle_sources)
        self._epoch, self._sequence, self._time = epoch, 0, 0.
        self._samples, self._available, self._reason = (), False, 'awaiting_first_packet'

    def ingest(self, epoch, sequence, simulation_time_s, samples):
        samples = tuple(samples)
        if epoch != self._epoch:
            raise ValueError('Wrong epoch')
        if type(sequence) is not int or sequence <= self._sequence:
            raise ValueError('Duplicate or out-of-order sequence')
        number(simulation_time_s, 'simulation time', self._time)
        if {s.vehicle_id for s in samples} != set(self._sources) or len(samples) != len(self._sources):
            raise ValueError('Require exactly one sample per configured vehicle')
        previous = {s.vehicle_id: s for s in self._samples}
        for sample in samples:
            if sample.source_id != self._sources[sample.vehicle_id] or sample.source_time_s > simulation_time_s:
                raise ValueError('Invalid source identity or time')
            old = previous.get(sample.vehicle_id)
            if old and (sample.source_time_s < old.source_time_s or
                        sample.received_monotonic_s < old.received_monotonic_s):
                raise ValueError('Sample clock rollback')
        self._sequence, self._time, self._samples = sequence, simulation_time_s, samples
        self._available, self._reason = True, None

    def disconnect(self, reason='simulator_disconnected'):
        if not isinstance(reason, str) or not reason or len(reason) > 256:
            raise ValueError('Bounded disconnect reason required')
        self._available, self._reason = False, reason

    def snapshot(self, now_monotonic_s):
        return snapshot_record(self.capabilities, self._epoch, self._sequence, self._time,
                               self._samples, now_monotonic_s, available=self._available, reason=self._reason)
