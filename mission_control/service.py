"""Single-owner orchestration, pacing and delivery; all motion belongs to Stage 8."""
import asyncio
from collections import deque
import json
import logging
from math import isfinite
import uuid
from drone_sim.fleet import FleetAction, FleetConfig, FleetSimulator, Member
from .records import encoded

LOG = logging.getLogger('mission_control')


def event(kind, **details):
    LOG.info(json.dumps({'event': kind, **details}, sort_keys=True))


def parse_config(payload):
    """Accept complete Stage 8 config or an explicit Stage 7 single-member wrapper."""
    if not isinstance(payload, dict) or set(payload) != {'kind', 'config'}:
        raise ValueError('Expected kind and config only')
    data = payload['config']
    if payload['kind'] == 'fleet':
        if not isinstance(data, dict) or not isinstance(data.get('members'), list) or not 1 <= len(data['members']) <= 4:
            raise ValueError('Backend requires 1–4 members')
        if sum(len(m['mission']['waypoints']) for m in data['members']) > 128:
            raise ValueError('Backend permits at most 128 route legs')
        config = FleetConfig.from_dict(data)
    elif payload['kind'] == 'mission':
        if set(data) != {'fleet_id', 'member', 'steps', 'actions'}:
            raise ValueError('Mission wrapper needs fleet_id, member, steps and actions')
        config = FleetConfig(data['fleet_id'], (Member.from_dict(data['member']),), data['steps'],
                             tuple(FleetAction(**a) for a in data['actions']))
    else:
        raise ValueError('kind must be fleet or mission')
    if len(config.members) > 4 or config.steps > 5000 or sum(len(m.mission.waypoints) for m in config.members) > 128:
        raise ValueError('Backend limits: four vehicles, 5000 ticks, 128 total route legs')
    if len(config.actions) > 1000:
        raise ValueError('At most 1000 scheduled commands')
    return config


class Service:
    def __init__(self, store, queue_size=8, history_size=64, max_viewers=16):
        if not 1 <= queue_size <= 64 or not 1 <= history_size <= 512 or not 1 <= max_viewers <= 16:
            raise ValueError('Delivery limits out of range')
        self.store = store
        self.lock = asyncio.Lock()
        self.wake = asyncio.Event()
        self.epoch = uuid.uuid4().hex
        self.sequence = 0
        self.history = deque(maxlen=history_size)
        self.viewers = set()
        self.queue_size, self.max_viewers = queue_size, max_viewers
        self.run_id = None
        self.engine = None
        self.state = 'empty'
        self.paused, self.speed = True, 1.
        self.rows, self.outcomes, self.pending, self.requests = [], [], {}, {}
        self.error = None
        self.latest = None
        self.recorded = False
        self.closing = False

    def status(self):
        return {'schema_version': 1, 'epoch': self.epoch, 'sequence': self.sequence,
                'run_id': self.run_id, 'state': self.state, 'playback_paused': self.paused,
                'playback_speed': self.speed, 'tick': self.engine.clock.tick if self.engine else None,
                'dt_s': self.engine.clock.dt_s if self.engine else None,
                'viewers': len(self.viewers), 'recorded': self.recorded, 'error': self.error}

    def _publish(self, row):
        self.sequence += 1
        truth = {k: v for k, v in row.items() if k != 'proximity'}
        message = {'schema_version': 1, 'type': 'telemetry', 'epoch': self.epoch,
                   'sequence': self.sequence, 'run_id': self.run_id, 'delivery': 'live',
                   'status': self.status(), 'simulation_truth': truth, 'estimated_tracking': None,
                   'advisories': {'proximity': row['proximity'], 'route_conflicts': self.engine.route_advisories}}
        # JSON-detached immutable-by-convention envelopes; consumers never see engine objects.
        self.latest = json.loads(encoded(message))
        self.history.append(self.latest)
        for queue in self.viewers:
            outgoing = self.latest
            if queue.full():
                dropped = queue.qsize()
                while not queue.empty(): queue.get_nowait()
                outgoing = dict(self.latest, delivery='resync', dropped_messages=dropped,
                                resync_reason='slow_consumer')
            queue.put_nowait(outgoing)

    async def load(self, config):
        async with self.lock:
            if self.state == 'loaded':
                raise ValueError('Finish the current run before loading another')
            self.store.capacity()
            engine = FleetSimulator(config)
            self.config, self.engine = config, engine
            self.run_id = uuid.uuid4().hex
            self.state, self.paused, self.speed = 'loaded', True, 1.
            self.rows = [engine.telemetry()]
            self.row_bytes = len(encoded(self.rows[0]))
            self.outcomes, self.pending, self.requests = [], {}, {}
            self.error, self.recorded = None, False
            self.schedule = {}
            for action in config.actions:
                self.schedule.setdefault(action.tick, []).append(action)
            self.history.clear()
            for queue in self.viewers:
                while not queue.empty(): queue.get_nowait()
            self._publish(self.rows[0])
            event('run_loaded', run_id=self.run_id, vehicles=len(config.members))
            return self.status()

    def _check(self, run_id):
        if self.run_id != run_id or self.state != 'loaded':
            raise ValueError('Stale run ID or run is not accepting control')

    async def playback(self, run_id, paused, speed):
        async with self.lock:
            self._check(run_id)
            if type(paused) is not bool or type(speed) not in (int,float) or not isfinite(speed) or not .1 <= speed <= 20:
                raise ValueError('Playback requires boolean paused and finite speed in [0.1,20]')
            self.paused, self.speed = paused, speed
            self.wake.set()
            event('playback', run_id=run_id, paused=paused, speed=speed)
            return self.status()

    async def command(self, run_id, request_id, command, vehicle_id=None, target_tick=None):
        async with self.lock:
            self._check(run_id)
            if not isinstance(request_id, str) or not 1 <= len(request_id) <= 64:
                raise ValueError('request_id must contain 1–64 characters')
            tick = self.engine.clock.tick if target_tick is None else target_tick
            action = FleetAction(tick, command, vehicle_id)
            # A retry without an explicit tick reuses the original acknowledgement.
            signature = (command, vehicle_id, target_tick)
            if request_id in self.requests:
                old_signature, ack = self.requests[request_id]
                if signature != old_signature: raise ValueError('request_id reused with different command')
                return dict(ack)
            if tick < self.engine.clock.tick or tick >= self.config.steps:
                raise ValueError('Command tick is outside remaining run')
            if vehicle_id is not None and vehicle_id not in {m.vehicle_id for m in self.config.members}:
                raise ValueError('Unknown vehicle ID')
            if sum(len(v) for v in self.pending.values()) >= 128 or len(self.requests) >= 1000:
                raise ValueError('Command queue or per-run request limit reached')
            order = len(self.requests)+1
            ack = {'request_id': request_id, 'run_id': run_id, 'accepted_for_processing': True,
                   'order': order, 'target_tick': tick, 'applied': False}
            self.pending.setdefault(tick, []).append((action, request_id, order))
            self.requests[request_id] = (signature, ack)
            event('command_queued', run_id=run_id, request_id=request_id, order=order, tick=tick)
            return dict(ack)

    async def tick(self):
        """Only the runner calls this in production. Lock defines ordering with REST writes."""
        async with self.lock:
            if self.state != 'loaded' or self.paused:
                return
            tick = self.engine.clock.tick
            scheduled = [(a, None, None) for a in self.schedule.get(tick, ())]
            interactive = self.pending.pop(tick, [])
            for action, request_id, order in scheduled+interactive:
                outcome = {'tick': tick, 'command': action.command, 'vehicle_id': action.vehicle_id,
                           'request_id': request_id, 'order': order, 'accepted': False}
                try:
                    affected = self.engine.command(action.command, action.vehicle_id)
                    outcome.update(accepted=True, affected_vehicle_ids=list(affected))
                except ValueError as error:
                    outcome['reason'] = str(error)
                self.outcomes.append(outcome)
                event('command_outcome', run_id=self.run_id, **outcome)
            try:
                row = self.engine.step()
                self.rows.append(row)
                # Bound memory and final recording size conservatively before serializing the whole run.
                self.row_bytes = getattr(self, 'row_bytes', 0)+len(encoded(row))
                if self.row_bytes > self.store.max_bytes//2:
                    raise ValueError('Run recording budget exceeded')
                if self.engine.clock.tick == self.config.steps:
                    self.state, self.paused = 'finished', True
                    await self._save()
                self._publish(row)
            except Exception as error:
                self.state, self.paused, self.error = 'failed', True, type(error).__name__+': '+str(error)
                event('run_failed', run_id=self.run_id, error=self.error)
                await self._save()
                self._publish(self.rows[-1])

    async def _save(self):
        try:
            await asyncio.to_thread(self.store.save, self.run_id, self.config, self.rows,
                                    self.outcomes, self.state, self.error)
            self.recorded = True
        except Exception as error:
            self.error = 'recording_failed: '+type(error).__name__+': '+str(error)
            self.state, self.paused = 'failed', True
            event('recording_failed', run_id=self.run_id, error=self.error)

    async def run(self):
        """Pace one fixed step per wait; never catch up by changing dt or skipping ticks."""
        while not self.closing:
            self.wake.clear()
            if self.state != 'loaded' or self.paused:
                await self.wake.wait()
                continue
            delay = self.config.dt_s/self.speed
            try:
                await asyncio.wait_for(self.wake.wait(), timeout=delay)
            except asyncio.TimeoutError:
                await self.tick()

    async def subscribe(self, epoch=None, after=None):
        async with self.lock:
            if self.latest is None: raise ValueError('Load a run before subscribing')
            if len(self.viewers) >= self.max_viewers: raise ValueError('Viewer limit reached')
            queue = asyncio.Queue(maxsize=self.queue_size)
            history = list(self.history)
            missing = [m for m in history if after is not None and m['sequence'] > after]
            can_resume = (epoch == self.epoch and type(after) is int and history[0]['sequence']-1 <= after <= self.sequence
                          and len(missing) <= self.queue_size)
            if can_resume and missing:
                for message in missing: queue.put_nowait(dict(message, delivery='replay'))
            else:
                queue.put_nowait(dict(self.latest, status=self.status(), delivery='snapshot' if after is None or can_resume else 'resync',
                                      resync_reason=None if after is None or can_resume else 'cursor_unavailable'))
            self.viewers.add(queue)
            return queue

    def unsubscribe(self, queue):
        self.viewers.discard(queue)
