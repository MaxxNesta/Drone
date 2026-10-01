"""Single-vehicle read-only state assembly. Input dictionaries are decoded packets.

No socket, command publisher, physics or inferred mission lifecycle lives here.
Real-runtime qualification is deliberately separate from fixture correctness.
"""
from collections import Counter
from dataclasses import dataclass
from math import atan2, cos, sin
from threading import RLock
import uuid
from .contracts import (Capabilities, VehicleSample, identifier, number, vector,
                        snapshot_record, enu_yaw_to_heading, ned_to_enu, wrap)


@dataclass(frozen=True)
class SitlConfig:
    vehicle_id: str = 'uav-1'
    world: str = 'default'
    model: str = 'x500_0'
    system_id: int = 1
    component_id: int = 1
    udp_port: int = 14550
    px4_boot_to_simulation_s: float = 0.
    world_to_enu_yaw_rad: float = 0.
    world_origin_enu_m: tuple = (0., 0., 0.)
    px4_origin_enu_m: tuple = (0., 0., 0.)

    def __post_init__(self):
        for name in ('vehicle_id', 'world', 'model'):
            identifier(getattr(self, name))
        for name in ('system_id', 'component_id'):
            if type(getattr(self, name)) is not int or not 1 <= getattr(self, name) <= 255:
                raise ValueError('Invalid MAVLink identity')
        if type(self.udp_port) is not int or not 1024 <= self.udp_port <= 65535:
            raise ValueError('Invalid local UDP port')
        for name in ('px4_boot_to_simulation_s', 'world_to_enu_yaw_rad'):
            number(getattr(self, name), name)
        for name in ('world_origin_enu_m', 'px4_origin_enu_m'):
            object.__setattr__(self, name, vector(getattr(self, name)))


def stamp(value):
    sec, nsec = value['sec'], value['nsec']
    if type(sec) is not int or sec < 0 or type(nsec) is not int or not 0 <= nsec < 1000000000:
        raise ValueError('Invalid Gazebo timestamp')
    return sec+nsec*1e-9


class RealSitlAdapter:
    capabilities = Capabilities('px4-gazebo', 'gazebo_simulation', False, False)

    def __init__(self, config=SitlConfig(), evidence='live_transport_unqualified', epoch=None):
        if evidence not in ('live_transport_unqualified', 'synthetic_fixture'):
            raise ValueError('Unsupported evidence label')
        self.config, self.evidence = config, evidence
        self.epoch = epoch or uuid.uuid4().hex
        identifier(self.epoch)
        self.lock = RLock()
        self.time_s, self.clock_receipt, self.sequence = 0., None, 0
        self.truth, self.position, self.attitude, self.estimator, self.heartbeat = (None,)*5
        self.entity_id, self.blocked = None, None
        self.rejections = Counter()
        self._stream_times = {}

    def reject(self, reason):
        self.rejections[reason] += 1
        return False

    def clock(self, packet, received):
        with self.lock:
            number(received, 'receipt time', 0)
            t = stamp(packet['sim'])
            if self.blocked:
                return self.reject('restart_required')
            if self.clock_receipt is not None and (t < self.time_s or received < self.clock_receipt):
                self.blocked = 'clock_rollback_restart_required'
                return self.reject('clock_rollback')
            self.time_s, self.clock_receipt = t, received
            self.sequence += 1
            return True

    def pose(self, packet, received):
        with self.lock:
            number(received, 'receipt time', 0)
            t = stamp(packet['stamp'])
            if self.blocked or self.clock_receipt is None:
                return self.reject('clock_unavailable')
            # Different topic callbacks can arrive in either order. Drop future
            # poses instead of advancing the authoritative clock from a pose.
            if t > self.time_s or (self.truth is not None and t <= self.truth.source_time_s):
                return self.reject('pose_timestamp')
            matches = [p for p in packet['poses'] if p['name'] == self.config.model]
            if len(matches) != 1:
                return self.reject('model_not_unique_or_missing')
            pose = matches[0]
            entity = pose['id']
            if type(entity) is not int or entity < 0:
                raise ValueError('Invalid Gazebo entity ID')
            if self.entity_id is not None and entity != self.entity_id:
                self.blocked = 'model_identity_changed_restart_required'
                return self.reject('model_identity_changed')
            x, y, z = vector(pose['position'])
            q = pose['orientation_wxyz']
            if len(q) != 4:
                raise ValueError('Invalid quaternion')
            for value in q:
                number(value, 'quaternion')
            if abs(sum(v*v for v in q)-1) > .001:
                raise ValueError('Quaternion must be normalized')
            w, qx, qy, qz = q
            yaw = atan2(2*(w*qz+qx*qy), 1-2*(qy*qy+qz*qz))
            angle = self.config.world_to_enu_yaw_rad
            rotated = (cos(angle)*x-sin(angle)*y, sin(angle)*x+cos(angle)*y, z)
            position = tuple(a+b for a, b in zip(rotated, self.config.world_origin_enu_m))
            self.truth = VehicleSample(self.config.vehicle_id, self.config.model, t, received,
                                       'simulation_truth', position_enu_m=position,
                                       heading_rad=enu_yaw_to_heading(yaw+angle))
            self.entity_id = entity
            self.sequence += 1
            return True

    def mavlink(self, kind, fields, system_id, component_id, received):
        with self.lock:
            number(received, 'receipt time', 0)
            if (system_id, component_id) != (self.config.system_id, self.config.component_id):
                return self.reject('mavlink_identity')
            if self.blocked:
                return self.reject('restart_required')
            if kind == 'HEARTBEAT':
                if fields['autopilot'] != 12 or fields['type'] != 2:  # PX4 / quadrotor
                    return self.reject('not_px4_quadcopter')
                if fields['base_mode'] & 128:
                    self.blocked = 'armed_source_outside_unarmed_scope'
                    return self.reject('armed_source')
                self.heartbeat = received
                self.sequence += 1
                return True
            if kind not in ('LOCAL_POSITION_NED', 'ATTITUDE', 'ESTIMATOR_STATUS'):
                return False
            if self.clock_receipt is None:
                return self.reject('clock_unavailable')
            raw = fields['time_usec']*1e-6 if kind == 'ESTIMATOR_STATUS' else fields['time_boot_ms']*.001
            number(raw, 'PX4 source time', 0)
            t = raw+self.config.px4_boot_to_simulation_s
            if t < 0 or t > self.time_s or self.time_s-t > .5:
                return self.reject('px4_clock_unmapped_or_old')
            previous = self._stream_times.get(kind)
            if previous is not None and t <= previous:
                if t < previous:
                    self.blocked = 'px4_clock_rollback_restart_required'
                return self.reject('px4_duplicate_or_rollback')
            if kind == 'LOCAL_POSITION_NED':
                position = ned_to_enu(tuple(fields[k] for k in ('x', 'y', 'z')))
                velocity = ned_to_enu(tuple(fields[k] for k in ('vx', 'vy', 'vz')))
                value = (t, received, tuple(a+b for a,b in zip(position, self.config.px4_origin_enu_m)), velocity)
                self.position = value
            elif kind == 'ATTITUDE':
                self.attitude = (t, received, wrap(fields['yaw']))
            else:
                flags = fields['flags']
                if type(flags) is not int or not 0 <= flags <= 65535:
                    raise ValueError('Invalid estimator flags')
                self.estimator = (t, received, flags)
            self._stream_times[kind] = t
            self.sequence += 1
            return True

    def snapshot(self, now_monotonic_s):
        """Stage 12 adapter contract selects Gazebo truth; estimates stay separate."""
        return self.telemetry(now_monotonic_s)['simulation_truth']

    def telemetry(self, now_monotonic_s):
        with self.lock:
            number(now_monotonic_s, 'monotonic time', 0)
            now = now_monotonic_s
            active = self.clock_receipt is not None and 0 <= now-self.clock_receipt < 2 and not self.blocked
            reason = self.blocked or (None if active else 'gazebo_clock_unavailable')
            truth = snapshot_record(self.capabilities, self.epoch, self.sequence, self.time_s,
                                    (self.truth,) if self.truth else (), now, available=bool(active), reason=reason)
            estimate = None
            validity = 'missing'
            if self.position:
                t, receipt, position, velocity = self.position
                # Every field is gated by fresh validity evidence and timestamp
                # compatibility. Missing status never implies a valid EKF.
                status = self.estimator
                valid_status = (active and self.heartbeat is not None and 0 <= now-self.heartbeat < 2
                                and status and abs(status[0]-t) <= .25 and 0 <= now-status[1] < .5)
                flags = status[2] if valid_status else 0
                pos_valid = bool(flags & (8|16) and flags & 32)
                vel_valid = bool(flags & 2 and flags & 4)
                att = self.attitude
                heading = att[2] if att and flags & 1 and abs(att[0]-t) <= .25 and 0 <= now-att[1] < .5 else None
                validity = 'valid' if pos_valid and vel_valid else 'invalid_or_unknown'
                sample = VehicleSample(self.config.vehicle_id, 'px4-'+str(self.config.system_id), t, receipt,
                                       'autopilot_estimate', position_enu_m=position if pos_valid else None,
                                       velocity_enu_mps=velocity if vel_valid else None, heading_rad=heading)
                estimate = snapshot_record(self.capabilities, self.epoch, self.sequence, self.time_s, (sample,), now,
                                           available=bool(active and self.heartbeat is not None and 0 <= now-self.heartbeat < 2),
                                           reason=reason or ('px4_heartbeat_unavailable' if self.heartbeat is None or now-self.heartbeat >= 2 else None))
            return {'schema_version': 1, 'type': 'sitl_telemetry', 'evidence': self.evidence,
                    'read_only': True, 'epoch': self.epoch, 'sequence': self.sequence,
                    'generated_monotonic_s': now, 'simulation_truth': truth,
                    'autopilot_estimate': estimate, 'estimator_validity': validity,
                    'mission_progress': None, 'diagnostics': dict(self.rejections), 'reason': reason}
