"""Bounded, read-only local collector handoff; never launches a transport."""
from dataclasses import fields, replace
import json
from pathlib import Path
import time
from simulator_adapter.contracts import VehicleSample, snapshot_record, number, identifier
from simulator_adapter.sitl import RealSitlAdapter


def read_sitl(path, now=None):
    if path is None:
        return {'configured': False, 'status': 'disabled', 'telemetry': None,
                'detail': 'SITL telemetry is not configured. Numerical simulation remains the default.'}
    now = time.monotonic() if now is None else now
    try:
        with Path(path).open('rb') as stream:
            raw = stream.read(128*1024+1)
        if len(raw) > 128*1024:
            raise ValueError('SITL snapshot exceeds 128 KiB')
        data = json.loads(raw)
        if (type(data['schema_version']) is not int or data['schema_version'] != 1
                or data['type'] != 'sitl_telemetry' or data['read_only'] is not True
                or data['evidence'] not in ('synthetic_fixture', 'live_transport_unqualified')
                or data['mission_progress'] is not None):
            raise ValueError('Unsupported SITL snapshot contract')
        identifier(data['epoch'])
        if type(data['sequence']) is not int or data['sequence'] < 0:
            raise ValueError('Invalid sequence')
        number(data['generated_monotonic_s'], 'generation time', 0)
        age = now-data['generated_monotonic_s']
        if age < 0:
            raise ValueError('Collector must use this host monotonic clock')
        if data['estimator_validity'] not in ('missing','valid','invalid_or_unknown'):
            raise ValueError('Invalid estimator validity')
        diagnostics = data['diagnostics']
        if (not isinstance(diagnostics, dict) or len(diagnostics) > 64 or
                any(not isinstance(k,str) or len(k)>80 or type(v) is not int or v<0 for k,v in diagnostics.items())):
            raise ValueError('Invalid diagnostics')
        reason = data['reason']
        if reason is not None and (not isinstance(reason, str) or len(reason)>256):
            raise ValueError('Invalid reason')
        names = {f.name for f in fields(VehicleSample)}
        snapshots = {}
        for key, provenance in [('simulation_truth','simulation_truth'), ('autopilot_estimate','autopilot_estimate')]:
            value = data[key]
            if value is None and key == 'autopilot_estimate':
                snapshots[key] = None
                continue
            if (value['schema_version'] != 1 or value['type'] != 'simulator_snapshot'
                    or value['backend'] != 'px4-gazebo' or value['coordinate_frame'] != 'ENU'
                    or value['time_domain'] != 'simulation' or value['epoch'] != data['epoch']
                    or value['sequence'] != data['sequence'] or len(value['vehicles']) > 1
                    or value['availability'] not in ('available','unavailable')):
                raise ValueError('Invalid source snapshot')
            source_reason = value.get('reason')
            if source_reason is not None and (not isinstance(source_reason, str) or len(source_reason)>256):
                raise ValueError('Invalid source reason')
            samples = []
            for sample in value['vehicles']:
                s = VehicleSample(**{k:sample[k] for k in names})
                if s.provenance != provenance or s.mission_state != 'unknown' or s.waypoint_count is not None:
                    raise ValueError('Unexpected provenance or invented mission progress')
                if key == 'autopilot_estimate' and age >= .5:
                    s = replace(s, position_enu_m=None, velocity_enu_mps=None, heading_rad=None)
                samples.append(s)
            snapshots[key] = snapshot_record(RealSitlAdapter.capabilities, data['epoch'], data['sequence'],
                                             value['simulation_time_s'], samples, now,
                                             available=age<2 and value['availability']=='available',
                                             reason='collector_stale' if age>=2 else source_reason)
        truth, estimate = snapshots['simulation_truth'], snapshots['autopilot_estimate']
        if estimate is not None:
            if truth['simulation_time_s'] != estimate['simulation_time_s']:
                raise ValueError('Source clocks must agree')
            if (truth['vehicles'] and estimate['vehicles'] and
                    truth['vehicles'][0]['vehicle_id'] != estimate['vehicles'][0]['vehicle_id']):
                raise ValueError('Source vehicle identities must agree')
        clean = {k:data[k] for k in ('schema_version','type','evidence','read_only','epoch','sequence',
                                    'generated_monotonic_s','estimator_validity','mission_progress','diagnostics','reason')}
        if age >= .5 and clean['estimator_validity'] == 'valid':
            clean['estimator_validity'] = 'invalid_or_unknown'
        clean.update(snapshots)
        return {'configured': True, 'status': 'stale' if age>=2 else 'live', 'telemetry': clean,
                'collector_age_s': age, 'detail': 'Read-only transport; real SITL qualification is not established by reception.'}
    except FileNotFoundError:
        return {'configured': True, 'status': 'waiting', 'telemetry': None, 'detail': 'Waiting for local SITL collector.'}
    except (OSError, ValueError, TypeError, KeyError, AttributeError, OverflowError, RecursionError) as error:
        return {'configured': True, 'status': 'error', 'telemetry': None, 'detail': 'Invalid or unreadable SITL snapshot: '+type(error).__name__}
