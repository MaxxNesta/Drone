"""Bounded local recordings and exact same-runtime replay verification."""
import hashlib
import json
import os
import platform
import re
from dataclasses import replace
from pathlib import Path
from drone_sim.fleet import FleetAction, FleetConfig, simulate, summarize


def encoded(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def digest(rows):
    return hashlib.sha256(encoded(rows)).hexdigest()


class RecordStore:
    def __init__(self, root, max_records=20, max_bytes=32*1024*1024):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.max_records, self.max_bytes = max_records, max_bytes

    def path(self, run_id):
        if not re.fullmatch(r'[a-f0-9]{32}', run_id):
            raise ValueError('Invalid run ID')
        return self.root/(run_id+'.json')

    def capacity(self):
        if len(list(self.root.glob('*.json'))) >= self.max_records:
            raise ValueError('Recording store full; archive local recordings before loading a run')

    def save(self, run_id, config, rows, outcomes, state, error=None):
        self.capacity()
        value = {'schema_version': 1, 'run_id': run_id, 'runtime': platform.python_version(),
                 'state': state, 'error': error, 'config': config.to_dict(), 'rows': rows,
                 'command_outcomes': outcomes, 'telemetry_sha256': digest(rows),
                 'summary': summarize(config, rows)}
        data = encoded(value)
        if len(data) > self.max_bytes:
            raise ValueError('Recording exceeds byte limit')
        path = self.path(run_id)
        temporary = path.with_suffix('.tmp')
        with temporary.open('xb') as stream:
            stream.write(data); stream.flush(); os.fsync(stream.fileno())
        os.replace(temporary, path)
        return value

    def read(self, run_id):
        path = self.path(run_id)
        if path.stat().st_size > self.max_bytes:
            raise ValueError('Recording exceeds byte limit')
        return json.loads(path.read_text())

    def list(self):
        return [p.stem for p in sorted(self.root.glob('*.json'))]

    def verify(self, run_id):
        record = self.read(run_id)
        if record['state'] != 'finished':
            raise ValueError('Only completed simulation runs can be replay-verified')
        if record['runtime'] != platform.python_version():
            raise ValueError('Exact replay requires the recording Python version')
        if digest(record['rows']) != record['telemetry_sha256']:
            raise ValueError('Recorded telemetry checksum mismatch')
        config = FleetConfig.from_dict(record['config'])
        actions = tuple(FleetAction(o['tick'], o['command'], o['vehicle_id'])
                        for o in record['command_outcomes'] if o['accepted'])
        replay = replace(config, actions=actions)
        actual = digest(list(simulate(replay)))
        return {'schema_version': 1, 'run_id': run_id, 'matches': actual == record['telemetry_sha256'],
                'expected_sha256': record['telemetry_sha256'], 'actual_sha256': actual,
                'samples': len(record['rows'])}
