"""Contract regression tests need no PX4, Gazebo, ROS or optional dependencies."""
import json
import math
from dataclasses import replace
from pathlib import Path
import unittest
from drone_sim.fleet import FleetConfig, simulate
from simulator_adapter import (MockAdapter, NumericalAdapter, VehicleSample,
                               enu_to_ned, ned_to_enu, enu_yaw_to_heading)


class AdapterTests(unittest.TestCase):
    def sample(self, **kwargs):
        return replace(VehicleSample('uav-1', 'x500-0', 1., 10., 'autopilot_estimate',
                                     position_enu_m=(1., 2., 3.)), **kwargs)

    def mock(self):
        return MockAdapter({'uav-1': 'x500-0'})

    def test_cardinal_axes_round_trip(self):
        for enu, ned in [((1, 0, 0), (0, 1, 0)), ((0, 1, 0), (1, 0, 0)),
                         ((0, 0, 1), (0, 0, -1)), ((12, -7, 2), (-7, 12, -2))]:
            self.assertEqual(enu_to_ned(enu), ned)
            self.assertEqual(ned_to_enu(ned), enu)

    def test_heading_convention(self):
        for yaw, heading in [(0, math.pi/2), (math.pi/2, 0), (-math.pi/2, -math.pi)]:
            self.assertAlmostEqual(enu_yaw_to_heading(yaw), heading)

    def test_invalid_numeric_and_progress(self):
        for changes in [{'battery_fraction': 1.1}, {'heading_rad': float('nan')},
                        {'position_enu_m': (1, 2)}, {'source_time_s': -1},
                        {'completed_waypoints': 2, 'waypoint_count': 1},
                        {'completed_waypoints': 0}, {'mission_state': 'armed'},
                        {'provenance': 'truth_maybe'}, {'velocity_enu_mps': (True, 0, 0)}]:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.sample(**changes)

    def test_unavailable_unknown_and_estimate_separation(self):
        adapter = self.mock()
        self.assertEqual(adapter.snapshot(10)['availability'], 'unavailable')
        adapter.ingest('mock-run', 1, 1, [self.sample()])
        sample = adapter.snapshot(10)['vehicles'][0]
        self.assertEqual(sample['provenance'], 'autopilot_estimate')
        self.assertIsNone(sample['battery_fraction'])
        self.assertEqual(sample['mission_state'], 'unknown')
        self.assertFalse(adapter.capabilities.exact_replay)
        self.assertEqual(adapter.capabilities.commands, ())

    def test_fresh_stale_lost_and_disconnect_are_distinct(self):
        adapter = self.mock()
        adapter.ingest('mock-run', 1, 1, [self.sample()])
        for now, state in [(10, 'fresh'), (10.5, 'stale'), (12, 'lost')]:
            self.assertEqual(adapter.snapshot(now)['vehicles'][0]['freshness'], state)
        adapter.disconnect()
        row = adapter.snapshot(10)
        self.assertEqual(row['availability'], 'unavailable')
        self.assertEqual(row['vehicles'][0]['freshness'], 'fresh')
        self.assertEqual(row['vehicles'][0]['mission_state'], 'unknown')

    def test_rejected_packets_are_atomic(self):
        adapter = self.mock()
        adapter.ingest('mock-run', 2, 1, [self.sample()])
        before = adapter.snapshot(10)
        cases = [('mock-run', 2, 1, [self.sample()]), ('mock-run', 1, 1, [self.sample()]),
                 ('other-epoch', 3, 1, [self.sample()]), ('mock-run', 3, .9, [self.sample()]),
                 ('mock-run', 3, 1, []), ('mock-run', 3, 1, [self.sample(source_id='other')]),
                 ('mock-run', 3, 1, [self.sample(source_time_s=.5)]),
                 ('mock-run', 3, 1, [self.sample(received_monotonic_s=9)])]
        for args in cases:
            with self.subTest(args=args), self.assertRaises(ValueError):
                adapter.ingest(*args)
            self.assertEqual(adapter.snapshot(10), before)

    def test_future_time_and_detached_snapshots(self):
        adapter = self.mock()
        with self.assertRaises(ValueError):
            adapter.ingest('mock-run', 1, .5, [self.sample()])
        adapter.ingest('mock-run', 1, 1, [self.sample()])
        with self.assertRaises(ValueError):
            adapter.snapshot(9)
        row = adapter.snapshot(10)
        row['vehicles'][0]['mission_state'] = 'completed'
        self.assertEqual(adapter.snapshot(10)['vehicles'][0]['mission_state'], 'unknown')
        json.dumps(adapter.snapshot(10), allow_nan=False)

    def test_identity_mapping_and_reconnect(self):
        for sources in [{'a': 'x', 'A': 'y'}, {'a': 'x', 'b': 'x'}, {}]:
            with self.assertRaises(ValueError):
                MockAdapter(sources)
        adapter = self.mock()
        adapter.ingest('mock-run', 1, 1, [self.sample()])
        adapter.disconnect()
        adapter.ingest('mock-run', 2, 2, [self.sample(source_time_s=2, received_monotonic_s=11)])
        self.assertEqual(adapter.snapshot(11)['availability'], 'available')

    def test_numerical_matches_original_full_run(self):
        config = FleetConfig.from_dict(json.loads(Path('scenarios/stage8/crossing.json').read_text()))
        adapter = NumericalAdapter(config)
        reference = list(simulate(config))
        for expected in reference:
            if expected['tick']:
                self.assertEqual(adapter.step(expected['simulation_time_s']), expected)
            row = adapter.snapshot(expected['simulation_time_s'])
            self.assertEqual(row['sequence'], expected['tick'])
            self.assertEqual(row['simulation_time_s'], expected['simulation_time_s'])
            for sample in row['vehicles']:
                original = expected['vehicles'][sample['vehicle_id']]
                self.assertEqual(sample['position_enu_m'], original['vehicle']['position_enu_m'])
                self.assertEqual(sample['mission_state'], original['mission_state'])
            self.assertEqual(adapter.snapshot(expected['simulation_time_s']), row)
        with self.assertRaises(ValueError):
            adapter.step(100)

    def test_numerical_commands_remain_engine_owned(self):
        config = FleetConfig.from_dict(json.loads(Path('scenarios/stage8/crossing.json').read_text()))
        adapter = NumericalAdapter(replace(config, actions=()))
        vehicle = config.members[0].vehicle_id
        adapter.command('start', vehicle)
        self.assertEqual(adapter.snapshot(0)['vehicles'][0]['mission_state'], 'running')
        with self.assertRaises(ValueError):
            adapter.command('arm', vehicle)
        with self.assertRaises(ValueError):
            adapter.step(-1)
        self.assertEqual(adapter.snapshot(0)['sequence'], 0)

    def test_old_source_cannot_be_refreshed_by_new_receipt(self):
        adapter = self.mock()
        adapter.ingest('mock-run', 1, 5, [self.sample(received_monotonic_s=12)])
        sample = adapter.snapshot(12)['vehicles'][0]
        self.assertEqual(sample['receipt_age_s'], 0)
        self.assertEqual(sample['simulation_age_s'], 4)
        self.assertEqual(sample['freshness'], 'lost')
