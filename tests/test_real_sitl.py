"""Synthetic protocol fixtures; never claim captured SITL evidence."""
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from simulator_adapter.sitl import RealSitlAdapter, SitlConfig
from simulator_adapter.sitl_runtime import atomic_json, PassiveMavlink, Collector
from mission_control.sitl import read_sitl
from tests.test_backend import AVAILABLE, CONTROL, VIEW
if AVAILABLE:
    from fastapi.testclient import TestClient
    from mission_control.app import create_app


def fixture():
    return json.loads(Path('tests/fixtures/stage13/packets.json').read_text())


def populated():
    f = fixture()
    adapter = RealSitlAdapter(evidence='synthetic_fixture', epoch='fixture-epoch')
    adapter.clock(f['clock'], 10)
    adapter.pose(f['pose'], 10)
    for kind,key in [('HEARTBEAT','heartbeat'), ('LOCAL_POSITION_NED','position'),
                     ('ATTITUDE','attitude'), ('ESTIMATOR_STATUS','estimator')]:
        adapter.mavlink(kind, f[key], 1, 1, 10)
    return adapter


class SitlTests(unittest.TestCase):
    def test_enu_ned_truth_and_estimates_are_separate(self):
        adapter = populated()
        row = adapter.telemetry(10)
        truth = row['simulation_truth']['vehicles'][0]
        estimate = row['autopilot_estimate']['vehicles'][0]
        self.assertEqual(truth['position_enu_m'], (1,2,3))
        self.assertEqual(estimate['position_enu_m'], (1,2,3))
        self.assertEqual(estimate['velocity_enu_mps'], (4,5,6))
        self.assertAlmostEqual(truth['heading_rad'], math.pi/2)
        self.assertAlmostEqual(estimate['heading_rad'], .4)
        self.assertEqual(estimate['provenance'], 'autopilot_estimate')
        self.assertIsNone(row['mission_progress'])
        self.assertIsNone(estimate['battery_fraction'])
        self.assertEqual(estimate['mission_state'], 'unknown')
        self.assertEqual(adapter.capabilities.commands, ())
        self.assertEqual(adapter.snapshot(10)['type'], 'simulator_snapshot')

    def test_invalid_and_expired_estimator_status_nulls_fields(self):
        adapter = populated()
        for status in (None, (1,10,0), (0,10,47)):
            adapter.estimator = status
            sample = adapter.telemetry(10)['autopilot_estimate']['vehicles'][0]
            self.assertIsNone(sample['position_enu_m'])
            self.assertIsNone(sample['velocity_enu_mps'])
            self.assertIsNone(sample['heading_rad'])
        sample = populated().telemetry(10.6)['autopilot_estimate']['vehicles'][0]
        self.assertIsNone(sample['position_enu_m'])

    def test_disconnect_does_not_invent_progress_or_retime(self):
        row = populated().telemetry(12)
        self.assertEqual(row['simulation_truth']['availability'], 'unavailable')
        self.assertEqual(row['simulation_truth']['vehicles'][0]['freshness'], 'lost')
        self.assertEqual(row['simulation_truth']['simulation_time_s'], 1)
        self.assertIsNone(row['mission_progress'])

    def test_gazebo_clock_rollback_requires_new_epoch(self):
        adapter = populated()
        self.assertFalse(adapter.clock({'sim':{'sec':0,'nsec':0}},11))
        self.assertFalse(adapter.clock({'sim':{'sec':2,'nsec':0}},12))
        self.assertEqual(adapter.telemetry(12)['reason'],'clock_rollback_restart_required')
        self.assertNotEqual(RealSitlAdapter().epoch, adapter.epoch)

    def test_wrong_identity_future_duplicates_and_armed_sources(self):
        adapter = populated();f=fixture()
        self.assertFalse(adapter.mavlink('HEARTBEAT',f['heartbeat'],2,1,10))
        self.assertFalse(adapter.mavlink('LOCAL_POSITION_NED',f['position'],1,1,10))
        self.assertFalse(adapter.mavlink('ATTITUDE',dict(f['attitude'],time_boot_ms=2000),1,1,10))
        self.assertFalse(adapter.mavlink('HEARTBEAT',dict(f['heartbeat'],base_mode=128),1,1,10))
        self.assertEqual(adapter.telemetry(10)['reason'],'armed_source_outside_unarmed_scope')

    def test_model_change_and_ambiguous_model(self):
        adapter = populated(); f=fixture()
        adapter.clock({'sim':{'sec':2,'nsec':0}},11)
        f['pose']['stamp']['sec']=2
        f['pose']['poses'].append(dict(f['pose']['poses'][0]))
        self.assertFalse(adapter.pose(f['pose'],11))
        f['pose']['poses'].pop();f['pose']['poses'][0]['id']=8
        self.assertFalse(adapter.pose(f['pose'],11))
        self.assertIn('identity_changed',adapter.telemetry(11)['reason'])

    def test_pose_waits_for_clock_and_checks_quaternion(self):
        adapter=RealSitlAdapter(); f=fixture()
        self.assertFalse(adapter.pose(f['pose'],10))
        adapter.clock(f['clock'],10)
        f['pose']['poses'][0]['orientation_wxyz']=[0,0,0,0]
        with self.assertRaises(ValueError):adapter.pose(f['pose'],10)
        self.assertIsNone(adapter.truth)

    def test_read_handoff_authenticity_labels_and_staleness(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'sample.json'
            self.assertEqual(read_sitl(None)['status'],'disabled')
            self.assertEqual(read_sitl(path,10)['status'],'waiting')
            atomic_json(path,populated().telemetry(10))
            self.assertEqual(read_sitl(path,10)['telemetry']['evidence'],'synthetic_fixture')
            self.assertEqual(read_sitl(path,12)['status'],'stale')
            self.assertEqual(read_sitl(path,12)['telemetry']['simulation_truth']['availability'],'unavailable')
            self.assertEqual(read_sitl(path,9)['status'],'error')
            path.write_text('x'*131073)
            self.assertEqual(read_sitl(path,10)['status'],'error')
            data=populated().telemetry(10);data['mission_progress']=100
            atomic_json(path,data)
            self.assertEqual(read_sitl(path,10)['status'],'error')

    def test_runtime_subscribes_only_and_cleans_up(self):
        from unittest.mock import Mock
        node=Mock(); node.subscribe.return_value=True
        adapter=populated(); decoder=Mock(); decoder.decode.return_value=[]
        collector=Collector(adapter,node,object,object,decoder)
        self.assertEqual(node.subscribe.call_count,2)
        self.assertFalse(collector.datagram(b'ignored',('192.0.2.1',14550),10))
        decoder.decode.assert_not_called()
        collector.close()
        self.assertEqual(node.unsubscribe.call_count,2)
        node.advertise.assert_not_called();node.request.assert_not_called()

    def test_handoff_rejects_mixed_vehicle_or_clock_and_preserves_reason(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'sample.json'
            row=populated().telemetry(10)
            row['autopilot_estimate']['availability']='unavailable'
            row['autopilot_estimate']['reason']='px4_heartbeat_unavailable'
            atomic_json(path,row)
            self.assertEqual(read_sitl(path,10)['telemetry']['autopilot_estimate']['reason'],'px4_heartbeat_unavailable')
            row['autopilot_estimate']['vehicles'][0]['vehicle_id']='another-vehicle'
            atomic_json(path,row)
            self.assertEqual(read_sitl(path,10)['status'],'error')
            row=populated().telemetry(10)
            row['autopilot_estimate']['simulation_time_s']=2
            atomic_json(path,row)
            self.assertEqual(read_sitl(path,10)['status'],'error')



@unittest.skipUnless(AVAILABLE,'Optional backend dependencies unavailable')
class SitlApiTests(unittest.TestCase):
    def test_authenticated_read_only_endpoint_and_default_unchanged(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'sitl.json'
            atomic_json(path,populated().telemetry(10))
            app=create_app(CONTROL,VIEW,folder,start_runner=False,sitl_snapshot_path=path)
            with TestClient(app,base_url='http://127.0.0.1:8000',client=('127.0.0.1',50000)) as client:
                headers={'Authorization':'Bearer '+VIEW}
                self.assertEqual(client.get('/v1/sitl').status_code,401)
                self.assertEqual(client.get('/v1/sitl',headers={**headers,'Origin':'https://evil.test'}).status_code,403)
                with patch('mission_control.sitl.time.monotonic',return_value=10):
                    row=client.get('/v1/sitl',headers=headers).json()
                self.assertEqual(row['status'],'live')
                self.assertEqual(row['telemetry']['evidence'],'synthetic_fixture')
                self.assertEqual(client.post('/v1/sitl',json={},headers={'Authorization':'Bearer '+CONTROL}).status_code,405)
                self.assertEqual(client.get('/v1/status',headers=headers).json()['state'],'empty')


class OptionalWireTests(unittest.TestCase):
    def test_pymavlink_wire_decoding_when_installed(self):
        try:
            from pymavlink.dialects.v20 import common
        except ImportError:
            self.skipTest('Optional pymavlink parser unavailable')
        # Generated protocol wire bytes, explicitly NOT a captured PX4 packet.
        sender=common.MAVLink(None,srcSystem=1,srcComponent=1)
        message=common.MAVLink_local_position_ned_message(1000,2,1,-3,5,4,-6)
        wire=message.pack(sender)
        decoded=PassiveMavlink().decode(wire)
        self.assertEqual(decoded[0].get_type(),'LOCAL_POSITION_NED')
        self.assertEqual(decoded[0].get_srcSystem(),1)
        fixture_data=json.loads(Path('tests/fixtures/stage13/mavlink-wire.json').read_text())
        self.assertFalse(fixture_data['captured_from_sitl'])
        for version in ('v1_hex','v2_hex'):
            self.assertEqual(PassiveMavlink().decode(bytes.fromhex(fixture_data[version]))[0].x,2)
        with self.assertRaises(Exception):PassiveMavlink().decode(wire[:-1]+bytes([wire[-1]^255]))


    def test_real_udp_receive_with_generated_wire_fixture(self):
        try:
            decoder = PassiveMavlink()
        except ImportError:
            self.skipTest('Optional pymavlink parser unavailable')
        import socket
        from unittest.mock import Mock
        data=json.loads(Path('tests/fixtures/stage13/mavlink-wire.json').read_text())
        adapter=RealSitlAdapter(evidence='synthetic_fixture')
        adapter.clock(fixture()['clock'],10)
        node=Mock();node.subscribe.return_value=True
        collector=Collector(adapter,node,object,object,decoder)
        try:
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as receiver, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
                receiver.bind(('127.0.0.1',0));receiver.settimeout(2)
                sender.sendto(bytes.fromhex(data['v2_hex']),receiver.getsockname())
                packet,peer=receiver.recvfrom(65535)
                collector.datagram(packet,peer,10)
                self.assertEqual(adapter.position[2],(1,2,3))
                # Without status/heartbeat this raw estimate is not valid state.
                self.assertIsNone(adapter.telemetry(10)['autopilot_estimate']['vehicles'][0]['position_enu_m'])
                self.assertFalse(collector.datagram(packet,('127.0.0.1',peer[1]+1),10))
        finally:
            collector.close()
