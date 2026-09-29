"""Optional transport tests plus standard-library coordinator regression tests."""
import asyncio
from dataclasses import replace
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from drone_sim.fleet import FleetConfig,simulate
from mission_control.records import RecordStore,digest
from mission_control.service import Service,parse_config

CONTROL='test-control-'+'a'*32
VIEW='test-view-'+'b'*32
AVAILABLE=all(importlib.util.find_spec(name) is not None for name in ('fastapi','httpx'))
if AVAILABLE:
    from mission_control.app import create_app
    from fastapi.testclient import TestClient
    from starlette.websockets import WebSocketDisconnect


def payload(steps=20,name='crossing'):
    data=json.loads(Path('scenarios/stage8',name+'.json').read_text())
    data['steps']=steps
    data['actions']=[a for a in data['actions'] if a['tick']<steps]
    return {'kind':'fleet','config':data}


class CoordinatorTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.directory=tempfile.TemporaryDirectory()
        self.store=RecordStore(self.directory.name)
        self.service=Service(self.store,queue_size=2,history_size=4)
        self.config=parse_config(payload())
        await self.service.load(self.config)

    async def asyncTearDown(self): self.directory.cleanup()

    async def run_all(self,service=None,speed=1):
        service=service or self.service
        await service.playback(service.run_id,False,speed)
        while service.state=='loaded': await service.tick()

    async def test_exact_clock_and_baseline_replay(self):
        await self.run_all()
        self.assertEqual(digest(self.service.rows),digest(list(simulate(self.config))))
        self.assertEqual(self.service.rows[-1]['tick'],20)
        self.assertTrue(self.store.verify(self.service.run_id)['matches'])

    async def test_playback_pause_and_speed_do_not_alter_motion(self):
        await self.service.tick()
        self.assertEqual(self.service.engine.clock.tick,0)
        await self.service.playback(self.service.run_id,False,20)
        await self.service.tick()
        await self.service.playback(self.service.run_id,True,.1)
        for _ in range(3): await self.service.tick()
        self.assertEqual(self.service.engine.clock.tick,1)
        await self.run_all(speed=.1)
        self.assertEqual(digest(self.service.rows),digest(list(simulate(self.config))))

    async def test_command_order_idempotency_and_rejection(self):
        run=self.service.run_id
        a=await self.service.command(run,'pause','pause','eastbound',0)
        self.assertEqual(a,await self.service.command(run,'pause','pause','eastbound',0))
        await self.service.command(run,'bad','start','eastbound',0)
        await self.service.command(run,'resume','resume','eastbound',0)
        await self.run_all()
        outcomes=self.service.outcomes[:5]
        self.assertEqual([o['command'] for o in outcomes],['start','pause','start','resume'])
        self.assertEqual([o['accepted'] for o in outcomes],[True,True,False,True])
        self.assertTrue(self.store.verify(run)['matches'])

    async def test_stale_and_unknown_commands_no_mutation(self):
        with self.assertRaises(ValueError): await self.service.command('old','x','abort')
        with self.assertRaises(ValueError): await self.service.command(self.service.run_id,'x','abort','nope')
        self.assertEqual(self.service.pending,{})
        await self.service.command(self.service.run_id,'x','pause')
        with self.assertRaises(ValueError): await self.service.command(self.service.run_id,'x','resume')

    async def test_bounded_command_queue(self):
        for i in range(128): await self.service.command(self.service.run_id,str(i),'pause')
        with self.assertRaises(ValueError): await self.service.command(self.service.run_id,'overflow','pause')
        self.assertEqual(sum(map(len,self.service.pending.values())),128)

    async def test_slow_viewer_queue_and_reconnect(self):
        queue=await self.service.subscribe()
        initial=await queue.get()
        await self.run_all()
        self.assertLessEqual(queue.qsize(),2)
        messages=[]
        while not queue.empty(): messages.append(queue.get_nowait())
        self.assertEqual(messages[-1]['sequence'],self.service.sequence)
        self.assertTrue(any(m['delivery']=='resync' for m in messages))
        reconnect=await self.service.subscribe(initial['epoch'],initial['sequence'])
        self.assertEqual((await reconnect.get())['delivery'],'resync')
        recent=await self.service.subscribe(self.service.epoch,self.service.sequence-1)
        self.assertEqual((await recent.get())['delivery'],'replay')
        self.service.unsubscribe(queue)
        self.assertNotIn(queue,self.service.viewers)

    async def test_initial_snapshot_truth_separation(self):
        message=self.service.latest
        self.assertIsNone(message['estimated_tracking'])
        self.assertNotIn('proximity',message['simulation_truth'])
        self.assertTrue(message['advisories']['route_conflicts'])
        self.assertEqual(message['simulation_truth']['time_domain'],'simulation')

    async def test_vehicle_failure_does_not_stop_peer(self):
        config=parse_config(payload(700,'one-battery-failure'))
        other=Service(self.store)
        await other.load(config); await self.run_all(other)
        record=self.store.read(other.run_id)
        self.assertEqual(record['summary']['vehicles']['bravo']['final_mission_state'],'completed')
        self.assertEqual(record['summary']['vehicles']['alpha']['stop_reason'],'critical_battery')
        self.assertTrue(self.store.verify(other.run_id)['matches'])

    async def test_invalid_scheduled_command_is_recorded_and_isolated(self):
        from drone_sim.fleet import FleetAction
        other=Service(self.store)
        await other.load(replace(self.config,actions=(FleetAction(0,'resume','eastbound'),FleetAction(0,'start'))))
        await self.run_all(other)
        self.assertFalse(other.outcomes[0]['accepted'])
        self.assertTrue(self.store.verify(other.run_id)['matches'])

    async def test_engine_exception_marks_partial_record_failed(self):
        await self.service.playback(self.service.run_id,False,1)
        with patch.object(self.service.engine,'step',side_effect=RuntimeError('fixture failure')):
            await self.service.tick()
        self.assertEqual(self.service.state,'failed')
        self.assertTrue(self.service.recorded)
        with self.assertRaises(ValueError): self.store.verify(self.service.run_id)

    async def test_recording_failure_is_not_success(self):
        with patch.object(self.store,'save',side_effect=OSError('disk fixture')):
            await self.run_all()
        self.assertEqual(self.service.state,'failed')
        self.assertFalse(self.service.recorded)
        self.assertIn('recording_failed',self.service.error)

    async def test_store_checksum_capacity_and_path_validation(self):
        await self.run_all()
        path=self.store.path(self.service.run_id)
        record=json.loads(path.read_text());record['rows'][0]['tick']=999
        path.write_text(json.dumps(record))
        with self.assertRaises(ValueError): self.store.verify(self.service.run_id)
        with self.assertRaises(ValueError): self.store.read('../secret')
        self.store.max_records=1
        with self.assertRaises(ValueError): await self.service.load(self.config)

    async def test_load_guard_and_replacement_resets_buffers(self):
        with self.assertRaises(ValueError): await self.service.load(self.config)
        await self.run_all(); old=self.service.run_id
        await self.service.load(self.config)
        self.assertNotEqual(old,self.service.run_id)
        self.assertEqual(len(self.service.history),1)
        self.assertEqual(self.service.outcomes,[])

    async def test_stage7_wrapper_and_limits(self):
        data=payload()['config']
        wrapper={'kind':'mission','config':{'fleet_id':'single','member':data['members'][0],
                                          'steps':20,'actions':data['actions']}}
        self.assertEqual(len(parse_config(wrapper).members),1)
        invalid=payload(5001)
        with self.assertRaises(ValueError): parse_config(invalid)


    async def test_real_runner_pause_resume_and_speed(self):
        task=asyncio.create_task(self.service.run())
        try:
            await asyncio.sleep(.03)
            self.assertEqual(self.service.engine.clock.tick,0)
            await self.service.playback(self.service.run_id,False,2)
            async def finished():
                while self.service.state=='loaded': await asyncio.sleep(.01)
            await asyncio.wait_for(finished(),2)
            self.assertEqual(digest(self.service.rows),digest(list(simulate(self.config))))
        finally:
            self.service.closing=True;self.service.wake.set();await task

    async def test_future_commands_and_concurrent_submission(self):
        run=self.service.run_id
        a,b=await asyncio.gather(self.service.command(run,'a','pause','eastbound',2),
                                 self.service.command(run,'b','resume','eastbound',2))
        self.assertEqual((a['order'],b['order']),(1,2))
        await self.run_all()
        self.assertEqual([o['tick'] for o in self.service.outcomes],[0,2,2])
        self.assertTrue(self.store.verify(run)['matches'])

    async def test_viewer_limits_and_invalid_cursor(self):
        for _ in range(16): await self.service.subscribe()
        with self.assertRaises(ValueError): await self.service.subscribe()
        with self.assertRaises(ValueError): Service(self.store,queue_size=0)

    async def test_recording_budget_stops_without_false_completion(self):
        self.store.max_bytes=1000
        await self.run_all()
        self.assertEqual(self.service.state,'failed')
        self.assertTrue(self.service.paused)


@unittest.skipUnless(AVAILABLE,'Optional backend dependencies unavailable')
class TransportTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory()
        self.app=create_app(CONTROL,VIEW,self.directory.name,start_runner=False,queue_size=2)
        self.client=TestClient(self.app,base_url='http://127.0.0.1:8000',client=('127.0.0.1',50000))
        self.client.__enter__()
        self.auth={'Authorization':'Bearer '+CONTROL}
        self.read={'Authorization':'Bearer '+VIEW}

    def tearDown(self):
        self.client.__exit__(None,None,None)
        self.directory.cleanup()

    def load(self,steps=20):
        result=self.client.post('/v1/runs',json=payload(steps),headers=self.auth)
        self.assertEqual(result.status_code,200,result.text)
        return result.json()['run_id']

    def tick(self): self.client.portal.call(self.app.state.service.tick)

    def test_auth_scope_and_origin(self):
        self.assertEqual(self.client.get('/v1/status').status_code,401)
        self.assertEqual(self.client.post('/v1/runs',json=payload(),headers=self.read).status_code,403)
        self.assertEqual(self.client.get('/v1/status',headers={**self.auth,'Origin':'https://evil.example'}).status_code,403)
        self.assertEqual(self.client.get('/v1/status',headers={**self.auth,'Host':'evil.example'}).status_code,400)
        self.assertEqual(self.client.get('/v1/status',headers=self.read).status_code,200)

    def test_validate_load_snapshot_and_strict_requests(self):
        valid=self.client.post('/v1/configurations/validate',json=payload(),headers=self.read)
        self.assertEqual(valid.status_code,200)
        self.assertEqual(self.client.get('/v1/status',headers=self.read).json()['state'],'empty')
        run=self.load()
        self.assertEqual(self.client.get('/v1/snapshot',headers=self.read).json()['run_id'],run)
        self.assertEqual(self.client.post('/v1/configurations/validate',json={'kind':'fleet','config':{}},headers=self.read).status_code,422)
        self.assertEqual(self.client.post('/v1/playback',json={'run_id':run,'paused':'false','speed':1},headers=self.auth).status_code,422)
        self.assertEqual(self.client.post('/v1/playback',json={'run_id':'old','paused':False,'speed':1},headers=self.auth).status_code,409)

    def test_body_limit_and_no_public_docs(self):
        response=self.client.post('/v1/runs',content=b'x'*(1024*1024+1),headers=self.auth)
        self.assertEqual(response.status_code,413)
        self.assertEqual(self.client.get('/docs').status_code,404)
        self.assertEqual(self.client.get('/v1/openapi.json',headers=self.read).status_code,200)

    def test_concurrent_fleet_and_vehicle_websockets(self):
        run=self.load()
        with self.client.websocket_connect('ws://127.0.0.1:8000/v1/telemetry') as fleet, self.client.websocket_connect('ws://127.0.0.1:8000/v1/telemetry') as vehicle:
            fleet.send_json({'token':VIEW}); vehicle.send_json({'token':VIEW,'vehicle_id':'eastbound'})
            a,b=fleet.receive_json(),vehicle.receive_json()
            self.assertEqual(a['sequence'],b['sequence'])
            self.assertEqual(b['type'],'vehicle_telemetry')
            self.assertEqual(b['simulation_truth']['vehicle']['vehicle_id'],'eastbound')
            self.client.post('/v1/playback',json={'run_id':run,'paused':False,'speed':2},headers=self.auth)
            self.tick()
            a,b=fleet.receive_json(),vehicle.receive_json()
            self.assertEqual(a['sequence'],b['sequence'])
            self.assertEqual(b['simulation_truth']['tick'],1)

    def test_websocket_auth_and_read_only(self):
        self.load()
        with self.client.websocket_connect('ws://127.0.0.1:8000/v1/telemetry') as socket:
            socket.send_json({'token':'bad'})
            with self.assertRaises(WebSocketDisconnect): socket.receive_json()
        with self.client.websocket_connect('ws://127.0.0.1:8000/v1/telemetry') as socket:
            socket.send_json({'token':VIEW});socket.receive_json()
            socket.send_json({'command':'abort'})
            with self.assertRaises(WebSocketDisconnect): socket.receive_json()
        with self.assertRaises(WebSocketDisconnect):
            with self.client.websocket_connect('ws://127.0.0.1:8000/v1/telemetry',headers={'Origin':'http://evil.example'}): pass

    def test_disconnect_reconnect_and_epoch(self):
        run=self.load()
        with self.client.websocket_connect('ws://127.0.0.1:8000/v1/telemetry') as socket:
            socket.send_json({'token':VIEW});first=socket.receive_json()
        self.client.post('/v1/playback',json={'run_id':run,'paused':False,'speed':1},headers=self.auth)
        self.tick()
        with self.client.websocket_connect('ws://127.0.0.1:8000/v1/telemetry') as socket:
            socket.send_json({'token':VIEW,'epoch':first['epoch'],'after_sequence':first['sequence']})
            message=socket.receive_json();self.assertEqual(message['delivery'],'replay')
            self.assertEqual(message['simulation_truth']['tick'],1)
        with self.client.websocket_connect('ws://127.0.0.1:8000/v1/telemetry') as socket:
            socket.send_json({'token':VIEW,'epoch':'old','after_sequence':0})
            self.assertEqual(socket.receive_json()['delivery'],'resync')
        self.assertEqual(len(self.app.state.service.viewers),0)

    def test_rest_commands_results_and_replay(self):
        run=self.load(4)
        ack=self.client.post('/v1/commands',json={'run_id':run,'request_id':'pause','command':'pause','vehicle_id':'eastbound','target_tick':1},headers=self.auth)
        self.assertEqual(ack.status_code,200)
        self.client.post('/v1/playback',json={'run_id':run,'paused':False,'speed':1},headers=self.auth)
        for _ in range(4): self.tick()
        self.assertIn(run,self.client.get('/v1/results',headers=self.read).json()['run_ids'])
        report=self.client.get('/v1/results/'+run,headers=self.read).json()
        self.assertEqual(report['state'],'finished')
        self.assertFalse(report['summary']['all_completed'])
        self.assertEqual(self.client.get('/v1/results/'+run+'/telemetry?limit=2',headers=self.read).json()['total'],5)
        self.assertTrue(self.client.post('/v1/results/'+run+'/verify-replay',headers=self.auth).json()['matches'])
        replay=self.client.post('/v1/results/'+run+'/replay',headers=self.auth)
        self.assertEqual(replay.status_code,200,replay.text)
        self.assertNotEqual(replay.json()['run_id'],run)

    def test_settings_and_remote_peer_rejection(self):
        with self.assertRaises(ValueError): create_app('short',VIEW,self.directory.name)
        with self.assertRaises(ValueError): create_app(CONTROL,VIEW,self.directory.name,origins=('https://example.com',))
        other=TestClient(self.app,base_url='http://127.0.0.1',client=('192.0.2.1',1))
        self.assertEqual(other.get('/v1/status',headers=self.auth).status_code,403)


    def test_single_server_ownership_and_shutdown_record(self):
        run=self.load()
        second=create_app(CONTROL,VIEW,self.directory.name,start_runner=False)
        with self.assertRaises(RuntimeError):
            with TestClient(second,base_url='http://127.0.0.1',client=('127.0.0.1',5)): pass
        self.client.__exit__(None,None,None)
        record=RecordStore(self.directory.name).read(run)
        self.assertEqual(record['state'],'interrupted')
        # Re-enter to leave normal tearDown ownership intact.
        self.client.__enter__()

    def test_invalid_websocket_payload_and_vehicle(self):
        self.load()
        for value in ({'token':VIEW,'vehicle_id':'missing'},{'token':VIEW,'after_sequence':True},
                      {'token':VIEW,'unknown':1},[]):
            with self.client.websocket_connect('ws://127.0.0.1:8000/v1/telemetry') as socket:
                socket.send_json(value)
                with self.assertRaises(WebSocketDisconnect): socket.receive_json()


if __name__=='__main__': unittest.main()
