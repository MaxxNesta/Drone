"""Reproducible loopback HTTP/WebSocket smoke test against a real Uvicorn subprocess."""
import argparse
import asyncio
import json
import os
from pathlib import Path
import secrets
import socket
import subprocess
import sys
import tempfile
import time
import httpx
from websockets.asyncio.client import connect


async def exercise(base, ws_url, control, view):
    headers={'Authorization':'Bearer '+control}
    async with httpx.AsyncClient(base_url=base,timeout=10) as client:
        for _ in range(100):
            try:
                response=await client.get('/v1/status',headers=headers)
                if response.status_code==200: break
            except httpx.ConnectError: pass
            await asyncio.sleep(.05)
        else: raise RuntimeError('Server did not become ready')
        assert (await client.get('/v1/status')).status_code==401
        config=json.loads(Path('scenarios/stage8/crossing.json').read_text())
        config['steps']=80
        response=await client.post('/v1/runs',json={'kind':'fleet','config':config},headers=headers)
        response.raise_for_status();run=response.json()['run_id']
        started=time.monotonic()
        async with connect(ws_url,origin=base) as fleet, connect(ws_url,origin=base) as vehicle:
            await fleet.send(json.dumps({'token':view}))
            await vehicle.send(json.dumps({'token':view,'vehicle_id':'eastbound'}))
            initial=json.loads(await fleet.recv()); individual=json.loads(await vehicle.recv())
            assert initial['sequence']==individual['sequence']
            async def drain_vehicle():
                while True:
                    sample=json.loads(await asyncio.wait_for(vehicle.recv(),10))
                    if sample['status']['state']=='finished': return sample
            vehicle_task=asyncio.create_task(drain_vehicle())
            response=await client.post('/v1/playback',json={'run_id':run,'paused':False,'speed':2},headers=headers)
            response.raise_for_status()
            messages=[]
            while True:
                message=json.loads(await asyncio.wait_for(fleet.recv(),10))
                messages.append(message)
                if message['status']['state']=='finished': break
            assert all(a['sequence']<b['sequence'] for a,b in zip(messages,messages[1:]))
            assert messages[-1]['simulation_truth']['tick']==80
            final_vehicle=await vehicle_task
            assert final_vehicle['sequence']==messages[-1]['sequence']
            elapsed=time.monotonic()-started
        # Reconnection after a finished run still obtains authoritative truth.
        async with connect(ws_url,origin=base) as ws:
            await ws.send(json.dumps({'token':view,'epoch':'lost-client-state','after_sequence':0}))
            reconnect=json.loads(await ws.recv())
            assert reconnect['delivery']=='resync'
            assert reconnect['simulation_truth']['tick']==80
        verification=await client.post('/v1/results/'+run+'/verify-replay',headers=headers)
        verification.raise_for_status(); assert verification.json()['matches']
        summary=(await client.get('/v1/results/'+run,headers=headers)).json()
        return {'schema_version':1,'transport':'real loopback Uvicorn + HTTP + WebSockets',
                'simulated_ticks':80,'simulation_duration_s':1.6,'requested_playback_speed':2,
                'observed_wall_duration_s':elapsed,'received_fleet_samples_after_initial':len(messages),
                'concurrent_fleet_and_vehicle_viewers':True,'unauthenticated_rest_rejected':True,
                'reconnection_resync':True,'deterministic_replay_matches':verification.json()['matches'],
                'run_state':summary['state'],'mission_all_completed':summary['summary']['all_completed']}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report',required=True)
    args=parser.parse_args()
    with socket.socket() as probe:
        probe.bind(('127.0.0.1',0));port=probe.getsockname()[1]
    control,view=secrets.token_urlsafe(32),secrets.token_urlsafe(32)
    with tempfile.TemporaryDirectory() as directory:
        environment=dict(os.environ,DRONE_CONTROL_TOKEN=control,DRONE_VIEW_TOKEN=view)
        with open(Path(directory)/'server.log','w+') as log:
            process=subprocess.Popen([sys.executable,'-m','mission_control','--port',str(port),
                                      '--record-dir',str(Path(directory)/'records')],env=environment,stdout=log,stderr=log)
            try:
                report=asyncio.run(exercise(f'http://127.0.0.1:{port}',f'ws://127.0.0.1:{port}/v1/telemetry',control,view))
                Path(args.report).write_text(json.dumps(report,indent=2)+'\n')
                print(json.dumps(report))
            finally:
                process.terminate()
                try: process.wait(timeout=10)
                except subprocess.TimeoutExpired: process.kill();process.wait()
                log.seek(0)
                logs=log.read()
                assert control not in logs and view not in logs, 'Credential leaked into server log'
                if process.returncode not in (0,-15): print(logs,file=sys.stderr)


if __name__=='__main__': main()
