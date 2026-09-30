"""Stage 11 real HTTP/WebSocket checks against unchanged reference simulation."""
import argparse
import asyncio
import json
import os
from pathlib import Path
import platform
import secrets
import socket
import subprocess
import sys
import tempfile
import time
import httpx
from websockets.asyncio.client import connect
from drone_sim.fleet import simulate
from .records import digest,encoded
from .service import parse_config


def check_clock(row):
    assert row['simulation_time_s']==row['tick']*row['dt_s']
    for vehicle in row['vehicles'].values():
        assert (vehicle['tick'],vehicle['dt_s'],vehicle['simulation_time_s'])==(row['tick'],row['dt_s'],row['simulation_time_s'])

async def run_case(client,base,control,view,name):
    config=json.loads(Path('scenarios/stage8',name+'.json').read_text())
    original=list(simulate(parse_config({'kind':'fleet','config':config})))
    expected=digest(original)
    headers={'Authorization':'Bearer '+control}
    result=await client.post('/v1/runs',json={'kind':'fleet','config':config},headers=headers)
    result.raise_for_status();run=result.json()['run_id']
    active=(await client.get('/v1/configurations/active',headers={'Authorization':'Bearer '+view})).json()
    assert active['run_id']==run and encoded(active['config'])==encoded(parse_config({'kind':'fleet','config':config}).to_dict())
    async def drain(ws):
        count=0;prior=0;warning=False;last=None
        while True:
            message=json.loads(await asyncio.wait_for(ws.recv(),90))
            if message['type']!='telemetry':continue
            assert message['sequence']>prior;prior=message['sequence'];count+=1
            truth=message['simulation_truth'];check_clock(truth)
            reference=original[truth['tick']]
            assert encoded(truth)==encoded({k:v for k,v in reference.items() if k!='proximity'})
            assert encoded(message['advisories']['proximity'])==encoded(reference['proximity'])
            warning=warning or bool(message['advisories']['proximity']);last=message
            if message['status']['state']=='finished':break
        return {'received_samples':count,'observed_proximity':warning,'last_sequence':last['sequence']}
    started=time.monotonic()
    async with connect(base.replace('http:','ws:')+'/v1/telemetry',origin=base,max_size=2*1024*1024) as a,connect(base.replace('http:','ws:')+'/v1/telemetry',origin=base,max_size=2*1024*1024) as b:
        await a.send(json.dumps({'token':view}));await b.send(json.dumps({'token':view}))
        tasks=[asyncio.create_task(drain(ws)) for ws in (a,b)]
        try:
            response=await client.post('/v1/playback',json={'run_id':run,'paused':False,'speed':20},headers=headers);response.raise_for_status()
            viewers=await asyncio.gather(*tasks)
        finally:
            for task in tasks:task.cancel()
            await asyncio.gather(*tasks,return_exceptions=True)
    duration=time.monotonic()-started
    rows=[]
    for offset in range(0,config['steps']+1,1000):
        response=await client.get(f'/v1/results/{run}/telemetry',params={'offset':offset,'limit':1000},headers=headers);response.raise_for_status();rows.extend(response.json()['rows'])
    assert len(rows)==len(original) and digest(rows)==expected
    for row in rows:check_clock(row)
    result=(await client.get('/v1/results/'+run,headers=headers)).json()
    assert result['telemetry_sha256']==expected
    verification=await client.post('/v1/results/'+run+'/verify-replay',headers=headers);verification.raise_for_status();assert verification.json()['matches']
    async with connect(base.replace('http:','ws:')+'/v1/telemetry',origin=base) as ws:
        await ws.send(json.dumps({'token':view,'epoch':active['epoch'],'after_sequence':1}))
        recovered=json.loads(await asyncio.wait_for(ws.recv(),10))
        assert recovered['delivery']=='resync' and recovered['simulation_truth']['tick']==config['steps']
    summary=result['summary']
    if name=='independent-surveys':assert summary['all_completed']
    if name=='crossing':assert summary['proximity_episodes']>0
    if name=='one-battery-failure':
        assert summary['vehicles']['alpha']['stop_reason']=='critical_battery'
        assert summary['vehicles']['bravo']['final_mission_state']=='completed'
    return {'scenario':name,'ticks':config['steps'],'simulation_duration_s':summary['duration_s'],'observed_wall_duration_s':duration,'viewers':viewers,'recorded_samples':len(rows),'matches_original_simulation':True,'deterministic_replay_matches':True,'stale_cursor_resync':True,'telemetry_sha256':expected,'summary':summary}

async def exercise(base,control,view):
    async with httpx.AsyncClient(base_url=base,timeout=90) as client:
        for _ in range(100):
            try:
                if (await client.get('/v1/status',headers={'Authorization':'Bearer '+view})).status_code==200:break
            except httpx.ConnectError:pass
            await asyncio.sleep(.05)
        else:raise RuntimeError('Backend startup timeout')
        assert (await client.get('/v1/configurations/active')).status_code==401
        cases=[]
        for name in ('independent-surveys','crossing','one-battery-failure'):
            cases.append(await run_case(client,base,control,view,name))
        return {'schema_version':1,'release_candidate':'0.1.0-rc.1','runtime':platform.python_version(),'transport':'actual loopback HTTP and two simultaneous WebSocket viewers','cases':cases}

def main():
    if not __debug__:raise RuntimeError("Run release verification without Python -O")
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--report',required=True);args=parser.parse_args()
    with socket.socket() as probe:probe.bind(('127.0.0.1',0));port=probe.getsockname()[1]
    control,view=secrets.token_urlsafe(32),secrets.token_urlsafe(32)
    with tempfile.TemporaryDirectory() as directory:
        with open(Path(directory)/'server.log','w+') as log:
            process=subprocess.Popen([sys.executable,'-m','mission_control','--port',str(port),'--record-dir',str(Path(directory)/'records')],env=dict(os.environ,DRONE_CONTROL_TOKEN=control,DRONE_VIEW_TOKEN=view),stdout=log,stderr=log)
            try:
                report=asyncio.run(exercise(f'http://127.0.0.1:{port}',control,view))
                output=Path(args.report);output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(report,indent=2)+'\n')
                print(json.dumps({'passed':len(report['cases']),'report':str(output)}))
            finally:
                process.terminate()
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:process.kill();process.wait()
                log.seek(0);logs=log.read();assert control not in logs and view not in logs

if __name__=='__main__':main()
