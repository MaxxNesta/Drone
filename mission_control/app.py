"""Authenticated loopback FastAPI/WebSocket boundary for the virtual fleet."""
import asyncio
from contextlib import asynccontextmanager, suppress
import fcntl
import ipaddress
import json
from pathlib import Path
import secrets
from urllib.parse import urlparse
from fastapi import FastAPI, Depends, HTTPException, Request, WebSocket, WebSocketDisconnect
from pydantic import BaseModel, ConfigDict, Field, StrictBool
from starlette.middleware.cors import CORSMiddleware
from starlette.middleware.trustedhost import TrustedHostMiddleware
from starlette.responses import JSONResponse
from .records import RecordStore
from .service import Service, parse_config


class LocalBoundary:
    """Reject non-loopback clients and bound HTTP bodies before JSON/Pydantic parsing."""
    def __init__(self, app, max_body=1024*1024):
        self.app, self.max_body = app, max_body

    async def __call__(self, scope, receive, send):
        if scope['type'] not in ('http','websocket'):
            return await self.app(scope, receive, send)
        try:
            local = ipaddress.ip_address(scope['client'][0]).is_loopback
        except (ValueError, TypeError):
            local = False
        if not local:
            if scope['type'] == 'websocket':
                await send({'type':'websocket.close','code':1008})
            else:
                await JSONResponse({'detail':'Loopback clients only'},403)(scope, receive, send)
            return
        if scope['type'] == 'http':
            chunks, length = [], 0
            while True:
                message = await receive()
                if message['type'] == 'http.disconnect': return
                length += len(message.get('body',b''))
                if length > self.max_body:
                    return await JSONResponse({'detail':'Request body too large'},413)(scope, receive, send)
                chunks.append(message.get('body',b''))
                if not message.get('more_body'): break
            delivered = False
            async def buffered():
                nonlocal delivered
                if not delivered:
                    delivered = True
                    return {'type':'http.request','body':b''.join(chunks),'more_body':False}
                return await receive()
            return await self.app(scope, buffered, send)
        await self.app(scope, receive, send)


class Playback(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)
    run_id: str
    paused: StrictBool
    speed: float = Field(ge=.1,le=20,allow_inf_nan=False)


class Command(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)
    run_id: str
    request_id: str = Field(min_length=1,max_length=64)
    command: str
    vehicle_id: str | None = None
    target_tick: int | None = None


def create_app(control_token, view_token, record_dir, origins=('http://127.0.0.1:8000','http://localhost:8000'),
               start_runner=True, queue_size=8, history_size=64, sitl_snapshot_path=None):
    if any(not isinstance(t,str) or len(t)<32 for t in (control_token,view_token)) or control_token == view_token:
        raise ValueError('Distinct control/view tokens of at least 32 characters required')
    for origin in origins:
        parsed = urlparse(origin)
        if (parsed.scheme != 'http' or parsed.hostname not in ('127.0.0.1','localhost','::1')
                or parsed.path or parsed.query or parsed.fragment or parsed.username):
            raise ValueError('Only explicit local HTTP browser origins are supported')
    store = RecordStore(record_dir)
    service = Service(store,queue_size,history_size)

    @asynccontextmanager
    async def lifespan(app):
        lock_file = (Path(record_dir)/'.server.lock').open('a')
        try:
            fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            lock_file.close()
            raise RuntimeError('Recording directory already owned by another backend')
        runner = asyncio.create_task(service.run()) if start_runner else None
        try:
            yield
        finally:
            if runner:
                service.closing = True
                service.wake.set()
                await runner
            async with service.lock:
                if service.state == 'loaded':
                    service.state,service.paused = 'interrupted',True
                    await service._save()
            lock_file.close()

    app = FastAPI(title='Drone virtual mission control',version='1.0.0',lifespan=lifespan,
                  docs_url=None,redoc_url=None,openapi_url=None)
    app.state.service = service
    app.state.connections = 0
    app.add_middleware(CORSMiddleware,allow_origins=list(origins),allow_methods=['GET','POST'],
                       allow_headers=['Authorization','Content-Type'])
    app.add_middleware(TrustedHostMiddleware,allowed_hosts=['127.0.0.1','localhost','[::1]'])
    app.add_middleware(LocalBoundary)

    def role(token):
        if secrets.compare_digest(token.encode(),control_token.encode()): return 'control'
        if secrets.compare_digest(token.encode(),view_token.encode()): return 'view'
        return None

    def authorize(request, required):
        origin = request.headers.get('origin')
        if origin is not None and origin not in origins:
            raise HTTPException(403,'Origin denied')
        header = request.headers.get('authorization','')
        identity = role(header[7:]) if header.startswith('Bearer ') else None
        if identity is None: raise HTTPException(401,'Bearer token required')
        if required == 'control' and identity != 'control': raise HTTPException(403,'Control token required')

    async def viewer(request: Request): authorize(request,'view')
    async def controller(request: Request): authorize(request,'control')

    @app.exception_handler(ValueError)
    async def invalid(request, error):
        return JSONResponse({'detail':str(error)},409)

    async def configuration(request):
        try:
            data = await request.json()
            # Reject NaN/Infinity and bound shapes through the existing constructors.
            json.dumps(data,allow_nan=False)
            return parse_config(data)
        except (ValueError,TypeError,KeyError,AttributeError,OverflowError,RecursionError) as error:
            raise HTTPException(422,'Invalid simulation configuration: '+str(error)) from error

    @app.post('/v1/configurations/validate',dependencies=[Depends(viewer)])
    async def validate(request: Request):
        config = await configuration(request)
        return {'schema_version':1,'valid':True,'normalized_config':config.to_dict()}

    @app.post('/v1/runs',dependencies=[Depends(controller)])
    async def load(request: Request):
        return await service.load(await configuration(request))

    @app.get('/v1/sitl',dependencies=[Depends(viewer)])
    async def sitl_snapshot():
        from .sitl import read_sitl
        return await asyncio.to_thread(read_sitl, sitl_snapshot_path)

    @app.get('/v1/status',dependencies=[Depends(viewer)])
    async def status(): return await service.read_status()

    @app.get('/v1/configurations/active',dependencies=[Depends(viewer)])
    async def active_configuration():
        return await service.active_configuration()

    @app.get('/v1/snapshot',dependencies=[Depends(viewer)])
    async def snapshot():
        value=await service.snapshot()
        if value is None: raise HTTPException(409,'No simulation loaded')
        return value

    @app.post('/v1/playback',dependencies=[Depends(controller)])
    async def playback(body: Playback):
        return await service.playback(body.run_id,body.paused,body.speed)

    @app.post('/v1/commands',dependencies=[Depends(controller)])
    async def command(body: Command): return await service.command(**body.model_dump())

    @app.get('/v1/commands',dependencies=[Depends(viewer)])
    async def outcomes():
        return {'run_id':service.run_id,'outcomes':service.outcomes,
                'pending':sum(len(v) for v in service.pending.values())}

    @app.get('/v1/results',dependencies=[Depends(viewer)])
    async def results(): return {'schema_version':1,'run_ids':store.list()}

    def record(run_id):
        try: return store.read(run_id)
        except FileNotFoundError: raise HTTPException(404,'Recording not found')

    @app.get('/v1/results/{run_id}',dependencies=[Depends(viewer)])
    async def result(run_id: str):
        return {k:v for k,v in record(run_id).items() if k!='rows'}

    @app.get('/v1/results/{run_id}/telemetry',dependencies=[Depends(viewer)])
    async def recorded_telemetry(run_id: str,offset: int=0,limit: int=100):
        if offset<0 or not 1<=limit<=1000: raise HTTPException(422,'Invalid page; limit must be 1–1000')
        rows=record(run_id)['rows']
        return {'schema_version':1,'run_id':run_id,'total':len(rows),'offset':offset,'rows':rows[offset:offset+limit]}

    @app.post('/v1/results/{run_id}/verify-replay',dependencies=[Depends(controller)])
    async def verify_replay(run_id: str):
        record(run_id)
        # Serialize expensive verification to bound worker/memory use.
        async with service.lock:
            return await asyncio.to_thread(store.verify,run_id)

    @app.post('/v1/results/{run_id}/replay',dependencies=[Depends(controller)])
    async def replay(run_id: str):
        data=record(run_id)
        async with service.lock:
            verified=await asyncio.to_thread(store.verify,run_id)
        if not verified['matches']: raise HTTPException(409,'Recording does not replay exactly')
        if data['state']!='finished': raise HTTPException(409,'Run was not completed')
        from dataclasses import replace
        from drone_sim.fleet import FleetConfig,FleetAction
        actions=tuple(FleetAction(o['tick'],o['command'],o['vehicle_id']) for o in data['command_outcomes'] if o['accepted'])
        return await service.load(replace(FleetConfig.from_dict(data['config']),actions=actions))

    @app.get('/v1/openapi.json',dependencies=[Depends(viewer)])
    async def schema(): return app.openapi()

    @app.websocket('/v1/telemetry')
    async def telemetry(socket: WebSocket):
        if socket.headers.get('origin') not in (None,*origins) or app.state.connections>=service.max_viewers:
            await socket.close(code=1008); return
        app.state.connections+=1
        queue=None
        tasks=[]
        try:
            await socket.accept()
            # Browser WebSocket API cannot set Authorization headers. No tokens in URLs/logs.
            auth=await asyncio.wait_for(socket.receive_json(),timeout=5)
            if not isinstance(auth,dict) or not isinstance(auth.get('token'),str) or role(auth['token']) is None:
                await socket.close(code=1008); return
            if set(auth)-{'token','epoch','after_sequence','vehicle_id'}:
                await socket.close(code=1008); return
            vehicle_id=auth.get('vehicle_id')
            if vehicle_id is not None and (not service.engine or vehicle_id not in {m.vehicle_id for m in service.config.members}):
                await socket.close(code=1008); return
            after=auth.get('after_sequence')
            if after is not None and (type(after) is not int or after<0):
                await socket.close(code=1008); return
            queue=await service.subscribe(auth.get('epoch'),after)
            async def sending():
                while True:
                    try:
                        message=await asyncio.wait_for(queue.get(),timeout=10)
                    except asyncio.TimeoutError:
                        message={'schema_version':1,'type':'heartbeat',**(await service.read_status())}
                    if vehicle_id is not None and message['type']=='telemetry':
                        truth=message['simulation_truth']
                        if vehicle_id not in truth['vehicles']:
                            await socket.close(code=1008); return
                        message=dict(message,type='vehicle_telemetry',simulation_truth={
                            'tick':truth['tick'],'simulation_time_s':truth['simulation_time_s'],
                            'dt_s':truth['dt_s'],'vehicle':truth['vehicles'][vehicle_id]},
                            advisories={k:[a for a in values if vehicle_id in a['vehicle_ids']]
                                        for k,values in message['advisories'].items()})
                    await asyncio.wait_for(socket.send_json(message),timeout=2)
            async def receiving():
                # Stream is read-only; all commands require authenticated REST authorization.
                await socket.receive_text()
                await socket.close(code=1008)
            tasks=[asyncio.create_task(sending()),asyncio.create_task(receiving())]
            done,_=await asyncio.wait(tasks,return_when=asyncio.FIRST_COMPLETED)
            for task in done: task.result()
        except (WebSocketDisconnect,ValueError,TypeError,asyncio.TimeoutError,RuntimeError):
            with suppress(RuntimeError,WebSocketDisconnect): await socket.close(code=1008)
        finally:
            for task in tasks: task.cancel()
            for task in tasks:
                with suppress(asyncio.CancelledError,WebSocketDisconnect,RuntimeError,ValueError,TypeError,asyncio.TimeoutError): await task
            if queue is not None: service.unsubscribe(queue)
            app.state.connections-=1

    return app
