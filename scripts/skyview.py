#!/usr/bin/env python3
"""Start the local SKYVIEW release; owns and stops only its two child processes."""
import argparse
import os
from pathlib import Path
import secrets
import shutil
import signal
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request

ROOT=Path(__file__).resolve().parents[1]
KEYS={'DRONE_CONTROL_TOKEN','DRONE_VIEW_TOKEN','DASHBOARD_ACCESS_KEY','BACKEND_PORT','DASHBOARD_PORT','BACKEND_PYTHON','RECORD_DIR'}

def initialize(path):
    values={name:secrets.token_urlsafe(32) for name in ('DRONE_CONTROL_TOKEN','DRONE_VIEW_TOKEN','DASHBOARD_ACCESS_KEY')}
    values.update(BACKEND_PORT='8000',DASHBOARD_PORT='3000',BACKEND_PYTHON='.venv-backend/bin/python',RECORD_DIR='artifacts/skyview/records')
    with os.fdopen(os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600),'w') as file:
        file.write('# Local secrets: do not commit or share this file.\n')
        file.writelines(f'{key}={value}\n' for key,value in values.items())

def configuration(path):
    values={}
    for line in path.read_text().splitlines():
        line=line.strip()
        if not line or line.startswith('#'): continue
        key,separator,value=line.partition('=')
        if not separator or key not in KEYS or key in values:
            raise ValueError('Unknown, duplicate or malformed environment setting')
        values[key]=value.strip()
    if set(values)!=KEYS: raise ValueError('Environment file must define exactly the documented settings')
    tokens=[values[k] for k in ('DRONE_CONTROL_TOKEN','DRONE_VIEW_TOKEN','DASHBOARD_ACCESS_KEY')]
    if len(set(tokens))!=3 or any(len(t)<32 or t.startswith('REPLACE') for t in tokens):
        raise ValueError('Initialize three distinct random credentials before starting')
    ports=[int(values[k]) for k in ('BACKEND_PORT','DASHBOARD_PORT')]
    if len(set(ports))!=2 or any(not 1024<=p<=65535 for p in ports): raise ValueError('Two distinct ports from 1024 to 65535 required')
    for key in ('BACKEND_PYTHON','RECORD_DIR'):
        p=Path(values[key]);values[key]=str(p if p.is_absolute() else ROOT/p)
    return values

def preflight(values,check_ports=True):
    node=shutil.which('node');npm=shutil.which('npm')
    if not node or not npm: raise ValueError('Node 24 and npm are required; see docs/STAGE11_INTEGRATION.md')
    if int(subprocess.check_output([node,'--version'],text=True).lstrip('v').split('.')[0])<24:
        raise ValueError('Use the tested Node 24 or newer runtime')
    subprocess.run([values['BACKEND_PYTHON'],'-c','import sys,fastapi,uvicorn; assert sys.version_info >= (3,12)'],check=True,cwd=ROOT)
    if not (ROOT/'dashboard/node_modules/next/package.json').is_file(): raise ValueError('Run npm ci in dashboard first')
    if check_ports:
        for key in ('BACKEND_PORT','DASHBOARD_PORT'):
            with socket.socket() as probe:
                try: probe.bind(('127.0.0.1',int(values[key])))
                except OSError as error: raise ValueError(f'{key} is already in use; stop its owner or choose another port') from error
    return node,npm

def ready(url,process,token=None):
    deadline=time.monotonic()+30
    while time.monotonic()<deadline:
        if process.poll() is not None: raise RuntimeError('A local server exited during startup')
        try:
            request=urllib.request.Request(url,headers={'Authorization':'Bearer '+token} if token else {})
            with urllib.request.urlopen(request,timeout=1) as response:
                if response.status==200:return
        except (urllib.error.URLError,TimeoutError):pass
        time.sleep(.1)
    raise RuntimeError('Local server did not become ready within 30 seconds')

def stop(children):
    for child in reversed(children):
        if child.poll() is None: child.terminate()
    for child in reversed(children):
        try:child.wait(timeout=10)
        except subprocess.TimeoutExpired:child.kill();child.wait()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--env',type=Path,default=ROOT/'.skyview.env')
    parser.add_argument('--init-env',action='store_true')
    parser.add_argument('--check',action='store_true')
    parser.add_argument('--no-build',action='store_true',help='Use an already verified production build')
    args=parser.parse_args();children=[]
    try:
        if args.init_env:
            initialize(args.env);print(f'Created private environment file: {args.env}');return 0
        values=configuration(args.env);node,npm=preflight(values)
        if args.check:print('Local release prerequisites and ports verified');return 0
        if not args.no_build:subprocess.run([npm,'run','build'],cwd=ROOT/'dashboard',check=True)
        elif not (ROOT/'dashboard/.next/BUILD_ID').is_file():raise ValueError('Production build missing; omit --no-build')
        environment=dict(os.environ,**values,DRONE_BACKEND_URL='http://127.0.0.1:'+values['BACKEND_PORT'])
        def interrupt(signum,frame):raise KeyboardInterrupt
        signal.signal(signal.SIGTERM,interrupt)
        backend=subprocess.Popen([values['BACKEND_PYTHON'],'-m','mission_control','--port',values['BACKEND_PORT'],'--record-dir',values['RECORD_DIR']],cwd=ROOT,env=environment,start_new_session=True)
        children.append(backend);ready('http://127.0.0.1:'+values['BACKEND_PORT']+'/v1/status',backend,values['DRONE_VIEW_TOKEN'])
        dashboard=subprocess.Popen([node,'server/index.mjs'],cwd=ROOT/'dashboard',env=environment,start_new_session=True)
        children.append(dashboard);ready('http://127.0.0.1:'+values['DASHBOARD_PORT'],dashboard)
        print('SKYVIEW ready: http://127.0.0.1:'+values['DASHBOARD_PORT'],flush=True)
        print(f'Unlock using DASHBOARD_ACCESS_KEY in {args.env}. Ctrl-C stops both local servers.',flush=True)
        while all(child.poll() is None for child in children):time.sleep(.25)
        raise RuntimeError('A server exited; stopping its companion')
    except KeyboardInterrupt:return 0
    except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as error:
        print('SKYVIEW startup failed: '+str(error),file=sys.stderr);return 1
    finally:stop(children)

if __name__=='__main__':sys.exit(main())
