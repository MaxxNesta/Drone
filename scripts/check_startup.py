"""Exercise the actual local release launcher and owned-process shutdown."""
import argparse
import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from scripts.skyview import ROOT,initialize,configuration


def main():
    if not __debug__:raise RuntimeError("Run release verification without Python -O")
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--report',required=True);args=parser.parse_args()
    with tempfile.TemporaryDirectory() as directory:
        directory=Path(directory);env=directory/'private.env';initialize(env)
        with socket.socket() as a,socket.socket() as b:
            a.bind(('127.0.0.1',0));b.bind(('127.0.0.1',0));ports=(a.getsockname()[1],b.getsockname()[1])
        text=env.read_text().replace('BACKEND_PORT=8000',f'BACKEND_PORT={ports[0]}').replace('DASHBOARD_PORT=3000',f'DASHBOARD_PORT={ports[1]}').replace('BACKEND_PYTHON=.venv-backend/bin/python','BACKEND_PYTHON='+sys.executable).replace('RECORD_DIR=artifacts/skyview/records','RECORD_DIR='+str(directory/'records'))
        env.write_text(text);values=configuration(env)
        with (directory/'launcher.log').open('w+') as log:
            process=subprocess.Popen([sys.executable,'scripts/skyview.py','--env',str(env),'--no-build'],cwd=ROOT,stdout=log,stderr=log)
            ready=False
            try:
                deadline=time.monotonic()+45
                while time.monotonic()<deadline and process.poll() is None:
                    try:
                        with urllib.request.urlopen(f'http://127.0.0.1:{ports[1]}',timeout=1) as response:ready=response.status==200
                        if ready:break
                    except (urllib.error.URLError,TimeoutError):pass
                    time.sleep(.1)
                assert ready,'Launcher did not become ready'
                request=urllib.request.Request(f'http://127.0.0.1:{ports[0]}/v1/configurations/active',headers={'Authorization':'Bearer '+values['DRONE_VIEW_TOKEN']})
                with urllib.request.urlopen(request,timeout=3) as response:assert json.load(response)['config'] is None
            finally:
                process.terminate()
                try:process.wait(timeout=25)
                except subprocess.TimeoutExpired:process.kill();process.wait();raise RuntimeError('Launcher failed to stop')
                log.seek(0);output=log.read()
                assert all(values[key] not in output for key in ('DRONE_CONTROL_TOKEN','DRONE_VIEW_TOKEN','DASHBOARD_ACCESS_KEY'))
            assert process.returncode==0
            for port in ports:
                with socket.socket() as probe:assert probe.connect_ex(('127.0.0.1',port))!=0,'Child server remained running'
            report={'release_candidate':'0.1.0-rc.1','production_dashboard_ready':ready,'authenticated_backend_ready':True,'both_owned_servers_stopped':True,'credentials_absent_from_logs':True}
            target=Path(args.report);target.parent.mkdir(parents=True,exist_ok=True);target.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))

if __name__=='__main__':main()
