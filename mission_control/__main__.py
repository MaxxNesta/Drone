"""Run one authenticated server bound exclusively to IPv4 loopback."""
import argparse
import logging
import os
from .app import create_app


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=8000)
    parser.add_argument('--record-dir',default='artifacts/stage9/records')
    args=parser.parse_args()
    if not 1024<=args.port<=65535: parser.error('Port must be 1024–65535')
    app=create_app(os.environ.get('DRONE_CONTROL_TOKEN',''),os.environ.get('DRONE_VIEW_TOKEN',''),args.record_dir,
                   origins=(f'http://127.0.0.1:{args.port}',f'http://localhost:{args.port}'),
                   sitl_snapshot_path=os.environ.get('SKYVIEW_SITL_SNAPSHOT') or None)
    logging.basicConfig(level=logging.INFO,format='%(message)s')
    import uvicorn
    uvicorn.run(app,host='127.0.0.1',port=args.port,workers=1,proxy_headers=False,
                access_log=False,ws_max_size=16384,ws_max_queue=4,timeout_graceful_shutdown=5)


if __name__=='__main__': main()
