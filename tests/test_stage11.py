"""Additive active-configuration contract and release lifecycle regressions."""
import json
import tempfile
import unittest
from mission_control.service import Service,parse_config
from mission_control.records import RecordStore
from tests.test_backend import AVAILABLE,CONTROL,VIEW,payload
if AVAILABLE:
    from mission_control.app import create_app
    from fastapi.testclient import TestClient

class ActiveConfigurationTests(unittest.IsolatedAsyncioTestCase):
    async def test_empty_loaded_detached_finished_and_replaced(self):
        with tempfile.TemporaryDirectory() as directory:
            service=Service(RecordStore(directory))
            empty=await service.active_configuration()
            self.assertIsNone(empty['config']);self.assertIsNone(empty['run_id'])
            await service.load(parse_config(payload(2)))
            before=service.status();first=await service.active_configuration()
            first['config']['members'][0]['mission']['home_enu_m'][0]=999
            self.assertNotEqual((await service.active_configuration())['config'],first['config'])
            self.assertEqual(before,service.status())
            await service.playback(service.run_id,False,1)
            await service.tick();await service.tick()
            final=await service.active_configuration()
            self.assertEqual(final['state'],'finished');self.assertEqual(final['run_id'],first['run_id'])
            await service.load(parse_config(payload(3,'one-battery-failure')))
            second=await service.active_configuration()
            self.assertNotEqual(second['run_id'],first['run_id'])
            self.assertEqual(second['config']['fleet_id'],'one-battery-failure')

    async def test_finished_status_and_snapshot_wait_for_final_publication(self):
        import asyncio
        from unittest.mock import patch
        with tempfile.TemporaryDirectory() as directory:
            service=Service(RecordStore(directory))
            await service.load(parse_config(payload(1)))
            await service.playback(service.run_id,False,1)
            entered,release=asyncio.Event(),asyncio.Event()
            original=service._save
            async def delayed_save():
                entered.set();await release.wait();await original()
            with patch.object(service,'_save',side_effect=delayed_save):
                tick=asyncio.create_task(service.tick());await entered.wait()
                snapshot=asyncio.create_task(service.snapshot())
                status=asyncio.create_task(service.read_status())
                await asyncio.sleep(0)
                self.assertFalse(snapshot.done());self.assertFalse(status.done())
                release.set();await tick
                self.assertEqual((await snapshot)['simulation_truth']['tick'],1)
                self.assertEqual((await status)['state'],'finished')

@unittest.skipUnless(AVAILABLE,'Optional backend dependencies unavailable')
class ActiveConfigurationTransportTests(unittest.TestCase):
    def test_authenticated_read_only_and_no_clock_mutation(self):
        with tempfile.TemporaryDirectory() as directory:
            app=create_app(CONTROL,VIEW,directory,start_runner=False)
            with TestClient(app,base_url='http://127.0.0.1:8000',client=('127.0.0.1',50000)) as client:
                url='/v1/configurations/active';read={'Authorization':'Bearer '+VIEW};control={'Authorization':'Bearer '+CONTROL}
                self.assertEqual(client.get(url).status_code,401)
                self.assertEqual(client.get(url,headers={**read,'Origin':'https://evil.test'}).status_code,403)
                self.assertEqual(client.get(url,headers=read).json()['config'],None)
                self.assertEqual(client.post(url,json={},headers=control).status_code,405)
                loaded=client.post('/v1/runs',json=payload(),headers=control).json()
                before=client.get('/v1/status',headers=read).json()
                active=client.get(url,headers=read).json()
                self.assertEqual(active['run_id'],loaded['run_id'])
                self.assertEqual(active['schema_version'],1)
                self.assertEqual(active['config'],json.loads(json.dumps(parse_config(payload()).to_dict())))
                self.assertEqual(client.get(url,headers=control).json(),active)
                self.assertEqual(client.get('/v1/status',headers=read).json(),before)
                self.assertNotIn(CONTROL,json.dumps(active));self.assertNotIn(VIEW,json.dumps(active))

class StartupConfigurationTests(unittest.TestCase):
    def test_private_initialization_and_literal_parsing(self):
        from scripts.skyview import initialize,configuration
        from pathlib import Path
        import stat
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'local.env';initialize(path)
            self.assertEqual(stat.S_IMODE(path.stat().st_mode),0o600)
            values=configuration(path)
            self.assertEqual(len({values[k] for k in ('DRONE_CONTROL_TOKEN','DRONE_VIEW_TOKEN','DASHBOARD_ACCESS_KEY')}),3)
            with self.assertRaises(FileExistsError):initialize(path)
            path.write_text(path.read_text()+'UNKNOWN=value\n')
            with self.assertRaises(ValueError):configuration(path)

    def test_duplicate_ports_and_placeholder_credentials_rejected(self):
        from scripts.skyview import initialize,configuration
        from pathlib import Path
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'local.env';initialize(path)
            original=path.read_text();path.write_text(original.replace('DASHBOARD_PORT=3000','DASHBOARD_PORT=8000'))
            with self.assertRaises(ValueError):configuration(path)
            path.write_text(Path('config/skyview.env.example').read_text())
            with self.assertRaises(ValueError):configuration(path)
