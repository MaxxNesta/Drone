import json
import subprocess
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

from drone_sim.fleet import Member, FleetAction, FleetConfig, FleetSimulator, simulate, summarize
from drone_sim.fleet.diagnostics import segment_separation, proximity, route_conflicts
from drone_sim.fleet.export import export
from drone_sim.planning import Mission, RouteWaypoint, Polygon, lawnmower
from drone_sim.planning.execution import to_scenario
from drone_sim.vehicle import VehicleConfig, VehicleState
from drone_sim.vehicle.model import BatteryConfig
from drone_sim.vehicle.scenario import simulate as single_simulate, Scenario

BOUNDARY = Polygon(((-40,-40),(40,-40),(40,40),(-40,40)))


def member(key='a', start=(-5,0,2), end=(5,0,2), **config):
    mission = Mission(key, start, start, end[2], 3, 2, BOUNDARY, (RouteWaypoint(end),))
    return Member(key, mission, VehicleConfig(initial_state=VehicleState(start), **config))


def fixture(**options):
    return FleetConfig('test', (member(), member('b',(-5,10,2),(5,10,2))), **options)


def base_row(row):
    return {k:v for k,v in row.items() if k not in ('vehicle_id','fleet_id','fleet_failure')}


class FleetConfigurationTests(unittest.TestCase):
    def test_unique_and_safe_ids(self):
        for key in ('../a','', 'a/b', 'x'*65):
            with self.assertRaises(ValueError): member(key)
        with self.assertRaises(ValueError): FleetConfig('x',(member(),member()))
        with self.assertRaises(ValueError): FleetConfig('x',())
        with self.assertRaises(ValueError): FleetConfig('x',(member('A'),member('a')))

    def test_mixed_clocks_rejected(self):
        with self.assertRaises(ValueError): FleetConfig('x',(member(),member('b',dt_s=.05)))

    def test_bad_schedule_and_thresholds(self):
        for values in ({'steps':0},{'steps':True},{'schema_version':2}, {'proximity_m':float('nan')},
                       {'route_conflict_m':0}, {'actions':(FleetAction(0,'start','unknown'),)},
                       {'steps':2,'actions':(FleetAction(2,'start'),)}):
            with self.assertRaises(ValueError): fixture(**values)
        with self.assertRaises(ValueError): FleetAction(-1,'start')
        with self.assertRaises(ValueError): FleetAction(0,'land')

    def test_invalid_mission_or_launch_rejected(self):
        a=member()
        with self.assertRaises(ValueError): Member('x',a.mission,VehicleConfig())
        with self.assertRaises(ValueError): Member('x',replace(a.mission,speed_limit_mps=100),a.vehicle)
        with self.assertRaises(ValueError): Member('x',a.mission,replace(a.vehicle,initial_state=VehicleState((-5,0,2),battery_fraction=.05)))

    def test_round_trip_and_order(self):
        config=fixture(steps=20)
        imported=FleetConfig.from_dict(json.loads(json.dumps(config.to_dict())))
        self.assertEqual(config,imported)
        self.assertEqual(list(simulate(config)),list(simulate(replace(config,members=tuple(reversed(config.members))))))

    def test_survey_and_home_are_preserved(self):
        area=Polygon(((0,0),(6,0),(6,4),(0,4)))
        plan=lawnmower(area,(-2,-2,2),flight_boundary=BOUNDARY,altitude_m=3)
        a=Member('survey',plan,VehicleConfig(initial_state=VehicleState(plan.start_enu_m)))
        imported=FleetConfig.from_dict(json.loads(json.dumps(FleetConfig('s',(a,)).to_dict())))
        self.assertEqual(imported.members[0].mission,plan)


class FleetLifecycleTests(unittest.TestCase):
    def test_authoritative_clock_all_states(self):
        fleet=FleetSimulator(fixture())
        fleet.command('start','a'); fleet.command('abort','b')
        for tick in range(1,30):
            row=fleet.step()
            self.assertEqual(row['tick'],tick)
            self.assertEqual({r['tick'] for r in row['vehicles'].values()},{tick})
            self.assertTrue(all(e.clock is fleet.clock for e in fleet._engines.values()))
            self.assertEqual(row['vehicles']['b']['vehicle']['position_enu_m'],(-5,10,2))

    def test_independent_execution_matches_stage6(self):
        config=fixture(steps=600)
        rows=list(simulate(config))
        for m in config.members:
            expected=list(single_simulate(to_scenario(m.mission,m.vehicle,config.steps)))
            self.assertEqual([base_row(r['vehicles'][m.vehicle_id]) for r in rows],expected)
        self.assertTrue(rows[-1]['summary']['all_completed'])

    def test_fleet_and_individual_lifecycle(self):
        fleet=FleetSimulator(fixture())
        self.assertEqual(fleet.telemetry()['summary']['status'],'idle')
        fleet.start(); fleet.step(); fleet.command('pause','a')
        self.assertEqual(fleet.telemetry()['summary']['state_counts'],{'paused':1,'running':1})
        self.assertEqual(fleet.pause(),('b',))
        before=fleet.telemetry()
        row=fleet.step()
        self.assertEqual(row['summary']['status'],'paused')
        self.assertLess(row['vehicles']['a']['vehicle']['battery_fraction'],before['vehicles']['a']['vehicle']['battery_fraction'])
        fleet.command('resume','a')
        self.assertEqual(fleet.resume(),('b',))
        fleet.command('emergency_stop','a'); fleet.command('emergency_stop','a')
        self.assertEqual(fleet.start(),())
        self.assertTrue(fleet.step()['summary']['degraded'])
        with self.assertRaises(ValueError): fleet.command('start','a')
        with self.assertRaises(ValueError): fleet.command('pause','missing')
        with self.assertRaises(ValueError): fleet.command('fly')

    def test_read_only_monitoring(self):
        fleet=FleetSimulator(fixture()); fleet.start()
        self.assertEqual(fleet.telemetry(),fleet.telemetry())
        snapshot=fleet.telemetry()
        snapshot['vehicles']['a']['events'][0]['kind']='mutated'
        self.assertEqual(fleet.telemetry()['vehicles']['a']['events'][0]['kind'],'mission_started')
        row=fleet.step()
        self.assertEqual(len(row['commands']),1)
        self.assertEqual(row['vehicles']['a']['events'][0]['kind'],'mission_started')
        self.assertEqual(fleet.step()['commands'],[])

    def test_one_battery_failure_other_completes(self):
        a=member(battery=BatteryConfig(idle_drain_per_s=1))
        config=FleetConfig('failure',(a,member('b',(-5,10,2),(5,10,2))),steps=600)
        rows=list(simulate(config)); report=summarize(config,rows)
        self.assertEqual(report['status'],'finished_with_failures')
        self.assertEqual(report['vehicles']['a']['stop_reason'],'critical_battery')
        self.assertEqual(report['vehicles']['b']['final_mission_state'],'completed')
        self.assertIsNone(report['first_all_completed_time_s'])
        self.assertEqual(report['completed_fraction'],.5)
        aborted=next(r['vehicles']['a'] for r in rows if r['vehicles']['a']['mission_state']=='aborted')
        self.assertEqual(aborted['vehicle'],rows[-1]['vehicles']['a']['vehicle'])

    def test_completion_does_not_stop_peers_or_clock(self):
        config=FleetConfig('lengths',(member(end=(-4,0,2)),member('b',(-5,10,2),(20,10,2))),steps=900)
        rows=list(simulate(config)); report=summarize(config,rows)
        self.assertLess(report['vehicles']['a']['completion_time_s'],report['vehicles']['b']['completion_time_s'])
        self.assertEqual(len(rows),901)
        self.assertEqual(report['first_all_completed_time_s'],report['vehicles']['b']['completion_time_s'])

    def test_completed_then_aborted_is_not_current_success(self):
        config=FleetConfig('late',(member(end=(-4,0,2)),),steps=600,
                           actions=(FleetAction(0,'start'),FleetAction(550,'abort','a')))
        report=summarize(config,list(simulate(config)))
        self.assertFalse(report['all_completed']); self.assertIsNotNone(report['first_all_completed_time_s'])

    def test_pause_resume_schedule_and_replay(self):
        config=fixture(steps=800,actions=(FleetAction(0,'start'),FleetAction(50,'pause'),
                                        FleetAction(100,'resume','a'),FleetAction(150,'resume')))
        rows=list(simulate(config))
        self.assertEqual(rows,list(simulate(config)))
        self.assertEqual(rows[51]['summary']['status'],'paused')
        self.assertTrue(rows[-1]['summary']['all_completed'])

    def test_polygon_failure_isolated(self):
        data=json.loads(Path('docs/diagnostics/stage7/concave_boundary-mission.json').read_text())
        mission=Mission.from_dict(data)
        a=Member('concave',mission,VehicleConfig(initial_state=VehicleState(mission.start_enu_m)))
        config=FleetConfig('boundary',(a,member('peer',(-10,-10,2),(-8,-10,2))),steps=2500)
        rows=list(simulate(config))
        self.assertEqual(rows[-1]['vehicles']['concave']['fleet_failure'],'predicted_flight_boundary_crossing')
        self.assertEqual(rows[-1]['vehicles']['peer']['mission_state'],'completed')
        for row in rows:
            self.assertTrue(mission.flight_boundary.contains(row['vehicles']['concave']['vehicle']['position_enu_m'][:2]))
        self._check_export_replay(config,rows)

    def _check_export_replay(self,config,rows):
        with tempfile.TemporaryDirectory() as directory:
            output=Path(directory)/'run'; export(config,rows,output)
            for member_ in config.members:
                path=output/'vehicles'/member_.vehicle_id
                scenario=Scenario.from_dict(json.loads((path/'config.json').read_text()))
                expected=json.loads(json.dumps(list(single_simulate(scenario))))
                actual=[base_row(json.loads(line)) for line in (path/'telemetry.jsonl').read_text().splitlines()]
                self.assertEqual(expected,actual)

    def test_command_exports_replay_stage6(self):
        config=fixture(steps=120,actions=(FleetAction(0,'start'),FleetAction(20,'pause'),
                                        FleetAction(40,'resume','a'),FleetAction(60,'emergency_stop','b')))
        self._check_export_replay(config,list(simulate(config)))


class FleetDiagnosticsTests(unittest.TestCase):
    def test_3d_segment_geometry(self):
        self.assertEqual(segment_separation((-1,0,0),(1,0,0),(0,-1,0),(0,1,0)),0)
        self.assertEqual(segment_separation((-1,0,0),(1,0,0),(0,-1,3),(0,1,3)),3)
        self.assertEqual(segment_separation((0,0,0),(0,0,0),(2,0,0),(3,0,0)),2)
        self.assertEqual(segment_separation((0,0,0),(3,0,0),(1,0,0),(2,0,0)),0)
        self.assertAlmostEqual(segment_separation((0,0,0),(1,0,0),(2,1,0),(2,2,0)),2**.5)

    def test_swept_crossing_not_missed(self):
        before={'a':(-1,0,2),'b':(1,0,2)}; after={'a':(1,0,2),'b':(-1,0,2)}
        alerts=proximity(before,after,.1)
        self.assertEqual(alerts[0]['minimum_separation_m'],0)
        self.assertEqual(alerts[0]['closest_tick_fraction'],.5)
        self.assertEqual(alerts[0]['end_separation_m'],2)

    def test_shared_time_not_independent_path_distance(self):
        # Same spatial line, vehicles moving together at fixed separation.
        self.assertEqual(proximity({'a':(0,0,0),'b':(2,0,0)}, {'a':(3,0,0),'b':(5,0,0)},1),[])

    def test_initial_overlap_and_vertical_separation(self):
        points={'a':(0,0,2),'b':(0,0,2)}
        self.assertEqual(proximity(points,points,.1)[0]['minimum_separation_m'],0)
        self.assertEqual(proximity(points,{'a':(0,0,2),'b':(0,0,2)},.1)[0]['closest_tick_fraction'],0)
        points['b']=(0,0,5)
        self.assertEqual(proximity(points,points,2),[])

    def test_route_conflict_independent_of_schedule(self):
        config=FleetConfig('cross',(member(),member('b',(0,-5,2),(0,5,2))),steps=1200,
                           actions=(FleetAction(0,'start','a'),FleetAction(600,'start','b')))
        self.assertEqual(len(route_conflicts(config)),1)
        report=summarize(config,list(simulate(config)))
        self.assertEqual(report['proximity_episodes'],0)
        self.assertTrue(report['all_completed'])

    def test_alerts_do_not_change_dynamics(self):
        config=FleetConfig('cross',(member(),member('b',(0,-5,2),(0,5,2))),steps=600)
        rows=list(simulate(config)); quiet=list(simulate(replace(config,proximity_m=.001,route_conflict_m=.001)))
        self.assertEqual([r['vehicles'] for r in rows],[r['vehicles'] for r in quiet])
        report=summarize(config,rows)
        self.assertEqual(report['proximity_episodes'],1)
        self.assertLess(report['minimum_alert_separation_m'],1e-8)
        self.assertTrue(report['all_completed'])

    def test_stationary_aborted_vehicle_is_visible(self):
        config=FleetConfig('stopped',(member(),member('b',(-5,0,2),(-5,10,2))),steps=4,
                           actions=(FleetAction(0,'abort','a'),FleetAction(0,'start','b')))
        rows=list(simulate(config))
        self.assertTrue(all(r['proximity'] for r in rows))

    def test_export_byte_reproducibility_and_cli(self):
        config=fixture(steps=10)
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            for name in ('a','b'): export(config,list(simulate(config)),root/name)
            for path in (root/'a').rglob('*'):
                if path.is_file(): self.assertEqual(path.read_bytes(),(root/'b'/path.relative_to(root/'a')).read_bytes())
            with self.assertRaises(FileExistsError): export(config,list(simulate(config)),root/'a')
            result=subprocess.run([sys.executable,'-m','drone_sim.fleet','--config',str(root/'a'/'config.json'),
                                   '--output-dir',str(root/'cli')],capture_output=True,text=True)
            self.assertEqual(result.returncode,2,result.stderr)  # unfinished is explicit
            self.assertEqual((root/'a'/'fleet.jsonl').read_bytes(),(root/'cli'/'fleet.jsonl').read_bytes())


class FleetFixtureTests(unittest.TestCase):
    def test_committed_scenarios_and_expected_outcomes(self):
        from scenarios.stage8.generate import scenarios
        for config in scenarios():
            with self.subTest(config=config.fleet_id):
                stored=FleetConfig.from_dict(json.loads(Path('scenarios/stage8',config.fleet_id+'.json').read_text()))
                self.assertEqual(stored,config)
                rows=list(simulate(config)); report=summarize(config,rows)
                failed='failure' in config.fleet_id
                self.assertEqual(report['all_completed'],not failed)
                self.assertEqual(report['degraded'],failed)
                self.assertEqual(report['proximity_episodes'],1 if config.fleet_id=='crossing' else 0)
                self.assertEqual(len(report['route_advisories']),1 if 'crossing' in config.fleet_id else 0)
                self.assertEqual({v['tick'] for r in rows for v in r['vehicles'].values()},set(range(config.steps+1)))

    def test_incomplete_export_rejected(self):
        config=fixture(steps=3)
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError): export(config,list(simulate(config))[:-1],Path(directory)/'partial')

    def test_optional_diagnostic_plot(self):
        import importlib.util
        if importlib.util.find_spec('matplotlib') is None:
            self.skipTest('Optional Matplotlib unavailable')
        from drone_sim.fleet.export import plot
        config=fixture(steps=2)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'plot.png'
            plot(config,list(simulate(config)),path)
            self.assertEqual(path.read_bytes()[:8],b'\x89PNG\r\n\x1a\n')


if __name__=='__main__': unittest.main()
