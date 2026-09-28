import csv
from dataclasses import replace
import importlib.util
import json
from math import pi
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from drone_sim.clock import FixedStepClock
from drone_sim.core import default_scenario, simulate as simulate_cameras
from drone_sim.vehicle import (VehicleSimulator, VehicleConfig, VehicleState, MovementLimits,
                               PDConfig, BatteryConfig, Geofence, Waypoint, integrate, pd_acceleration)
from drone_sim.vehicle.model import norm, wrap
from drone_sim.vehicle.scenario import Scenario, Action, simulate, summarize
from drone_sim.vehicle.export import export

ROOT = Path(__file__).parents[1]


class ClockTests(unittest.TestCase):
    def test_shared_camera_vehicle_timestamps(self):
        for dt in (.01,.02,.05,.1):
            cameras=list(simulate_cameras(default_scenario(dt_s=dt,steps=50)))
            vehicle=list(simulate(Scenario(vehicle=VehicleConfig(dt_s=dt),steps=50)))
            self.assertEqual([f.simulation_time_s for f in cameras],[r['simulation_time_s'] for r in vehicle])

    def test_integer_tick_no_accumulated_time(self):
        clock=FixedStepClock(.02)
        for _ in range(10000): clock=clock.advance()
        self.assertEqual(clock.time_s,200.)
        self.assertEqual(FixedStepClock(.02).tick,0)

    def test_invalid_clocks(self):
        for kwargs in ({'dt_s':0},{'dt_s':float('nan')},{'tick':-1},{'tick':.5}):
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError): FixedStepClock(**kwargs)

    def test_external_clock_order_rejected_without_mutation(self):
        s=VehicleSimulator()
        before=s.telemetry()
        for clock in (FixedStepClock(.02,0),FixedStepClock(.02,2),FixedStepClock(.01,1)):
            with self.assertRaises(ValueError): s.step(clock)
            self.assertEqual(s.telemetry(),before)
        self.assertEqual(s.step(FixedStepClock(.02,1))['tick'],1)


class DynamicsTests(unittest.TestCase):
    def test_semi_implicit_analytic_update(self):
        config=VehicleConfig(dt_s=.1,drag_per_s=0)
        state=integrate(VehicleState(),(1,0,0),config)
        self.assertAlmostEqual(state.velocity_enu_mps[0],.1)
        self.assertAlmostEqual(state.position_enu_m[0],.01)
        self.assertEqual(state.acceleration_enu_mps2,(1.,0.,0.))

    def test_drag_matches_discrete_solution(self):
        config=VehicleConfig(drag_per_s=.5)
        state=VehicleState(velocity_enu_mps=(2,0,0))
        for _ in range(100): state=integrate(state,(0,0,0),config)
        self.assertAlmostEqual(state.velocity_enu_mps[0],2*(1-.5*.02)**100,places=12)

    def test_diagonal_norm_limits_and_realized_acceleration(self):
        config=VehicleConfig(limits=MovementLimits(2,1,1))
        state=VehicleState()
        maximum=0
        for _ in range(1000):
            old=state
            state=integrate(state,(100,100,100),config)
            maximum=max(maximum,norm(state.velocity_enu_mps))
            self.assertLessEqual(norm(state.velocity_enu_mps),2+1e-12)
            self.assertLessEqual(norm(state.acceleration_enu_mps2),1+1e-12)
            self.assertEqual(state.acceleration_enu_mps2,tuple((v-u)/.02 for u,v in zip(old.velocity_enu_mps,state.velocity_enu_mps)))
        self.assertAlmostEqual(maximum,2)

    def test_integrator_rejects_initial_overspeed(self):
        with self.assertRaises(ValueError):
            integrate(VehicleState(velocity_enu_mps=(6,0,0)),(0,0,0),VehicleConfig())

    def test_pd_command_limits_and_derivative_sign(self):
        limits=MovementLimits()
        state=VehicleState(velocity_enu_mps=(1,0,0))
        acceleration=pd_acceleration(state,state.position_enu_m,PDConfig(),limits)
        self.assertLess(acceleration[0],0)
        self.assertLessEqual(norm(pd_acceleration(state,(100,100,100),PDConfig(),limits)),3+1e-12)

    def test_heading_rate_wrap_and_stationary_heading(self):
        config=VehicleConfig(drag_per_s=0,limits=MovementLimits(5,3,.4))
        state=VehicleState(velocity_enu_mps=(0,-1,0),heading_rad=pi-.001)
        new=integrate(state,(0,0,0),config)
        self.assertLessEqual(abs(wrap(new.heading_rad-state.heading_rad)),.4*.02+1e-12)
        self.assertAlmostEqual(wrap(new.heading_rad-pi),0)
        state=VehicleState(heading_rad=.7)
        self.assertAlmostEqual(integrate(state,(0,0,0),config).heading_rad,.7)

    def test_battery_drain_analytic_and_clamp(self):
        config=VehicleConfig(dt_s=.1,drag_per_s=0,battery=BatteryConfig(.01,.02,.03))
        state=integrate(VehicleState(velocity_enu_mps=(1,0,0)),(1,0,0),config)
        self.assertAlmostEqual(state.battery_fraction,1-.1*(.01+.02*1.1+.03))
        empty=integrate(VehicleState(battery_fraction=.000001),(0,0,0),config)
        self.assertEqual(empty.battery_fraction,0)

    def test_invalid_physics_configurations(self):
        constructors=[lambda:VehicleConfig(dt_s=.2),lambda:VehicleConfig(drag_per_s=-1),
            lambda:MovementLimits(max_speed_mps=0),lambda:PDConfig(kp=float('nan')),
            lambda:VehicleConfig(dt_s=.1,controller=PDConfig(1,30)),
            lambda:BatteryConfig(critical_fraction=.9,low_fraction=.5),
            lambda:VehicleState(battery_fraction=1.1),lambda:VehicleState(position_enu_m=(1,2,float('inf'))),
            lambda:Geofence((1,0,0),(0,1,1)),
            lambda:VehicleConfig(initial_state=VehicleState(position_enu_m=(51,0,2))),
            lambda:VehicleConfig(initial_state=VehicleState(velocity_enu_mps=(6,0,0)))]
        for i,constructor in enumerate(constructors):
            with self.subTest(case=i),self.assertRaises(ValueError): constructor()


class MissionTests(unittest.TestCase):
    def test_three_axis_convergence_multiple_stepsizes(self):
        for dt in (.01,.02,.05,.1):
            s=VehicleSimulator([Waypoint((8,-6,7))],VehicleConfig(dt_s=dt))
            s.start()
            for _ in range(round(30/dt)): s.step()
            self.assertEqual(s.status,'completed')
            self.assertLess(norm(tuple(p-t for p,t in zip(s.state.position_enu_m,(8,-6,7)))),1e-5)
            self.assertLess(norm(s.state.velocity_enu_mps),1e-5)

    def test_stationary_target_requires_settle_duration(self):
        s=VehicleSimulator([Waypoint((0,0,2),settle_s=.1)])
        s.start()
        for _ in range(4): s.step()
        self.assertEqual(s.status,'running')
        s.step()
        self.assertEqual(s.status,'completed')
        self.assertEqual(s.state.position_enu_m,(0,0,2))

    def test_position_alone_not_waypoint_completion(self):
        config=VehicleConfig(initial_state=VehicleState(velocity_enu_mps=(1,0,0)))
        s=VehicleSimulator([Waypoint((0,0,2),position_tolerance_m=1,speed_tolerance_mps=.01,settle_s=0)],config)
        s.start(); s.step()
        self.assertEqual(s.completed_waypoints,0)

    def test_waypoint_order_and_no_teleport(self):
        rows=list(simulate(Scenario()))
        events=[e for r in rows for e in r['events'] if e['kind']=='waypoint_reached']
        self.assertEqual([e['waypoint_index'] for e in events],[0,1,2])
        self.assertEqual(rows[-1]['mission_state'],'completed')
        for old,new in zip(rows,rows[1:]):
            distance=norm(tuple(p-q for p,q in zip(new['vehicle']['position_enu_m'],old['vehicle']['position_enu_m'])))
            self.assertLessEqual(distance,5*.02+1e-12)

    def test_pause_keeps_clock_battery_running_and_resumes(self):
        s=VehicleSimulator([Waypoint((20,0,5))]); s.start()
        for _ in range(100): s.step()
        s.pause()
        hold=s.hold_position
        battery=s.state.battery_fraction
        for _ in range(800): s.step()
        self.assertEqual(s.status,'paused')
        self.assertEqual(s.completed_waypoints,0)
        self.assertLess(norm(tuple(p-q for p,q in zip(s.state.position_enu_m,hold))),.001)
        self.assertLess(s.state.battery_fraction,battery)
        self.assertEqual(s.clock.tick,900)
        s.resume()
        for _ in range(1500): s.step()
        self.assertEqual(s.status,'completed')

    def test_pause_resets_settling(self):
        s=VehicleSimulator([Waypoint((0,0,2),settle_s=.1)]); s.start()
        for _ in range(4): s.step()
        s.pause(); s.resume(); s.step()
        self.assertEqual(s.status,'running')
        self.assertEqual(s.settle_ticks,1)

    def test_invalid_lifecycle_operations(self):
        s=VehicleSimulator([Waypoint((1,0,2))])
        for command in (s.pause,s.resume):
            with self.assertRaises(ValueError): command()
        s.start()
        with self.assertRaises(ValueError): s.start()
        s.abort()
        for command in (s.start,s.pause,s.resume):
            with self.assertRaises(ValueError): command()
        with self.assertRaises(ValueError): VehicleSimulator().start()

    def test_emergency_stop_latched_and_explicit_discontinuity(self):
        s=VehicleSimulator([Waypoint((30,0,2))]); s.start()
        for _ in range(100): s.step()
        position=s.state.position_enu_m
        velocity=s.state.velocity_enu_mps
        s.emergency_stop()
        s.abort()
        row=s.step()
        event=next(e for e in row['events'] if e['kind']=='virtual_stop')
        self.assertEqual(event['velocity_reset_enu_mps'],tuple(-v for v in velocity))
        self.assertEqual(s.stop_reason,'emergency_stop')
        state=s.state
        for _ in range(100): s.step()
        self.assertEqual(s.state,state)
        self.assertEqual(s.state.position_enu_m,position)
        self.assertEqual(s.state.velocity_enu_mps,(0,0,0))
        self.assertEqual(s.status,'aborted')

    def test_geofence_rejects_attempted_crossing(self):
        config=VehicleConfig(initial_state=VehicleState((49.99,0,2),(5,0,0)))
        s=VehicleSimulator([Waypoint((0,0,2))],config); s.start(); row=s.step()
        self.assertEqual(s.stop_reason,'geofence_predicted_crossing')
        self.assertEqual(s.state.position_enu_m,(49.99,0,2))
        self.assertEqual(s.state.velocity_enu_mps,(0,0,0))
        self.assertTrue(row['emergency_stopped'])
        with self.assertRaises(ValueError): VehicleSimulator([Waypoint((100,0,2))])

    def test_low_then_critical_battery_stop(self):
        config=VehicleConfig(battery=BatteryConfig(idle_drain_per_s=.1),initial_state=VehicleState(battery_fraction=.21))
        s=VehicleSimulator([Waypoint((20,0,5))],config); s.start()
        events=[]
        for _ in range(100): events.extend(s.step()['events'])
        self.assertTrue(s.low_battery)
        self.assertEqual(s.status,'aborted')
        self.assertEqual(s.stop_reason,'critical_battery')
        self.assertEqual(sum(e['kind']=='battery_low' for e in events),1)
        self.assertEqual(sum(e['kind']=='virtual_stop' for e in events),1)
        self.assertGreaterEqual(s.state.battery_fraction,0)

    def test_initial_critical_battery_cannot_start(self):
        s=VehicleSimulator([Waypoint((1,0,2))],VehicleConfig(initial_state=VehicleState(battery_fraction=.1)))
        self.assertEqual(s.status,'aborted')
        with self.assertRaises(ValueError): s.start()

    def test_idle_holds_position_without_mission_progress(self):
        s=VehicleSimulator()
        initial=s.state.position_enu_m
        for _ in range(10): s.step()
        self.assertEqual(s.status,'idle')
        self.assertEqual(s.state.position_enu_m,initial)
        self.assertIsNone(s.telemetry()['waypoint_error_m'])

    def test_settle_and_fence_validation(self):
        for constructor in (lambda:Waypoint((0,0,2),settle_s=-1),lambda:Waypoint((0,0,2),speed_tolerance_mps=0)):
            with self.assertRaises(ValueError): constructor()
        self.assertTrue(Geofence().contains((-50,50,0)))


class ScenarioExportTests(unittest.TestCase):
    def test_all_committed_scenarios_reproducible_and_safe(self):
        expected={'waypoints':'completed','pause_resume':'completed','emergency_stop':'aborted',
                  'low_battery':'aborted','geofence':'aborted'}
        for path in (ROOT/'scenarios/stage6').glob('*.json'):
            scenario=Scenario.from_dict(json.loads(path.read_text()))
            rows=list(simulate(scenario))
            self.assertEqual(rows,list(simulate(scenario)))
            self.assertEqual(rows[-1]['mission_state'],expected[path.stem])
            for r in rows:
                self.assertTrue(scenario.vehicle.geofence.contains(r['vehicle']['position_enu_m']))
                self.assertLessEqual(norm(r['vehicle']['velocity_enu_mps']),scenario.vehicle.limits.max_speed_mps+1e-12)

    def test_configuration_round_trip(self):
        s=Scenario()
        self.assertEqual(s,Scenario.from_dict(json.loads(json.dumps(s.to_dict()))))
        for data in ({'schema_version':2},{'steps':-1},{'actions':[{'tick':3000,'command':'start'}]},
                     {'actions':[{'tick':0,'command':'physical_arm'}]}):
            with self.assertRaises(ValueError): Scenario.from_dict(data)

    def test_zero_step_idle(self):
        rows=list(simulate(Scenario(steps=0,actions=())))
        self.assertEqual(len(rows),1)
        self.assertIsNone(summarize(rows)['completion_time_s'])

    def test_csv_jsonl_consistency_and_cli_replay(self):
        with tempfile.TemporaryDirectory() as folder:
            a,b=Path(folder)/'a',Path(folder)/'b'
            args=[sys.executable,'-m','drone_sim.vehicle','--config',str(ROOT/'scenarios/stage6/emergency_stop.json')]
            subprocess.run(args+['--output-dir',str(a)],check=True,capture_output=True)
            subprocess.run(args+['--output-dir',str(b)],check=True,capture_output=True)
            for name in ('config.json','telemetry.jsonl','telemetry.csv','summary.json'):
                self.assertEqual((a/name).read_bytes(),(b/name).read_bytes())
            rows=[json.loads(x) for x in (a/'telemetry.jsonl').read_text().splitlines()]
            with open(a/'telemetry.csv') as stream: table=list(csv.DictReader(stream))
            self.assertEqual(len(rows),len(table))
            for row,line in zip(rows,table):
                self.assertEqual(float(line['simulation_time_s']),row['simulation_time_s'])
                self.assertEqual(float(line['east_m']),row['vehicle']['position_enu_m'][0])
                self.assertEqual(json.loads(line['events_json']),row['events'])
            with self.assertRaises(FileExistsError): export(Scenario(),[],a)

    @unittest.skipUnless(importlib.util.find_spec('matplotlib'), 'optional plotting dependency unavailable')
    def test_diagnostic_plot_export(self):
        from drone_sim.vehicle.export import plot
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'plot.png'
            plot(list(simulate(Scenario(steps=10))),path)
            self.assertTrue(path.read_bytes().startswith(b'\x89PNG'))
            self.assertGreater(path.stat().st_size,10000)
