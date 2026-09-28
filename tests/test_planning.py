from dataclasses import replace
import importlib.util
import json
from math import hypot
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from drone_sim.planning import (Polygon, PlanningError, Mission, RouteWaypoint, lawnmower, return_home,
                                validate, estimates, sample_coverage, coverage_segments)
from drone_sim.planning.geometry import route_inside, distance
from drone_sim.planning.routes import lane_segments
from drone_sim.planning.execution import capabilities, to_scenario, execute
from drone_sim.vehicle import VehicleConfig, VehicleState, BatteryConfig
from drone_sim.vehicle.scenario import simulate

ROOT=Path(__file__).parents[1]
RECT=Polygon(((0,0),(12,0),(12,8),(0,8)))
BOUNDARY=Polygon(((-5,-5),(20,-5),(20,15),(-5,15)))
L_SHAPE=Polygon(((0,0),(12,0),(12,4),(4,4),(4,12),(0,12)))


def survey(**changes):
    values={'area':RECT,'home':(-2,-2,2),'flight_boundary':BOUNDARY,'lane_spacing_m':4.,'swath_width_m':4.}
    values.update(changes)
    return lawnmower(**values)


def vehicle_for(plan,**kwargs):
    return VehicleConfig(initial_state=VehicleState(position_enu_m=plan.start_enu_m),**kwargs)


class PolygonTests(unittest.TestCase):
    def test_canonical_ring_orientation_and_closing_vertex(self):
        p=Polygon(((12,8),(12,0),(0,0),(0,8),(12,8)))
        self.assertEqual(p,RECT)
        self.assertEqual(p.area_m2,96.)
        self.assertEqual(Polygon(((0,0),(6,0),(12,0),(12,8),(0,8))),RECT)

    def test_invalid_geometry(self):
        cases=[((0,0),(1,1)),((0,0),(2,2),(0,2),(2,0)),((0,0),(1,0),(2,0)),
               ((0,0),(2,0),(2,2),(0,0),(0,2)),((0,0),(2,0),(1,0),(1,2),(0,2)),
               ((0,0),(2,0),(2,float('nan'))),((0,0),(2,0),(2,float('inf'))),
               ((0,0),(1e7,0),(0,2)),((0,0),(True,0),(0,1)),
               ((0,0),(2,0),(2,2),(1,0),(0,2))]
        for case in cases:
            with self.subTest(case=case),self.assertRaises(PlanningError): Polygon(case)

    def test_boundary_and_notch_membership(self):
        self.assertTrue(L_SHAPE.contains((4,8)))
        self.assertTrue(L_SHAPE.contains((2,10)))
        self.assertFalse(L_SHAPE.contains((8,8)))
        self.assertFalse(L_SHAPE.contains_segment((2,10),(10,2)))
        self.assertTrue(L_SHAPE.contains_segment((2,10),(4,4)))

    def test_segment_test_catches_excursion_with_inside_midpoint(self):
        p=Polygon(((0,0),(12,0),(12,10),(8,10),(8,3),(7,3),(7,10),(3,10),(3,3),(2,3),(2,10),(0,10)))
        self.assertTrue(p.contains((6,6)))
        self.assertFalse(p.contains_segment((1,6),(11,6)))

    def test_concave_visibility_path(self):
        start,end=(2,10),(10,2)
        path=route_inside(L_SHAPE,start,end)
        self.assertGreater(len(path),1)
        previous=start
        for p in path:
            self.assertTrue(L_SHAPE.contains_segment(previous,p));previous=p
        self.assertEqual(path[-1],end)
        self.assertEqual(path,route_inside(L_SHAPE,start,end))

    def test_visibility_rejects_outside_start(self):
        with self.assertRaises(PlanningError): route_inside(RECT,(-1,0),(1,1))


class CoverageTests(unittest.TestCase):
    def test_rectangle_analytic_lane_count_length_and_spacing(self):
        segments=lane_segments(RECT,4,90)
        self.assertEqual(len(segments),2)
        self.assertEqual([round(distance(a,b),8) for a,b in segments],[12.,12.])
        self.assertEqual(sorted(a[1] for a,b in segments),[2.,6.])
        self.assertGreater(segments[0][1][0],segments[0][0][0])
        self.assertLess(segments[1][1][0],segments[1][0][0])

    def test_nondivisible_extent_and_spacing_larger_than_area(self):
        for spacing in (3.,100.):
            plan=survey(lane_spacing_m=spacing,swath_width_m=spacing)
            result=sample_coverage(RECT,coverage_segments(plan),spacing,.25)
            self.assertEqual(result['covered_fraction'],1.)

    def test_slanted_and_concave_full_ideal_coverage(self):
        triangle=Polygon(((0,0),(12,0),(3,9)))
        for shape in (triangle,L_SHAPE):
            for angle in (0,35,90,135):
                p=survey(area=shape,orientation_deg=angle,lane_spacing_m=2,swath_width_m=2)
                self.assertTrue(validate(p,capabilities(VehicleConfig()))['valid'])
                report=sample_coverage(shape,coverage_segments(p),2,.25)
                self.assertEqual(report['covered_fraction'],1.,(shape,angle,report))

    def test_multiple_intervals_on_a_concave_scanline(self):
        area=Polygon(((0,0),(12,0),(12,12),(8,12),(8,4),(4,4),(4,12),(0,12)))
        self.assertEqual(len(lane_segments(area,4,90)),5)
        p=lawnmower(area,(1,1,2),lane_spacing_m=4,swath_width_m=4)
        self.assertTrue(validate(p,capabilities(VehicleConfig()))['valid'])
        self.assertEqual(sample_coverage(area,coverage_segments(p),4,.25)['covered_fraction'],1.)

    def test_half_open_scanline_through_vertex(self):
        p=Polygon(((0,0),(8,0),(8,3),(4,3),(4,9),(0,9)))
        segments=lane_segments(p,6,90)
        self.assertTrue(segments)
        for a,b in segments: self.assertTrue(p.contains_segment(a,b))

    def test_lane_spacing_gap_is_rejected(self):
        with self.assertRaises(PlanningError): survey(lane_spacing_m=5,swath_width_m=4)
        for value in (0,-1,float('nan')):
            with self.assertRaises(PlanningError): survey(lane_spacing_m=value)

    def test_removed_leg_rejected_even_when_waypoints_inside(self):
        p=survey()
        index=next(i for i,w in enumerate(p.waypoints) if w.leg_kind=='survey_lane')
        changed=replace(p,waypoints=p.waypoints[:index]+p.waypoints[index+1:])
        self.assertIn('incomplete_survey_route',validate(changed,capabilities(VehicleConfig()))['reasons'])

    def test_sampling_does_not_claim_coverage_without_route(self):
        report=sample_coverage(RECT,(),4)
        self.assertEqual(report['covered_fraction'],0.)
        self.assertIsNone(report['maximum_distance_to_route_m'])

    def test_bounded_work_for_tiny_spacing(self):
        with self.assertRaises(PlanningError): survey(lane_spacing_m=1e-6)
        with self.assertRaises(PlanningError): sample_coverage(RECT,(),4,1e-6)


class MissionTests(unittest.TestCase):
    def test_round_trip_and_repeatability(self):
        p=survey(metadata={'purpose':'survey','tags':['civilian']})
        self.assertEqual(p,Mission.from_dict(json.loads(json.dumps(p.to_dict()))))
        self.assertEqual(p,survey(metadata={'purpose':'survey','tags':['civilian']}))
        self.assertEqual(p,survey(area=Polygon(tuple(reversed(RECT.vertices))),metadata=p.metadata))

    def test_unknown_schema_coordinates_and_metadata_rejected(self):
        p=survey().to_dict()
        for changes in ({'schema_version':2},{'coordinate_frame':'GPS'},{'metadata':{'x':float('nan')}},
                        {'speed_limit_mps':-1},{'waypoints':[]}):
            with self.subTest(changes=changes),self.assertRaises(PlanningError): Mission.from_dict({**p,**changes})

    def test_vehicle_speed_acceleration_and_altitude_rejections(self):
        cap=capabilities(VehicleConfig())
        for kwargs,reason in (({'speed_limit_mps':6},'speed_exceeds_vehicle'),
                              ({'acceleration_limit_mps2':4},'acceleration_exceeds_vehicle'),
                              ({'altitude_m':40},'altitude_outside_vehicle_geofence')):
            report=validate(survey(**kwargs),cap)
            self.assertFalse(report['valid']);self.assertIn(reason,report['reasons'])

    def test_home_and_route_geofence_rejections(self):
        p=survey()
        changed=replace(p,home_enu_m=(60,0,2))
        self.assertIn('point_outside_vehicle_geofence',validate(changed,capabilities(VehicleConfig()))['reasons'])
        changed=replace(p,flight_boundary=Polygon(((0,0),(1,0),(1,1),(0,1))))
        self.assertIn('leg_outside_flight_boundary',validate(changed,capabilities(VehicleConfig()))['reasons'])

    def test_survey_not_contained_rejected(self):
        with self.assertRaises(PlanningError): survey(flight_boundary=Polygon(((0,0),(1,0),(1,1),(0,1))))

    def test_distance_and_duration_known_single_leg(self):
        p=Mission('straight',(0,0,2),(0,0,2),2,2,1,BOUNDARY,(RouteWaypoint((10,0,2)),),settle_s=.5,settling_allowance_s=0)
        result=estimates(p)
        self.assertEqual(result['route_distance_m'],10.)
        self.assertEqual(result['cruise_only_time_s'],5.)
        self.assertEqual(result['approximate_duration_s'],7.5)

    def test_stationary_start_and_no_silent_relocation(self):
        p=survey()
        with self.assertRaises(PlanningError): to_scenario(p,VehicleConfig())
        config=vehicle_for(p)
        with self.assertRaises(PlanningError): to_scenario(p,replace(config,initial_state=replace(config.initial_state,velocity_enu_mps=(1,0,0))))
        with self.assertRaises(PlanningError): to_scenario(p,replace(config,initial_state=replace(config.initial_state,battery_fraction=.01)))


class ReturnHomeTests(unittest.TestCase):
    def test_climb_transit_descend_and_concave_routing(self):
        p=return_home((2,10,3),(10,2,2),L_SHAPE,6)
        self.assertEqual(p.waypoints[0].position_enu_m,(2,10,6))
        self.assertEqual(p.waypoints[-1].position_enu_m,(10,2,2))
        self.assertTrue(validate(p,capabilities(VehicleConfig()))['valid'])
        self.assertGreater(len(p.waypoints),3)

    def test_invalid_return_cases(self):
        for current,home,altitude in (((1,1,3),(2,2,2),2),((1,1,2),(50,50,2),5),
                                     ((50,50,2),(1,1,2),5),((1,1,2),(1,1,2),5)):
            with self.subTest(current=current,home=home),self.assertRaises(PlanningError):
                return_home(current,home,BOUNDARY,altitude)

    def test_return_home_executes_in_existing_engine(self):
        p=return_home((10,5,3),(0,0,2),BOUNDARY,6)
        scenario,rows,report=execute(p,vehicle_for(p),10000)
        self.assertEqual(report['final_mission_state'],'completed')
        self.assertLess(distance(rows[-1]['vehicle']['position_enu_m'],p.home_enu_m),.1)
        self.assertEqual(rows,list(simulate(scenario)))


class ExecutionTests(unittest.TestCase):
    def test_survey_completion_estimate_comparison_and_limits(self):
        p=survey()
        scenario,rows,report=execute(p,vehicle_for(p),15000)
        self.assertEqual(report['final_mission_state'],'completed')
        self.assertEqual(report['completed_waypoints'],len(p.waypoints))
        self.assertIsNotNone(report['duration_error_s'])
        self.assertAlmostEqual(report['duration_error_s'],report['completion_time_s']-estimates(p)['approximate_duration_s'])
        self.assertLessEqual(report['maximum_speed_mps'],p.speed_limit_mps+1e-12)
        self.assertEqual(rows,list(simulate(scenario)))

    def test_predicted_polygon_violation_reuses_stop_without_leaving(self):
        # A valid nominal corner route; a loose arrival tolerance permits dynamic corner cutting.
        p=Mission('corner',(2,10,2),(2,10,2),2,3,2,L_SHAPE,
                  (RouteWaypoint((4,4,2)),RouteWaypoint((10,2,2))),position_tolerance_m=1,speed_tolerance_mps=3,settle_s=0)
        self.assertTrue(validate(p,capabilities(VehicleConfig()))['valid'])
        scenario,rows,report=execute(p,vehicle_for(p),10000)
        self.assertEqual(report['execution_failure'],'predicted_flight_boundary_crossing')
        self.assertEqual(report['final_mission_state'],'aborted')
        self.assertEqual(report['stop_reason'],'emergency_stop')
        self.assertTrue(all(L_SHAPE.contains(r['vehicle']['position_enu_m'][:2]) for r in rows))
        self.assertEqual(rows,list(simulate(scenario)))

    def test_return_home_can_fail_from_battery_during_execution(self):
        p=return_home((10,5,3),(0,0,2),BOUNDARY,6)
        cfg=vehicle_for(p,battery=BatteryConfig(idle_drain_per_s=.9))
        _,rows,report=execute(p,cfg,1000)
        self.assertEqual(report['execution_failure'],'critical_battery')
        self.assertEqual(report['final_mission_state'],'aborted')
        self.assertIsNone(report['completion_time_s'])

    def test_survey_altitude_tampering_rejected(self):
        p=survey()
        waypoints=list(p.waypoints)
        index=next(i for i,w in enumerate(waypoints) if w.leg_kind=='survey_lane')
        w=waypoints[index]
        waypoints[index]=replace(w,position_enu_m=(*w.position_enu_m[:2],w.position_enu_m[2]+1))
        result=validate(replace(p,waypoints=tuple(waypoints)),capabilities(VehicleConfig()))
        self.assertIn('survey_leg_wrong_altitude',result['reasons'])

    def test_timeout_not_reported_as_completion(self):
        p=survey()
        _,rows,report=execute(p,vehicle_for(p),2)
        self.assertEqual(report['execution_failure'],'step_budget_exhausted')
        self.assertIsNone(report['completion_time_s'])
        self.assertIsNone(report['duration_error_s'])

    def test_planner_cli_replay_and_import(self):
        with tempfile.TemporaryDirectory() as folder:
            a,b=Path(folder)/'a',Path(folder)/'b'
            command=[sys.executable,'-m','drone_sim.planning']
            subprocess.run(command+['--request',str(ROOT/'scenarios/stage7/rectangle.json'),'--output-dir',str(a)],check=True,capture_output=True)
            subprocess.run(command+['--mission',str(a/'mission.json'),'--output-dir',str(b)],check=True,capture_output=True)
            self.assertEqual((a/'mission.json').read_bytes(),(b/'mission.json').read_bytes())
            self.assertEqual((a/'report.json').read_bytes(),(b/'report.json').read_bytes())

    def test_cli_rejection_has_machine_readable_report(self):
        with tempfile.TemporaryDirectory() as folder:
            request=Path(folder)/'request.json'
            data=json.loads((ROOT/'scenarios/stage7/rectangle.json').read_text())
            data['speed_limit_mps']=50
            request.write_text(json.dumps(data))
            output=Path(folder)/'result'
            result=subprocess.run([sys.executable,'-m','drone_sim.planning','--request',str(request),'--simulate',
                                   '--output-dir',str(output)],capture_output=True)
            self.assertEqual(result.returncode,2)
            report=json.loads((output/'report.json').read_text())
            self.assertIn('speed_exceeds_vehicle',report['validation']['reasons'])
            self.assertIsNone(report['execution'])

    @unittest.skipUnless(importlib.util.find_spec('matplotlib'), 'optional plotting dependency unavailable')
    def test_route_plot(self):
        from drone_sim.planning.diagnostics import plot_route
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'route.png';plot_route(survey(),path)
            self.assertTrue(path.read_bytes().startswith(b'\x89PNG'))
