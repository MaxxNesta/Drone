from dataclasses import replace
import math
import unittest
from drone_sim import Camera, Target, default_scenario, simulate
from drone_sim.triangulation import TriangulationConfig, camera_ray, triangulate


class TriangulationTests(unittest.TestCase):
    def setUp(self):
        self.scenario=default_scenario(steps=0)
        self.frame=next(simulate(self.scenario))

    def result(self, observations=None, cameras=None, now=0., config=TriangulationConfig()):
        return triangulate(cameras or self.scenario.cameras,
                           self.frame.detections if observations is None else observations,now,config)

    def test_exact_recovery_and_reordered_observations(self):
        for observations in [self.frame.detections,tuple(reversed(self.frame.detections))]:
            result=self.result(observations)
            self.assertTrue(result.valid,result.reason)
            self.assertLess(math.dist(result.position_enu_m,self.frame.truth.position_enu_m),1e-10)
            self.assertLess(result.ray_separation_m,1e-10)
            self.assertLess(result.condition_number,100)

    def test_rotated_tilted_cameras(self):
        point=(3.,12.,8.)
        cameras=(Camera.look_at('a',(-4.,1.,2.),point),Camera.look_at('b',(8.,2.,3.),point))
        scenario=replace(self.scenario,cameras=cameras,target=Target(point,(0.,0.,0.)))
        frame=next(simulate(scenario))
        result=triangulate(cameras,frame.detections,0.)
        self.assertTrue(result.valid)
        self.assertLess(math.dist(result.position_enu_m,point),1e-10)
        ray=camera_ray(cameras[0],frame.detections[0].pixel)
        self.assertAlmostEqual(sum(x*x for x in ray),1.)

    def test_invalid_stale_future_and_mismatched_observations(self):
        a,b=self.frame.detections
        for observations,now,reason in [((replace(a,valid=False),b),0,'invalid_observation'),
                ((a,b),1,'stale_observation'),((replace(a,capture_time_s=1),b),0,'future_observation'),
                ((replace(a,target_id='other'),b),0,'target_mismatch'),
                ((replace(a,capture_time_s=.02),b),.02,'timestamp_mismatch'),
                ((replace(a,pixel=(math.nan,0)),b),0,'invalid_pixel'),
                ((a,a),0,'duplicate_camera'),((replace(a,camera_id='unknown'),b),0,'unknown_camera'),
                ((replace(a,image_size=(1,1)),b),0,'wrong_image_size'),
                ((a,),0,'requires_two_observations')]:
            with self.subTest(reason=reason):
                result=self.result(observations,now=now)
                self.assertFalse(result.valid)
                self.assertEqual(result.reason,reason)
                self.assertIsNone(result.position_enu_m)

    def test_compatible_timestamp_skew(self):
        a,b=self.frame.detections
        result=self.result((replace(a,capture_time_s=.005),b),now=.005)
        self.assertTrue(result.valid)
        self.assertEqual(result.timestamp_s,.0025)

    def test_parallel_and_nearly_parallel(self):
        a,b=self.frame.detections
        for offset in [0.,1e-5]:
            result=self.result((replace(a,pixel=(320.,240.)),replace(b,pixel=(320.+offset,240.))))
            self.assertFalse(result.valid)
            self.assertEqual(result.reason,'ill_conditioned')

    def test_antiparallel_rays(self):
        a,b=self.scenario.cameras
        b=Camera.look_at(b.camera_id,b.position_enu_m,(5.,-1.,2.))
        obs=tuple(replace(o,pixel=(320.,240.)) for o in self.frame.detections)
        self.assertEqual(self.result(obs,(a,b)).reason,'ill_conditioned')

    def test_same_origin_rejected(self):
        a,b=self.scenario.cameras
        result=self.result(cameras=(a,replace(b,position_enu_m=a.position_enu_m)))
        self.assertEqual(result.reason,'insufficient_baseline')

    def test_backward_intersection_rejected(self):
        a,b=self.frame.detections
        result=self.result((replace(a,pixel=(220.,240.)),replace(b,pixel=(420.,240.))))
        self.assertEqual(result.reason,'behind_camera')

    def test_skew_ray_residual(self):
        a,b=self.frame.detections
        result=self.result((a,replace(b,pixel=(b.pixel[0],b.pixel[1]+50))))
        self.assertEqual(result.reason,'excessive_residual')
        self.assertGreater(result.ray_separation_m,.25)

    def test_range_limit(self):
        result=self.result(config=TriangulationConfig(max_range_m=10))
        self.assertEqual(result.reason,'out_of_range')

    def test_condition_and_policy_validation(self):
        with self.assertRaises(ValueError): TriangulationConfig(max_condition_number=1)
        with self.assertRaises(ValueError): TriangulationConfig(max_age_s=math.nan)
        with self.assertRaises(ValueError): self.result(now=math.inf)

    def test_noiseless_moving_trajectory(self):
        scenario=default_scenario(steps=500)
        for frame in simulate(scenario):
            result=triangulate(scenario.cameras,frame.detections,frame.simulation_time_s)
            self.assertTrue(result.valid,result.reason)
            self.assertLess(math.dist(result.position_enu_m,frame.truth.position_enu_m),1e-9)

    def test_range_checked_for_both_cameras(self):
        point=(0.,10.,2.)
        cameras=(Camera.look_at('near',(0.,0.,2.),point),
                 Camera.look_at('far',(100.,0.,2.),point))
        scenario=replace(self.scenario,cameras=cameras,target=Target(point,(0.,0.,0.)))
        observations=next(simulate(scenario)).detections
        result=triangulate(cameras,observations,0.,TriangulationConfig(max_range_m=50))
        self.assertEqual(result.reason,'out_of_range')
        self.assertLess(result.ranges_m[0],50)
        self.assertGreater(result.ranges_m[1],50)
