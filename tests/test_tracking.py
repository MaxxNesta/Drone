from dataclasses import replace
import math
import unittest
from drone_sim import Camera, Detection, default_scenario, simulate
from drone_sim.tracking import CameraTracker, FilterConfig


class TrackingTests(unittest.TestCase):
    def setUp(self):
        self.camera = Camera.look_at('c',(0.,0.,0.),(0.,1.,0.))
        self.tracker = CameraTracker(self.camera,'target')

    def observation(self, time=0., sequence=0, pixel=(320.,240.), **changes):
        return replace(Detection('c',sequence,time,(640,480),'target',pixel,True,'visible'),**changes)

    def test_initialization_uses_measurement(self):
        result = self.tracker.step(0.,self.observation(pixel=(450.,120.)))
        self.assertEqual(result.pixel,(450.,120.))
        self.assertEqual(result.velocity_px_s,(0.,0.))
        self.assertEqual(result.status,'tracking')
        self.assertFalse(result.centered)
        self.assertEqual(result.covariance[0][0],1.)

    def test_invalid_cannot_initialize(self):
        result = self.tracker.step(0.,self.observation(valid=False))
        self.assertEqual(result.status,'lost')
        self.assertIsNone(result.pixel)
        self.assertIsNone(result.centered)

    def test_stationary_center_is_valid(self):
        for i in range(100):
            result = self.tracker.step(i*.02,self.observation(i*.02,i))
            self.assertEqual(result.status,'tracking')
            self.assertTrue(result.centered)
            self.assertTrue(result.observation.valid)
            self.assertEqual(result.pixel,(320.,240.))

    def test_coasting_stale_lost_and_reacquisition(self):
        self.tracker.step(0.,self.observation())
        for time,status in [(.1,'coasting'),(.25,'coasting'),(.26,'stale'),(.75,'stale'),(.76,'lost')]:
            result = self.tracker.step(time)
            self.assertEqual(result.status,status)
            self.assertFalse(result.observation.valid)
            self.assertEqual(result.last_measurement_time_s,0.)
        result = self.tracker.step(.8,self.observation(.8,40,(500.,100.)))
        self.assertEqual(result.pixel,(500.,100.))
        self.assertEqual(result.velocity_px_s,(0.,0.))
        self.assertEqual(result.status,'tracking')

    def test_constant_velocity_convergence_and_covariance(self):
        for i in range(251):
            t=i*.02
            result=self.tracker.step(t,self.observation(t,i,(100+12*t,200-5*t)))
            p=result.covariance
            self.assertEqual(p[0][2],p[2][0])
            self.assertGreaterEqual(p[0][0]*p[2][2]-p[0][2]**2,-1e-10)
        self.assertAlmostEqual(result.velocity_px_s[0],12,places=2)
        self.assertAlmostEqual(result.velocity_px_s[1],-5,places=2)
        self.assertLess(math.dist(result.pixel,(160,175)),.01)
        coast=self.tracker.step(5.1)
        self.assertLess(math.dist(coast.pixel,(161.2,174.5)),.02)
        self.assertGreater(coast.covariance[0][0],result.covariance[0][0])

    def test_innovation_gate_does_not_refresh_age(self):
        self.tracker.step(0.,self.observation())
        result=self.tracker.step(.02,self.observation(.02,1,(600,400)))
        self.assertEqual(result.reason,'innovation_rejected')
        self.assertEqual(result.last_measurement_time_s,0.)
        self.assertEqual(result.pixel,(320,240))

    def test_reject_identity_schema_pixels_and_timing(self):
        changes=[({'target_id':'other'},'wrong_target'),({'camera_id':'other'},'wrong_camera'),
                 ({'time_domain':'wall'},'unsupported_schema_or_time_domain'),
                 ({'schema_version':2},'unsupported_schema_or_time_domain'),
                 ({'pixel':(math.nan,1)},'invalid_pixel'),({'pixel':(640,0)},'outside_image'),
                 ({'pixel':None},'invalid_observation'),({'image_size':(10,10)},'wrong_image_size'),
                 ({'capture_time_s':2.},'future_observation'),
                 ({'capture_time_s':0.},'stale_observation'),
                 ({'capture_time_s':.9},'delayed_observation')]
        for delta,reason in changes:
            with self.subTest(reason=reason):
                tracker=CameraTracker(self.camera,'target')
                result=tracker.step(1.,replace(self.observation(1.,1),**delta))
                self.assertEqual(result.reason,reason)
                self.assertFalse(result.accepted)

    def test_duplicate_and_clock_reversal(self):
        observation=self.observation()
        self.tracker.step(0.,observation)
        self.assertEqual(self.tracker.step(0.,observation).reason,'duplicate_or_out_of_order')
        self.assertEqual(self.tracker.step(.1,self.observation(.1,0)).reason,'duplicate_or_out_of_order')
        with self.assertRaises(ValueError): self.tracker.step(.05)
        with self.assertRaises(ValueError): self.tracker.step(math.nan)

    def test_dt_scaled_covariance(self):
        a,b=CameraTracker(self.camera,'target'),CameraTracker(self.camera,'target')
        for tracker in (a,b): tracker.step(0.,self.observation())
        end_a=a.step(.2)
        b.step(.1)
        end_b=b.step(.2)
        for row_a,row_b in zip(end_a.covariance,end_b.covariance):
            for x,y in zip(row_a,row_b): self.assertAlmostEqual(x,y,places=10)

    def test_invalid_filter_parameters(self):
        for kwargs in [dict(measurement_variance_px2=0),dict(lost_after_s=.1),
                       dict(centered_radius_px=-1),dict(acceleration_spectral_density=math.inf)]:
            with self.assertRaises(ValueError): FilterConfig(**kwargs)

    def test_one_step_matches_hand_computed_kalman_gain(self):
        config=FilterConfig(acceleration_spectral_density=0.,initial_velocity_variance=4.)
        tracker=CameraTracker(self.camera,'target',config)
        tracker.step(0.,self.observation(pixel=(100.,200.)))
        # dt=.5: P_pre=[[2,2],[2,4]], S=3, K=[2/3,2/3].
        result=tracker.step(.5,self.observation(.5,1,(103.,200.)))
        self.assertAlmostEqual(result.pixel[0],102.)
        self.assertAlmostEqual(result.velocity_px_s[0],2.)
        self.assertAlmostEqual(result.covariance[0][0],2/3)
        self.assertAlmostEqual(result.covariance[0][2],2/3)
        self.assertAlmostEqual(result.covariance[2][2],8/3)

    def test_stale_packet_never_refreshes_last_measurement(self):
        first=self.observation()
        self.tracker.step(0.,first)
        result=self.tracker.step(.3,first)
        self.assertEqual(result.reason,'stale_observation')
        self.assertEqual(result.status,'stale')
        self.assertEqual(result.last_measurement_time_s,0.)
        self.assertFalse(result.observation.valid)
