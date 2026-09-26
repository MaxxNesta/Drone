import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path
import unittest
from dataclasses import replace
from drone_sim import Camera, Intrinsics, Target, default_scenario, simulate
from drone_sim.__main__ import diagnostics


class SimulatorTests(unittest.TestCase):
    def setUp(self):
        self.camera = Camera.look_at('test', (0., 0., 0.), (0., 1., 0.))

    def test_center_and_signed_offsets(self):
        for point, expected in [((0,10,0),(320,240)), ((1,10,0),(360,240)),
                                ((-1,10,0),(280,240)), ((0,10,1),(320,200)),
                                ((0,10,-1),(320,280))]:
            with self.subTest(point=point):
                self.assertEqual(self.camera.project(point), (expected, 'visible'))

    def test_nondefault_intrinsics(self):
        camera = replace(self.camera, intrinsics=Intrinsics(800,600,200,300,100,150))
        self.assertEqual(camera.project((1,10,1))[0], (120,120))

    def test_rotated_translated_camera(self):
        camera = Camera.look_at('east', (10,20,3), (11,20,3))
        self.assertEqual(camera.project((20,20,3))[0], (320,240))
        self.assertEqual(camera.project((20,19,4))[0], (360,200))

    def test_tilted_camera_center(self):
        camera = Camera.look_at('tilted', (1,2,3), (4,6,8))
        pixel, reason = camera.project((4,6,8))
        self.assertEqual(reason, 'visible')
        self.assertAlmostEqual(pixel[0], 320)
        self.assertAlmostEqual(pixel[1], 240)

    def test_behind_and_on_plane(self):
        for point in [(0,-1,0), (1,0,1), (0,0,0)]:
            self.assertEqual(self.camera.project(point), (None, 'behind_or_on_camera_plane'))

    def test_image_bounds(self):
        for point in [(-8,10,0), (0,10,6)]:
            self.assertIsNotNone(self.camera.project(point)[0])
        for point in [(8,10,0), (0,10,-6), (-8.01,10,0), (0,10,6.01)]:
            self.assertEqual(self.camera.project(point), (None, 'outside_image'))

    def test_parallax_matches_baseline_depth(self):
        frame = next(simulate(default_scenario()))
        left, right = frame.detections
        self.assertEqual(left.pixel, (420,180))
        self.assertEqual(right.pixel, (220,180))
        self.assertEqual(left.pixel[0]-right.pixel[0], 400*10/20)
        self.assertNotEqual(left.camera_id, right.camera_id)

    def test_independent_view_validity(self):
        scenario = replace(default_scenario(steps=0), target=Target((15,20,2)))
        left, right = next(simulate(scenario)).detections
        self.assertFalse(left.valid)
        self.assertIsNone(left.pixel)
        self.assertTrue(right.valid)

    def test_fixed_step_trajectory_and_timestamps(self):
        scenario = default_scenario(dt_s=.03, steps=1000)
        frames = list(simulate(scenario))
        self.assertEqual(len(frames), 1001)
        for tick, frame in enumerate(frames):
            self.assertEqual(frame.tick, tick)
            self.assertEqual(frame.simulation_time_s, tick*.03)
            for p, start, velocity in zip(frame.truth.position_enu_m,
                                         scenario.target.position_enu_m,
                                         scenario.target.velocity_enu_mps):
                self.assertAlmostEqual(p, start+velocity*tick*.03, places=10)
            for detection in frame.detections:
                self.assertEqual(detection.frame_sequence, tick)
                self.assertEqual(detection.capture_time_s, frame.simulation_time_s)
                self.assertEqual(detection.time_domain, 'simulation')
        self.assertEqual(scenario.target.position_enu_m, (0,20,5))

    def test_stationary_target_and_zero_steps(self):
        scenario = replace(default_scenario(steps=20), target=Target(velocity_enu_mps=(0,0,0)))
        self.assertTrue(all(f.truth == scenario.target for f in simulate(scenario)))
        self.assertEqual(len(list(simulate(replace(scenario, steps=0)))), 1)

    def test_repeatability(self):
        scenario = default_scenario(steps=30)
        self.assertEqual(list(simulate(scenario)), list(simulate(scenario)))
        command = [sys.executable, '-m', 'drone_sim', '--steps', '3']
        first = subprocess.check_output(command)
        self.assertEqual(first, subprocess.check_output(command))
        rows = [json.loads(row) for row in first.splitlines()]
        self.assertEqual([r['type'] for r in rows], ['scenario']+['frame']*4+['summary'])
        self.assertEqual(rows[-1]['valid_detections_by_camera'], {'camera-0':4,'camera-1':4})

    def test_cli_file_output_matches_stdout(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "observations.jsonl"
            command = [sys.executable, "-m", "drone_sim", "--steps", "2"]
            subprocess.run(command + ["--output", str(output)], check=True)
            self.assertEqual(output.read_bytes(), subprocess.check_output(command))

    def test_diagnostics_motion_error(self):
        summary = list(diagnostics(default_scenario()))[-1]
        self.assertLess(summary['max_motion_absolute_error_m'], 1e-10)
        self.assertEqual(summary['frame_count'], 101)

    def test_reject_invalid_inputs(self):
        for dt in [0,-1,math.nan,math.inf]:
            with self.assertRaises(ValueError): default_scenario(dt_s=dt)
        for steps in [-1,1.5,True]:
            with self.assertRaises(ValueError): default_scenario(steps=steps)
        for kwargs in [dict(fx=0),dict(fy=-1),dict(cx=math.nan),dict(width=0),dict(height=1.5)]:
            with self.assertRaises(ValueError): Intrinsics(**kwargs)
        with self.assertRaises(ValueError): Target((math.nan,0,0))
        with self.assertRaises(ValueError): self.camera.project((0,math.inf,0))
        with self.assertRaises(ValueError): Camera.look_at('bad',(0,0,0),(0,0,1))
        with self.assertRaises(ValueError): replace(self.camera, right=(-1,0,0))
        with self.assertRaises(ValueError): replace(default_scenario(), cameras=(self.camera,self.camera))
        with self.assertRaises(ValueError): replace(default_scenario(), cameras=(self.camera,replace(self.camera,camera_id='other')))

    def test_cli_rejects_invalid_configuration(self):
        result = subprocess.run([sys.executable,'-m','drone_sim','--dt','nan'], capture_output=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, b'')
        self.assertIn(b'dt_s must be finite and positive', result.stderr)


if __name__ == '__main__':
    unittest.main()
