from dataclasses import replace
import json
import subprocess
import sys
import unittest
from drone_sim import Target, default_scenario
from drone_sim.evaluate import evaluate


class EvaluationTests(unittest.TestCase):
    def test_noiseless_stationary_recovery(self):
        scenario=replace(default_scenario(steps=100),target=Target(velocity_enu_mps=(0.,0.,0.)))
        summary=list(evaluate(scenario))[-1]
        self.assertEqual(summary['availability']['filtered'],1.)
        self.assertLess(summary['position_error_m']['filtered']['rmse'],1e-10)

    def test_noisy_moving_improves_pixel_and_position_error(self):
        summary=list(evaluate(default_scenario(steps=500),noise_std_px=1.,seed=7))[-1]
        for key in ['pixel_error_px','position_error_m']:
            self.assertLess(summary[key]['filtered']['rmse'],.8*summary[key]['raw']['rmse'])
        self.assertGreater(summary['availability']['filtered'],.95)

    def test_dropout_availability_and_states(self):
        missing=range(20,70)
        rows=list(evaluate(default_scenario(steps=100),missing_ticks=missing))
        summary=rows[-1]
        self.assertAlmostEqual(summary['availability']['filtered'],51/101)
        for state in ['tracking','coasting','stale','lost']:
            self.assertGreater(summary['track_status_counts'][state],0)
        for row in rows[1:-1]:
            if row['tick'] in missing:
                self.assertFalse(row['triangulation']['filtered']['valid'])

    def test_cli_repeatability_and_summary(self):
        command=[sys.executable,'-m','drone_sim.evaluate','--steps','5','--seed','7']
        data=subprocess.check_output(command)
        self.assertEqual(data,subprocess.check_output(command))
        self.assertEqual(json.loads(data.splitlines()[-1])['frames'],6)

    def test_seed_changes_noise(self):
        scenario=default_scenario(steps=2)
        self.assertNotEqual(list(evaluate(scenario,1.,1)),list(evaluate(scenario,1.,2)))
