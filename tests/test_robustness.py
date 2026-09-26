"""Regression tests for each deterministic fault and the estimator/truth boundary."""
from dataclasses import fields, replace
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from drone_sim.core import Target, default_scenario
from drone_sim.robustness.config import RobustnessConfig, CameraFaults, Occlusion
from drone_sim.robustness.world import Packet, Evidence, generate
from drone_sim.robustness.association import CameraAssociation, assign
from drone_sim.robustness.runner import run, IdentityMetrics, MatchedErrors


ROOT=Path(__file__).resolve().parents[1]


def load(name):
    return RobustnessConfig.from_dict(json.loads((ROOT/'scenarios/stage3'/f'{name}.json').read_text()))


def summary(config):
    for row in run(config):
        pass
    return row


class FailureScenarioTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.results={path.stem:summary(load(path.stem)) for path in (ROOT/'scenarios/stage3').glob('*.json')}

    def test_baseline_unchanged_and_identity_stable(self):
        result=self.results['baseline']
        self.assertEqual(result['observation_availability'],1.)
        self.assertEqual(result['position_availability']['filtered'],1.)
        self.assertEqual(result['identity']['identity_switches'],0)
        self.assertLess(result['matched_position_error_m']['raw']['rmse'],1e-9)

    def test_noise_reduces_matched_errors_but_reports_fragmentation(self):
        result=self.results['noise']
        for field in ['matched_pixel_error_px','matched_position_error_m']:
            error=result[field]
            self.assertEqual(error['raw']['samples'],error['filtered']['samples'])
            self.assertLess(error['filtered']['rmse'],.8*error['raw']['rmse'])
        self.assertGreater(result['identity']['identity_switches'],0)
        # The raw-only residual failure must be absent from BOTH error arrays.
        self.assertLess(result['matched_position_error_m']['matched_samples'],
                        result['valid_position_outputs']['filtered'])

    def test_dropped_frames_reduce_availability(self):
        result=self.results['dropped_frames']
        self.assertGreater(result['generation']['dropped_frames'],0)
        self.assertLess(result['observation_availability'],.9)
        self.assertGreater(result['observation_availability'],.6)
        self.assertLess(result['position_availability']['filtered'],.8)
        self.assertGreater(result['identity']['reacquisitions'],0)

    def test_occlusion_states_and_reacquisition(self):
        result=self.results['occlusion']
        self.assertEqual(result['generation']['occluded_observations'],160)
        for state in ['tracking','coasting','stale','lost']:
            self.assertGreater(result['tracking_states'][state],0)
        self.assertEqual(result['identity']['reacquisitions'],4)
        self.assertEqual(result['identity']['same_id_reacquisitions'],2)
        self.assertAlmostEqual(result['position_availability']['filtered'],221/301)
        self.assertGreater(result['tracking_transitions']['stale->lost'],0)

    def test_different_rates_are_not_interpolated(self):
        result=self.results['camera_rates']
        self.assertEqual(result['generation']['scheduled_frames'],402)
        self.assertAlmostEqual(result['position_availability']['filtered'],101/301)
        self.assertEqual(result['stereo_rejections']['timestamp_mismatch'],200)

    def test_fresh_latency_preserves_capture_time_and_drains_queue(self):
        result=self.results['latency']
        self.assertEqual(result['delivery_ticks'],305)
        self.assertEqual(result['accepted_observations'],602)
        self.assertEqual(result['position_availability']['filtered'],1.)
        self.assertAlmostEqual(result['transport_latency_s']['mean'],.08)
        self.assertAlmostEqual(result['matched_position_latency_s']['mean'],.08)
        self.assertAlmostEqual(result['matched_position_error_m']['filtered']['rmse'],
                               self.results['baseline']['matched_position_error_m']['filtered']['rmse'])
        records=run(replace(load('latency'),steps=0))
        for row in records:
            if row['type']=='tick' and row['deliveries']:
                d=row['deliveries'][0]
                self.assertEqual(d['result']['associated'][0]['filtered']['capture_time_s'],0.)
                self.assertAlmostEqual(row['delivery_time_s'],.08)

    def test_stale_latency_has_no_positions_or_zero_error_claim(self):
        result=self.results['stale_latency']
        self.assertEqual(result['packet_rejections']['stale_packet'],602)
        self.assertEqual(result['accepted_observations'],0)
        self.assertEqual(result['position_availability']['filtered'],0.)
        self.assertIsNone(result['matched_position_error_m']['filtered']['rmse'])

    def test_timestamp_skew_is_not_silently_corrected(self):
        result=self.results['timestamp_skew']
        self.assertEqual(result['position_availability']['filtered'],0.)
        self.assertGreater(result['stereo_rejections']['timestamp_mismatch'],0)
        self.assertAlmostEqual(result['transport_latency_s']['mean'],0.)
        self.assertGreater(result['reported_age_s']['mean'],0.)

    def test_duplicate_packets_do_not_inflate_metrics(self):
        result=self.results['duplicates']
        self.assertEqual(result['packet_rejections']['duplicate_packet'],150)
        self.assertEqual(result['accepted_observations'],602)
        self.assertEqual(result['matched_position_error_m'],self.results['baseline']['matched_position_error_m'])
        self.assertEqual(result['unique_delivered_observations'],602)

    def test_out_of_order_rejected_without_clock_reversal(self):
        result=self.results['out_of_order']
        self.assertEqual(result['packet_rejections']['out_of_order_packet'],120)
        self.assertEqual(result['accepted_observations'],482)
        self.assertEqual(result['identity']['identity_switches'],0)

    def test_crossing_reports_ambiguity_and_identity_switches(self):
        result=self.results['crossing']
        self.assertGreater(result['stereo_rejections']['ambiguous_stereo'],0)
        self.assertGreater(result['identity']['identity_switches'],0)
        self.assertGreater(result['identity']['track_label_changes'],0)
        self.assertGreater(result['position_availability']['filtered'],.8)
        self.assertEqual(result['cross_view_identity_mismatches'],0)

    def test_combined_faults_expose_false_stereo_association(self):
        result=self.results['combined']
        self.assertGreater(result['packet_rejections']['duplicate_packet'],0)
        self.assertGreater(result['packet_rejections']['out_of_order_packet'],0)
        self.assertGreater(result['cross_view_identity_mismatches'],0)
        self.assertLess(result['position_availability']['filtered'],.3)


class AssociationBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.camera=default_scenario().cameras[0]

    def test_packet_has_no_ground_truth_identity(self):
        self.assertEqual({f.name for f in fields(Packet)},
                         {'camera_id','sequence','timestamp_s','pixels'})
        events,evidence,_=generate(load('crossing'))
        self.assertTrue(evidence)
        self.assertTrue(all(not hasattr(e.packet,'target_id') for e in events))

    def test_relabeling_truth_does_not_change_estimator_outputs(self):
        config=replace(load('crossing'),steps=25)
        renamed=replace(config,objects=tuple(replace(o,target_id='renamed-'+str(i))
                                            for i,o in enumerate(config.objects)))
        events1,evidence1,_=generate(config)
        events2,evidence2,_=generate(renamed)
        self.assertEqual(events1,events2)
        self.assertNotEqual(evidence1,evidence2)
        a,b=list(run(config)),list(run(renamed))
        for x,y in zip(a[1:-1],b[1:-1]):
            for field in ['deliveries','tracks','transitions','stereo']:
                self.assertEqual(x[field],y[field])
        self.assertEqual(a[-1],b[-1])

    def test_shuffled_observation_order_is_not_identity(self):
        estimator=CameraAssociation(self.camera)
        first=estimator.consume(Packet(self.camera.camera_id,0,0.,((200.,200.),(400.,240.))),0.)
        first_ids={a.raw.pixel:a.track_id for a in first['accepted']}
        next_result=estimator.consume(Packet(self.camera.camera_id,1,.02,((400.1,240.),(200.1,200.))),.02)
        for a in next_result['accepted']:
            self.assertEqual(a.track_id,first_ids[(200.,200.) if a.raw.pixel[0]<300 else (400.,240.)])

    def test_assignment_is_global_and_one_to_one(self):
        self.assertEqual(assign([[1.,2.],[1.1,None]]),((0,1),(1,0)))
        self.assertEqual(assign([[None],[None]]),())

    def test_snapshot_does_not_consume_capture_clock(self):
        estimator=CameraAssociation(self.camera)
        estimator.consume(Packet(self.camera.camera_id,0,0.,((320.,240.),)),.1)
        estimator.snapshots(.1)
        result=estimator.consume(Packet(self.camera.camera_id,1,.02,((320.2,240.),)),.12)
        self.assertEqual(len(result['accepted']),1)
        self.assertEqual(result['accepted'][0].track_id,'camera-0:track-0')
        self.assertEqual(result['accepted'][0].filtered.capture_time_s,.02)

    def test_invalid_and_future_packets_rejected(self):
        estimator=CameraAssociation(self.camera)
        self.assertEqual(estimator.consume(Packet(self.camera.camera_id,0,1.,()),0.)['reason'],'future_packet')
        self.assertEqual(estimator.consume(Packet(self.camera.camera_id,1,math.nan,()),0.)['reason'],'invalid_timestamp')
        result=estimator.consume(Packet(self.camera.camera_id,2,.02,((math.nan,1.),)),.02)
        self.assertEqual(result['accepted'],[])
        self.assertEqual(result['rejections'][0]['reason'],'invalid_pixel')

    def test_duplicate_and_out_of_order_have_distinct_reasons(self):
        estimator=CameraAssociation(self.camera)
        packet=Packet(self.camera.camera_id,2,.04,((320.,240.),))
        estimator.consume(packet,.04)
        self.assertEqual(estimator.consume(packet,.06)['reason'],'duplicate_packet')
        self.assertEqual(estimator.consume(Packet(self.camera.camera_id,1,.02,()),.06)['reason'],'out_of_order_packet')

    def test_identity_metrics_score_known_swaps_and_recovery(self):
        metrics=IdentityMetrics()
        metrics.observe('cam','track-0',Evidence('a',0.,(0,0)),0.,.02)
        metrics.observe('cam','track-1',Evidence('a',.02,(0,0)),.02,.02)
        metrics.observe('cam','track-1',Evidence('b',.04,(0,0)),.04,.02)
        metrics.observe('cam','track-1',Evidence('a',1.,(0,0)),1.,.02)
        report=metrics.report()
        self.assertEqual(report['identity_switches'],1)
        self.assertEqual(report['track_label_changes'],2)
        self.assertEqual(report['reacquisitions'],1)
        self.assertEqual(report['same_id_reacquisitions'],1)

    def test_matched_error_vectors_have_identical_denominators(self):
        metrics=MatchedErrors()
        metrics.add((3.,4.),(0.,1.),(0.,0.))
        report=metrics.report()
        self.assertEqual(report['matched_samples'],1)
        self.assertEqual(report['raw']['rmse'],5.)
        self.assertEqual(report['filtered']['rmse'],1.)


class ConfigurationTests(unittest.TestCase):
    def test_json_roundtrip(self):
        for path in (ROOT/'scenarios/stage3').glob('*.json'):
            config=load(path.stem)
            self.assertEqual(config,RobustnessConfig.from_dict(json.loads(json.dumps(config.to_dict()))))

    def test_invalid_parameters(self):
        for kwargs in [dict(period_ticks=0),dict(phase_ticks=1),dict(noise_std_px=math.nan),
                       dict(drop_probability=2),dict(latency_ticks=-1),dict(jitter_ticks=.5)]:
            with self.assertRaises(ValueError): CameraFaults(**kwargs)
        with self.assertRaises(ValueError): RobustnessConfig(steps=-1)
        with self.assertRaises(ValueError): RobustnessConfig(objects=(Target(),Target()))
        with self.assertRaises(ValueError): Occlusion(10,5)
        with self.assertRaises(ValueError):
            RobustnessConfig(cameras=(CameraFaults(occlusions=(Occlusion(0,1,'typo'),)),CameraFaults()))
        with self.assertRaises(TypeError): RobustnessConfig.from_dict({'unknown':True})

    def test_full_dropout_is_explicitly_unavailable(self):
        config=RobustnessConfig(steps=5,cameras=(CameraFaults(drop_probability=1.),)*2)
        result=summary(config)
        self.assertEqual(result['generation']['dropped_frames'],12)
        self.assertEqual(result['observation_availability'],0.)
        self.assertEqual(result['position_availability']['filtered'],0.)

    def test_phase_and_explicit_drop_ticks(self):
        config=RobustnessConfig(steps=10,cameras=(CameraFaults(period_ticks=3,phase_ticks=1,drop_ticks=(4,)),)*2)
        events,_,counts=generate(config)
        self.assertEqual([e.tick for e in events if e.packet.camera_id=='camera-0'],[1,7,10])
        self.assertEqual(counts['dropped_frames'],2)

    def test_object_specific_occlusion_does_not_hide_other_object(self):
        config=replace(load('crossing'),steps=2,cameras=(CameraFaults(occlusions=(Occlusion(0,3,'object-a'),)),)*2)
        events,evidence,counts=generate(config)
        self.assertTrue(all(len(e.packet.pixels)==1 for e in events))
        self.assertEqual({e.object_id for e in evidence.values()},{'object-b'})
        self.assertEqual(counts['occluded_observations'],6)

    def test_seed_controls_randomness_and_replays_exactly(self):
        config=replace(load('combined'),steps=30)
        self.assertEqual(list(run(config)),list(run(config)))
        self.assertNotEqual(generate(config)[0],generate(replace(config,seed=8))[0])

    def test_cli_file_matches_stdout_and_invalid_config_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            config=Path(directory)/'config.json'
            output=Path(directory)/'output.jsonl'
            config.write_text(json.dumps(replace(load('combined'),steps=12).to_dict()))
            command=[sys.executable,'-m','drone_sim.robustness','--config',str(config)]
            data=subprocess.check_output(command,cwd=ROOT)
            subprocess.run(command+['--output',str(output)],cwd=ROOT,check=True)
            self.assertEqual(data,output.read_bytes())
            self.assertEqual(data,subprocess.check_output(command,cwd=ROOT))
            rows=[json.loads(row) for row in data.splitlines()]
            self.assertEqual(rows[0]['type'],'configuration')
            self.assertEqual(rows[-1]['type'],'summary')
            config.write_text('{"dt_s":0}')
            result=subprocess.run(command,cwd=ROOT,capture_output=True)
            self.assertNotEqual(result.returncode,0)
            self.assertEqual(result.stdout,b'')

    def test_suite_output_and_empty_suite(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory)
            command=[sys.executable,'-m','drone_sim.robustness','--suite',str(folder),'--summary-only']
            self.assertNotEqual(subprocess.run(command,capture_output=True,cwd=ROOT).returncode,0)
            for i in range(2):
                (folder/f'{i}.json').write_text(json.dumps(RobustnessConfig(name=f'suite-{i}',steps=2).to_dict()))
            output=subprocess.check_output(command,cwd=ROOT)
            rows=[json.loads(row) for row in output.splitlines()]
            self.assertEqual([r['type'] for r in rows],['configuration','summary']*2)
            self.assertEqual([r['scenario'] for r in rows if r['type']=='summary'],['suite-0','suite-1'])
