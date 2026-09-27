import json
import tempfile
import unittest
from dataclasses import replace
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
from drone_sim.core import default_scenario, simulate
from drone_sim.vision import Observation, ObservationFrame, ObservationTracker, from_simulation
from drone_sim.vision.backends import YoloDetector, video_frames, webcam_frames, process, fingerprint
from drone_sim.vision.evaluate import evaluate


def observation(x=100, category=0, confidence=.9):
    return Observation((x,100), (x-5,95,x+5,105), confidence, category)


def frame(seq=0, stamp=10., objects=None):
    return ObservationFrame('fixture:synthetic', 'camera-A', seq, stamp, (640,480),
        (observation(),) if objects is None else tuple(objects), 'video_pts', 'synthetic_pts',
        round(stamp*1000), (1,1000))


def rows_for(frames):
    tracker = ObservationTracker()
    return [{'observation':f.to_dict(), 'tracking':tracker.consume(f)} for f in frames]


def label(f, ids):
    return {'observation':f.to_dict(), 'truth_ids':ids}


class RecordsTests(unittest.TestCase):
    def test_roundtrip_and_preservation(self):
        f=frame(stamp=-2.5)
        actual=ObservationFrame.from_dict(json.loads(json.dumps(f.to_dict())))
        self.assertEqual(actual, f)
        self.assertEqual(actual.capture_time_s,-2.5)
        self.assertEqual(actual.observations[0].confidence,.9)

    def test_invalid_records(self):
        for change in ({'schema_version':1}, {'capture_time_s':float('nan')}, {'image_size':(0,1)},
                       {'frame_sequence':-1}, {'pts':None}, {'time_base':(1,0)},
                       {'observations':(observation(confidence=2),)},
                       {'observations':(Observation((640,100)),)},
                       {'observations':(Observation((100,100),(0,0,2,2)),)},
                       {'observations':(observation(category=-1),)}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                replace(frame(), **change)

    def test_simulation_uses_same_tracker_without_truth_ids(self):
        simulation=next(simulate(default_scenario()))
        detection=simulation.detections[0]
        f=from_simulation(detection,'simulation:camera-A')
        self.assertNotIn('target_id',json.dumps(f.to_dict()))
        self.assertIsNone(f.observations[0].confidence)
        self.assertEqual(ObservationTracker().consume(f)['tracks'][0]['status'],'tracking')


class TrackingTests(unittest.TestCase):
    def test_initialized_and_distinct_states(self):
        tracker=ObservationTracker()
        outputs=[tracker.consume(frame(i,t,objects)) for i,t,objects in
                 [(0,10,None),(1,10.1,()),(2,10.4,()),(3,10.9,()),(4,11,None)]]
        self.assertEqual([o['tracks'][0]['status'] for o in outputs],
                         ['tracking','coasting','stale','lost','tracking'])
        self.assertEqual(outputs[0]['tracks'][0]['pixel'],(100,100))
        self.assertEqual(outputs[1]['detection_state'],'no_detection')
        self.assertNotEqual(outputs[0]['assignments'][0]['track_id'], outputs[4]['assignments'][0]['track_id'])

    def test_duplicate_and_out_of_order_do_not_advance(self):
        t=ObservationTracker()
        t.consume(frame())
        self.assertEqual(t.consume(frame())['reason'],'duplicate_or_out_of_order')
        self.assertEqual(t.consume(frame(1,9))['reason'],'duplicate_or_out_of_order')
        self.assertEqual(t.consume(frame(1,10.1))['tracks'][0]['status'],'tracking')

    def test_source_dimensions_camera_clock_binding(self):
        for changes in ({'source_id':'other'}, {'camera_id':'other'}, {'image_size':(800,600)},
                        {'time_domain':'simulation'}):
            t=ObservationTracker(); t.consume(frame())
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                t.consume(replace(frame(1,10.1),**changes))

    def test_class_partition_and_capacity(self):
        objects=[observation(100+i,category=i%2) for i in range(20)]
        result=ObservationTracker().consume(frame(objects=objects))
        self.assertEqual(len(result['assignments']),16)
        self.assertEqual(len(result['rejections']),4)
        self.assertEqual(len(set(a['track_id'] for a in result['assignments'])),16)

    def test_replay_deterministic_and_motion_continuity(self):
        frames=[frame(i,10+i*.1,[observation(100+i)]) for i in range(10)]
        a,b=rows_for(frames),rows_for(frames)
        self.assertEqual(a,b)
        self.assertEqual(len({r['tracking']['assignments'][0]['track_id'] for r in a}),1)
        self.assertGreater(a[-1]['tracking']['tracks'][0]['velocity_px_s'][0],0)


class BackendTests(unittest.TestCase):
    def test_missing_weights_no_import_or_download(self):
        with self.assertRaises(ValueError):
            YoloDetector('/does/not/exist.pt')

    def test_yolo_conversion_without_dependencies(self):
        def tensor(values):
            return NS(cpu=lambda:NS(tolist=lambda:values))
        boxes=NS(xyxy=tensor([[95,95,105,105]]),conf=tensor([.8]),cls=tensor([2.]))
        model=NS(predict=Mock(return_value=[NS(boxes=boxes)]))
        with tempfile.NamedTemporaryFile() as weights:
            detector=YoloDetector(weights.name,factory=lambda _:model)
            self.assertEqual(detector.detect('image'),(observation(category=2,confidence=.8),))
            model.predict.assert_called_once_with('image',conf=.4,imgsz=320,verbose=False)

    def test_pts_vfr_source_fingerprint_and_close(self):
        def image(pts):
            return NS(pts=pts,time_base=Fraction(1,1000),width=640,height=480,
                      to_ndarray=lambda format:'pixels')
        container=Mock()
        container.__enter__=Mock(return_value=container)
        container.__exit__=Mock(return_value=False)
        container.decode=Mock(return_value=iter([image(10000),image(10033),image(10100)]))
        with tempfile.NamedTemporaryFile() as video:
            frames=list(video_frames(video.name,'A',opener=lambda _:container))
            self.assertEqual([f.capture_time_s for f,_ in frames],[10,10.033,10.1])
            self.assertEqual(frames[0][0].source_id,fingerprint(video.name))
        container.__exit__.assert_called_once()

    def test_missing_pts_rejected(self):
        class Container:
            def __enter__(self): return self
            def __exit__(self,*args): pass
            def decode(self,**kwargs): return iter([NS(pts=None,time_base=Fraction(1,30))])
        with tempfile.NamedTemporaryFile() as f, self.assertRaises(ValueError):
            list(video_frames(f.name,'A',opener=lambda _:Container()))

    def test_pipeline_injected_compute_clock(self):
        ticks=iter([0,.001,.004,.005,.006])
        detector=NS(detect=lambda _: (observation(),),metadata={'kind':'synthetic'})
        result=list(process([(frame(objects=()),'image')],detector,clock=lambda:next(ticks)))
        self.assertAlmostEqual(result[0]['timing_ms']['total'],5)
        self.assertEqual(result[0]['observation']['capture_time_s'],10)
        self.assertEqual(result[0]['tracking']['tracks'][0]['status'],'tracking')

    def test_webcam_opt_in_receipt_time_and_release(self):
        capture=NS(isOpened=lambda:True,read=lambda:(True,NS(shape=(480,640,3))),release=Mock())
        result=list(webcam_frames(0,'A',1,factory=lambda _:capture,clock=lambda:42.))
        self.assertEqual(result[0][0].time_domain,'monotonic_receive')
        self.assertEqual(result[0][0].capture_time_s,42.)
        capture.release.assert_called_once()

    def test_webcam_failed_read_never_replayed(self):
        capture=NS(isOpened=lambda:True,read=lambda:(False,None),release=Mock())
        with self.assertRaises(ValueError):
            list(webcam_frames(0,'A',1,factory=lambda _:capture))
        capture.release.assert_called_once()


class EvaluationTests(unittest.TestCase):
    def test_counts_identity_and_unmeasured_latency(self):
        frames=[frame(),frame(1,10.9,[]),frame(2,11,[observation(),observation(300)])]
        labels=[label(frame(),['object']),label(frame(1,10.9),['object']),label(frame(2,11),['object'])]
        result=evaluate(rows_for(frames),labels)
        self.assertEqual((result['true_positives'],result['false_positives'],result['false_negatives']),(2,1,1))
        self.assertEqual(result['identity_switches'],1)
        self.assertEqual(result['reacquisitions'],1)
        self.assertIsNone(result['processing_latency_ms']['mean'])

    def test_wrong_class_and_duplicate_predictions(self):
        f=frame(objects=[observation(category=1),observation(),observation()])
        result=evaluate(rows_for([f]),[label(frame(),['a'])])
        self.assertEqual(result['true_positives'],1)
        self.assertEqual(result['false_positives'],2)

    def test_no_labels_not_perfect_accuracy(self):
        result=evaluate(rows_for([frame()]),[])
        self.assertIsNone(result['precision'])
        self.assertIsNone(result['identity_switches'])
        self.assertEqual(result['unlabeled_frames'],1)

    def test_mismatch_missing_duplicate_annotations_fail(self):
        for labels in ([label(frame(1,10.1),['a'])],
                       [label(replace(frame(),image_size=(800,600)),['a'])],
                       [label(frame(),['a']),label(frame(),['a'])]):
            with self.subTest(labels=labels),self.assertRaises(ValueError):
                evaluate(rows_for([frame()]),labels)

class FixtureTests(unittest.TestCase):
    def test_committed_fixture_replay_cli_and_report(self):
        import subprocess
        import sys
        root=Path(__file__).parent/'fixtures'/'vision'
        with tempfile.TemporaryDirectory() as folder:
            result=Path(folder)/'results.jsonl'
            report=Path(folder)/'summary.json'
            command=[sys.executable,'-m','drone_sim.vision','replay','--input',str(root/'observations.jsonl'),'--output',str(result)]
            subprocess.run(command,check=True,capture_output=True)
            first=result.read_bytes()
            subprocess.run(command,check=True,capture_output=True)
            self.assertEqual(first,result.read_bytes())
            subprocess.run([sys.executable,'-m','drone_sim.vision','evaluate','--input',str(result),
                '--annotations',str(root/'annotations.jsonl'),'--output',str(report)],check=True,capture_output=True)
            expected=json.loads((Path(__file__).parents[1]/'docs/diagnostics/stage4_fixture.json').read_text())
            self.assertEqual(json.loads(report.read_text()),expected)

    def test_same_video_bytes_do_not_become_independent_sources(self):
        with tempfile.TemporaryDirectory() as directory:
            a,b=Path(directory)/'a.mp4',Path(directory)/'b.mp4'
            a.write_bytes(b'synthetic bytes'); b.write_bytes(a.read_bytes())
            self.assertEqual(fingerprint(a),fingerprint(b))

    def test_annotations_cannot_change_tracking(self):
        frames=[frame(i,10+i*.1) for i in range(3)]
        rows=rows_for(frames)
        original=json.dumps(rows,sort_keys=True)
        a=evaluate(rows,[label(f,['first']) for f in frames])
        b=evaluate(rows,[label(f,['renamed']) for f in frames])
        self.assertEqual(a,b)
        self.assertEqual(json.dumps(rows,sort_keys=True),original)

    def test_stationary_centered_is_not_lost(self):
        o=Observation((320,240),(315,235,325,245),.8,0)
        rows=rows_for([frame(i,10+i*.1,[o]) for i in range(12)])
        for row in rows:
            track=row['tracking']['tracks'][0]
            self.assertTrue(track['centered'])
            self.assertEqual(track['status'],'tracking')
            self.assertEqual(track['pixel'],(320,240))

    def test_repeated_video_pts_are_rejected(self):
        f=NS(pts=0,time_base=Fraction(1,30),width=640,height=480,to_ndarray=lambda **kw:None)
        class Container:
            def __enter__(self): return self
            def __exit__(self,*args): pass
            def decode(self,**kwargs): return iter([f,f])
        with tempfile.NamedTemporaryFile() as path, self.assertRaises(ValueError):
            list(video_frames(path.name,'A',opener=lambda _:Container()))
