import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace as NS
from drone_sim.vision.validate import measure, summarize, interval, distribution
from drone_sim.vision.annotations import make_template, convert
from tests.test_vision import frame, observation


class ValidationTests(unittest.TestCase):
    def run_frames(self, frames, suppress=()):
        def clock():
            clock.tick += .001
            return clock.tick
        clock.tick = 0
        return list(measure(((f,'image') for f in frames),NS(detect=lambda _: (observation(),)),
                            lambda image,row:None,suppress,clock=clock))

    def test_timing_components_and_original_pts(self):
        rows = self.run_frames([frame()])
        times = rows[0]['timing_ms']
        self.assertAlmostEqual(times['total'],3.)
        self.assertAlmostEqual(times['pipeline_with_render'],4.)
        self.assertEqual(rows[0]['observation']['capture_time_s'],10.)
        summary = summarize(rows,2.)
        self.assertEqual(summary['export_throughput_fps'],.5)
        self.assertIsNone(summary['accuracy']['precision'])
        self.assertEqual(summary['accuracy_status'],'blocked_no_manual_annotations')

    def test_suppression_lifecycle_keeps_detector_evidence(self):
        frames = [frame(i,t) for i,t in enumerate([10,10.1,10.4,10.9,11])]
        # Use .09 start to avoid the binary representation of 10.1-10 at a boundary.
        rows = self.run_frames(frames,[(.09,1.)])
        self.assertEqual([r['tracking']['tracks'][0]['status'] for r in rows],
                         ['tracking','coasting','stale','lost','tracking'])
        self.assertEqual(len(rows[2]['raw_observation']['observations']),1)
        self.assertEqual(rows[2]['observation']['observations'],())
        self.assertEqual(summarize(rows,1.)['suppressed_frames'],3)

    def test_multiple_detections_and_limits(self):
        detector = NS(detect=lambda _: (observation(),observation(300)))
        rows=list(measure([(frame(),'image'),(frame(1,10.1),'image')],detector,lambda *a:None,max_frames=1))
        summary=summarize(rows,1.)
        self.assertEqual(summary['frames'],1)
        self.assertEqual(summary['raw_multiple_detection_frames'],1)
        self.assertEqual(len(rows[0]['tracking']['assignments']),2)

    def test_natural_missing_detections_and_no_labels(self):
        rows=list(measure([(frame(),'image')],NS(detect=lambda _:()),lambda *a:None))
        summary=summarize(rows,1.)
        self.assertEqual(summary['raw_zero_detection_frames'],1)
        self.assertIsNone(summary['accuracy']['identity_switches'])
        self.assertIsNone(distribution([])['p95'])

    def test_failure_closes_decoder(self):
        closed=[]
        def frames():
            try:
                yield frame(),'image'
            finally:
                closed.append(True)
        def fail(*args): raise RuntimeError('export failure')
        with self.assertRaises(RuntimeError):
            list(measure(frames(),NS(detect=lambda _:()),fail))
        self.assertEqual(closed,[True])

    def test_invalid_intervals_and_statistics(self):
        for text in ('-1:2','2:1','nan:2','0:inf'):
            with self.assertRaises(ValueError): interval(text)
        with self.assertRaises(ValueError): summarize([],0)
        with self.assertRaises(ValueError): distribution([float('nan')])
        self.assertEqual(interval('1:2'),(1.,2.))

    def test_template_is_unfilled_and_cannot_score(self):
        rows=self.run_frames([frame()])
        template=make_template(rows,[0])
        self.assertIsNone(template['frames'][0]['objects'])
        self.assertFalse(template['frames'][0]['reviewed'])
        self.assertNotIn('observations',template['frames'][0]['metadata'])
        with self.assertRaises(ValueError): convert(template)

    def test_manual_conversion_and_explicit_empty_frame(self):
        template=make_template(self.run_frames([frame(),frame(1,10.1)]),[0,1])
        for f in template['frames']:
            f.update(reviewed=True,objects=[])
        template['frames'][0]['objects']=[{'bbox_xyxy':[95,95,105,105],'class_id':0,'truth_id':'manual-1'}]
        rows=convert(template)
        self.assertEqual(rows[0]['truth_ids'],['manual-1'])
        self.assertIsNone(rows[0]['observation']['observations'][0]['confidence'])
        self.assertEqual(rows[1]['truth_ids'],[])

    def test_missing_template_frames_fail(self):
        with self.assertRaises(ValueError): make_template(self.run_frames([frame()]),[99])


@unittest.skipUnless(all(importlib.util.find_spec(x) is not None for x in ('av','cv2','numpy')), 'optional video dependencies unavailable')
class CodecTests(unittest.TestCase):
    def test_actual_codec_vfr_overlay_roundtrip(self):
        import av
        import numpy as np
        from drone_sim.vision.validate import DemoWriter
        from drone_sim.vision.backends import video_frames
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'demo.mp4'
            writer=DemoWriter(path,{0:'person'})
            records=[frame(i,t) for i,t in enumerate([10,10.04,10.12])]
            try:
                rows=list(measure(((f,np.zeros((480,640,3),dtype=np.uint8)) for f in records),
                                  NS(detect=lambda _: (observation(),)),writer))
            finally:
                writer.close()
            decoded=list(video_frames(path,'export'))
            self.assertEqual(len(decoded),3)
            self.assertEqual([f.image_size for f,_ in decoded],[(640,480)]*3)
            for (f,image),expected in zip(decoded,[0,.04,.12]):
                self.assertAlmostEqual(f.capture_time_s,expected,places=5)
                self.assertGreater(image.sum(),0)
