"""Image-plane bridge to Stage 2/3 tracking, with no camera pose or focal length."""
from types import SimpleNamespace
from ..robustness.association import CameraAssociation
from ..robustness.world import Packet


class ObservationTracker:
    def __init__(self):
        self.binding = None
        self.origin = None
        self.last_sequence = -1
        self.last_time = None
        self.groups = {}
        self.states = {}

    def consume(self, frame):
        binding = (frame.source_id, frame.camera_id, frame.image_size, frame.time_domain)
        if self.binding is not None and binding != self.binding:
            raise ValueError('Source, camera, dimensions and clock must remain fixed per tracker')
        if frame.frame_sequence <= self.last_sequence or (self.last_time is not None and frame.capture_time_s <= self.last_time):
            return {'reason': 'duplicate_or_out_of_order', 'assignments': [], 'tracks': [], 'transitions': []}
        self.binding = binding
        if self.origin is None:
            self.origin = frame.capture_time_s
        self.last_sequence, self.last_time = frame.frame_sequence, frame.capture_time_s
        now = frame.capture_time_s-self.origin
        w,h = frame.image_size
        plane = SimpleNamespace(camera_id=frame.camera_id,
            intrinsics=SimpleNamespace(width=w, height=h, cx=w/2, cy=h/2))
        for o in frame.observations:
            if o.class_id not in self.groups:
                self.groups[o.class_id] = CameraAssociation(plane)
        output = {'reason': 'accepted', 'assignments': [], 'tracks': [], 'transitions': [], 'rejections': [],
                  'detection_state': 'detected' if frame.observations else 'no_detection'}
        for category, manager in self.groups.items():
            indices = [i for i,o in enumerate(frame.observations) if o.class_id == category]
            indices.sort(key=lambda i: (-(frame.observations[i].confidence or 0), i))
            for i in indices[8:]:
                output['rejections'].append({'index': i, 'reason': 'observation_capacity'})
            indices = indices[:8]
            packet = Packet(frame.camera_id, frame.frame_sequence, now,
                            tuple(frame.observations[i].pixel for i in indices))
            result = manager.consume(packet, now)
            prefix = str(category)+':'
            for a in result['accepted']:
                output['assignments'].append({'index': indices[a.sample[2]], 'track_id': prefix+a.track_id})
            for r in result['rejections']:
                output['rejections'].append({'index': indices[r['index']], 'reason': r['reason']})
            for s in manager.snapshots(now):
                identity = prefix+s.target_id
                previous = self.states.get(identity)
                if previous != s.status:
                    output['transitions'].append({'track_id': identity, 'from': previous, 'to': s.status})
                self.states[identity] = s.status
                output['tracks'].append({'track_id': identity, 'status': s.status, 'centered': s.centered,
                    'pixel': s.pixel, 'velocity_px_s': s.velocity_px_s, 'age_s': s.age_s,
                    'timestamp_s': frame.capture_time_s})
        return output
