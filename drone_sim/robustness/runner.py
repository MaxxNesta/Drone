"""Experiment orchestration and truth-owned scoring, outside estimator modules."""
from collections import Counter, defaultdict
from dataclasses import asdict
from ..core import default_scenario
from ..evaluate import error_norm, statistics
from ..tracking import FilterConfig
from ..triangulation import TriangulationConfig
from .association import CameraAssociation, stereo_pairs
from .world import generate, position_at, sample_key


class MatchedErrors:
    """Both algorithms contribute together or neither does; never different masks."""
    def __init__(self):
        self.raw,self.filtered=[],[]

    def add(self,raw,filtered,truth):
        self.raw.append(error_norm(raw,truth))
        self.filtered.append(error_norm(filtered,truth))

    def report(self):
        return {'matched_samples':len(self.raw),
                'raw':statistics(self.raw),'filtered':statistics(self.filtered)}


class IdentityMetrics:
    """Ground-truth labels are allowed here only, never in association."""
    def __init__(self):
        self.by_object={}
        self.by_track={}
        self.switches=self.track_label_changes=self.reacquisitions=self.same_id_reacquisitions=0
        self.recovery_delays=[]

    def observe(self,camera,identity,evidence,arrival_s,period_s):
        key=(camera,evidence.object_id)
        previous=self.by_object.get(key)
        recovered=switched=same_id=False
        recovery_delay=None
        if previous:
            last_id,last_capture=previous
            if identity != last_id:
                self.switches+=1
                switched=True
            if evidence.true_capture_s-last_capture > period_s*1.5+1e-9:
                self.reacquisitions+=1
                self.same_id_reacquisitions+=int(identity == last_id)
                recovery_delay=arrival_s-(last_capture+period_s)
                self.recovery_delays.append(recovery_delay)
                recovered=True
                same_id=identity == last_id
        label_changed=identity in self.by_track and self.by_track[identity] != evidence.object_id
        if label_changed:
            self.track_label_changes+=1
        self.by_object[key]=(identity,evidence.true_capture_s)
        self.by_track[identity]=evidence.object_id
        return {'reacquired':recovered,'same_id_reacquired':same_id,'identity_switch':switched,
                'track_label_change':label_changed,'recovery_delay_s':recovery_delay}

    def report(self):
        return {'identity_switches':self.switches,'track_label_changes':self.track_label_changes,
                'reacquisitions':self.reacquisitions,'same_id_reacquisitions':self.same_id_reacquisitions,
                'recovery_delay_s':statistics(self.recovery_delays)}


def run(config):
    events,evidence,generation=generate(config)
    cameras=default_scenario().cameras
    # Only calibration and observation-policy parameters cross this boundary.
    estimators={c.camera_id:CameraAssociation(c,config.association_gate_px) for c in cameras}
    faults={c.camera_id:f for c,f in zip(cameras,config.cameras)}
    objects={o.target_id:o for o in config.objects}
    cache={c.camera_id:[] for c in cameras}
    calendar=defaultdict(list)
    for event in events:
        calendar[event.tick].append(event.packet)
    horizon=max([config.steps]+list(calendar))
    pixels,positions=MatchedErrors(),MatchedErrors()
    identity=IdentityMetrics()
    packet_rejections,sample_rejections,stereo_rejections=Counter(),Counter(),Counter()
    states,transitions=Counter(),Counter()
    previous_states={}
    latency,reported_ages,position_latency=[],[],[]
    delivered,accepted,seen_pairs=set(),set(),set()
    coverage={'raw':set(),'filtered':set()}
    valid_positions=Counter()
    mismatches=0
    geometric_stereo=set()
    for tick in range(config.steps+1):
        for obj in config.objects:
            point=position_at(obj,tick*config.dt_s)
            if all(c.project(point)[0] is not None for c in cameras):
                geometric_stereo.add((obj.target_id,tick))
    yield {'type':'configuration','schema_version':1,'configuration':config.to_dict(),
           'calibration':[asdict(c) for c in cameras],
           'policy':{'filter':asdict(FilterConfig()),'triangulation':asdict(TriangulationConfig()),
                     'max_tracks_per_camera':32,
                     'latency_policy':'in-order capture-time filtering; no retimestamping',
                     'stereo_policy':'geometry-only, latest packet per camera, no interpolation'}}
    for tick in range(horizon+1):
        now=tick*config.dt_s
        packet_records,scoring_records=[],[]
        for packet in calendar.get(tick,()):
            # True transport latency is scorer-only; estimator sees reported age.
            policy=faults[packet.camera_id]
            true_capture=(policy.phase_ticks+packet.sequence*policy.period_ticks)*config.dt_s
            latency.append(now-true_capture)
            reported_ages.append(now-packet.timestamp_s)
            delivered.update(sample_key(packet,i) for i in range(len(packet.pixels)))
            result=estimators[packet.camera_id].consume(packet,now)
            if result['reason'] == 'accepted':
                cache[packet.camera_id]=result['accepted']
            else:
                packet_rejections[result['reason']]+=1
            for rejection in result['rejections']:
                sample_rejections[rejection['reason']]+=1
            for association in result['accepted']:
                accepted.add(association.sample)
                truth=evidence[association.sample]
                pixels.add(association.raw.pixel,association.filtered.pixel,truth.ideal_pixel)
                identity_event=identity.observe(packet.camera_id,association.track_id,truth,now,
                                           faults[packet.camera_id].period_ticks*config.dt_s)
                scoring_records.append({'sample':association.sample,'true_object_id':truth.object_id,
                                        'ideal_pixel':truth.ideal_pixel,
                                        'matched_pixel_error_px':{
                                            'raw':error_norm(association.raw.pixel,truth.ideal_pixel),
                                            'filtered':error_norm(association.filtered.pixel,truth.ideal_pixel)},
                                        **identity_event})
            packet_records.append({'packet':asdict(packet),'reported_age_s':now-packet.timestamp_s,
                                   'evaluation':{'true_capture_s':true_capture,'transport_latency_s':now-true_capture},
                                   'result':{'reason':result['reason'],'rejections':result['rejections'],
                                             'associated':[asdict(a) for a in result['accepted']]}})
        snapshots=[]
        changes=[]
        for estimator in estimators.values():
            for snapshot in estimator.snapshots(now):
                states[snapshot.status]+=1
                previous=previous_states.get(snapshot.target_id,'uninitialized')
                if previous != snapshot.status:
                    transition=f'{previous}->{snapshot.status}'
                    transitions[transition]+=1
                    changes.append({'track_id':snapshot.target_id,'from':previous,'to':snapshot.status})
                previous_states[snapshot.target_id]=snapshot.status
                snapshots.append({'track_id':snapshot.target_id,'status':snapshot.status,
                                  'centered':snapshot.centered,'age_s':snapshot.age_s,
                                  'pixel':snapshot.pixel})
        pair_records=[]
        if packet_records:
            pairs,diagnostics=stereo_pairs(cameras,cache[cameras[0].camera_id],cache[cameras[1].camera_id],
                                          now,config.epipolar_gate_m,config.ambiguity_margin_m)
            stereo_rejections.update(diagnostics)
            if not cache[cameras[0].camera_id] or not cache[cameras[1].camera_id]:
                stereo_rejections['missing_camera_observations']+=1
            for a,b,raw,filtered in pairs:
                key=(a.sample,b.sample)
                if key in seen_pairs:
                    continue
                seen_pairs.add(key)
                ea,eb=evidence[a.sample],evidence[b.sample]
                same_object=ea.object_id == eb.object_id
                actual_time=(ea.true_capture_s+eb.true_capture_s)/2
                truth_point=position_at(objects[ea.object_id],actual_time) if same_object else None
                errors={}
                for name,result in (('raw',raw),('filtered',filtered)):
                    if result.valid:
                        valid_positions[name]+=1
                        if same_object:
                            coverage_key=(ea.object_id,round(actual_time/config.dt_s))
                            if coverage_key in geometric_stereo:
                                coverage[name].add(coverage_key)
                    else:
                        stereo_rejections[f'{name}:{result.reason}']+=1
                    errors[name]=error_norm(result.position_enu_m,truth_point) if result.valid and same_object else None
                matched=raw.valid and filtered.valid and same_object
                if matched:
                    positions.add(raw.position_enu_m,filtered.position_enu_m,truth_point)
                    position_latency.append(now-actual_time)
                if not same_object:
                    mismatches+=1
                pair_records.append({'samples':key,'track_ids':(a.track_id,b.track_id),
                                     'raw':asdict(raw),'filtered':asdict(filtered),
                                     'evaluation':{'same_true_object':same_object,'matched_error_sample':matched,
                                                   'truth_enu_m':truth_point,'true_capture_midpoint_s':actual_time,
                                                   'position_error_m':errors,'delivery_latency_s':now-actual_time}})
        yield {'type':'tick','tick':tick,'delivery_time_s':now,'deliveries':packet_records,
               'tracks':snapshots,'transitions':changes,'stereo':pair_records,
               'evaluation':scoring_records}
    denominator=generation.get('geometric_opportunities',0)
    stereo_denominator=len(geometric_stereo)
    yield {'type':'summary','scenario':config.name,'generation':generation,
           'delivery_ticks':horizon+1,'unique_delivered_observations':len(delivered),
           'accepted_observations':len(accepted),
           'observation_availability':len(accepted)/denominator if denominator else None,
           'geometric_stereo_ticks':stereo_denominator,
           'position_availability':{k:len(v)/stereo_denominator if stereo_denominator else None for k,v in coverage.items()},
           'valid_position_outputs':dict(valid_positions),'matched_pixel_error_px':pixels.report(),
           'matched_position_error_m':positions.report(),
           'transport_latency_s':statistics(latency),'reported_age_s':statistics(reported_ages),
           'matched_position_latency_s':statistics(position_latency),'identity':identity.report(),
           'cross_view_identity_mismatches':mismatches,'packet_rejections':dict(packet_rejections),
           'observation_rejections':dict(sample_rejections),'stereo_rejections':dict(stereo_rejections),
           'tracking_states':dict(states),'tracking_transitions':dict(transitions)}
