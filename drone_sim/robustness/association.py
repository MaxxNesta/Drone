"""Estimator-only association. No truth records or true object IDs are accepted."""
from copy import deepcopy
from dataclasses import dataclass, replace
from math import hypot, isfinite, log
from ..core import Detection
from ..tracking import CameraTracker, FilterConfig, detection_error
from ..triangulation import TriangulationConfig, triangulate


def assign(costs):
    """Maximum-cardinality, minimum-cost one-to-one matching (<=8 columns).

    Dynamic programming over detection masks avoids a greedy nearest-neighbor
    decision at crossings. None denotes a gated-out edge. Ties are deterministic.
    """
    states = {0:(0.,())}
    for row, values in enumerate(costs):
        updated = dict(states)
        for mask,(cost,pairs) in states.items():
            for column,value in enumerate(values):
                if value is None or mask & (1 << column):
                    continue
                key = mask | (1 << column)
                candidate = (cost+value,pairs+((row,column),))
                if key not in updated or candidate < updated[key]:
                    updated[key] = candidate
        states = updated
    return min(states.values(),key=lambda v:(-len(v[1]),v[0],v[1]))[1]


@dataclass(frozen=True)
class Associated:
    sample: tuple  # (camera ID, packet sequence, shuffled index), not an object ID
    track_id: str
    raw: Detection
    filtered: Detection


class CameraAssociation:
    """In-order capture-time filtering behind an arrival-time freshness gate.

    Delayed packets are not retimestamped. Snapshots predict copies to wall/sim
    delivery time, so state inspection cannot consume the capture-time clock.
    """
    def __init__(self,camera,gate_px=30.,max_tracks=32):
        self.camera,self.gate_px,self.max_tracks = camera,gate_px,max_tracks
        self.tracks,self.latest = {},{}
        self.retired = []
        self.seen = set()
        self.last_sequence,self.last_stamp = -1,-1.
        self.next_id = 0
        self.filter_config = FilterConfig()

    def consume(self,packet,arrival_s):
        result = {'reason':'accepted','accepted':[],'rejections':[]}
        if packet.camera_id != self.camera.camera_id:
            result['reason']='wrong_camera'
        elif packet.sequence in self.seen:
            result['reason']='duplicate_packet'
        else:
            self.seen.add(packet.sequence)
            age=arrival_s-packet.timestamp_s
            if not isfinite(packet.timestamp_s) or packet.timestamp_s < 0:
                result['reason']='invalid_timestamp'
            elif age < -1e-9:
                result['reason']='future_packet'
            elif age > self.filter_config.stale_after_s:
                result['reason']='stale_packet'
            elif packet.sequence <= self.last_sequence or packet.timestamp_s <= self.last_stamp:
                result['reason']='out_of_order_packet'
            elif len(packet.pixels) > 8:
                result['reason']='too_many_observations'
        if result['reason'] != 'accepted':
            return result
        self.last_sequence,self.last_stamp=packet.sequence,packet.timestamp_s
        valid=[]
        k=self.camera.intrinsics
        for index,pixel in enumerate(packet.pixels):
            d=Detection(self.camera.camera_id,packet.sequence,packet.timestamp_s,
                        (k.width,k.height),'unassigned',pixel,True,'visible')
            error=detection_error(d,self.camera)
            if error:
                result['rejections'].append({'index':index,'reason':error})
            else:
                valid.append((index,d))
        ids, predictions=[],[]
        for identity,tracker in list(self.tracks.items()):
            prediction=deepcopy(tracker).step(packet.timestamp_s)
            if prediction.status == 'lost':
                self.retired.append(prediction)
                del self.tracks[identity]
                del self.latest[identity]
            else:
                ids.append(identity)
                predictions.append(prediction)
        costs=[]
        for prediction in predictions:
            values=[]
            for _,d in valid:
                distance=hypot(*(p-q for p,q in zip(d.pixel,prediction.pixel)))
                variances=[prediction.covariance[i][i]+self.filter_config.measurement_variance_px2
                           for i in range(2)]
                mahal=sum((d.pixel[i]-prediction.pixel[i])**2/variances[i] for i in range(2))
                # Gaussian negative log likelihood, omitting shared constants.
                # Mahalanobis alone incorrectly favors old high-uncertainty tracks.
                cost=mahal+sum(log(v) for v in variances)
                values.append(cost if distance <= self.gate_px and
                              mahal <= self.filter_config.gate_squared_mahalanobis else None)
            costs.append(values)
        matches=dict(assign(costs))
        used=set(matches.values())
        for row,identity in enumerate(ids):
            observation=replace(valid[matches[row]][1],target_id=identity) if row in matches else None
            estimate=self.tracks[identity].step(packet.timestamp_s,observation)
            self.latest[identity]=estimate
            if observation is not None:
                index=valid[matches[row]][0]
                if estimate.accepted:
                    result['accepted'].append(Associated((packet.camera_id,packet.sequence,index),identity,
                                                        observation,estimate.observation))
                else:
                    result['rejections'].append({'index':index,'reason':estimate.reason})
        # Initial identities depend on measured geometry, never simulator object order.
        for column in sorted(set(range(len(valid)))-used,key=lambda i:valid[i][1].pixel):
            index,d=valid[column]
            if len(self.tracks) >= self.max_tracks:
                result['rejections'].append({'index':index,'reason':'track_capacity'})
                continue
            identity=f'{self.camera.camera_id}:track-{self.next_id}'
            self.next_id+=1
            tracker=CameraTracker(self.camera,identity,self.filter_config)
            raw=replace(d,target_id=identity)
            estimate=tracker.step(packet.timestamp_s,raw)
            self.tracks[identity],self.latest[identity]=tracker,estimate
            result['accepted'].append(Associated((packet.camera_id,packet.sequence,index),identity,
                                                raw,estimate.observation))
        return result

    def snapshots(self,now_s):
        snapshots=self.retired
        self.retired=[]
        for identity,tracker in list(self.tracks.items()):
            latest=self.latest[identity]
            snapshot=latest if latest.timestamp_s == now_s else deepcopy(tracker).step(now_s)
            snapshots.append(snapshot)
            if snapshot.status == 'lost':
                del self.tracks[identity]
                del self.latest[identity]
        return snapshots


def stereo_pairs(cameras,left,right,now_s,epipolar_gate_m=.5,ambiguity_margin_m=.01):
    """Geometry-only cross-view matching, shared by raw and filtered comparisons.

    Candidate IDs are explicitly assigned here only after local association;
    the Stage 2 solver never receives ground-truth object identity.
    """
    costs=[]
    diagnostics=[]
    policy=TriangulationConfig()
    for a in left:
        row=[]
        for b in right:
            observations=(replace(a.raw,target_id='candidate'),replace(b.raw,target_id='candidate'))
            result=triangulate(cameras,observations,now_s,policy)
            usable=result.reason in ('accepted','excessive_residual')
            cost=result.ray_separation_m if usable else None
            if cost is not None and cost > epipolar_gate_m:
                cost=None
            row.append(cost)
            if cost is None:
                diagnostics.append(result.reason if not usable else 'epipolar_gate')
        costs.append(row)
    # Reject locally ambiguous edges instead of inventing cross-camera identity.
    gated=[row[:] for row in costs]
    for i,row in enumerate(costs):
        for j,cost in enumerate(row):
            if cost is None:
                continue
            rivals=[c for k,c in enumerate(row) if k != j and c is not None]
            rivals += [r[j] for k,r in enumerate(costs) if k != i and r[j] is not None]
            if rivals and min(rivals) <= cost+ambiguity_margin_m:
                gated[i][j]=None
                diagnostics.append('ambiguous_stereo')
    pairs=[]
    for i,j in assign(gated):
        a,b=left[i],right[j]
        pair_id=f'{a.track_id}|{b.track_id}'
        raw=triangulate(cameras,(replace(a.raw,target_id=pair_id),replace(b.raw,target_id=pair_id)),now_s)
        filtered=triangulate(cameras,(replace(a.filtered,target_id=pair_id),
                                     replace(b.filtered,target_id=pair_id)),now_s)
        pairs.append((a,b,raw,filtered))
    return pairs,diagnostics
