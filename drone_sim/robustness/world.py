"""Truth-owned sensor and delivery generator. Estimators see Packet only."""
from collections import Counter
from dataclasses import dataclass
import random
from ..core import default_scenario


@dataclass(frozen=True)
class Packet:
    camera_id: str
    sequence: int
    timestamp_s: float
    pixels: tuple  # shuffled unlabeled (u,v) observations; no stable object identifiers


@dataclass(frozen=True)
class Delivery:
    tick: int
    packet: Packet


@dataclass(frozen=True)
class Evidence:
    object_id: str
    true_capture_s: float
    ideal_pixel: tuple


def sample_key(packet, index):
    return (packet.camera_id,packet.sequence,index)


def position_at(target, timestamp):
    return tuple(p+v*timestamp for p,v in zip(target.position_enu_m,target.velocity_enu_mps))


def generate(config):
    """Return delivery schedule, scorer-only evidence, and sensor-generation counts.

    Packets after the configured end tick are retained and drained by the runner.
    Random streams are keyed by camera, frame and effect to avoid coupling faults.
    """
    cameras = default_scenario().cameras
    events, evidence, counts = [], {}, Counter()
    for camera, faults in zip(cameras,config.cameras):
        sequence = 0
        for tick in range(faults.phase_ticks,config.steps+1,faults.period_ticks):
            timestamp = tick*config.dt_s
            rng = lambda effect: random.Random(f'{config.seed}:{camera.camera_id}:{sequence}:{effect}')
            counts['scheduled_frames'] += 1
            visible = []
            for obj in config.objects:
                pixel, _ = camera.project(position_at(obj,timestamp))
                if pixel is not None:
                    counts['geometric_opportunities'] += 1
                    visible.append((obj,pixel))
            if tick in faults.drop_ticks or rng('drop').random() < faults.drop_probability:
                counts['dropped_frames'] += 1
                counts['dropped_observations'] += len(visible)
                sequence += 1
                continue
            points = []
            noise = rng('noise')
            for obj,pixel in visible:
                if any(o.start <= tick < o.stop and o.object_id in ('*',obj.target_id) for o in faults.occlusions):
                    counts['occluded_observations'] += 1
                    continue
                measured = tuple(p+noise.gauss(0,faults.noise_std_px) for p in pixel)
                points.append((measured,Evidence(obj.target_id,timestamp,pixel)))
            rng('shuffle').shuffle(points)
            packet = Packet(camera.camera_id,sequence,timestamp+faults.timestamp_offset_s,
                            tuple(p for p,_ in points))
            for index,(_,truth) in enumerate(points):
                evidence[sample_key(packet,index)] = truth
            counts['generated_observations'] += len(points)
            delay = faults.latency_ticks + rng('latency').randint(0,faults.jitter_ticks)
            if faults.reorder_every and (sequence+1) % faults.reorder_every == 0:
                delay += faults.reorder_delay_ticks
                counts['reorder_injections'] += 1
            events.append(Delivery(tick+delay,packet))
            if faults.duplicate_every and (sequence+1) % faults.duplicate_every == 0:
                events.append(Delivery(tick+delay+faults.duplicate_delay_ticks,packet))
                counts['duplicate_injections'] += 1
            sequence += 1
    events.sort(key=lambda e:(e.tick,e.packet.camera_id,e.packet.sequence))
    return events,evidence,dict(counts)
