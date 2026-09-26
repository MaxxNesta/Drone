"""Reproducible Stage 2 benchmark: python3 -m drone_sim.evaluate --help."""
import argparse
from collections import Counter
from dataclasses import asdict, replace
import json
from math import isfinite, sqrt
from pathlib import Path
import random
from .core import Target, default_scenario, simulate
from .tracking import CameraTracker, FilterConfig, detection_error
from .triangulation import TriangulationConfig, triangulate


def error_norm(a, b):
    return sqrt(sum((x-y)**2 for x,y in zip(a,b)))


def statistics(errors):
    return {'samples':len(errors),
            'rmse':sqrt(sum(e*e for e in errors)/len(errors)) if errors else None,
            'mean':sum(errors)/len(errors) if errors else None,
            'max':max(errors) if errors else None}


def evaluate(scenario, noise_std_px=0.0, seed=7, missing_ticks=()):
    """Truth is used only to generate observations and score outputs, never by estimators."""
    if not isfinite(noise_std_px) or noise_std_px < 0:
        raise ValueError('Noise standard deviation must be finite and nonnegative')
    missing = frozenset(missing_ticks)
    rngs = {c.camera_id:random.Random(f'{seed}:{c.camera_id}') for c in scenario.cameras}
    trackers = [CameraTracker(c,scenario.target.target_id) for c in scenario.cameras]
    pixels = {'raw':[], 'filtered':[]}
    positions = {'raw':[], 'filtered':[]}
    rejected = {'raw':Counter(), 'filtered':Counter()}
    states = Counter()
    count = 0
    yield {'type':'configuration','schema_version':1,'scenario':asdict(scenario),
           'noise_std_px':noise_std_px,'seed':seed,'missing_ticks':sorted(missing),
           'filter':asdict(FilterConfig()),'triangulation':asdict(TriangulationConfig())}
    for frame in simulate(scenario):
        count += 1
        observations = []
        for truth in frame.detections:
            observation = truth
            if frame.tick in missing:
                observation = replace(truth,valid=False,pixel=None,reason='synthetic_dropout')
            elif truth.valid and noise_std_px:
                observation = replace(truth,pixel=tuple(p+rngs[truth.camera_id].gauss(0,noise_std_px)
                                                       for p in truth.pixel))
            observations.append(observation)
        tracks = [tracker.step(frame.simulation_time_s,observation)
                  for tracker,observation in zip(trackers,observations)]
        results = {
            'raw':triangulate(scenario.cameras,observations,frame.simulation_time_s),
            'filtered':triangulate(scenario.cameras,[t.observation for t in tracks],frame.simulation_time_s)}
        for truth,observation,track in zip(frame.detections,observations,tracks):
            states[track.status] += 1
            if truth.valid:
                camera = next(c for c in scenario.cameras if c.camera_id == observation.camera_id)
                if detection_error(observation,camera) is None:
                    pixels['raw'].append(error_norm(observation.pixel,truth.pixel))
                if track.accepted:
                    pixels['filtered'].append(error_norm(track.pixel,truth.pixel))
        frame_errors = {}
        for name,result in results.items():
            if result.valid:
                error = error_norm(result.position_enu_m,frame.truth.position_enu_m)
                positions[name].append(error)
                frame_errors[name] = error
            else:
                rejected[name][result.reason] += 1
                frame_errors[name] = None
        yield {'type':'estimate','tick':frame.tick,'time_s':frame.simulation_time_s,
               'truth_enu_m':frame.truth.position_enu_m,
               'observations':[asdict(o) for o in observations],
               'tracks':[asdict(t) for t in tracks],
               'triangulation':{name:asdict(result) for name,result in results.items()},
               'position_error_m':frame_errors}
    yield {'type':'summary','frames':count,'track_status_counts':dict(states),
           'pixel_error_px':{key:statistics(value) for key,value in pixels.items()},
           'position_error_m':{key:statistics(value) for key,value in positions.items()},
           'availability':{key:len(value)/count for key,value in positions.items()},
           'rejections':{key:dict(value) for key,value in rejected.items()}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario',choices=['moving','stationary','dropout','degenerate'],default='moving')
    parser.add_argument('--steps',type=int,default=500)
    parser.add_argument('--noise-std',type=float,default=1.0)
    parser.add_argument('--seed',type=int,default=7)
    parser.add_argument('--output',type=Path)
    args = parser.parse_args()
    try:
        scenario = default_scenario(steps=args.steps)
        if not isfinite(args.noise_std) or args.noise_std < 0:
            raise ValueError('Noise standard deviation must be finite and nonnegative')
        if args.scenario == 'stationary':
            scenario = replace(scenario,target=Target(velocity_enu_mps=(0.,0.,0.)))
        if args.scenario == 'degenerate':
            # Collinear origins looking north: a target on that axis gives parallel rays.
            from .core import Camera
            cameras = (Camera.look_at('camera-0',(0.,0.,2.),(0.,1.,2.)),
                       Camera.look_at('camera-1',(0.,1.,2.),(0.,2.,2.)))
            scenario = replace(scenario,cameras=cameras,target=Target((0.,20.,2.),(0.,0.,0.)))
        missing = list(range(100,120))+list(range(200,250)) if args.scenario == 'dropout' else []
    except ValueError as error:
        parser.error(str(error))
    def write(stream):
        for record in evaluate(scenario,args.noise_std,args.seed,missing):
            stream.write(json.dumps(record,sort_keys=True,allow_nan=False)+'\n')
    if args.output:
        with args.output.open('w',encoding='utf-8') as stream:
            write(stream)
    else:
        import sys
        write(sys.stdout)


if __name__ == '__main__':
    main()
