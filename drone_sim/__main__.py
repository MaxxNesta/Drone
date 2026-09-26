"""Run with python3 -m drone_sim; emit portable JSONL diagnostics."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
from .core import default_scenario, simulate


def diagnostics(scenario):
    yield {'type': 'scenario', 'schema_version': 1, 'world_frame': 'ENU',
           'optical_frame': 'right-down-forward', 'distortion': 'none',
           'time_domain': 'simulation', 'configuration': asdict(scenario)}
    counts = {c.camera_id: 0 for c in scenario.cameras}
    max_error = 0.0
    final = None
    for frame in simulate(scenario):
        final = frame
        expected = tuple(p + v * frame.simulation_time_s for p, v in
                         zip(scenario.target.position_enu_m, scenario.target.velocity_enu_mps))
        error = max(abs(p-e) for p, e in zip(frame.truth.position_enu_m, expected))
        max_error = max(max_error, error)
        for detection in frame.detections:
            counts[detection.camera_id] += int(detection.valid)
        yield {'type': 'frame', **asdict(frame)}
    yield {'type': 'summary', 'frame_count': scenario.steps + 1,
           'valid_detections_by_camera': counts,
           'max_motion_absolute_error_m': max_error,
           'final_truth': asdict(final.truth)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--steps', type=int, default=100)
    parser.add_argument('--dt', type=float, default=0.02, help='Fixed step in seconds')
    parser.add_argument('--output', type=Path, help='JSONL path; default is stdout')
    args = parser.parse_args()
    try:
        scenario = default_scenario(args.dt, args.steps)
    except ValueError as error:
        parser.error(str(error))
    def write(stream):
        for record in diagnostics(scenario):
            stream.write(json.dumps(record, sort_keys=True, allow_nan=False) + '\n')
    if args.output:
        with args.output.open('w', encoding='utf-8') as stream:
            write(stream)
    else:
        import sys
        write(sys.stdout)


if __name__ == '__main__':
    main()
