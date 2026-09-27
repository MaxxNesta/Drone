"""Recorded-video inference and dependency-free observation replay/evaluation."""
import argparse
import json
from .records import ObservationFrame
from .tracker import ObservationTracker
from .evaluate import evaluate


def read_jsonl(path):
    with open(path) as stream:
        return [json.loads(line) for line in stream if line.strip()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    run = commands.add_parser('run')
    source = run.add_mutually_exclusive_group(required=True)
    source.add_argument('--video')
    source.add_argument('--webcam', type=int)
    run.add_argument('--max-frames', type=int, default=300)
    run.add_argument('--weights', required=True)
    run.add_argument('--camera-id', required=True)
    run.add_argument('--confidence', type=float, default=.4)
    run.add_argument('--image-size', type=int, default=320)
    run.add_argument('--output', required=True)
    replay = commands.add_parser('replay')
    replay.add_argument('--input', required=True)
    replay.add_argument('--output', required=True)
    score = commands.add_parser('evaluate')
    score.add_argument('--input', required=True)
    score.add_argument('--annotations', required=True)
    score.add_argument('--output', required=True)
    args = parser.parse_args()
    if args.command == 'run':
        from .backends import YoloDetector, video_frames, webcam_frames, process
        detector = YoloDetector(args.weights, args.confidence, args.image_size)
        frames = (video_frames(args.video, args.camera_id) if args.video is not None else
                  webcam_frames(args.webcam, args.camera_id, args.max_frames))
        rows = process(frames, detector)
    elif args.command == 'replay':
        tracker = ObservationTracker()
        frames = [ObservationFrame.from_dict(row) for row in read_jsonl(args.input)]
        rows = ({'observation':f.to_dict(), 'tracking':tracker.consume(f),
                 'timing_ms':{}, 'detector':{'kind':'recorded_observation_replay'}} for f in frames)
    else:
        result = evaluate(read_jsonl(args.input), read_jsonl(args.annotations))
        with open(args.output, 'w') as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
            stream.write('\n')
        return
    with open(args.output, 'w') as stream:
        for row in rows:
            stream.write(json.dumps(row, sort_keys=True, allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
