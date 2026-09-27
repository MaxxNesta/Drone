"""Create unfilled labeling templates or strictly convert reviewed manual labels."""
import argparse
import json
from pathlib import Path
from .records import Observation, ObservationFrame
from .backends import fingerprint, video_frames


def make_template(rows, sequences):
    selected = set(sequences)
    if not selected or any(type(x) is not int or x < 0 for x in selected):
        raise ValueError('Select nonnegative frame sequences')
    frames = []
    for row in rows:
        f = ObservationFrame.from_dict(row['observation'])
        if f.frame_sequence in selected:
            metadata = f.to_dict()
            del metadata['observations']
            frames.append({'metadata':metadata,'reviewed':False,'objects':None})
    if len(frames) != len(selected):
        raise ValueError('Missing or duplicate requested frames')
    return {'schema_version':1,'annotation_kind':'manual_template_not_ground_truth','frames':frames}


def convert(template):
    if template.get('schema_version') != 1 or not template.get('frames'):
        raise ValueError('Unsupported or empty template')
    rows = []
    keys = set()
    for item in template['frames']:
        if item.get('reviewed') is not True or not isinstance(item.get('objects'),list):
            raise ValueError('Every selected frame needs explicit manual review; null is not an empty scene')
        observations, identities = [], []
        for o in item['objects']:
            b = tuple(o['bbox_xyxy'])
            if len(b) != 4 or type(o['class_id']) is not int:
                raise ValueError('Manual box and class required')
            identities.append(o['truth_id'])
            observations.append(Observation(((b[0]+b[2])/2,(b[1]+b[3])/2),b,None,o['class_id']))
        if len(set(identities)) != len(identities) or any(not isinstance(x,str) or not x for x in identities):
            raise ValueError('Unique nonempty instance IDs required')
        f = ObservationFrame.from_dict({**item['metadata'],'observations':[
            {'pixel':o.pixel,'bbox_xyxy':o.bbox_xyxy,'confidence':None,'class_id':o.class_id} for o in observations]})
        key = (f.source_id,f.camera_id,f.frame_sequence)
        if key in keys:
            raise ValueError('Duplicate labeled frame')
        keys.add(key)
        rows.append({'observation':f.to_dict(),'truth_ids':identities})
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='command',required=True)
    prepare = sub.add_parser('prepare')
    prepare.add_argument('--telemetry',required=True)
    prepare.add_argument('--video',required=True)
    prepare.add_argument('--frames',required=True,help='Comma-separated original frame sequences')
    prepare.add_argument('--output-dir',required=True)
    complete = sub.add_parser('convert')
    complete.add_argument('--template',required=True)
    complete.add_argument('--output',required=True)
    args = p.parse_args()
    if args.command == 'convert':
        rows = convert(json.loads(Path(args.template).read_text()))
        with open(args.output,'x') as stream:
            for row in rows:
                stream.write(json.dumps(row,sort_keys=True)+'\n')
        return
    with open(args.telemetry) as stream:
        rows = [json.loads(line) for line in stream if line.strip()]
    template = make_template(rows,[int(x) for x in args.frames.split(',')])
    source = fingerprint(args.video)
    if any(f['metadata']['source_id'] != source for f in template['frames']):
        raise ValueError('Video fingerprint does not match telemetry')
    folder = Path(args.output_dir)
    folder.mkdir(parents=True,exist_ok=False)
    selected = {f['metadata']['frame_sequence']:f['metadata'] for f in template['frames']}
    from PIL import Image
    frames = video_frames(args.video,template['frames'][0]['metadata']['camera_id'])
    try:
        for record,image in frames:
            if record.frame_sequence in selected:
                metadata = selected.pop(record.frame_sequence)
                if record.capture_time_s != metadata['capture_time_s'] or list(record.image_size) != list(metadata['image_size']):
                    raise ValueError('Decoded frame does not match telemetry')
                Image.fromarray(image[:,:,::-1]).save(folder/f'frame-{record.frame_sequence:06d}.png')
            if not selected:
                break
    finally:
        frames.close()
    if selected:
        raise ValueError('Selected frames were not decoded')
    (folder/'template.json').write_text(json.dumps(template,indent=2)+'\n')


if __name__ == '__main__':
    main()
