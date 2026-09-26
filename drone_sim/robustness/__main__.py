"""Run reproducible JSON-configured Stage 3 failure experiments."""
import argparse
import json
from pathlib import Path
from .config import RobustnessConfig
from .runner import run


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    source=parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--config',type=Path)
    source.add_argument('--suite',type=Path,help='Run all JSON configurations in this directory')
    parser.add_argument('--output',type=Path,help='JSONL file; defaults to stdout')
    parser.add_argument('--summary-only',action='store_true')
    args=parser.parse_args()
    try:
        paths=sorted(args.suite.glob('*.json')) if args.suite else [args.config]
        if not paths:
            raise ValueError('Scenario suite contains no JSON configurations')
        configs=[RobustnessConfig.from_dict(json.loads(path.read_text())) for path in paths]
    except (OSError,ValueError,TypeError,KeyError) as error:
        parser.error(str(error))
    def write(stream):
        for config in configs:
            for row in run(config):
                if not args.summary_only or row['type'] in ('configuration','summary'):
                    stream.write(json.dumps(row,sort_keys=True,allow_nan=False)+'\n')
    if args.output:
        with args.output.open('w',encoding='utf-8') as stream:
            write(stream)
    else:
        import sys
        write(sys.stdout)


if __name__ == '__main__':
    main()
