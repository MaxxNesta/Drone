"""Local JSON replay CLI; no transport or hardware integration."""
import argparse
import json
from . import FleetConfig, simulate
from .export import export, plot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--plots', action='store_true')
    args = parser.parse_args()
    with open(args.config) as stream:
        config = FleetConfig.from_dict(json.load(stream))
    rows = list(simulate(config))
    report = export(config, rows, args.output_dir)
    if args.plots:
        from pathlib import Path
        plot(config, rows, Path(args.output_dir)/'fleet.png')
    print(json.dumps({'fleet_id': config.fleet_id, 'status': report['status'],
                      'all_completed': report['all_completed'], 'proximity_episodes': report['proximity_episodes']}))
    return 0 if report['all_completed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
