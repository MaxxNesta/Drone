"""Run a deterministic virtual waypoint scenario and export telemetry/diagnostics."""
import argparse
import json
from pathlib import Path
from .scenario import Scenario, simulate
from .export import export, plot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',required=True)
    parser.add_argument('--output-dir',required=True)
    parser.add_argument('--plots',action='store_true',help='Optional Matplotlib PNG diagnostics')
    args = parser.parse_args()
    scenario = Scenario.from_dict(json.loads(Path(args.config).read_text()))
    rows = list(simulate(scenario))
    summary = export(scenario,rows,args.output_dir)
    if args.plots:
        plot(rows,Path(args.output_dir)/'diagnostics.png')
    print(json.dumps(summary,sort_keys=True,allow_nan=False))


if __name__ == '__main__':
    main()
