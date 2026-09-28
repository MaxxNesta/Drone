"""Generate/import a civilian ENU mission, validate, export and optionally simulate."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
from . import PlanningError, Mission, lawnmower, return_home, validate, estimates, sample_coverage, coverage_segments
from .execution import capabilities, execute
from ..vehicle import VehicleConfig
from ..vehicle.scenario import Scenario
from ..vehicle.export import export


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    source=parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--request',help='JSON planner request with operation survey or return_home')
    source.add_argument('--mission',help='Previously exported mission JSON')
    parser.add_argument('--output-dir',required=True)
    parser.add_argument('--vehicle-config',help='Stage 6 vehicle configuration object')
    parser.add_argument('--simulate',action='store_true')
    parser.add_argument('--max-steps',type=int,default=30000)
    parser.add_argument('--plots',action='store_true')
    args=parser.parse_args()
    if args.max_steps<=0: parser.error('--max-steps must be positive')
    if args.request:
        request=json.loads(Path(args.request).read_text())
        operation=request.pop('operation')
        if operation not in ('survey','return_home'): parser.error('Unsupported operation')
        mission=(lawnmower if operation=='survey' else return_home)(**request)
    else:
        mission=Mission.from_dict(json.loads(Path(args.mission).read_text()))
    if args.vehicle_config:
        config=Scenario.from_dict({'vehicle':json.loads(Path(args.vehicle_config).read_text()),'waypoints':[],
                                   'actions':[],'steps':0}).vehicle
    else:
        default=VehicleConfig()
        config=replace(default,initial_state=replace(default.initial_state,position_enu_m=mission.start_enu_m))
    validation=validate(mission,capabilities(config))
    folder=Path(args.output_dir); folder.mkdir(parents=True,exist_ok=False)
    def write(name,value):
        (folder/name).write_text(json.dumps(value,sort_keys=True,indent=2,allow_nan=False)+'\n')
    write('mission.json',mission.to_dict())
    report={'schema_version':1,'validation':validation,
            'validation_scope':'nominal route geometry and capability limits; execution may still fail',
            'coverage_model':'ideal disk swath along nominal survey legs, not measured sensor coverage',
            'estimates':estimates(mission),
            'planned_coverage':sample_coverage(mission.survey_area,coverage_segments(mission),mission.swath_width_m) if mission.survey_area else None,
            'execution':None}
    rows=[]
    if args.simulate and validation['valid']:
        try:
            scenario,rows,result=execute(mission,config,args.max_steps)
        except PlanningError as error:
            report['execution']={'execution_failure':error.code,'final_mission_state':None}
        else:
            export(scenario,rows,folder/'execution')
            report['execution']=result
    write('report.json',report)
    if args.plots:
        from .diagnostics import plot_route
        plot_route(mission,folder/'route.png',rows)
    print(json.dumps(report,sort_keys=True,allow_nan=False))
    if not validation['valid'] or (report['execution'] and report['execution']['execution_failure']):
        raise SystemExit(2)


if __name__ == '__main__': main()
