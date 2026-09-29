"""Deterministic fleet and per-vehicle numeric exports; optional static diagnostics."""
import json
from pathlib import Path
from dataclasses import replace
from ..planning.execution import to_scenario
from ..vehicle.scenario import Action
from ..vehicle.export import export as export_vehicle
from .engine import summarize


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False)+'\n')


def export(config, rows, folder):
    ids = {m.vehicle_id for m in config.members}
    if len(rows) != config.steps+1:
        raise ValueError('Export requires the complete configured run')
    for tick, row in enumerate(rows):
        if (row['tick'] != tick or row['dt_s'] != config.dt_s or row['fleet_id'] != config.fleet_id
                or set(row['vehicles']) != ids
                or any(r['tick'] != tick or r['dt_s'] != config.dt_s for r in row['vehicles'].values())):
            raise ValueError('Export requires synchronized matching fleet records')
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=False)
    write_json(folder/'config.json', config.to_dict())
    report = summarize(config, rows)
    write_json(folder/'summary.json', report)
    with (folder/'fleet.jsonl').open('w') as stream:
        for row in rows:
            stream.write(json.dumps(row, sort_keys=True, allow_nan=False)+'\n')
    for member in config.members:
        key = member.vehicle_id
        selected = [r['vehicles'][key] for r in rows]
        # Record effective commands, including any polygon stop, for Stage 6 replay.
        actions = []
        for row in rows:
            for command in row['commands']:
                if key in command['affected_vehicle_ids']:
                    actions.append(Action(command['tick'], command['command']))
            for event in row['vehicles'][key]['events']:
                if event['kind'] == 'virtual_stop' and event['reason'] == 'emergency_stop' and row['vehicles'][key]['fleet_failure']:
                    actions.append(Action(event['tick'], 'emergency_stop'))
        scenario = replace(to_scenario(member.mission, member.vehicle, config.steps), actions=tuple(actions))
        export_vehicle(scenario, selected, folder/'vehicles'/key)
        write_json(folder/'vehicles'/key/'mission.json', member.mission.to_dict())
    return report


def plot(config, rows, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(1, 2, figsize=(12, 5), layout='constrained')
    for member in config.members:
        key = member.vehicle_id
        points = [r['vehicles'][key]['vehicle']['position_enu_m'] for r in rows]
        line, = axes[0].plot([p[0] for p in points], [p[1] for p in points], label=key)
        axes[0].scatter(*points[0][:2], marker='o', color=line.get_color())
        axes[0].scatter(*member.mission.home_enu_m[:2], marker='*', color=line.get_color())
        axes[1].plot([r['simulation_time_s'] for r in rows],
                     [100*r['vehicles'][key]['vehicle']['battery_fraction'] for r in rows], label=key)
    alerts = [r for r in rows if r['proximity']]
    for row in alerts:
        axes[1].axvline(row['simulation_time_s'], color='red', alpha=.03)
    axes[0].set(xlabel='East (m)', ylabel='North (m)', title='Accepted paths; dots = launch, stars = home')
    axes[0].set_aspect('equal')
    axes[1].set(xlabel='Simulation time (s)', ylabel='Battery (%)', title='Red intervals: advisory proximity')
    for axis in axes:
        axis.legend(); axis.grid(alpha=.3)
    figure.suptitle(config.fleet_id+' — virtual fleet (no avoidance)')
    try:
        figure.savefig(path, dpi=120)
    finally:
        plt.close(figure)
