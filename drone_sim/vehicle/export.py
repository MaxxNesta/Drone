"""Standard-library structured exports; plotting imports remain optional."""
import csv
import json
from pathlib import Path
from .scenario import summarize


def export(scenario, rows, folder):
    folder = Path(folder)
    folder.mkdir(parents=True,exist_ok=False)
    (folder/'config.json').write_text(json.dumps(scenario.to_dict(),indent=2,sort_keys=True,allow_nan=False)+'\n')
    with open(folder/'telemetry.jsonl','w') as stream:
        for row in rows:
            stream.write(json.dumps(row,sort_keys=True,allow_nan=False)+'\n')
    with open(folder/'telemetry.csv','w',newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['schema_version','tick','simulation_time_s','mission_state','completed_waypoints',
                         'east_m','north_m','up_m','ve_mps','vn_mps','vu_mps','ae_mps2','an_mps2','au_mps2',
                         'heading_rad','battery_fraction','waypoint_error_m','low_battery','stop_reason','events_json'])
        for r in rows:
            s = r['vehicle']
            writer.writerow([r['schema_version'],r['tick'],r['simulation_time_s'],r['mission_state'],r['completed_waypoints'],
                *s['position_enu_m'],*s['velocity_enu_mps'],*s['acceleration_enu_mps2'],s['heading_rad'],s['battery_fraction'],
                r['waypoint_error_m'],r['low_battery'],r['stop_reason'],json.dumps(r['events'],sort_keys=True)])
    summary = summarize(rows)
    (folder/'summary.json').write_text(json.dumps(summary,indent=2,sort_keys=True,allow_nan=False)+'\n')
    return summary


def plot(rows, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from .model import norm
    t = [r['simulation_time_s'] for r in rows]
    figure, axes = plt.subplots(4,1,figsize=(10,11),sharex=True,layout='constrained')
    for i,label in enumerate(('East','North','Up')):
        axes[0].plot(t,[r['vehicle']['position_enu_m'][i] for r in rows],label=label)
        axes[1].plot(t,[r['vehicle']['velocity_enu_mps'][i] for r in rows],label=label)
    axes[1].plot(t,[norm(r['vehicle']['velocity_enu_mps']) for r in rows],color='black',linestyle='--',label='Speed')
    axes[2].plot(t,[r['waypoint_error_m'] if r['waypoint_error_m'] is not None else float('nan') for r in rows])
    axes[3].plot(t,[100*r['vehicle']['battery_fraction'] for r in rows])
    for axis,label in zip(axes,('Position (m, ENU)','Velocity (m/s)','Waypoint error (m)','Battery (%)')):
        axis.set_ylabel(label)
        axis.grid(alpha=.3)
        for r in rows:
            for e in r['events']:
                if e['kind'] in ('waypoint_reached','virtual_stop'):
                    axis.axvline(e['time_s'],color='gray',alpha=.3,linewidth=.8)
    axes[0].legend(ncol=3)
    axes[1].legend(ncol=4)
    axes[-1].set_xlabel('Simulation time (s)')
    battery = [100*r['vehicle']['battery_fraction'] for r in rows]
    margin = max(.2,(max(battery)-min(battery))*.1)
    axes[-1].set_ylim(max(0,min(battery)-margin),min(100,max(battery)+margin))
    figure.suptitle('Virtual UAV — simplified point-mass simulation')
    try:
        figure.savefig(path,dpi=130,metadata={'Software':'Drone Stage 6'})
    finally:
        plt.close(figure)
