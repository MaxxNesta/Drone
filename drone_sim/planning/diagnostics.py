"""Standalone route plots, independent of terrain, websites and camera models."""
def plot_route(mission, path, rows=()):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure,axis=plt.subplots(figsize=(9,8),layout='constrained')
    boundary=mission.flight_boundary.vertices+mission.flight_boundary.vertices[:1]
    axis.plot(*zip(*boundary),color='red',linestyle='--',label='Allowed flight boundary')
    if mission.survey_area:
        polygon=mission.survey_area.vertices
        axis.fill(*zip(*polygon),color='tab:green',alpha=.1,label='Survey area')
    colors={'transit':'gray','survey_lane':'tab:blue','survey_boundary':'tab:orange','return_home':'tab:purple'}
    seen=set(); labels={}; previous=mission.start_enu_m
    for index,w in enumerate(mission.waypoints):
        p=w.position_enu_m
        axis.plot([previous[0],p[0]],[previous[1],p[1]],color=colors[w.leg_kind],linewidth=1.4,
                  label=w.leg_kind if w.leg_kind not in seen else None)
        if len(mission.waypoints)<=30:
            labels.setdefault(tuple(round(x,6) for x in p[:2]),[]).append(str(index+1))
        seen.add(w.leg_kind); previous=p
    for position,indices in labels.items():
        axis.annotate(','.join(indices),position,fontsize=7,xytext=(4,4),textcoords='offset points')
    if rows:
        points=[r['vehicle']['position_enu_m'] for r in rows]
        axis.plot([p[0] for p in points],[p[1] for p in points],color='black',linestyle=':',linewidth=1,label='Simulated trajectory')
        if rows[-1]['mission_state']=='aborted':
            axis.scatter(*points[-1][:2],marker='x',s=90,color='red',label='Virtual stop')
    axis.scatter(*mission.home_enu_m[:2],marker='*',s=130,color='black',label='Home')
    axis.scatter(*mission.start_enu_m[:2],marker='o',s=35,facecolors='none',edgecolors='black',label='Start')
    axis.set(xlabel='East (m)',ylabel='North (m)',title=mission.mission_id+(' — '+rows[-1]['mission_state'] if rows else ' — nominal plan'))
    axis.set_aspect('equal'); axis.grid(alpha=.25); axis.legend(fontsize=8,loc='best')
    try: figure.savefig(path,dpi=130,metadata={'Software':'Drone Stage 7'})
    finally: plt.close(figure)
