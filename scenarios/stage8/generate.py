"""Regenerate expanded Stage 8 fixtures: python3 -m scenarios.stage8.generate."""
import json
from dataclasses import replace
from pathlib import Path
from drone_sim.fleet import Member, FleetConfig, FleetAction
from drone_sim.planning import Mission, RouteWaypoint, Polygon, lawnmower
from drone_sim.vehicle import VehicleConfig, VehicleState
from drone_sim.vehicle.model import BatteryConfig

BOUNDARY = Polygon(((-40,-40),(40,-40),(40,40),(-40,40)))


def transit(key, start, finish, battery=None):
    mission = Mission(key, start, start, finish[2], 3, 2, BOUNDARY, (RouteWaypoint(finish),))
    config = VehicleConfig(initial_state=VehicleState(start))
    if battery is not None:
        config = replace(config, battery=battery)
    return Member(key, mission, config)


def scenarios():
    surveys = []
    for key, offset in (('west',-15),('east',5)):
        area = Polygon(((offset,0),(offset+6,0),(offset+6,4),(offset,4)))
        home = (offset-2,-2,2)
        mission = lawnmower(area,home,flight_boundary=BOUNDARY,altitude_m=4,
                            lane_spacing_m=2,swath_width_m=2,mission_id=key+'-survey')
        surveys.append(Member(key,mission,VehicleConfig(initial_state=VehicleState(home))))
    yield FleetConfig('independent-surveys',tuple(surveys),steps=5000)
    cross = (transit('eastbound',(-5,0,2),(5,0,2)),transit('northbound',(0,-5,2),(0,5,2)))
    yield FleetConfig('crossing',cross,steps=700)
    yield FleetConfig('staggered-crossing',cross,steps=1300,
                      actions=(FleetAction(0,'start','eastbound'),FleetAction(600,'start','northbound')))
    separate = (transit('alpha',(-5,-10,2),(5,-10,2)),transit('bravo',(-5,10,2),(5,10,2)))
    yield FleetConfig('fleet-lifecycle',separate,steps=900,
                      actions=(FleetAction(0,'start'),FleetAction(50,'pause'),
                               FleetAction(100,'resume','alpha'),FleetAction(150,'resume')))
    drained = replace(separate[0], vehicle=replace(separate[0].vehicle,battery=BatteryConfig(idle_drain_per_s=1)))
    yield FleetConfig('one-battery-failure',(drained,separate[1]),steps=700)
    area = Polygon(((0,0),(12,0),(12,4),(4,4),(4,12),(0,12)))
    mission = lawnmower(area,(1,1,2),altitude_m=4,lane_spacing_m=3,swath_width_m=3,
                        mission_id='concave-survey')
    concave = Member('concave',mission,VehicleConfig(initial_state=VehicleState(mission.start_enu_m)))
    yield FleetConfig('one-boundary-failure',(concave,separate[0]),steps=2700)


if __name__ == '__main__':
    for config in scenarios():
        (Path(__file__).parent/(config.fleet_id+'.json')).write_text(json.dumps(config.to_dict(),indent=2,sort_keys=True)+'\n')
