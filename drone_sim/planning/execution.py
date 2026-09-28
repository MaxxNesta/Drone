"""Thin adapter only: all dynamics, mission states and stopping remain owned by Stage 6."""
from copy import deepcopy
from dataclasses import replace
from ..vehicle import VehicleConfig, VehicleSimulator, Waypoint
from ..vehicle.scenario import Scenario, Action, summarize
from .geometry import PlanningError, distance, EPS
from .mission import Capabilities, validate, estimates


def capabilities(config):
    return Capabilities(config.limits.max_speed_mps,config.limits.max_acceleration_mps2,
                        config.geofence.minimum_enu_m,config.geofence.maximum_enu_m)


def to_scenario(mission, vehicle=None, max_steps=30000):
    vehicle=vehicle or VehicleConfig()
    result=validate(mission,capabilities(vehicle))
    if not result['valid']: raise PlanningError(','.join(result['reasons']))
    if distance(vehicle.initial_state.position_enu_m,mission.start_enu_m)>EPS:
        raise PlanningError('initial_position_does_not_match_plan')
    if distance(vehicle.initial_state.velocity_enu_mps,(0,0,0))>EPS:
        raise PlanningError('stationary_start_required')
    if vehicle.initial_state.battery_fraction<=vehicle.battery.critical_fraction:
        raise PlanningError('critical_battery_no_start')
    try:
        limited=replace(vehicle,limits=replace(vehicle.limits,max_speed_mps=mission.speed_limit_mps,
                                              max_acceleration_mps2=mission.acceleration_limit_mps2))
    except ValueError as error:
        raise PlanningError('initial_state_incompatible_with_mission_limits') from error
    waypoints=tuple(Waypoint(w.position_enu_m,mission.position_tolerance_m,
                            mission.speed_tolerance_mps,mission.settle_s) for w in mission.waypoints)
    return Scenario(mission.mission_id,limited,waypoints,(Action(0,'start'),),max_steps)


def execute(mission, vehicle=None, max_steps=30000):
    """Preview each existing-engine step on a copy; stop before crossing the polygon.

    No new lifecycle or integrator. On rejection call Stage 6 emergency_stop at the
    last valid state. Already aborted simulators are never restarted by this API.
    """
    scenario=to_scenario(mission,vehicle,max_steps)
    sim=VehicleSimulator(scenario.waypoints,scenario.vehicle)
    rows=[sim.telemetry(consume_events=True)]
    sim.start()
    failure=None
    for _ in range(max_steps):
        candidate=deepcopy(sim)
        row=candidate.step(sim.clock.advance())
        if not mission.flight_boundary.contains_segment(sim.state.position_enu_m[:2],candidate.state.position_enu_m[:2]):
            failure='predicted_flight_boundary_crossing'
            sim.emergency_stop()
            row=sim.step(sim.clock.advance())
        else:
            sim=candidate
        rows.append(row)
        if sim.status in ('completed','aborted'): break
    if sim.status!='completed' and failure is None:
        failure=sim.stop_reason or 'step_budget_exhausted'
    result=summarize(rows)
    estimate=estimates(mission)
    result.update({'execution_failure':failure,'planning_estimates':estimate,
                   'duration_error_s':result['completion_time_s']-estimate['approximate_duration_s'] if sim.status=='completed' else None})
    replay_actions=scenario.actions
    if failure=='predicted_flight_boundary_crossing':
        replay_actions += (Action(sim.clock.tick-1,'emergency_stop'),)
    replay=replace(scenario,steps=sim.clock.tick,actions=replay_actions)
    return replay,rows,result
