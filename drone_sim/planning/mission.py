"""Mission data and capability validation. No vehicle physics or mission state machine."""
from dataclasses import dataclass, field, asdict
import json
from math import isfinite, sqrt
from .geometry import Polygon, PlanningError, point, distance, EPS


def positive(value, name, zero=False):
    if type(value) not in (int,float) or not isfinite(value) or (value<0 if zero else value<=0):
        raise PlanningError('invalid_'+name)


@dataclass(frozen=True)
class RouteWaypoint:
    position_enu_m: tuple
    leg_kind: str = 'transit'  # describes incoming leg

    def __post_init__(self):
        object.__setattr__(self,'position_enu_m',point(self.position_enu_m,3))
        if self.leg_kind not in ('transit','survey_lane','survey_boundary','return_home'):
            raise PlanningError('unknown_leg_kind')


@dataclass(frozen=True)
class Mission:
    mission_id: str
    home_enu_m: tuple
    start_enu_m: tuple
    altitude_m: float
    speed_limit_mps: float
    acceleration_limit_mps2: float
    flight_boundary: Polygon
    waypoints: tuple
    survey_area: Polygon = None
    swath_width_m: float = None
    lane_spacing_m: float = None
    orientation_deg: float = None  # north-zero, clockwise towards east
    settle_s: float = .5
    position_tolerance_m: float = .1
    speed_tolerance_mps: float = .1
    settling_allowance_s: float = 2.
    metadata: dict = field(default_factory=dict)
    schema_version: int = 1
    coordinate_frame: str = 'ENU'

    def __post_init__(self):
        if type(self.schema_version) is not int or self.schema_version!=1 or self.coordinate_frame!='ENU' or not isinstance(self.mission_id,str) or not self.mission_id:
            raise PlanningError('invalid_mission_header')
        object.__setattr__(self,'home_enu_m',point(self.home_enu_m,3))
        object.__setattr__(self,'start_enu_m',point(self.start_enu_m,3))
        if type(self.altitude_m) not in (int,float) or not isfinite(self.altitude_m) or abs(self.altitude_m)>1e6: raise PlanningError('invalid_altitude')
        for name in ('speed_limit_mps','acceleration_limit_mps2','position_tolerance_m','speed_tolerance_mps'):
            positive(getattr(self,name),name)
        for name in ('settle_s','settling_allowance_s'): positive(getattr(self,name),name,True)
        if not 1 <= len(self.waypoints) <= 2000: raise PlanningError('waypoint_count')
        object.__setattr__(self,'waypoints',tuple(self.waypoints))
        if not isinstance(self.metadata,dict): raise PlanningError('metadata_must_be_object')
        try: object.__setattr__(self,'metadata',json.loads(json.dumps(self.metadata,allow_nan=False)))
        except (TypeError,ValueError): raise PlanningError('invalid_metadata')
        if self.survey_area is not None:
            positive(self.swath_width_m,'swath_width')
            positive(self.lane_spacing_m,'lane_spacing')
            if self.lane_spacing_m>self.swath_width_m: raise PlanningError('lane_spacing_exceeds_swath')
            if type(self.orientation_deg) not in (int,float) or not isfinite(self.orientation_deg): raise PlanningError('invalid_orientation')

    @classmethod
    def from_dict(cls, data):
        data=dict(data)
        data['flight_boundary']=Polygon(tuple(data['flight_boundary']))
        if data.get('survey_area') is not None: data['survey_area']=Polygon(tuple(data['survey_area']))
        data['waypoints']=tuple(RouteWaypoint(**w) for w in data['waypoints'])
        return cls(**data)

    def to_dict(self):
        data=asdict(self)
        data['flight_boundary']=self.flight_boundary.vertices
        data['survey_area']=self.survey_area.vertices if self.survey_area else None
        return data


@dataclass(frozen=True)
class Capabilities:
    max_speed_mps: float
    max_acceleration_mps2: float
    geofence_minimum_enu_m: tuple
    geofence_maximum_enu_m: tuple

    def __post_init__(self):
        positive(self.max_speed_mps,'max_speed'); positive(self.max_acceleration_mps2,'max_acceleration')
        object.__setattr__(self,'geofence_minimum_enu_m',point(self.geofence_minimum_enu_m,3))
        object.__setattr__(self,'geofence_maximum_enu_m',point(self.geofence_maximum_enu_m,3))
        if any(a>=b for a,b in zip(self.geofence_minimum_enu_m,self.geofence_maximum_enu_m)):
            raise PlanningError('invalid_capability_geofence')


def validate(mission, capabilities):
    reasons=[]
    if mission.speed_limit_mps>capabilities.max_speed_mps: reasons.append('speed_exceeds_vehicle')
    if mission.acceleration_limit_mps2>capabilities.max_acceleration_mps2: reasons.append('acceleration_exceeds_vehicle')
    positions=[mission.home_enu_m,mission.start_enu_m]+[w.position_enu_m for w in mission.waypoints]
    for p in positions:
        if not all(a<=x<=b for a,x,b in zip(capabilities.geofence_minimum_enu_m,p,capabilities.geofence_maximum_enu_m)):
            reasons.append('point_outside_vehicle_geofence')
        if not mission.flight_boundary.contains(p[:2]): reasons.append('point_outside_flight_boundary')
    if not capabilities.geofence_minimum_enu_m[2]<=mission.altitude_m<=capabilities.geofence_maximum_enu_m[2]:
        reasons.append('altitude_outside_vehicle_geofence')
    if mission.survey_area:
        from .routes import coverage_structure_valid
        if not coverage_structure_valid(mission): reasons.append('incomplete_survey_route')
        for p in mission.survey_area.vertices:
            if not all(a<=x<=b for a,x,b in zip(capabilities.geofence_minimum_enu_m,(*p,mission.altitude_m),capabilities.geofence_maximum_enu_m)):
                reasons.append('survey_outside_vehicle_geofence')
        for a,b in mission.survey_area.edges:
            if not mission.flight_boundary.contains_segment(a,b): reasons.append('survey_outside_flight_boundary')
    previous=mission.start_enu_m
    for w in mission.waypoints:
        p=w.position_enu_m
        if distance(previous,p)<=EPS: reasons.append('zero_length_leg')
        if not mission.flight_boundary.contains_segment(previous[:2],p[:2]): reasons.append('leg_outside_flight_boundary')
        if w.leg_kind.startswith('survey_'):
            if mission.survey_area is None or not mission.survey_area.contains_segment(previous[:2],p[:2]):
                reasons.append('invalid_survey_leg')
            if abs(previous[2]-mission.altitude_m)>EPS or abs(p[2]-mission.altitude_m)>EPS:
                reasons.append('survey_leg_wrong_altitude')
        previous=p
    return {'valid':not reasons,'reasons':sorted(set(reasons))}


def estimates(mission):
    previous=mission.start_enu_m
    total=coverage=seconds=0.
    v,a=mission.speed_limit_mps,mission.acceleration_limit_mps2
    for w in mission.waypoints:
        d=distance(previous,w.position_enu_m)
        total+=d
        if w.leg_kind.startswith('survey_'): coverage+=d
        seconds+=(2*sqrt(d/a) if d<v*v/a else d/v+v/a)+mission.settle_s+mission.settling_allowance_s
        previous=w.position_enu_m
    return {'route_distance_m':total,'survey_distance_m':coverage,'transit_distance_m':total-coverage,
            'cruise_only_time_s':total/v,'approximate_duration_s':seconds,
            'duration_model':'rest-to-rest acceleration-limited legs + configured dwell/settling allowance; excludes drag/PD lag'}
