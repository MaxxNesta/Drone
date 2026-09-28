"""Deterministic lawnmower and virtual return-to-home polyline generation."""
from math import ceil, sin, cos, radians, isfinite
from .geometry import EPS, PlanningError, Polygon, point, distance, route_inside
from .mission import Mission, RouteWaypoint, positive


def lane_segments(area, spacing, orientation_deg):
    positive(spacing,'lane_spacing')
    if type(orientation_deg) not in (int,float) or not isfinite(orientation_deg):
        raise PlanningError('invalid_orientation')
    angle=radians(orientation_deg%360)
    sine,cosine=sin(angle),cos(angle)
    sine=0. if abs(sine)<1e-14 else sine
    cosine=0. if abs(cosine)<1e-14 else cosine
    direction=(sine,cosine); normal=(cosine,-sine)
    projected=[(sum(x*y for x,y in zip(p,direction)),sum(x*y for x,y in zip(p,normal))) for p in area.vertices]
    low,high=min(p[1] for p in projected),max(p[1] for p in projected)
    ratio=(high-low)/spacing
    if ratio>1000+1e-12: raise PlanningError('too_many_lanes')
    count=max(1,ceil(ratio-1e-12))
    pitch=(high-low)/count
    def world(x,y): return (direction[0]*x+normal[0]*y,direction[1]*x+normal[1]*y)
    segments=[]
    for lane in range(count):
        y=low+(lane+.5)*pitch
        cuts=[]
        for a,b in zip(projected,projected[1:]+projected[:1]):
            if (a[1]<=y<b[1]) or (b[1]<=y<a[1]):
                cuts.append(a[0]+(y-a[1])*(b[0]-a[0])/(b[1]-a[1]))
        cuts.sort()
        if len(cuts)%2: raise PlanningError('ambiguous_scanline')
        intervals=[(a,b) for a,b in zip(cuts[::2],cuts[1::2]) if b-a>EPS]
        if lane%2: intervals=[(b,a) for a,b in reversed(intervals)]
        segments.extend((world(a,y),world(b,y)) for a,b in intervals)
    if not segments: raise PlanningError('no_survey_lanes')
    return tuple(segments)


class _Builder:
    def __init__(self,start,boundary,altitude):
        self.current=point(start,3); self.boundary=boundary; self.altitude=altitude; self.waypoints=[]
        if not boundary.contains(self.current[:2]): raise PlanningError('start_outside_boundary')

    def append(self,p,kind='transit'):
        p=point(p,3)
        if distance(self.current,p)>EPS:
            self.waypoints.append(RouteWaypoint(p,kind)); self.current=p
        if len(self.waypoints)>2000: raise PlanningError('too_many_waypoints')

    def horizontal_to(self,p,kind='transit'):
        for intermediate in route_inside(self.boundary,self.current[:2],p):
            self.append((*intermediate,self.altitude),kind)


def lawnmower(area, home, altitude_m=5., speed_limit_mps=3., acceleration_limit_mps2=2.,
              lane_spacing_m=4., orientation_deg=90., swath_width_m=4., flight_boundary=None,
              mission_id='survey', metadata=None, **mission_options):
    if not isinstance(area,Polygon): area=Polygon(tuple(area))
    boundary=flight_boundary or area
    if not isinstance(boundary,Polygon): boundary=Polygon(tuple(boundary))
    home=point(home,3)
    positive(swath_width_m,'swath_width'); positive(lane_spacing_m,'lane_spacing')
    if lane_spacing_m>swath_width_m: raise PlanningError('lane_spacing_exceeds_swath')
    if not isfinite(altitude_m): raise PlanningError('invalid_altitude')
    if any(not boundary.contains_segment(a,b) for a,b in area.edges):
        raise PlanningError('survey_outside_flight_boundary')
    builder=_Builder(home,boundary,altitude_m)
    builder.append((*home[:2],altitude_m))
    for a,b in lane_segments(area,lane_spacing_m,orientation_deg):
        builder.horizontal_to(a)
        builder.append((*b,altitude_m),'survey_lane')
    # Required perimeter pass closes ideal footprint gaps at sloped/concave edges.
    builder.horizontal_to(area.vertices[0])
    for p in area.vertices[1:]+area.vertices[:1]: builder.append((*p,altitude_m),'survey_boundary')
    builder.horizontal_to(home[:2],'return_home')
    builder.append(home,'return_home')
    return Mission(mission_id,home,home,altitude_m,speed_limit_mps,acceleration_limit_mps2,boundary,
                   tuple(builder.waypoints),area,swath_width_m,lane_spacing_m,orientation_deg%360,
                   metadata={} if metadata is None else metadata,**mission_options)


def return_home(current, home, flight_boundary, altitude_m, speed_limit_mps=3., acceleration_limit_mps2=2.,
                mission_id='virtual-return-home', metadata=None, **mission_options):
    current,home=point(current,3),point(home,3)
    if not isinstance(flight_boundary,Polygon): flight_boundary=Polygon(tuple(flight_boundary))
    if not isfinite(altitude_m) or altitude_m<max(current[2],home[2]):
        raise PlanningError('return_altitude_below_endpoints')
    if distance(current,home)<=EPS: raise PlanningError('already_at_home')
    if not flight_boundary.contains(home[:2]): raise PlanningError('home_outside_boundary')
    builder=_Builder(current,flight_boundary,altitude_m)
    builder.append((*current[:2],altitude_m),'return_home')
    builder.horizontal_to(home[:2],'return_home')
    builder.append(home,'return_home')
    return Mission(mission_id,home,current,altitude_m,speed_limit_mps,acceleration_limit_mps2,
                   flight_boundary,tuple(builder.waypoints),metadata={} if metadata is None else metadata,**mission_options)


def coverage_segments(mission):
    previous=mission.start_enu_m; segments=[]
    for w in mission.waypoints:
        if w.leg_kind.startswith('survey_'): segments.append((previous[:2],w.position_enu_m[:2]))
        previous=w.position_enu_m
    return tuple(segments)


def coverage_structure_valid(mission):
    """Require all generated lanes and polygon edges, rather than trusting sample coverage."""
    actual=coverage_segments(mission)
    expected=lane_segments(mission.survey_area,mission.lane_spacing_m,mission.orientation_deg)+mission.survey_area.edges
    return all(any((distance(a,c)<=EPS and distance(b,d)<=EPS) or
                   (distance(a,d)<=EPS and distance(b,c)<=EPS) for c,d in actual) for a,b in expected)


def sample_coverage(area, segments, swath_width_m, sample_spacing_m=.5):
    """Finite diagnostic samples (grid plus vertices); not a proof of real sensor coverage."""
    from .geometry import segment_distance
    positive(sample_spacing_m,'sample_spacing'); positive(swath_width_m,'swath_width')
    low=[min(p[i] for p in area.vertices) for i in (0,1)]
    high=[max(p[i] for p in area.vertices) for i in (0,1)]
    ratios=[(b-a)/sample_spacing_m for a,b in zip(low,high)]
    if any(r>100000 for r in ratios): raise PlanningError('too_many_coverage_samples')
    counts=[max(1,ceil(r)) for r in ratios]
    if (counts[0]+1)*(counts[1]+1)>100000: raise PlanningError('too_many_coverage_samples')
    samples=set(area.vertices)
    for i in range(counts[0]+1):
        for j in range(counts[1]+1):
            p=(low[0]+i*(high[0]-low[0])/counts[0],low[1]+j*(high[1]-low[1])/counts[1])
            if area.contains(p): samples.add(p)
    distances=[min((segment_distance(p,a,b) for a,b in segments),default=float('inf')) for p in sorted(samples)]
    covered=sum(d<=swath_width_m/2+EPS for d in distances)
    return {'samples':len(samples),'covered_samples':covered,'covered_fraction':covered/len(samples),
            'maximum_distance_to_route_m':max(distances) if segments else None,
            'sample_spacing_m':sample_spacing_m,'assumed_swath_width_m':swath_width_m}
