"""Advisory geometry only. No avoidance, scheduling or vehicle mutation."""
from itertools import combinations
from math import sqrt
from ..planning.geometry import distance, segment_distance


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x-y for x, y in zip(a, b))


def segment_separation(a, b, c, d):
    """Minimum 3D separation between closed segments, including zero lengths."""
    minimum = min(segment_distance(a, c, d), segment_distance(b, c, d),
                  segment_distance(c, a, b), segment_distance(d, a, b))
    u, v, w = sub(b, a), sub(d, c), sub(a, c)
    uu, uv, vv, uw, vw = dot(u, u), dot(u, v), dot(v, v), dot(u, w), dot(v, w)
    denominator = uu*vv-uv*uv
    if denominator > 1e-14*uu*vv:
        s, t = (uv*vw-vv*uw)/denominator, (uu*vw-uv*uw)/denominator
        if 0 <= s <= 1 and 0 <= t <= 1:
            minimum = min(minimum, sqrt(sum((w[i]+s*u[i]-t*v[i])**2 for i in range(3))))
    return minimum


def route_conflicts(config):
    """One minimum-distance advisory per pair; ignores launch times and speed."""
    alerts = []
    for left, right in combinations(config.members, 2):
        a = [left.mission.start_enu_m]+[w.position_enu_m for w in left.mission.waypoints]
        b = [right.mission.start_enu_m]+[w.position_enu_m for w in right.mission.waypoints]
        closest = min((segment_separation(p, q, r, s), i, j)
                      for i, (p, q) in enumerate(zip(a, a[1:]))
                      for j, (r, s) in enumerate(zip(b, b[1:])))
        if closest[0] <= config.route_conflict_m:
            alerts.append({'vehicle_ids': [left.vehicle_id, right.vehicle_id],
                           'minimum_route_separation_m': closest[0], 'leg_indices': list(closest[1:]),
                           'threshold_m': config.route_conflict_m, 'advisory_only': True,
                           'kind': 'spatial_route_conflict'})
    return alerts


def proximity(previous, current, threshold):
    """Synchronized linear sweep between accepted samples, not independent segments."""
    alerts = []
    for a, b in combinations(sorted(current), 2):
        start = sub(previous[a], previous[b])
        end = sub(current[a], current[b])
        change = sub(end, start)
        size = dot(change, change)
        fraction = max(0., min(1., -dot(start, change)/size)) if size else 0.
        separation = sqrt(sum((x+fraction*y)**2 for x, y in zip(start, change)))
        if separation <= threshold:
            alerts.append({'kind': 'simulated_proximity', 'vehicle_ids': [a, b],
                           'minimum_separation_m': separation, 'end_separation_m': distance(current[a], current[b]),
                           'closest_tick_fraction': fraction, 'threshold_m': threshold, 'advisory_only': True})
    return alerts
