"""Bounded local-ENU polygon geometry; simple rings only, no GIS dependencies."""
from dataclasses import dataclass
from heapq import heappop, heappush
from math import hypot, isfinite

EPS = 1e-8  # local-meter boundary tolerance, not geodetic accuracy


class PlanningError(ValueError):
    def __init__(self, code):
        self.code = code
        super().__init__(code)


def point(value, dimensions=2):
    if (len(value) != dimensions or any(type(x) not in (int,float) or not isfinite(x) or abs(x)>1e6 for x in value)):
        raise PlanningError('invalid_local_coordinate')
    return tuple(float(x) for x in value)


def distance(a,b):
    return hypot(*(x-y for x,y in zip(a,b)))


def cross(a,b,c):
    return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])


def segment_distance(p,a,b):
    length2=sum((y-x)**2 for x,y in zip(a,b))
    if length2 == 0:
        return distance(p,a)
    t=max(0.,min(1.,sum((x-y)*(z-y) for x,y,z in zip(p,a,b))/length2))
    return distance(p,tuple(x+t*(y-x) for x,y in zip(a,b)))


def intersects(a,b,c,d):
    if any(segment_distance(p,u,v)<=EPS for p,u,v in ((a,c,d),(b,c,d),(c,a,b),(d,a,b))):
        return True
    return cross(a,b,c)*cross(a,b,d)<0 and cross(c,d,a)*cross(c,d,b)<0


@dataclass(frozen=True)
class Polygon:
    vertices: tuple

    def __post_init__(self):
        vertices=[point(v) for v in self.vertices]
        if len(vertices)>1 and vertices[0]==vertices[-1]:
            vertices.pop()  # conventional closed rings are accepted
        if not 3 <= len(vertices) <= 64:
            raise PlanningError('polygon_vertex_count')
        for i,a in enumerate(vertices):
            if any(distance(a,b)<=EPS for b in vertices[i+1:]):
                raise PlanningError('duplicate_polygon_vertex')
        # Normalize redundant straight vertices, but reject boundary backtracking.
        changed=True
        while changed and len(vertices)>=3:
            changed=False
            for i,b in enumerate(vertices):
                a,c=vertices[i-1],vertices[(i+1)%len(vertices)]
                if abs(cross(a,b,c)) <= EPS*max(distance(a,c),1.):
                    if segment_distance(b,a,c)>EPS:
                        raise PlanningError('polygon_backtracking')
                    vertices.pop(i); changed=True; break
        if len(vertices)<3:
            raise PlanningError('degenerate_polygon')
        edges=list(zip(vertices,vertices[1:]+vertices[:1]))
        for i,(a,b) in enumerate(edges):
            for j,(c,d) in enumerate(edges[i+1:],i+1):
                if j==i+1 or (i==0 and j==len(edges)-1):
                    continue
                if intersects(a,b,c,d):
                    raise PlanningError('self_intersecting_polygon')
        signed=sum(a[0]*b[1]-a[1]*b[0] for a,b in edges)/2
        if abs(signed)<=EPS:
            raise PlanningError('degenerate_polygon')
        if signed<0: vertices.reverse()
        first=min(range(len(vertices)),key=lambda i:vertices[i])
        object.__setattr__(self,'vertices',tuple(vertices[first:]+vertices[:first]))

    @property
    def edges(self):
        return tuple(zip(self.vertices,self.vertices[1:]+self.vertices[:1]))

    @property
    def area_m2(self):
        return sum(a[0]*b[1]-a[1]*b[0] for a,b in self.edges)/2

    def contains(self,p):
        p=point(p)
        if any(segment_distance(p,a,b)<=EPS for a,b in self.edges): return True
        inside=False
        for a,b in self.edges:
            if (a[1]>p[1]) != (b[1]>p[1]):
                x=a[0]+(p[1]-a[1])*(b[0]-a[0])/(b[1]-a[1])
                if p[0]<x: inside=not inside
        return inside

    def contains_segment(self,a,b):
        a,b=point(a),point(b)
        if not self.contains(a) or not self.contains(b): return False
        if distance(a,b)<=EPS: return True
        dx,dy=b[0]-a[0],b[1]-a[1]
        cuts=[0.,1.]
        for c,d in self.edges:
            ex,ey=d[0]-c[0],d[1]-c[1]
            denominator=dx*ey-dy*ex
            if abs(denominator)>EPS:
                t=((c[0]-a[0])*ey-(c[1]-a[1])*ex)/denominator
                u=((c[0]-a[0])*dy-(c[1]-a[1])*dx)/denominator
                if -EPS<=t<=1+EPS and -EPS<=u<=1+EPS: cuts.append(max(0.,min(1.,t)))
            elif segment_distance(c,a,b)<=EPS or segment_distance(d,a,b)<=EPS:
                length2=dx*dx+dy*dy
                cuts.extend(max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/length2)) for p in (c,d))
        cuts=sorted(set(cuts))
        return all(self.contains((a[0]+dx*(lo+hi)/2,a[1]+dy*(lo+hi)/2)) for lo,hi in zip(cuts,cuts[1:]))


def route_inside(boundary,start,end):
    """Shortest visibility-graph polyline, deterministic ties; no obstacles/holes."""
    start,end=point(start),point(end)
    if not boundary.contains(start) or not boundary.contains(end):
        raise PlanningError('route_endpoint_outside_boundary')
    if boundary.contains_segment(start,end): return (end,)
    nodes=(start,end)+boundary.vertices
    edges={i:[] for i in range(len(nodes))}
    for i,a in enumerate(nodes):
        for j in range(i+1,len(nodes)):
            if boundary.contains_segment(a,nodes[j]):
                cost=distance(a,nodes[j]); edges[i].append((j,cost)); edges[j].append((i,cost))
    queue=[(0.,(0,),0)]; best={0:(0.,(0,))}
    while queue:
        cost,path,node=heappop(queue)
        if best[node]!=(cost,path): continue
        if node==1: return tuple(nodes[i] for i in path[1:])
        for neighbor,weight in edges[node]:
            if neighbor in path: continue
            candidate=(cost+weight,path+(neighbor,))
            if neighbor not in best or candidate<best[neighbor]:
                best[neighbor]=candidate; heappush(queue,(*candidate,neighbor))
    raise PlanningError('no_route_inside_boundary')
