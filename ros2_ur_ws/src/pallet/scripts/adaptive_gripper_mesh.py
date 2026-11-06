"""Plane cut for the proposed visual wrist; source CAD is never modified."""
import numpy as np


def split_pad(triangles, point, normal):
    """Clip both halves, capping a single simple cross-section.

    Reject a different cut topology rather than guessing a surface.
    """
    pieces = [[], []]
    segments = []
    for triangle in triangles:
        distances = (triangle-point)@normal
        intersections = []
        for i in range(3):
            j = (i+1)%3
            if distances[i]*distances[j] < 0:
                intersections.append(triangle[i]+distances[i]/(distances[i]-distances[j])*(triangle[j]-triangle[i]))
        if intersections:
            if len(intersections) != 2:
                raise ValueError('Unexpected pad cut intersection')
            segments.append(intersections)
        for side, sign in enumerate([-1, 1]):
            polygon = []
            for i in range(3):
                j = (i+1)%3
                inside, next_inside = sign*distances[i]>=0, sign*distances[j]>=0
                if inside:
                    polygon.append(triangle[i])
                if inside != next_inside:
                    polygon.append(triangle[i]+distances[i]/(distances[i]-distances[j])*(triangle[j]-triangle[i]))
            for k in range(1,len(polygon)-1):
                pieces[side].append([polygon[0],polygon[k],polygon[k+1]])
    vertices, inverse = np.unique(np.round(np.array(segments).reshape(-1,3),12),axis=0,return_inverse=True)
    adjacency = {i:[] for i in range(len(vertices))}
    for a,b in inverse.reshape(-1,2):
        adjacency[a].append(b); adjacency[b].append(a)
    if not adjacency or any(len(v)!=2 for v in adjacency.values()):
        raise ValueError('Pad cut is not a closed ring')
    order, previous, current = [],None,0
    while current not in order:
        order.append(current)
        next_point = next(v for v in adjacency[current] if v != previous)
        previous,current = current,next_point
    if current != 0 or len(order)!=len(vertices):
        raise ValueError('Pad cut has multiple contours')
    ring = vertices[order]
    tangent = np.array([normal[1],-normal[0],0.])
    flat = np.column_stack([ring@tangent,ring[:,2]])
    def cross(a,b):
        return a[0]*b[1]-a[1]*b[0]

    signed_area = sum(cross(flat[i],flat[(i+1)%len(flat)]) for i in range(len(flat)))
    indices = list(range(len(flat)))
    if signed_area < 0:
        indices.reverse()
    caps = []
    while len(indices)>3:
        found = False
        for k,b in enumerate(indices):
            a,c = indices[k-1],indices[(k+1)%len(indices)]
            turn = cross(flat[b]-flat[a],flat[c]-flat[b])
            if abs(turn)<1e-14:
                # Preserve collinear boundary vertices with a degenerate cap
                # triangle, so the cut boundary remains geometrically sealed.
                caps.append([a,b,c]);indices.pop(k);found=True;break
            if turn < 0:
                continue
            inside = False
            for v in indices:
                if v in (a,b,c):
                    continue
                q = flat[v]
                if (cross(flat[b]-flat[a],q-flat[a])>=-1e-15 and
                    cross(flat[c]-flat[b],q-flat[b])>=-1e-15 and
                    cross(flat[a]-flat[c],q-flat[c])>=-1e-15):
                    inside=True;break
            if not inside:
                caps.append([a,b,c]);indices.pop(k);found=True;break
        if not found:
            raise ValueError('Pad cut cap cannot be triangulated safely')
    caps.append(indices)
    for indices in caps:
        cap = ring[indices]
        if np.dot(np.cross(cap[1]-cap[0],cap[2]-cap[0]),normal)<0:
            cap=cap[[0,2,1]]
        pieces[0].append(cap)
        pieces[1].append(cap[[0,2,1]])
    return tuple(np.array(part) for part in pieces)
