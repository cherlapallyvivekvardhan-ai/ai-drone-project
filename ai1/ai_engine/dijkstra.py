"""Dijkstra baseline on the same 3D occupancy grid."""
from heapq import heappush, heappop
from math import sqrt
from .astar import MOVES

def dijkstra(occupied, start, goal, size):
    if start in occupied or goal in occupied:
        return None, 0
    q = [(0.0, start)]
    dist = {start: 0.0}
    came = {}
    expanded = 0

    def inside(p):
        return 0 <= p[0] < size[0] and 0 <= p[1] < size[1] and 0 <= p[2] < size[2]

    while q:
        d, cur = heappop(q)
        if d != dist.get(cur):
            continue
        expanded += 1
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return list(reversed(path)), expanded
        for dx,dy,dz in MOVES:
            nxt = (cur[0]+dx, cur[1]+dy, cur[2]+dz)
            if not inside(nxt) or nxt in occupied:
                continue
            nd = d + sqrt(dx*dx + dy*dy + dz*dz)
            if nd < dist.get(nxt, float("inf")):
                dist[nxt] = nd
                came[nxt] = cur
                heappush(q, (nd, nxt))
    return None, expanded
