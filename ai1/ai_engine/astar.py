"""3D occupancy-grid A* with 26-neighbour motion."""
from heapq import heappush, heappop
from math import sqrt

MOVES = [
    (dx,dy,dz)
    for dx in (-1,0,1)
    for dy in (-1,0,1)
    for dz in (-1,0,1)
    if not (dx == dy == dz == 0)
]

def heuristic(a, b):
    dx, dy, dz = abs(a[0]-b[0]), abs(a[1]-b[1]), abs(a[2]-b[2])
    # Euclidean is admissible for uniform 26-neighbour costs.
    return sqrt(dx*dx + dy*dy + dz*dz)

def astar(occupied, start, goal, size):
    if start in occupied or goal in occupied:
        return None, 0
    open_heap = []
    heappush(open_heap, (heuristic(start, goal), 0.0, start))
    came = {}
    g = {start: 0.0}
    expanded = 0

    def inside(p):
        return 0 <= p[0] < size[0] and 0 <= p[1] < size[1] and 0 <= p[2] < size[2]

    while open_heap:
        _, current_g, cur = heappop(open_heap)
        if current_g != g.get(cur):
            continue
        expanded += 1
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            path.reverse()
            return path, expanded

        for dx,dy,dz in MOVES:
            nxt = (cur[0]+dx, cur[1]+dy, cur[2]+dz)
            if not inside(nxt) or nxt in occupied:
                continue
            step = sqrt(dx*dx + dy*dy + dz*dz)
            tentative = current_g + step
            if tentative < g.get(nxt, float("inf")):
                g[nxt] = tentative
                came[nxt] = cur
                heappush(open_heap, (tentative + heuristic(nxt, goal), tentative, nxt))
    return None, expanded
