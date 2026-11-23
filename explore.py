#!/usr/bin/env python3
"""explore.py - planning for autonomous mapping: frontiers, safety margin, A* path. Pure numpy, no ROS.

Used by scripts/mapping.py on the live SLAM map. Works offline on saved maps too, and draws what it
would do (frontiers, chosen target, path) into a picture:

    ./ros.sh python3 explore.py maps/barrel.yaml            -> maps/barrel_plan.png (start = map 0,0)
    ./ros.sh python3 explore.py maps/barrel.yaml 1.0 -0.5   -> start at x=1.0, y=-0.5 m on the map

Map arrays are like nav_msgs/OccupancyGrid: [row = y, col = x], -1 unknown, 0 free, 100 occupied.
"""
import heapq
import math
import os
import sys

import numpy as np

PLAN_RES = 0.05        # m, planning grid (the SLAM map is 1-2 cm; A* on that would be far too slow)
ROBOT_RADIUS = 0.16    # m, robot centre to its farthest corner: 26 x 18 cm -> sqrt(0.13² + 0.09²)
MARGIN = 0.05          # m, extra distance to walls. Passages need 2 * (ROBOT_RADIUS + MARGIN) = 0.42 m
WALL_COST_DIST = 0.5   # m, paths closer than this to a wall cost more: keeps the robot in the middle
MIN_FRONTIER = 0.3     # m, shorter frontier pieces are ignored (noise, gaps between scan rays)
VIEW_DIST = 2.5        # m, the robot looks at a frontier from up to this far away (it doesn't have to fit
                       # into narrow spots: looking into an obstacle course from its entrance is enough)


class Grid:
    """A map on the planning grid, with helpers to go between cells (row, col) and metres (x, y)."""

    def __init__(self, data, res, ox, oy):
        k = max(1, round(PLAN_RES / res))
        self.cells = downsample(data, k)
        self.res, self.ox, self.oy = res * k, ox, oy
        occupied = self.cells >= 50
        self.blocked = inflate(occupied, math.ceil((ROBOT_RADIUS + MARGIN) / self.res)) | (self.cells == -1)
        self.near_wall = inflate(occupied, math.ceil(WALL_COST_DIST / self.res))

    def cell(self, x, y):
        return int((y - self.oy) / self.res), int((x - self.ox) / self.res)

    def xy(self, rc):
        return self.ox + (rc[1] + 0.5) * self.res, self.oy + (rc[0] + 0.5) * self.res

    def inside(self, rc):
        return 0 <= rc[0] < self.cells.shape[0] and 0 <= rc[1] < self.cells.shape[1]


# ---------- grid helpers ----------

def downsample(a, k):
    """k x k blocks -> one cell: occupied if any cell is, free if at least half are free, else unknown."""
    if k == 1:
        return a.copy()
    h, w = (a.shape[0] // k) * k, (a.shape[1] // k) * k
    b = a[:h, :w].reshape(h // k, k, w // k, k)
    out = np.full((h // k, w // k), -1, np.int8)
    out[((b >= 0) & (b < 50)).sum(axis=(1, 3)) >= (k * k) // 2] = 0
    out[(b >= 50).any(axis=(1, 3))] = 100
    return out


def shift(m, dy, dx):
    """Move a 2D array by (dy, dx) cells, filling with False (no wrap-around)."""
    out = np.zeros_like(m)
    h, w = m.shape
    out[max(dy, 0):h + min(dy, 0), max(dx, 0):w + min(dx, 0)] = m[max(-dy, 0):h + min(-dy, 0), max(-dx, 0):w + min(-dx, 0)]
    return out


def inflate(mask, r):
    """Grow True cells by a disc of r cells (the robot's size around obstacles)."""
    out = mask.copy()
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if (dx or dy) and dx * dx + dy * dy <= r * r:
                out |= shift(mask, dy, dx)
    return out


# ---------- 1. frontiers ----------

def frontiers(g):
    """Groups of free cells next to unknown space, largest first: {'size': m, 'centre': (row, col)}.
    Also in narrow spots and right at walls, where the robot itself can't go: it looks at them from a
    viewpoint (see viewpoint())."""
    unknown = g.cells == -1
    near_unknown = shift(unknown, 1, 0) | shift(unknown, -1, 0) | shift(unknown, 0, 1) | shift(unknown, 0, -1)
    todo = set(map(tuple, np.argwhere((g.cells == 0) & near_unknown)))
    groups = []
    while todo:
        seed = todo.pop()
        group, stack = [seed], [seed]
        while stack:  # flood fill over the 8 neighbours
            r, c = stack.pop()
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    n = (r + dr, c + dc)
                    if n in todo:
                        todo.remove(n)
                        group.append(n)
                        stack.append(n)
        size = len(group) * g.res
        if size >= MIN_FRONTIER:
            cr, cc = np.mean(group, axis=0)
            centre = min(group, key=lambda p: (p[0] - cr) ** 2 + (p[1] - cc) ** 2)  # a cell of the group
            groups.append({'size': size, 'centre': centre})
    return sorted(groups, key=lambda f: -f['size'])


def key(g, f):
    """Stable name for a frontier between map updates: its centre rounded to 0.3 m."""
    x, y = g.xy(f['centre'])
    return round(x / 0.3), round(y / 0.3)


def blocked_from(g, start):
    """Where the robot can't be, seen from start: grown walls + unknown. Around the robot itself only
    real walls count, so it can drive away when it stands close to something."""
    blocked = g.blocked.copy()
    r = math.ceil((ROBOT_RADIUS + MARGIN) / g.res)
    around = (slice(max(0, start[0] - r), start[0] + r + 1), slice(max(0, start[1] - r), start[1] + r + 1))
    blocked[around] &= g.cells[around] >= 50
    return blocked


def reachable(blocked, start):
    """Steps (8-neighbour) from start to every cell the robot can reach; -1 = not reachable."""
    steps = np.full(blocked.shape, -1, np.int32)
    if blocked[start]:
        return steps
    steps[start] = 0
    frontier, n = [start], 0
    h, w = blocked.shape
    while frontier:
        n += 1
        nxt = []
        for r, c in frontier:
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < h and 0 <= cc < w and steps[rr, cc] < 0 and not blocked[rr, cc]:
                        steps[rr, cc] = n
                        nxt.append((rr, cc))
        frontier = nxt
    return steps


def viewpoint(g, steps, f):
    """Reachable cell from which the robot sees the frontier: within VIEW_DIST, nothing but free floor in
    between (line of sight), as close to the frontier as possible. None if there is none."""
    walls = g.cells != 0  # walls and unknown block the view
    cr, cc = f['centre']
    rad = int(VIEW_DIST / g.res)
    r0, r1 = max(0, cr - rad), min(g.cells.shape[0], cr + rad + 1)
    c0, c1 = max(0, cc - rad), min(g.cells.shape[1], cc + rad + 1)
    cand = np.argwhere(steps[r0:r1, c0:c1] >= 0) + (r0, c0)
    if len(cand) == 0:
        return None
    d2 = (cand[:, 0] - cr) ** 2 + (cand[:, 1] - cc) ** 2
    order = np.argsort(d2)
    for i in order[:400]:  # closest first; the first one with a free view wins
        if d2[i] > rad * rad:
            break
        p = (int(cand[i, 0]), int(cand[i, 1]))
        if free_view(walls, p, f["centre"]):
            return p
    return None


def free_view(walls, a, b):
    """Line of sight from a to b: no wall/unknown cell in between (the end cell b itself may be anything)."""
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2) + 1
    for i in range(n):  # stops before b
        r = round(a[0] + (b[0] - a[0]) * i / n)
        c = round(a[1] + (b[1] - a[1]) * i / n)
        if walls[r, c] and (r, c) != tuple(b):
            return False
    return True


# ---------- 2. path ----------

def plan(g, start, goal):
    """A* from start to goal (cells), around blocked cells, preferring the middle of passages.
    Returns the path as a few corner points (cells), or None if the goal can't be reached."""
    blocked = blocked_from(g, start)
    blocked[goal] = False
    if not (g.inside(start) and g.inside(goal)) or blocked[start]:
        return None
    h, w = blocked.shape
    steps = [(dr, dc, math.hypot(dr, dc)) for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc]
    dist = {start: 0.0}
    came = {}
    todo = [(0.0, start)]
    while todo:
        _, cur = heapq.heappop(todo)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return simplify(blocked, path[::-1])
        for dr, dc, step in steps:
            n = (cur[0] + dr, cur[1] + dc)
            if not (0 <= n[0] < h and 0 <= n[1] < w) or blocked[n]:
                continue
            d = dist[cur] + step * (3 if g.near_wall[n] else 1)
            if d < dist.get(n, math.inf):
                dist[n] = d
                came[n] = cur
                heapq.heappush(todo, (d + math.hypot(goal[0] - n[0], goal[1] - n[1]), n))
    return None


def free_line(blocked, a, b):
    """True if the straight line from cell a to cell b crosses no blocked cell."""
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2) + 1
    for i in range(n + 1):
        r = round(a[0] + (b[0] - a[0]) * i / n)
        c = round(a[1] + (b[1] - a[1]) * i / n)
        if blocked[r, c]:
            return False
    return True


def simplify(blocked, path):
    """Keep only the corners: skip every point that can be reached in a straight line."""
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not free_line(blocked, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def next_target(g, x, y, skip=(), max_range=None, origin=None):
    """The nearest frontier the robot can look at from (x, y) in metres.
    Returns (frontier, waypoints in metres) or (None, None) when nothing is left.
    frontier gets 'view' (cell to drive to) and 'key'. skip: frontier keys to leave out.
    max_range/origin: ignore frontiers farther than max_range m from origin (x, y)."""
    start = g.cell(x, y)
    blocked = blocked_from(g, start)
    steps = reachable(blocked, start)
    options = []
    for f in frontiers(g):
        f['key'] = key(g, f)
        if f['key'] in skip:
            continue
        if max_range and math.dist(g.xy(f['centre']), origin) > max_range:
            continue
        v = viewpoint(g, steps, f)
        if v is not None:
            f['view'] = v
            options.append((steps[v], f))
    for _, f in sorted(options, key=lambda o: o[0]):  # fewest steps to drive first
        path = plan(g, start, f['view'])
        if path:
            return f, [g.xy(p) for p in path[1:]]
    return None, None


# ---------- offline: load a saved map, draw the plan ----------

def load_map(yaml_path):
    """maps/<name>.yaml + .pgm (map_server format) -> (array like OccupancyGrid, res, origin x, origin y)."""
    info = {}
    for line in open(yaml_path):
        if ':' in line:
            k, v = line.split(':', 1)
            info[k.strip()] = v.strip()
    res = float(info['resolution'])
    ox, oy = (float(v) for v in info['origin'].strip('[]').split(',')[:2])
    with open(os.path.join(os.path.dirname(yaml_path), info['image']), 'rb') as f:
        assert f.readline().strip() == b'P5'
        w, h = map(int, f.readline().split())
        f.readline()
        img = np.frombuffer(f.read(w * h), np.uint8).reshape(h, w)[::-1]  # image top = largest y
    a = np.full(img.shape, -1, np.int8)
    a[img == 254] = 0
    a[img == 0] = 100
    return a, res, ox, oy


def draw(g, frontier_list, target, waypoints, start_xy, out_path):
    from PIL import Image, ImageDraw
    c = np.full(g.cells.shape + (3,), 150, np.uint8)            # unknown: grey
    c[g.cells == 0] = (255, 255, 255)                             # free: white
    c[(g.cells == 0) & g.blocked] = (255, 205, 205)               # too close to a wall: pink
    c[g.cells >= 50] = (0, 0, 0)                                  # walls: black
    scale = 4
    img = Image.fromarray(c[::-1]).resize((c.shape[1] * scale, c.shape[0] * scale), Image.NEAREST)
    d = ImageDraw.Draw(img)
    H = g.cells.shape[0]
    px = lambda rc: ((rc[1] + 0.5) * scale, (H - rc[0] - 0.5) * scale)  # noqa: E731
    for f in frontier_list:
        x, y = px(f['centre'])
        d.ellipse((x - 5, y - 5, x + 5, y + 5), outline=(0, 90, 255), width=2)
    pts = [px(g.cell(*start_xy))] + [px(g.cell(x, y)) for x, y in waypoints or []]
    if len(pts) > 1:
        d.line(pts, fill=(230, 0, 0), width=3)
    x, y = pts[0]
    d.ellipse((x - 6, y - 6, x + 6, y + 6), fill=(255, 122, 26))
    if target:
        x, y = px(target['view'])
        fx, fy = px(target['centre'])
        d.line((x, y, fx, fy), fill=(0, 170, 0), width=1)  # line of sight to the frontier
        d.ellipse((fx - 6, fy - 6, fx + 6, fy + 6), fill=(0, 90, 255))
        d.ellipse((x - 7, y - 7, x + 7, y + 7), fill=(0, 170, 0))
    img.save(out_path)


if __name__ == '__main__':
    if len(sys.argv) not in (2, 4):
        sys.exit(__doc__)
    import time
    data, res, ox, oy = load_map(sys.argv[1])
    sx, sy = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) == 4 else (0.0, 0.0)
    t = time.time()
    g = Grid(data, res, ox, oy)
    found = frontiers(g)
    target, waypoints = next_target(g, sx, sy)
    print(f'map {data.shape[1]}x{data.shape[0]} cells at {res * 100:.0f} cm -> planning grid '
          f'{g.cells.shape[1]}x{g.cells.shape[0]} at {g.res * 100:.0f} cm')
    print(f'{len(found)} frontiers (>= {MIN_FRONTIER} m), largest: '
          + ', '.join(f"{f['size']:.1f} m" for f in found[:5]))
    if target:
        fx, fy = g.xy(target['centre'])
        vx, vy = g.xy(target['view'])
        print(f'next: frontier of {target["size"]:.1f} m at ({fx:.2f}, {fy:.2f}) m, seen from ({vx:.2f}, {vy:.2f}), '
              f'path: ' + ' -> '.join(f'({x:.2f}, {y:.2f})' for x, y in waypoints))
    else:
        print('nothing reachable left to explore')
    print(f'took {time.time() - t:.2f} s')
    out = sys.argv[1].rsplit('.', 1)[0] + '_plan.png'
    draw(g, found, target, waypoints, (sx, sy), out)
    print('picture:', out, '(white free, black wall, pink too close to a wall, grey unknown,',
          'blue frontiers (filled = chosen), green viewpoint + line of sight, red path, orange start)')
