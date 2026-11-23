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
ROBOT_RADIUS = 0.25    # m, robot centre to its farthest corner (estimate)
MARGIN = 0.05          # m, extra distance to walls. Passages need 2 * (ROBOT_RADIUS + MARGIN) = 0.6 m
WALL_COST_DIST = 0.5   # m, paths closer than this to a wall cost more: keeps the robot in the middle
MIN_FRONTIER = 0.3     # m, shorter frontier pieces are ignored (noise, gaps between scan rays)


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
    """Groups of reachable free cells next to unknown space, largest first.
    Each: {'size': m, 'target': (row, col)}, target = the group's cell closest to its centre."""
    unknown = g.cells == -1
    near_unknown = shift(unknown, 1, 0) | shift(unknown, -1, 0) | shift(unknown, 0, 1) | shift(unknown, 0, -1)
    # free and not too close to walls (the robot has to be able to stand there), next to unknown
    candidates = (g.cells == 0) & ~inflate(g.cells >= 50, math.ceil((ROBOT_RADIUS + MARGIN) / g.res)) & near_unknown
    todo = set(map(tuple, np.argwhere(candidates)))
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
            target = min(group, key=lambda p: (p[0] - cr) ** 2 + (p[1] - cc) ** 2)
            groups.append({'size': size, 'target': target})
    return sorted(groups, key=lambda f: -f['size'])


# ---------- 2. path ----------

def plan(g, start, goal):
    """A* from start to goal (cells), around blocked cells, preferring the middle of passages.
    Returns the path as a few corner points (cells), or None if the goal can't be reached."""
    blocked = g.blocked.copy()
    # the robot may stand close to a wall itself: free its own surroundings so it can leave
    r = math.ceil((ROBOT_RADIUS + MARGIN) / g.res)
    around = (slice(max(0, start[0] - r), start[0] + r + 1), slice(max(0, start[1] - r), start[1] + r + 1))
    blocked[around] &= g.cells[around] >= 50  # around the robot only real walls stay blocked
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


def next_target(g, x, y, skip=()):
    """The nearest reachable frontier from (x, y) in metres.
    Returns (frontier, waypoints in metres) or (None, None) when there is nothing left to explore.
    skip: targets (cells) that failed before."""
    start = g.cell(x, y)
    found = [f for f in frontiers(g) if f['target'] not in skip]
    found.sort(key=lambda f: math.dist(f['target'], start))
    for f in found[:8]:  # nearest few by straight line; the first one with a path wins
        path = plan(g, start, f['target'])
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
        x, y = px(f['target'])
        d.ellipse((x - 5, y - 5, x + 5, y + 5), outline=(0, 90, 255), width=2)
    pts = [px(g.cell(*start_xy))] + [px(g.cell(x, y)) for x, y in waypoints or []]
    if len(pts) > 1:
        d.line(pts, fill=(230, 0, 0), width=3)
    x, y = pts[0]
    d.ellipse((x - 6, y - 6, x + 6, y + 6), fill=(255, 122, 26))
    if target:
        x, y = px(target['target'])
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
        print(f'next target: frontier of {target["size"]:.1f} m at ({g.xy(target["target"])[0]:.2f}, '
              f'{g.xy(target["target"])[1]:.2f}) m, path: ' + ' -> '.join(f'({x:.2f}, {y:.2f})' for x, y in waypoints))
    else:
        print('nothing reachable left to explore')
    print(f'took {time.time() - t:.2f} s')
    out = sys.argv[1].rsplit('.', 1)[0] + '_plan.png'
    draw(g, found, target, waypoints, (sx, sy), out)
    print('picture:', out, '(white free, black wall, pink too close to a wall, grey unknown,',
          'blue frontiers, green target, red path, orange start)')
