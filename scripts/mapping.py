# Autonomous mapping: drive to the nearest unexplored edge ("frontier") of the SLAM map, again and
# again, until nothing reachable is left. Then save the map.
#
# Needs: ./start.sh --slam, robot on the floor, STOP in the web page at hand.
# Run:   web page -> mapping.py -> Run   (or ./ros.sh python3 scripts/mapping.py)
# Try first with DRIVE = False: it only plans and prints what it would do.
#
# Planning (frontiers, A* path around walls) is in explore.py: test it offline on a saved map with
#   ./ros.sh python3 explore.py maps/<name>.yaml   -> maps/<name>_plan.png
import json
import math
import time
import urllib.request

import numpy as np
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

import edurobot as core
import explore
from motion import *

DRIVE = True            # False: only plan and print, the robot doesn't move
SPEED = 0.12            # m/s, slow = clean map
TURN = 25               # degrees/s for turning on the spot
# The robot only turns on the spot and drives forward (no strafing): least wheel slip = least map error.
# Room ~5 x 11.5 m, the lidar (12 m) sees most of it from anywhere: what's left to explore are mostly
# the shadows behind the obstacle course walls, so exploring concentrates on the course by itself.
MAX_RANGE = 7.0         # m from the start: frontiers farther away are ignored (don't leave through a door)
WAYPOINTS_PER_PLAN = 3  # drive this many corners of a path, then plan again with the newer map
MAX_ROUNDS = 60         # safety limit
SETTLE = 3.0            # s to wait before planning: SLAM publishes the map every 3 s
SAVE_AS = 'mapping_' + time.strftime('%Y%m%d_%H%M')

# ---- the live SLAM map ----
latest = {}
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
core._node.create_subscription(OccupancyGrid, 'map', lambda m: latest.update(map=m), qos)


def current_grid():
    m = latest.get('map')
    if m is None:
        return None
    data = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
    return explore.Grid(data, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y)


def save_map(name):
    """Ask the web page server to save the map (maps/<name>.pgm + .yaml)."""
    try:
        req = urllib.request.Request('http://localhost:8080/savemap', data=json.dumps({'name': name}).encode(),
                                     headers={'Content-Type': 'application/json'})
        print('Saved:', json.load(urllib.request.urlopen(req, timeout=10))['msg'])
    except Exception as e:
        print(f'Could not save the map ({e}): save it in the web page.')


# ---- start ----
start = get_map_pose()
if start is None:
    raise SystemExit('No SLAM map pose: start with ./start.sh --slam')
print(f'Start on the map: x={start[0]:.2f} y={start[1]:.2f} yaw={start[2]:.0f} deg'
      + ('' if DRIVE else '   (DRIVE = False: planning only)'))
for _ in range(50):
    if latest.get('map'):
        break
    time.sleep(0.1)

failed = {}  # frontier target cell -> how often we couldn't get there
for round_no in range(1, MAX_ROUNDS + 1):
    time.sleep(SETTLE)
    g = current_grid()
    x, y, _ = get_map_pose()
    # ignore frontiers outside the room and ones that failed twice
    skip = [t for t, n in failed.items() if n >= 2]
    skip += [f['target'] for f in explore.frontiers(g) if math.dist(g.xy(f['target']), start[:2]) > MAX_RANGE]
    target, waypoints = explore.next_target(g, x, y, skip=skip)
    if target is None:
        print(f'Round {round_no}: no reachable frontier left, map complete.')
        break
    tx, ty = g.xy(target['target'])
    print(f'Round {round_no}: robot ({x:.2f}, {y:.2f}) -> frontier {target["size"]:.1f} m at ({tx:.2f}, {ty:.2f}), '
          f'{len(waypoints)} waypoint(s): ' + ' '.join(f'({wx:.2f},{wy:.2f})' for wx, wy in waypoints))
    if not DRIVE:
        break
    for wx, wy in waypoints[:WAYPOINTS_PER_PLAN]:
        if not goto_map(SPEED, wx, wy, forward_only=True, turn_speed=TURN):
            failed[target['target']] = failed.get(target['target'], 0) + 1
            print('  could not get there, planning again')
            break
else:
    print(f'Stopped after {MAX_ROUNDS} rounds (MAX_ROUNDS).')

print_pose()
if DRIVE:
    save_map(SAVE_AS)
