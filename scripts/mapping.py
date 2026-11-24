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

import edurobot
import explore
from motion import get_map_pose, goto_map, print_pose

DRIVE = True  # False: only plan and print, the robot doesn't move

# ---- the robot ----
ROBOT_LENGTH = 0.26  # m, front to back
ROBOT_WIDTH = 0.18  # m, side to side
LIDAR_X = 0.0  # m, lidar in front of the robot's centre (0 = mounted in the centre)

# Clearances. Driving straight needs the robot's width. Turning on the spot needs the circle its corners
# sweep (sqrt(0.13² + 0.09²) = 0.16 m), so passages need 2 * (ROBOT_WIDTH/2 + PLAN_MARGIN) = 0.30 m,
# and the robot only turns where that circle + TURN_CLEARANCE is free.
PLAN_MARGIN = 0.06  # m, planned paths keep this much between the robot's side and walls
TURN_CLEARANCE = 0.02  # m, free space around the turning circle (planning and live lidar check)
DRIVE_CLEARANCE = 0.03  # m, live lidar check: free space beside the robot while driving
LOOK_AHEAD = 0.20  # m, live lidar check: free space in front of the robot while driving

SPEED = 0.12  # m/s, slow = clean map
TURN = 25  # degrees/s for turning on the spot

# The robot only turns on the spot and drives forward (no strafing): least wheel slip = least map error.
# Room ~5 x 11.5 m, the lidar (12 m) sees most of it from anywhere: what's left to explore are mostly
# the shadows behind the obstacle course walls, so exploring concentrates on the course by itself.
MAX_RANGE = 7.0  # m from the start: frontiers farther away are ignored (don't leave through a door)
WAYPOINTS_PER_PLAN = 3  # drive this many corners of a path, then plan again with the newer map
MAX_ROUNDS = 60  # safety limit
SETTLE = 3.0  # s to wait before planning: SLAM publishes the map every 3 s
SAVE_AS = 'mapping_' + time.strftime('%Y%m%d_%H%M')

# Hand the sizes to the planner (explore.py) and to the lidar collision check (edurobot.py)
explore.HALF_WIDTH = ROBOT_WIDTH / 2
explore.MARGIN = PLAN_MARGIN
explore.TURN_RADIUS = math.hypot(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2) + TURN_CLEARANCE
edurobot.ROBOT_LENGTH = ROBOT_LENGTH
edurobot.ROBOT_WIDTH = ROBOT_WIDTH
edurobot.LIDAR_X = LIDAR_X

# ---- the live SLAM map ----
slam_msgs = {}  # 'map' -> the newest OccupancyGrid message from SLAM
map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
edurobot._node.create_subscription(OccupancyGrid, 'map', lambda msg: slam_msgs.update(map=msg), map_qos)


def current_grid():
    """The newest SLAM map as an explore.Grid, or None if no map has arrived yet."""
    msg = slam_msgs.get('map')
    if msg is None:
        return None
    cells = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    origin = msg.info.origin.position
    return explore.Grid(cells, msg.info.resolution, origin.x, origin.y)


def save_map(name):
    """Ask the web page server to save the map (maps/<name>.pgm + .yaml)."""
    try:
        body = json.dumps({'name': name}).encode()
        req = urllib.request.Request('http://localhost:8080/savemap', data=body,
                                     headers={'Content-Type': 'application/json'})
        reply = json.load(urllib.request.urlopen(req, timeout=10))
        print('Saved:', reply['msg'])
    except Exception as e:
        print(f'Could not save the map ({e}): save it in the web page.')


# ---- start ----
start = get_map_pose()
if start is None:
    raise SystemExit('No SLAM map pose: start with ./start.sh --slam')
print(f'Start on the map: x={start[0]:.2f} y={start[1]:.2f} yaw={start[2]:.0f} deg'
      + ('' if DRIVE else '   (DRIVE = False: planning only)'))

# Wait until the first map has arrived
for _ in range(50):
    if slam_msgs.get('map'):
        break
    time.sleep(0.1)

# frontier key -> how often we went to look at it (still there after 2 visits = can't see in)
visits = {}

for round_no in range(1, MAX_ROUNDS + 1):
    time.sleep(SETTLE)
    grid = current_grid()
    robot_x, robot_y, _ = get_map_pose()

    # Frontiers we have already looked at twice are given up on
    skip = {key for key, count in visits.items() if count >= 2}
    target, waypoints = explore.next_target(grid, robot_x, robot_y, skip=skip,
                                            max_range=MAX_RANGE, origin=start[:2])
    if target is None:
        print(f'Round {round_no}: no frontier left that can be looked at, map complete.')
        break

    visits[target['key']] = visits.get(target['key'], 0) + 1
    centre_x, centre_y = grid.xy(target['centre'])
    view_x, view_y = grid.xy(target['view'])
    visit_note = f' [visit {visits[target["key"]]}]' if visits[target['key']] > 1 else ''
    print(f'Round {round_no}: robot ({robot_x:.2f}, {robot_y:.2f}) -> frontier {target["size"]:.1f} m '
          f'at ({centre_x:.2f}, {centre_y:.2f}), look from ({view_x:.2f}, {view_y:.2f}), '
          f'{len(waypoints)} waypoint(s)' + visit_note)

    if not DRIVE:
        break

    for wx, wy in waypoints[:WAYPOINTS_PER_PLAN]:
        arrived = goto_map(SPEED, wx, wy, forward_only=True, turn_speed=TURN, look_ahead=LOOK_AHEAD,
                           clearance=DRIVE_CLEARANCE, turn_clearance=TURN_CLEARANCE)
        if not arrived:
            print('  could not get there, planning again')
            break
else:
    print(f'Stopped after {MAX_ROUNDS} rounds (MAX_ROUNDS).')

print_pose()
if DRIVE:
    save_map(SAVE_AS)
