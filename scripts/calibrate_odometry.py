# Odometry calibration with the lidar: finds the real wheel radius and sets it on the robot.
#
# Idea: the robot drives a distance measured by its odometry (wheel encoders × wheel radius), and the
# lidar measures how much closer to a wall it really got. If the odometry says 0.600 m but the robot
# really moved 0.612 m, the wheels are 2 % bigger than the robot thinks:
#     new radius = old radius × (lidar distance / odometry distance)
#
# Setup: robot ~1 m in front of a straight, flat wall (no chair legs or boxes in front of it),
#        roughly facing it. The script squares itself up to the wall with the lidar.
#        Free space: the robot drives DRIVE_DIST towards the wall and back again.
# Run:   web page -> calibrate_odometry.py -> Run   (or ./ros.sh python3 scripts/calibrate_odometry.py)
# Then run it a second time: the ratio should now be ~1.000.
import math
import time

import numpy as np
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

import edurobot as core
from motion import backward, delay, forward, rotate_left, rotate_right

DRIVE_DIST = 0.60     # m driven per run (longer = more accurate)
SPEED = 0.10          # m/s, slow = less wheel slip
ROUND_TRIPS = 2       # each round trip = one run towards the wall + one back
APPLY = True          # False: only measure and print, don't change the robot

WALL_VIEW = 20        # degrees left and right of straight ahead that are used to find the wall
SCANS = 10            # lidar scans averaged (median) per measurement
FRONT_GAP = 0.35      # m, the robot's front must stay at least this far from the wall
MAX_CHANGE = 0.15     # don't apply a change of more than 15 % (something must be wrong)
MAX_SPREAD = 0.02     # don't apply if the single runs differ by more than 2 %

ROBOT_NODE = '/Edurob'
WHEEL_PARAMS = ['wheel_radius_0', 'wheel_radius_1', 'wheel_radius_2', 'wheel_radius_3']


# ---------- lidar: where is the wall? ----------

def fit_wall(scan):
    """Fit a straight line to the lidar points in front.
    Returns (distance to the wall in m, how far the robot is turned away from facing it in degrees,
    + = turned left), or None if there is no clear wall in front."""
    ranges = np.asarray(scan.ranges, dtype=float)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    angles = np.arctan2(np.sin(angles), np.cos(angles))  # -pi..pi, 0 = straight ahead
    use = np.isfinite(ranges) & (ranges > 0.2) & (ranges < 4.0) & (np.abs(angles) < math.radians(WALL_VIEW))
    x = ranges[use] * np.cos(angles[use])  # forward
    y = ranges[use] * np.sin(angles[use])  # left
    if len(x) < 10:
        return None

    # Wall as a line  x = a + b * y.  Things in front of the wall (chair legs, boxes) pull a plain fit
    # off, so: fit, keep the 70 % of points closest to that line, fit again, keep only the points
    # within 3 cm of it, final fit.
    b, a = np.polyfit(y, x, 1)
    off = np.abs(x - (a + b * y))
    closest = off <= np.percentile(off, 70)
    b, a = np.polyfit(y[closest], x[closest], 1)
    on_wall = np.abs(x - (a + b * y)) < 0.03
    if on_wall.sum() < max(10, len(x) / 2):  # most of the view has to be one straight wall
        return None
    b, a = np.polyfit(y[on_wall], x[on_wall], 1)

    distance = a / math.sqrt(1 + b * b)  # perpendicular distance: the same whichever way the robot faces
    turned = math.degrees(math.atan(b))   # wall further away on the left = robot turned left
    return distance, turned


def measure_wall():
    """Median of SCANS fresh lidar scans: (distance to the wall in m, robot turned away by degrees)."""
    distances, turns = [], []
    last_scan = None
    give_up = time.monotonic() + 10
    while len(distances) < SCANS:
        if time.monotonic() > give_up:
            raise SystemExit('No wall found in front of the robot (or no lidar data on /scan). '
                             'Place the robot ~1 m in front of a straight wall, facing it.')
        scan = core._scan
        if scan is None or scan is last_scan:  # wait for the next scan
            time.sleep(0.02)
            continue
        last_scan = scan
        wall = fit_wall(scan)
        if wall:
            distances.append(wall[0])
            turns.append(wall[1])
    return float(np.median(distances)), float(np.median(turns))


def face_wall():
    """Turn on the spot until the robot faces the wall squarely (within 1 degree)."""
    for _ in range(3):
        _, turned = measure_wall()
        if abs(turned) < 1.0:
            break
        print(f'  turned {turned:+.1f} deg away from the wall, correcting')
        if turned > 0:
            rotate_right(turned, 10)
        else:
            rotate_left(-turned, 10)
        delay(500)
    distance, turned = measure_wall()
    print(f'  facing the wall: {distance:.3f} m away, {turned:+.1f} deg')
    return distance


# ---------- odometry ----------

def odometry_xy():
    """Where the odometry thinks the robot is (m). The robot has to stand still."""
    x, y, _ = core._update_pose()
    return x, y


def measure_run(drive, name):
    """Drive one run, return lidar distance / odometry distance."""
    wall_before, turned = measure_wall()
    x0, y0 = odometry_xy()

    drive(SPEED, DRIVE_DIST)
    delay(1000)  # stand still, so lidar and odometry are up to date

    wall_after, _ = measure_wall()
    x1, y1 = odometry_xy()

    odometry_dist = math.hypot(x1 - x0, y1 - y0)
    # Driving at a small angle to the wall: the wall distance changes a bit less than the robot moved
    lidar_dist = abs(wall_after - wall_before) / math.cos(math.radians(turned))
    ratio = lidar_dist / odometry_dist
    print(f'  {name:<9} odometry {odometry_dist:.3f} m   lidar {lidar_dist:.3f} m   ratio {ratio:.4f}')
    return ratio


# ---------- wheel radius on the robot ----------

def wait(future, what):
    end = time.monotonic() + 5
    while not future.done() and time.monotonic() < end:
        time.sleep(0.05)
    if not future.done():
        raise SystemExit(f'No answer from {ROBOT_NODE} ({what})')
    return future.result()


params = AsyncParameterClient(core._node, ROBOT_NODE)


def get_wheel_radii():
    if not params.wait_for_services(timeout_sec=5):
        raise SystemExit(f'{ROBOT_NODE} parameter service not found: is the robot connected?')
    values = wait(params.get_parameters(WHEEL_PARAMS), 'get wheel radius').values
    radii = [v.double_value for v in values]
    if not all(r > 0 for r in radii):
        raise SystemExit(f'Could not read {", ".join(WHEEL_PARAMS)} from {ROBOT_NODE}')
    return radii


def set_wheel_radii(radii):
    new = [Parameter(name, Parameter.Type.DOUBLE, r) for name, r in zip(WHEEL_PARAMS, radii)]
    results = wait(params.set_parameters(new), 'set wheel radius').results
    failed = [f'{name}: {r.reason}' for name, r in zip(WHEEL_PARAMS, results) if not r.successful]
    if failed:
        raise SystemExit('The robot refused: ' + ', '.join(failed))


# ---------- calibration ----------

print('Odometry calibration with the lidar')
old_radii = get_wheel_radii()
print('Wheel radius now: ' + ', '.join(f'{r * 1000:.2f} mm' for r in old_radii))

print('Lining up with the wall...')
distance = face_wall()
closest_front = distance - DRIVE_DIST - core.ROBOT_LENGTH / 2
if closest_front < FRONT_GAP:
    raise SystemExit(f'Too close to the wall: move the robot back to at least '
                     f'{DRIVE_DIST + core.ROBOT_LENGTH / 2 + FRONT_GAP:.2f} m from it.')

print(f'Driving {DRIVE_DIST:.2f} m towards the wall and back, {ROUND_TRIPS} times...')
ratios = []
for trip in range(ROUND_TRIPS):
    ratios.append(measure_run(forward, 'forward'))
    ratios.append(measure_run(backward, 'backward'))

ratio = float(np.mean(ratios))
spread = max(ratios) - min(ratios)
new_radii = [r * ratio for r in old_radii]
print()
print(f'Result: the robot really moves {ratio:.4f} x what the odometry says '
      f'(runs differ by {spread * 100:.1f} %)')
print('New wheel radius: ' + ', '.join(f'{r * 1000:.2f} mm' for r in new_radii))

if abs(ratio - 1) < 0.002:
    print('Already accurate (within 0.2 %), nothing changed.')
elif abs(ratio - 1) > MAX_CHANGE:
    print(f'Not applied: change of {(ratio - 1) * 100:+.1f} % is too big to be right. '
          f'Wall straight? Wheels slipping?')
elif spread > MAX_SPREAD:
    print(f'Not applied: the runs differ too much ({spread * 100:.1f} %). '
          f'Wheels slipping? Something moving near the wall? Try again.')
elif not APPLY:
    print('Not applied (APPLY = False).')
else:
    set_wheel_radii(new_radii)
    print(f'Set on the robot. Run this script again to check: the ratio should be ~1.000.')

print('To set these by hand (e.g. after a robot reboot):')
for name, r in zip(WHEEL_PARAMS, new_radii):
    print(f'  ros2 param set {ROBOT_NODE} {name} {r:.5f}')
