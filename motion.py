"""motion.py - all movement helpers for the EduRob (and turtlesim), Arduino style.

Use it in a script like this (see scripts/ for examples):

    from motion import *

    calibrate_gyro(5)
    forward(0.2, 0.5)          # 0.2 m/s, 50 cm, soft start/stop (ramp=True is the default)
    rotate_left(90, 30)        # 90 degrees at 30 degrees/s
    orbit_left(0.1, 0.5, 360)  # once around an object 50 cm in front, always facing it
    print_pose()

`from motion import *` also brings the helpers from edurobot.py: delay, millis, stop,
calibrate_gyro, get_x, get_y, get_yaw, print_pose. Importing connects to the robot (or turtle)
and waits for odometry.

Every movement blocks until it is done and then stops the robot, like delay(). They all use
odometry: they keep going until the robot has really moved that far (NOTES.md explains how).
Units: speeds in m/s (driving) or degrees/s (turning), distances in m, angles in degrees.
Positive angles = left / counterclockwise, like everywhere in ROS.
"""
import math
import time

import edurobot as core
from edurobot import *  # noqa: F401,F403  delay, millis, stop, calibrate_gyro, get_x/y/yaw, print_pose
from edurobot import stop

__all__ = core.__all__ + [
    'forward', 'backward', 'strafe_left', 'strafe_right', 'move_direction',
    'rotate_left', 'rotate_right', 'turn_to',
    'curve_left', 'curve_right', 'orbit_left', 'orbit_right', 'rotate_around',
    'goto_xy', 'goto_pose', 'move_spinning', 'drive',
    'get_map_pose', 'goto_map',
]

# Speed ramps (ramp=True)
LINEAR_ACCEL = 0.5     # m/s², speed up / slow down this fast
ANGULAR_ACCEL = 90.0   # degrees/s²
MIN_LINEAR = 0.02      # m/s, never slower than this, or it would never arrive
MIN_ANGULAR = 5.0      # degrees/s

# goto_xy / goto_pose / turn_to
POS_TOLERANCE = 0.005  # m, close enough to the target point
YAW_TOLERANCE = 0.5    # degrees
DEFAULT_TURN = 30.0    # degrees/s, goto_pose when only turning
MAX_TURN = 90.0        # degrees/s, goto_pose turning limit


# ---------- direct control ----------

def drive(x, y, turn, ms):
    """Set all three speeds directly (m/s, m/s, degrees/s) for ms milliseconds, then stop. No odometry."""
    end = time.monotonic() + ms / 1000
    try:
        while time.monotonic() < end:
            core._publish(x, y, turn)
            time.sleep(core.CHECK_INTERVAL)
    finally:
        stop()


# ---------- straight lines, the robot doesn't turn ----------

def move_direction(vel, distance, angle, ramp=True):
    """Any direction: angle 0 = forward, 90 = left, -90 = right, 180 = backward, 45 = front-left."""
    speed, goal = abs(vel), abs(distance)
    x, y = speed * math.cos(math.radians(angle)), speed * math.sin(math.radians(angle))
    sx, sy, _ = core._update_pose()

    def step():
        done = math.hypot(core._pose[0] - sx, core._pose[1] - sy)
        if done >= goal:
            return None
        f = core._ramp_factor(done, goal - done, speed, LINEAR_ACCEL, MIN_LINEAR) if ramp else 1
        return x * f, y * f, 0

    core._run_move('distance', core._timeout(goal, speed), step)


def forward(vel, distance, ramp=True):
    move_direction(vel, distance, 0, ramp)


def backward(vel, distance, ramp=True):
    move_direction(vel, distance, 180, ramp)


def strafe_left(vel, distance, ramp=True):
    """Slide sideways to the left."""
    move_direction(vel, distance, 90, ramp)


def strafe_right(vel, distance, ramp=True):
    move_direction(vel, distance, -90, ramp)


# ---------- constant speeds, stop after turning `degree` ----------

def _move_angle(x, y, turn, degree, ramp):
    """Send (x, y, turn) until the heading has changed by `degree` degrees.
    Used for rotate (x = y = 0), curve (x), orbit (y) and rotate_around (x and y)."""
    goal, turn_speed, linear_speed = abs(degree), abs(turn), math.hypot(x, y)
    last_yaw = core._update_pose()[2]
    turned = 0.0  # degrees, summed up so turns larger than 180 degrees work too

    def step():
        nonlocal last_yaw, turned
        turned += math.degrees(core._wrap(core._pose[2] - last_yaw))
        last_yaw = core._pose[2]
        done = abs(turned)
        if done >= goal:
            return None
        f = 1.0
        if ramp:
            f = core._ramp_factor(done, goal - done, turn_speed, ANGULAR_ACCEL, MIN_ANGULAR)
            if linear_speed > 0:
                # Same ramp for the driving part: meters driven = degrees turned * linear / turn speed
                m_per_deg = linear_speed / turn_speed
                f = min(f, core._ramp_factor(done * m_per_deg, (goal - done) * m_per_deg, linear_speed,
                                        LINEAR_ACCEL, MIN_LINEAR))
        # One factor for all three speeds, so curves keep their radius while ramping
        return x * f, y * f, turn * f

    core._run_move('angle', core._timeout(goal, turn_speed), step)


def rotate_left(degree, vel, ramp=True):
    """Turn BY degree, counterclockwise, at vel degrees/s."""
    _move_angle(0, 0, abs(vel), degree, ramp)


def rotate_right(degree, vel, ramp=True):
    """Turn BY degree, clockwise, at vel degrees/s."""
    _move_angle(0, 0, -abs(vel), degree, ramp)


def turn_to(degree, vel, ramp=True):
    """Turn TO a heading measured from the start (0 = start direction, 90 = left of it).
    Takes the shorter way round. Doesn't add up errors like repeated rotate_* calls."""
    delta = math.degrees(core._wrap(math.radians(degree) - core._relative_pose()[2]))
    if abs(delta) < YAW_TOLERANCE:
        return
    _move_angle(0, 0, math.copysign(abs(vel), delta), delta, ramp)


def _circle_turn(vel, radius):
    """Turning speed for a circle: omega = v / r, in degrees/s."""
    return math.degrees(abs(vel) / abs(radius))


def curve_left(vel, radius, degree, ramp=True):
    """Drive forward along a circle (radius m, vel m/s) until turned `degree` degrees, counterclockwise."""
    _move_angle(abs(vel), 0, _circle_turn(vel, radius), degree, ramp)


def curve_right(vel, radius, degree, ramp=True):
    _move_angle(abs(vel), 0, -_circle_turn(vel, radius), degree, ramp)


def orbit_left(vel, radius, degree, ramp=True):
    """Circle around a point `radius` m in front, always facing it, strafing left at vel m/s.
    Strafing left around a centre in front means circling it clockwise, so the robot turns right."""
    _move_angle(0, abs(vel), -_circle_turn(vel, radius), degree, ramp)


def orbit_right(vel, radius, degree, ramp=True):
    _move_angle(0, -abs(vel), _circle_turn(vel, radius), degree, ramp)


def rotate_around(dx, dy, degree, vel, ramp=True):
    """Rotate by `degree` around the point (dx m in front, dy m left) at vel degrees/s.
    Keeps the point still: its speed (x - w*dy, y + w*dx) must be 0."""
    turn = math.copysign(abs(vel), degree)
    w = math.radians(turn)
    _move_angle(w * dy, -w * dx, turn, degree, ramp)


# ---------- go to a position (x, y measured from the start, like get_x/get_y) ----------

def _move_to(vel, tx, ty, use_yaw, tyaw, ramp):
    speed = abs(vel)
    px, py, pyaw = core._relative_pose()
    dist0 = math.hypot(tx - px, ty - py)
    yaw_err0 = math.degrees(core._wrap(math.radians(tyaw) - pyaw)) if use_yaw else 0.0

    # Turn just fast enough to finish turning when the robot arrives
    turn_speed = abs(yaw_err0) / (dist0 / speed) if dist0 > POS_TOLERANCE else DEFAULT_TURN
    turn_speed = min(turn_speed, MAX_TURN)

    state = {'pos_done': dist0 <= POS_TOLERANCE, 'yaw_done': abs(yaw_err0) <= YAW_TOLERANCE, 'last': dist0}
    timeout = max(core._timeout(dist0, speed), core._timeout(yaw_err0, turn_speed) if turn_speed > 0 else 0)

    def step():
        x, y, yaw = core._relative_pose()
        ex, ey = tx - x, ty - y
        dist = math.hypot(ex, ey)
        yaw_err = math.degrees(core._wrap(math.radians(tyaw) - yaw))
        # Done when close enough, or when it just went past (distance growing again / error changed sign)
        if dist <= POS_TOLERANCE or (dist > state['last'] and dist < speed * 0.1 + POS_TOLERANCE):
            state['pos_done'] = True
        state['last'] = dist
        if not use_yaw or abs(yaw_err) <= YAW_TOLERANCE or yaw_err * yaw_err0 < 0:
            state['yaw_done'] = True
        if state['pos_done'] and state['yaw_done']:
            return None
        cx = cy = ct = 0.0
        if not state['pos_done']:
            f = core._ramp_factor(dist0 - dist, dist, speed, LINEAR_ACCEL, MIN_LINEAR) if ramp else 1
            d = math.atan2(ey, ex) - yaw  # direction to the target, seen from the robot
            cx, cy = speed * f * math.cos(d), speed * f * math.sin(d)
        if not state['yaw_done']:
            done = abs(yaw_err0) - abs(yaw_err)
            f = core._ramp_factor(done, abs(yaw_err), turn_speed, ANGULAR_ACCEL, MIN_ANGULAR) if ramp else 1
            ct = math.copysign(turn_speed * f, yaw_err)
        return cx, cy, ct

    core._run_move('target', timeout, step)


def goto_xy(vel, x, y, ramp=True):
    """Drive in a straight line to (x, y) without turning, steering towards it the whole way."""
    _move_to(vel, x, y, False, 0, ramp)


def goto_pose(vel, x, y, yaw, ramp=True):
    """Like goto_xy, and turn to heading `yaw` (degrees) at the same time, arriving at both together."""
    _move_to(vel, x, y, True, yaw, ramp)


# ---------- straight line while spinning ----------

def move_spinning(vel, distance, angle, spin, ramp=True):
    """Drive `distance` m in direction `angle` (fixed when the call starts) while turning at
    `spin` degrees/s (> 0 = counterclockwise). The path stays straight."""
    speed, goal = abs(vel), abs(distance)
    sx, sy, syaw = core._update_pose()
    world_dir = syaw + math.radians(angle)  # fixed direction in the room

    def step():
        x, y, yaw = core._pose
        done = math.hypot(x - sx, y - sy)
        if done >= goal:
            return None
        f = core._ramp_factor(done, goal - done, speed, LINEAR_ACCEL, MIN_LINEAR) if ramp else 1
        # The robot keeps this command until the next one, so aim for its heading halfway through
        yaw += math.radians(spin * f) * core.CMD_INTERVAL / 2
        d = world_dir - yaw  # room direction, seen from the robot
        return speed * f * math.cos(d), speed * f * math.sin(d), spin * f

    core._run_move('distance', core._timeout(goal, speed), step)


# ---------- on the SLAM map (needs ./start.sh --slam) ----------
# goto_xy & co. use odometry, which drifts. These use the robot's position on the SLAM map instead:
# corrected by the lidar, and the same coordinates as the map in the web page / maps/*.yaml.

def _wait_map_pose(timeout=8.0):
    """The map pose, waiting a moment for it: a new program gets SLAM's map frame only after ~2-5 s."""
    end = time.monotonic() + timeout
    while (p := core._map_pose()) is None and time.monotonic() < end:
        time.sleep(0.1)
    return p


def get_map_pose():
    """(x m, y m, yaw degrees) on the SLAM map, or None if SLAM isn't running."""
    p = _wait_map_pose()
    return None if p is None else (p[0], p[1], math.degrees(p[2]))


def goto_map(vel, x, y, ramp=True, stop_distance=0.3, forward_only=False, turn_speed=25):
    """Drive to (x, y) on the SLAM map. Returns True when arrived, False if blocked, timed out or no SLAM.
    Stops early if the lidar sees something closer than stop_distance m in the driving direction.

    forward_only=False: straight there without turning (strafing as needed).
    forward_only=True:  turn on the spot towards the target (turn_speed degrees/s), then drive forward
                        with small steering corrections. Least wheel slip = least odometry error, best
                        for mapping (mecanum wheels slip most when strafing)."""
    speed = abs(vel)
    p = _wait_map_pose()
    if p is None:
        print('goto_map: no SLAM map pose (./start.sh --slam)', flush=True)
        return False
    dist0 = math.hypot(x - p[0], y - p[1])
    s = {'blocked': False, 'last': dist0, 'turning': forward_only, 'start': dist0}

    def step():
        p = core._map_pose()
        if p is None:
            s['blocked'] = True
            return None
        ex, ey = x - p[0], y - p[1]
        dist = math.hypot(ex, ey)
        # arrived, or just went past it (distance growing again close to the target)
        if dist <= 0.03 or (dist > s['last'] and dist < 0.08):
            return None
        s['last'] = dist
        d = core._wrap(math.atan2(ey, ex) - p[2])  # direction to the target, seen from the robot
        if forward_only:
            err = math.degrees(d)
            now = time.monotonic()
            if now < s.get('settle', 0):
                return 0, 0, 0  # let the robot really stop before the next phase (commands act ~0.6 s late)
            # turn on the spot until facing the target (±3°); turn again if it drifts off by > 15°
            if s['turning'] and abs(err) < 3 or not s['turning'] and abs(err) > 15:
                s['turning'] = not s['turning']
                s['start'] = dist  # ramp up again after each turn
                s['settle'] = now + core.LATENCY
                return 0, 0, 0  # stop at once: otherwise the turn would go on until the next command
            if s['turning']:
                turn = math.copysign(min(turn_speed, max(MIN_ANGULAR, abs(err) * 1.5)), err)  # slower when close
                return 0, 0, turn
            d = 0.0  # obstacle check: only straight ahead
        if core._obstacle_ahead(d, stop_distance):
            print(f'goto_map: obstacle within {stop_distance} m, stopped', flush=True)
            s['blocked'] = True
            return None
        f = core._ramp_factor(s['start'] - dist, dist, speed, LINEAR_ACCEL, MIN_LINEAR) if ramp else 1
        if forward_only:
            steer = max(-10.0, min(10.0, math.degrees(core._wrap(math.atan2(ey, ex) - p[2]))))  # gentle
            return speed * f, 0, steer
        return speed * f * math.cos(d), speed * f * math.sin(d), 0

    timeout = core._timeout(dist0, speed) + (360 / turn_speed if forward_only else 0)  # + time for turning
    finished = core._run_move('target', timeout, step)
    return finished and not s['blocked']
