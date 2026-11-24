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

# Speed ramps (ramp=True): speed goes up and down gradually, so the robot doesn't jerk
LINEAR_ACCEL = 0.5  # m/s², speed up / slow down this fast
ANGULAR_ACCEL = 90.0  # degrees/s²
MIN_LINEAR = 0.02  # m/s, never slower than this, or it would never arrive
MIN_ANGULAR = 5.0  # degrees/s

# goto_xy / goto_pose / turn_to
POS_TOLERANCE = 0.005  # m, close enough to the target point
YAW_TOLERANCE = 0.5  # degrees
DEFAULT_TURN = 30.0  # degrees/s, goto_pose when only turning
MAX_TURN = 90.0  # degrees/s, goto_pose turning limit


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
    """Drive `distance` m at `vel` m/s in any direction.
    angle 0 = forward, 90 = left, -90 = right, 180 = backward, 45 = front-left."""
    speed, goal = abs(vel), abs(distance)
    x_speed = speed * math.cos(math.radians(angle))
    y_speed = speed * math.sin(math.radians(angle))
    start_x, start_y, _ = core._update_pose()

    def step():
        travelled = math.hypot(core._pose[0] - start_x, core._pose[1] - start_y)
        if travelled >= goal:
            return None
        factor = core._ramp_factor(travelled, goal - travelled, speed, LINEAR_ACCEL, MIN_LINEAR) if ramp else 1
        return x_speed * factor, y_speed * factor, 0

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

def _move_angle(x_speed, y_speed, spin, degree, ramp):
    """Send (x_speed, y_speed, spin) until the heading has changed by `degree` degrees.
    Used for rotate (x and y are 0), curve (x), orbit (y) and rotate_around (x and y)."""
    goal = abs(degree)
    spin_speed = abs(spin)
    drive_speed = math.hypot(x_speed, y_speed)
    last_yaw = core._update_pose()[2]
    turned = 0.0  # degrees, summed up so turns larger than 180 degrees work too

    def step():
        nonlocal last_yaw, turned
        turned += math.degrees(core._wrap(core._pose[2] - last_yaw))
        last_yaw = core._pose[2]
        done = abs(turned)
        if done >= goal:
            return None
        factor = 1.0
        if ramp:
            factor = core._ramp_factor(done, goal - done, spin_speed, ANGULAR_ACCEL, MIN_ANGULAR)
            if drive_speed > 0:
                # Same ramp for the driving part: metres driven = degrees turned * drive speed / spin speed
                m_per_deg = drive_speed / spin_speed
                factor = min(factor, core._ramp_factor(done * m_per_deg, (goal - done) * m_per_deg,
                                                       drive_speed, LINEAR_ACCEL, MIN_LINEAR))
        # One factor for all three speeds, so curves keep their radius while ramping
        return x_speed * factor, y_speed * factor, spin * factor

    core._run_move('angle', core._timeout(goal, spin_speed), step)


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
    spin = math.copysign(abs(vel), degree)
    w = math.radians(spin)
    _move_angle(w * dy, -w * dx, spin, degree, ramp)


# ---------- go to a position (x, y measured from the start, like get_x/get_y) ----------

def _move_to(vel, tx, ty, use_yaw, tyaw, ramp):
    """Drive to (tx, ty) and, if use_yaw, turn to heading tyaw, arriving at both together."""
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

        x_speed = y_speed = spin = 0.0
        if not state['pos_done']:
            factor = core._ramp_factor(dist0 - dist, dist, speed, LINEAR_ACCEL, MIN_LINEAR) if ramp else 1
            direction = math.atan2(ey, ex) - yaw  # direction to the target, seen from the robot
            x_speed = speed * factor * math.cos(direction)
            y_speed = speed * factor * math.sin(direction)
        if not state['yaw_done']:
            done = abs(yaw_err0) - abs(yaw_err)
            factor = core._ramp_factor(done, abs(yaw_err), turn_speed, ANGULAR_ACCEL, MIN_ANGULAR) if ramp else 1
            spin = math.copysign(turn_speed * factor, yaw_err)
        return x_speed, y_speed, spin

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
    start_x, start_y, start_yaw = core._update_pose()
    world_dir = start_yaw + math.radians(angle)  # fixed direction in the room

    def step():
        x, y, yaw = core._pose
        travelled = math.hypot(x - start_x, y - start_y)
        if travelled >= goal:
            return None
        factor = core._ramp_factor(travelled, goal - travelled, speed, LINEAR_ACCEL, MIN_LINEAR) if ramp else 1
        # The robot keeps this command until the next one, so aim for its heading halfway through
        yaw += math.radians(spin * factor) * core.CMD_INTERVAL / 2
        direction = world_dir - yaw  # room direction, seen from the robot
        return (speed * factor * math.cos(direction),
                speed * factor * math.sin(direction),
                spin * factor)

    core._run_move('distance', core._timeout(goal, speed), step)


# ---------- on the SLAM map (needs ./start.sh --slam) ----------
# goto_xy & co. use odometry, which drifts. These use the robot's position on the SLAM map instead:
# corrected by the lidar, and the same coordinates as the map in the web page / maps/*.yaml.

def _wait_map_pose(timeout=8.0):
    """The map pose, waiting a moment for it: a new program gets SLAM's map frame only after ~2-5 s."""
    end = time.monotonic() + timeout
    while (pose := core._map_pose()) is None and time.monotonic() < end:
        time.sleep(0.1)
    return pose


def get_map_pose():
    """(x m, y m, yaw degrees) on the SLAM map, or None if SLAM isn't running."""
    pose = _wait_map_pose()
    return None if pose is None else (pose[0], pose[1], math.degrees(pose[2]))


def goto_map(vel, x, y, ramp=True, forward_only=False, turn_speed=25, look_ahead=0.2, clearance=0.03,
             turn_clearance=0.02):
    """Drive to (x, y) on the SLAM map. Returns True when arrived, False if blocked, timed out or no SLAM.

    forward_only=False: straight there without turning (strafing as needed).
    forward_only=True:  turn on the spot towards the target (turn_speed degrees/s), then drive forward
                        with small steering corrections. Least wheel slip = least odometry error, best
                        for mapping (mecanum wheels slip most when strafing).
    Collision check with the lidar and the robot's size (core.ROBOT_LENGTH / ROBOT_WIDTH):
      driving: stops if something is in the strip the robot sweeps in the next look_ahead m
               (robot width + clearance on each side)
      turning: doesn't turn if something is inside the circle the corners sweep (+ turn_clearance);
               then it backs up 10 cm (if free behind) and tries again once"""
    speed = abs(vel)
    pose = _wait_map_pose()
    if pose is None:
        print('goto_map: no SLAM map pose (./start.sh --slam)', flush=True)
        return False

    dist0 = math.hypot(x - pose[0], y - pose[1])
    state = {
        'blocked': False,  # False, True, or 'turn' (blocked while turning on the spot)
        'last': dist0,  # distance to the target at the previous step
        'turning': forward_only,  # forward_only: True while turning on the spot, False while driving
        'start': dist0,  # distance at the start of the current ramp
        'backed_up': False,
    }

    def step():
        pose = core._map_pose()
        if pose is None:
            state['blocked'] = True
            return None

        ex, ey = x - pose[0], y - pose[1]
        dist = math.hypot(ex, ey)
        # Arrived, or just went past it (distance growing again close to the target)
        if dist <= 0.03 or (dist > state['last'] and dist < 0.08):
            return None
        state['last'] = dist

        bearing = core._wrap(math.atan2(ey, ex) - pose[2])  # direction to the target, seen from the robot
        if forward_only:
            bearing_deg = math.degrees(bearing)
            now = time.monotonic()
            if now < state.get('settle', 0):
                # Let the robot really stop before the next phase (commands act ~0.6 s late)
                return 0, 0, 0
            # Turn on the spot until facing the target (±3°); turn again if it drifts off by > 15°
            switch_to_turn = not state['turning'] and abs(bearing_deg) > 15
            switch_to_drive = state['turning'] and abs(bearing_deg) < 3
            if switch_to_turn or switch_to_drive:
                state['turning'] = not state['turning']
                state['start'] = dist  # ramp up again after each turn
                state['settle'] = now + core.LATENCY
                return 0, 0, 0  # stop at once: otherwise the turn would go on until the next command
            if state['turning']:
                if core._turn_blocked(turn_clearance):
                    state['blocked'] = 'turn'
                    return None
                # Slower when close to the target direction
                spin = math.copysign(min(turn_speed, max(MIN_ANGULAR, abs(bearing_deg) * 1.5)), bearing_deg)
                return 0, 0, spin
            bearing = 0.0  # driving straight ahead

        if core._path_blocked(bearing, look_ahead, clearance):
            print('goto_map: obstacle in the way, stopped', flush=True)
            state['blocked'] = True
            return None

        factor = core._ramp_factor(state['start'] - dist, dist, speed, LINEAR_ACCEL, MIN_LINEAR) if ramp else 1
        if forward_only:
            # Gentle steering correction, at most 10 degrees/s
            steer = max(-10.0, min(10.0, math.degrees(core._wrap(math.atan2(ey, ex) - pose[2]))))
            return speed * factor, 0, steer
        return speed * factor * math.cos(bearing), speed * factor * math.sin(bearing), 0

    timeout = core._timeout(dist0, speed) + (360 / turn_speed if forward_only else 0)  # + time for turning
    while True:
        finished = core._run_move('target', timeout, step)
        if state['blocked'] == 'turn' and not state['backed_up'] and not core._path_blocked(math.pi, 0.12, clearance):
            print('goto_map: too close to turn, backing up 10 cm', flush=True)
            state['backed_up'] = True
            backward(min(speed, 0.08), 0.10)
            state.update(blocked=False, turning=True, last=math.inf)
            continue
        if state['blocked'] == 'turn':
            print('goto_map: too close to something to turn, stopped', flush=True)
        return finished and not state['blocked']
