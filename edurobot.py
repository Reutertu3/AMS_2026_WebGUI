"""edurobot.py - the ROS plumbing behind motion.py: connection, odometry, sending speeds.
Like the Arduino core: normally you don't touch this. Scripts use `from motion import *`.

Importing it connects to ROS and waits for odometry (up to 10 s). Ctrl+C (or Stop in the web
page) stops the robot, also after an error.

How the robot is driven: geometry_msgs/Twist on "cmd_vel", at most every CMD_INTERVAL (the robot
can't handle more, see README.md). Odometry: real robot odom -> base_link on /tf, turtlesim
"turtle_pose" (ros.sh --turtle remaps it). Movements keep going until the measured distance / angle
is reached.
"""
import atexit
import math
import signal
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from turtlesim_msgs.msg import Pose as TurtlePose

try:  # built from src/ with colcon, see README.md
    from edurob_interfaces.srv import CalibrateGyro
except ImportError:
    CalibrateGyro = None

__all__ = [
    'delay', 'millis', 'calibrate_gyro', 'stop',
    'get_x', 'get_y', 'get_yaw', 'print_pose',
]

CHECK_INTERVAL = 0.05  # s, how often the odometry is checked
# The real robot (micro-ROS over WiFi) handles only ~2 commands/s. Faster ones pile up and are
# executed seconds late (it keeps driving after stop). Measured: 0.5 s ok, 0.3 s already lags.
CMD_INTERVAL = 0.5     # s, speed commands are sent at most this often (stop always)
# Real robot: odometry arrives late and commands take effect late (~0.6 s together, measured).
# The pose is predicted this far ahead with the last command, and stop() waits this long.
# Robot overshoots -> increase, stops short -> decrease. Not used for the turtle (no delay there).
LATENCY = 0.6          # s
WAIT_FOR_POSE = 10.0   # s
# Real robot: what its wheels can do. A mecanum wheel turns at  forward ± sideways ± WHEEL_LEVER × rotation,
# so the fastest wheel needs  |x| + |y| + WHEEL_LEVER × |turn in rad/s|. If that is more than a wheel can
# do, the robot stalls for several seconds instead of just driving slower (measured: an orbit = strafe +
# turn loads two diagonal wheels double and stopped every ~5 s for ~13 s; pure strafing or turning at the
# same speeds ran smoothly). So every command is scaled down until the fastest wheel stays within
# WHEEL_MAX, all three speeds by the same factor: curves and orbits keep their shape, just slower.
# Not used for the turtle. Robot still stalls -> lower WHEEL_MAX or raise WHEEL_LEVER.
WHEEL_MAX = 0.16       # m/s, fastest a wheel goes (commanded 0.25 m/s forward gave ~0.16 m/s)
WHEEL_LEVER = 0.25     # m, half length + half width of the wheel base (estimate)



# ---------- connection (set up on import) ----------

rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
signal.signal(signal.SIGTERM, signal.default_int_handler)  # treat "kill" like Ctrl+C: stop the robot
_node = rclpy.create_node('edurobot')
_pub = _node.create_publisher(Twist, 'cmd_vel', 1)
_tf = Buffer()
_tf_listener = TransformListener(_tf, _node)
_turtle = None
_node.create_subscription(TurtlePose, 'turtle_pose', lambda m: globals().update(_turtle=(m.x, m.y, m.theta)), 1)
_scan = None       # latest lidar scan and when it arrived (for the obstacle stop)
_scan_time = 0.0
_node.create_subscription(LaserScan, 'scan', lambda m: globals().update(_scan=m, _scan_time=time.monotonic()),
                          qos_profile_sensor_data)
_executor = SingleThreadedExecutor()
_executor.add_node(_node)
_spin_thread = threading.Thread(target=_executor.spin, daemon=True)  # handles incoming messages in the background
_spin_thread.start()

_start_time = time.monotonic()
_real_robot = False          # odometry comes from /tf (not the turtle)
_last_cmd = (0.0, 0.0, 0.0)  # last sent command: m/s, m/s, rad/s
_last_pub = 0.0


_limit_warned = False


def _speed_limit(x, y, turn_deg):
    """Factor 0..1 that keeps a command within what the real robot's wheels can do (see WHEEL_MAX)."""
    global _limit_warned
    if not _real_robot:
        return 1.0
    demand = (abs(x) + abs(y) + WHEEL_LEVER * abs(math.radians(turn_deg))) / WHEEL_MAX
    if demand <= 1:
        return 1.0
    if not _limit_warned:
        _limit_warned = True
        print(f'Note: slowed down to what the wheels can do ({WHEEL_MAX} m/s per wheel, see WHEEL_MAX in '
              f'edurobot.py)', flush=True)
    return 1 / demand


def _publish(x, y, turn_deg):
    """Send a speed command (rate-limited, speed-limited). Returns the speed factor that was applied."""
    global _last_cmd, _last_pub
    now = time.monotonic()
    is_stop = x == 0 and y == 0 and turn_deg == 0
    f = _speed_limit(x, y, turn_deg)
    if now - _last_pub < CMD_INTERVAL and (not is_stop or _last_cmd == (0, 0, 0)):
        return f  # the robot still drives the last command (or already stands), see CMD_INTERVAL
    x, y, turn_deg = x * f, y * f, turn_deg * f
    _last_pub = now
    _last_cmd = (x, y, math.radians(turn_deg))
    m = Twist()
    m.linear.x, m.linear.y, m.angular.z = float(x), float(y), math.radians(turn_deg)
    _pub.publish(m)
    return f


def _read_pose():
    """Current (x, y, yaw rad) in the odometry frame, or None."""
    global _real_robot
    try:
        t = _tf.lookup_transform('odom', 'base_link', rclpy.time.Time()).transform
    except Exception:
        return _turtle
    q = t.rotation
    x, y = t.translation.x, t.translation.y
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    _real_robot = True
    # Predict where the robot will be once the delay has passed (see LATENCY)
    vx, vy, w = _last_cmd
    x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * LATENCY
    y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * LATENCY
    return x, y, yaw + w * LATENCY


def _wait_for_pose():
    end = time.monotonic() + WAIT_FOR_POSE
    while time.monotonic() < end:
        p = _read_pose()
        if p:
            return p
        time.sleep(0.1)
    print('No odometry received. Is the robot (or turtlesim) running? (./start.sh, robot rebooted?)',
          file=sys.stderr, flush=True)
    sys.exit(1)


def _excepthook(kind, value, tb):
    if issubclass(kind, KeyboardInterrupt):
        print('Stopped.', file=sys.stderr, flush=True)  # Ctrl+C / Stop button, no traceback needed
    else:
        sys.__excepthook__(kind, value, tb)


sys.excepthook = _excepthook


@atexit.register
def _shutdown():
    # Runs at the end of the script, also after Ctrl+C or an error: the robot must stop
    if rclpy.ok():
        m = Twist()
        _pub.publish(m)
        time.sleep(0.1)
        _pub.publish(m)
        time.sleep(0.1)
        _executor.shutdown(timeout_sec=1)
        _spin_thread.join(1)
        _node.destroy_node()
        rclpy.try_shutdown()


_pose = _wait_for_pose()  # last known position
_start_pose = _pose       # get_x() etc. are relative to it


# ---------- helpers ----------

def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _update_pose():
    global _pose
    p = _read_pose()
    if p:
        _pose = p
    return _pose


def _relative_pose():
    """Pose seen from the start pose (start = 0, 0, 0 facing +x), yaw in radians."""
    x, y, yaw = _update_pose()
    sx, sy, syaw = _start_pose
    dx, dy = x - sx, y - sy
    c, s = math.cos(syaw), math.sin(syaw)
    return c * dx + s * dy, -s * dx + c * dy, _wrap(yaw - syaw)


def _timeout(amount, vel):
    """Safety limit: twice the expected time + 5 s (e.g. robot stuck)."""
    return 5.0 if vel == 0 else abs(amount) / abs(vel) * 2 + 5


def _ramp_factor(done, remaining, speed, accel, min_speed):
    """Speed factor 0..1: speed up after the start, slow down before the goal. v = sqrt(2 a s)."""
    if speed <= 0:
        return 1.0
    v = min(speed, math.sqrt(2 * accel * max(done, 0.0)), math.sqrt(2 * accel * max(remaining, 0.0)))
    return max(v, min(min_speed, speed)) / speed


def _run_move(what, timeout, step):
    """The odometry loop shared by all movements: every CHECK_INTERVAL read the pose, ask step()
    for (x, y, turn) or None when done. Returns False after a timeout."""
    start = time.monotonic()
    slowest = 1.0  # speed limit factor: a slowed-down move gets correspondingly more time
    try:
        while True:
            _update_pose()
            c = step()
            if c is None:
                break
            if time.monotonic() - start > timeout / slowest:
                print(f'Timeout, {what} not reached', file=sys.stderr, flush=True)
                return False
            slowest = min(slowest, _publish(*c))
            time.sleep(CHECK_INTERVAL)
        return True
    finally:
        stop()


def _map_pose():
    """Robot pose on the SLAM map (x, y, yaw rad), drift-corrected unlike odometry; None without SLAM.
    Predicted LATENCY ahead with the last command, like the odometry (see _read_pose)."""
    try:
        t = _tf.lookup_transform('map', 'base_fix', rclpy.time.Time()).transform
    except Exception:
        return None
    q = t.rotation
    x, y = t.translation.x, t.translation.y
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    vx, vy, w = _last_cmd
    x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * LATENCY
    y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * LATENCY
    return x, y, yaw + w * LATENCY


def _obstacle_ahead(direction, distance, half_width=math.radians(35)):
    """True if the lidar sees something closer than `distance` m within ±half_width of `direction`
    (rad, robot frame, 0 = front), or if there is no current scan (can't see = don't drive)."""
    m = _scan
    if m is None or time.monotonic() - _scan_time > 1.0:
        return True
    for i, r in enumerate(m.ranges):
        if 0.12 < r < distance:  # < 0.12 m: the robot itself / noise
            a = m.angle_min + i * m.angle_increment  # lidar frame = robot frame (not rotated, measured)
            if abs(_wrap(a - direction)) < half_width:
                return True
    return False


# ---------- Arduino-like helpers ----------

def millis():
    """Milliseconds since the start of the script."""
    return int((time.monotonic() - _start_time) * 1000)


def delay(ms):
    """Wait ms milliseconds."""
    time.sleep(ms / 1000)


def calibrate_gyro(seconds=5):
    """Calibrate the robot's gyro (IMU): measures its drift for `seconds` while the robot stands
    still. Stops the robot first. Returns False if it failed (no robot, e.g. turtlesim)."""
    if CalibrateGyro is None:
        print('calibrate_gyro: edurob_interfaces not built (see README.md)', file=sys.stderr, flush=True)
        return False
    stop()
    client = _node.create_client(CalibrateGyro, 'calibrategyro')
    try:
        if not client.wait_for_service(timeout_sec=3):
            print('calibrate_gyro: no /calibrategyro service (real robot connected?)', file=sys.stderr, flush=True)
            return False
        req = CalibrateGyro.Request()
        req.time = max(1, min(255, int(seconds)))
        print(f'Calibrating gyro for {req.time} s, keep the robot still...', flush=True)
        future = client.call_async(req)
        end = time.monotonic() + req.time + 5
        while not future.done() and time.monotonic() < end:
            time.sleep(0.05)
        if not future.done():
            print('calibrate_gyro: no answer from the robot', file=sys.stderr, flush=True)
            return False
        o = future.result().offsets
        print(f'Gyro calibrated, offsets x={o.x:.5f} y={o.y:.5f} z={o.z:.5f}', flush=True)
        return True
    finally:
        _node.destroy_client(client)


# ---------- stop ----------

def stop():
    """All speeds to 0. On the real robot it waits until the robot has really stopped."""
    _publish(0, 0, 0)
    if _real_robot:
        time.sleep(LATENCY)  # let the robot really stop and the odometry catch up


# ---------- odometry (where the robot is, measured from the start of the script) ----------

def get_x():
    """m, forward at start = +x."""
    return _relative_pose()[0]


def get_y():
    """m, left at start = +y."""
    return _relative_pose()[1]


def get_yaw():
    """degrees, -180..180, positive = turned left."""
    return math.degrees(_relative_pose()[2])


def print_pose():
    """Prints e.g. "x=0.100 m  y=0.000 m  yaw=-90.0 deg"."""
    x, y, yaw = _relative_pose()
    print(f'x={x:.3f} m  y={y:.3f} m  yaw={math.degrees(yaw):.1f} deg', flush=True)
