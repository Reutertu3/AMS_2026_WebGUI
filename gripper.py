"""gripper.py - gripper helpers for the EduRob, Arduino style (GRIPPER.md explains everything).

Use it in a script like this (together with motion.py, or alone):

    from motion import *
    from gripper import *

    release()                  # open, waits until the jaws stop
    forward(0.1, 0.2)
    if grip():                 # close; True = stopped on an object
        print('got it')
    print_gripper()

Every command blocks until the jaws have stopped moving, like the movements in motion.py.
The gripper runs on the robot itself (firmware servo.cpp): these functions only call its ROS
services (/gripper/grip, /release, /sweep, /torque), set its gripper_* parameters and read
/gripper/state. Units: angles in degrees, speed and force in percent of the servo's limits.
"""
import math
import sys
import time

from rcl_interfaces.msg import ParameterType
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

import edurobot as core

__all__ = [
    'grip', 'release', 'sweep', 'gripper_torque', 'set_gripper', 'get_gripper_settings',
    'get_gripper_angle', 'get_gripper_current', 'is_gripping', 'print_gripper',
]

ROBOT_NODE = '/Edurob'   # the firmware's node, owns the gripper_* parameters
# Calibration knobs, same meaning as in the firmware (servo.cpp)
CURRENT_LIMIT_MA = 1750  # XL330 factory current limit, force % refers to it
GRIP_DETECT_RATIO = 0.7  # gripping = |current| >= this × force limit while not moving
STILL_RPM = 1.0          # slower than this counts as stopped
MOVE_TOLERANCE = 2.0     # degrees, closer than this to the closed end stop = nothing in the jaws
TIMEOUT = 15.0           # s per move: 88 degrees take ~0.7 s at 20 % speed, ~15 s at 1 %

_state = None  # latest /gripper/state: (time, degrees, rpm, mA)


def _on_state(m):
    global _state
    if m.position:
        _state = (time.monotonic(), math.degrees(m.position[0]),
                  m.velocity[0] * 60 / (2 * math.pi) if m.velocity else 0.0,
                  m.effort[0] if m.effort else 0.0)


core._node.create_subscription(JointState, 'gripper/state', _on_state, 10)
_params = AsyncParameterClient(core._node, ROBOT_NODE)
_clients = {}


def _wait(future, what, timeout=5.0):
    end = time.monotonic() + timeout
    while not future.done() and time.monotonic() < end:
        time.sleep(0.05)
    if not future.done():
        raise RuntimeError(f'gripper: no answer from the robot ({what})')
    return future.result()


def _call(srv_type, name, req):
    client = _clients.get(name)
    if client is None:
        client = _clients[name] = core._node.create_client(srv_type, name)
    if not client.wait_for_service(timeout_sec=3):
        raise RuntimeError(f'gripper: no {name} service (robot connected, gripper firmware flashed?)')
    res = _wait(client.call_async(req), name)
    if not res.success:
        raise RuntimeError(f'gripper: {name} {res.message}')


def _wait_still(still_s, timeout):
    """Block until the jaws haven't moved for still_s seconds. The firmware's own job flag isn't published."""
    start = time.monotonic()
    time.sleep(core.LATENCY)  # the command needs a moment to arrive and start the servo
    still_since = None
    while time.monotonic() - start < timeout:
        s = _state
        if s is not None and time.monotonic() - s[0] < 1.0 and abs(s[2]) < STILL_RPM:
            still_since = still_since or time.monotonic()
            if time.monotonic() - still_since >= still_s:
                return True
        else:
            still_since = None
        time.sleep(0.05)
    print('gripper: still moving after the timeout (or no /gripper/state)', file=sys.stderr, flush=True)
    return False


# ---------- settings (robot RAM: they stay until the robot reboots) ----------

_NAMES = {'speed': 'gripper_speed_pct', 'force': 'gripper_force_pct', 'open_deg': 'gripper_open_deg',
          'closed_deg': 'gripper_closed_deg', 'sweep_count': 'gripper_sweep_count'}


def set_gripper(speed=None, force=None, open_deg=None, closed_deg=None, sweep_count=None):
    """Change the gripper settings on the robot; leave out what should stay.
    speed, force: 1..100 % of the servo's limits. open_deg, closed_deg: end stops 0..360 degrees."""
    given = {'speed': speed, 'force': force, 'open_deg': open_deg, 'closed_deg': closed_deg,
             'sweep_count': sweep_count}
    new = [Parameter(_NAMES[k], Parameter.Type.INTEGER, int(v)) if k == 'sweep_count'
           else Parameter(_NAMES[k], Parameter.Type.DOUBLE, float(v))
           for k, v in given.items() if v is not None]
    if not new:
        return
    if not _params.wait_for_services(timeout_sec=5):
        raise RuntimeError(f'gripper: {ROBOT_NODE} parameter service not found (robot connected?)')
    results = _wait(_params.set_parameters(new), 'set parameters').results
    failed = [f'{p.name}={p.value}' for p, r in zip(new, results) if not r.successful]
    if failed:
        raise ValueError('gripper: the robot refused ' + ', '.join(failed) + ' (out of range, see GRIPPER.md)')


def get_gripper_settings():
    """The robot's gripper settings as a dict: speed, force, open_deg, closed_deg, sweep_count."""
    if not _params.wait_for_services(timeout_sec=5):
        raise RuntimeError(f'gripper: {ROBOT_NODE} parameter service not found (robot connected?)')
    values = _wait(_params.get_parameters(list(_NAMES.values())), 'get parameters').values
    return {k: v.integer_value if v.type == ParameterType.PARAMETER_INTEGER else v.double_value
            for k, v in zip(_NAMES, values)}


# ---------- commands, each waits until the jaws stop ----------

def grip(speed=None, force=None):
    """Close the jaws. Stops on an object and keeps holding it with the force limit.
    Returns True if it holds something, False if it closed all the way (nothing in the jaws)."""
    set_gripper(speed=speed, force=force)
    _call(Trigger, 'gripper/grip', Trigger.Request())
    _wait_still(0.4, TIMEOUT)
    closed = get_gripper_settings()['closed_deg']
    angle = get_gripper_angle()
    return angle is not None and abs(angle - closed) > MOVE_TOLERANCE


def release(speed=None, force=None):
    """Open the jaws to the open end stop."""
    set_gripper(speed=speed, force=force)
    _call(Trigger, 'gripper/release', Trigger.Request())
    _wait_still(0.4, TIMEOUT)


def sweep(count=None, speed=None, force=None):
    """Open and close `count` times (test / show-off). Stops early if something blocks the jaws.
    Ends closed."""
    set_gripper(speed=speed, force=force, sweep_count=count)
    _call(Trigger, 'gripper/sweep', Trigger.Request())
    # 1 s still: at each end stop the servo turns around, briefly standing still
    _wait_still(1.0, TIMEOUT * 2 * (count or get_gripper_settings()['sweep_count']))


def gripper_torque(on):
    """on=False: servo goes limp, the jaws can be moved by hand. on=True: holds where it is now."""
    _call(SetBool, 'gripper/torque', SetBool.Request(data=bool(on)))


# ---------- reading the state ----------

def _fresh_state():
    s = _state
    if s is None or time.monotonic() - s[0] > 1.0:
        time.sleep(0.5)  # a new script gets the first message after a moment
        s = _state
    return s


def get_gripper_angle():
    """Jaw servo angle in degrees (about 92 closed .. 180 open), None if there is no gripper data."""
    s = _fresh_state()
    return None if s is None else s[1]


def get_gripper_current():
    """Motor current in mA (sign = direction). Proportional to the grip force."""
    s = _fresh_state()
    return None if s is None else s[3]


def is_gripping():
    """True if the jaws push against something right now (current near the force limit, not moving)."""
    s = _fresh_state()
    if s is None:
        return False
    force_ma = get_gripper_settings()['force'] / 100 * CURRENT_LIMIT_MA
    return abs(s[3]) >= GRIP_DETECT_RATIO * force_ma and abs(s[2]) < STILL_RPM


def print_gripper():
    s = _fresh_state()
    if s is None:
        print('gripper: no /gripper/state (robot connected, gripper firmware flashed?)', flush=True)
    else:
        print(f'gripper: {s[1]:.1f} deg, {s[2]:.1f} rpm, {s[3]:.0f} mA', flush=True)
