# Gripper

The robot has a gripper driven by one Dynamixel XL330 servo. The firmware (`Edurob_Firmware/main/servo.cpp`)
does the real work on the robot. `gripper.py` is the Python side: helpers to use in scripts, like
`motion.py` for driving.

```python
from motion import *
from gripper import *

release()                 # open
forward(0.1, 0.2)         # drive to the object
if grip():                # close; True = stopped on an object
    print('got it')
    backward(0.1, 0.2)
    release()
```

Try it: `scripts/gripper_demo.py` (web page → Scripts → Run). In the web page's command box
`gripper.py` is already imported, so `grip()` or `release()` work there directly.

---

## 1. Functions

Every command **waits until the jaws have stopped**, like the movements in `motion.py`. Then the
next line runs.

`speed` and `force` are **percentages** (1 … 100): `speed=30` means 30 % of the servo's top speed,
`force=40` means 40 % of its maximum current. Leave them out to use the current settings
(default speed 20 %, force 17 %, see 2.).

| Function | Does |
| --- | --- |
| `grip()` | Close. Stops on an object and keeps holding it with the force limit. Returns `True` if it holds something, `False` if it closed all the way. |
| `grip(speed=10, force=40)` | The same, slowly (10 %) and squeezing harder (40 % ≈ 700 mA). |
| `release()` | Open to the open end stop. |
| `release(speed=50)` | Open fast (50 %). |
| `sweep()` | Open and close 3 times (test). Stops early if something blocks the jaws. Ends closed. |
| `sweep(5)` / `sweep(2, speed=40)` | 5 times / 2 times at 40 % speed. |
| `gripper_torque(on)` | `False`: the servo goes limp, so you can move the jaws by hand. `True`: it holds where it is now. |
| `set_gripper(speed=30, force=25)` | Change settings, leave out what should stay. Also `open_deg=`, `closed_deg=`, `sweep_count=` (see 2.). |
| `get_gripper_settings()` | The settings as a dict: `speed`, `force`, `open_deg`, `closed_deg`, `sweep_count`. |
| `get_gripper_angle()` | Jaw angle in degrees (about 92 closed … 180 open). `None` if there's no gripper data. |
| `get_gripper_current()` | Motor current in mA. It's proportional to the grip force; the sign shows the direction. |
| `is_gripping()` | `True` if the jaws push against something right now. |
| `print_gripper()` | Prints angle, speed and current, e.g. `gripper: 178.9 deg, 0.0 rpm, 11 mA`. |

`speed`, `force` and the count given to `grip`/`release`/`sweep` **are kept** for the following
commands (the same as calling `set_gripper` first): after `grip(force=40)`, a plain `grip()` also
squeezes with 40 %.

If the robot doesn't answer, refuses a value or isn't ready, the functions raise an error, which
stops the script.

## 2. Settings

The settings are ROS parameters of the robot node `/Edurob`. Unlike the wheel parameters, they are
**only kept in RAM**: after every robot reboot they are back to the firmware defaults.

| `set_gripper(...)` | Parameter | Default | Range | Meaning |
| --- | --- | --- | --- | --- |
| `speed` | `gripper_speed_pct` | 20 | >0 … 100 % | % of the servo's maximum speed. 20 % ≈ 120 °/s, fully open → closed in ~0.7 s. |
| `force` | `gripper_force_pct` | 17 | >0 … 100 % | % of the servo's current limit (1750 mA): 17 % ≈ 300 mA. Higher = squeezes harder. |
| `open_deg` | `gripper_open_deg` | 180 | 0 … 360° | open end stop |
| `closed_deg` | `gripper_closed_deg` | 92 | 0 … 360° | closed end stop, jaws touching with nothing in between |
| `sweep_count` | `gripper_sweep_count` | 3 | ≥ 1 | open/close cycles of `sweep()` |

The robot refuses values outside the range (`set_gripper(force=150)` → error).

**Finding the end stops:** `gripper_torque(False)`, move the jaws by hand to fully open / fully
closed, read `get_gripper_angle()` each time, then `gripper_torque(True)` and
`set_gripper(open_deg=…, closed_deg=…)`. To keep them after a reboot, change `DefaultOpenDeg` /
`DefaultClosedDeg` in `Edurob_Firmware/main/include/servo.h` and flash the robot.

## 3. How it works

### On the robot (`servo.cpp`)

- The servo runs in **current-based position mode**: it moves to a goal angle, but its motor
  current never goes above the force limit. When the jaws close on an object they stall there and
  squeeze with exactly that limited force. That's why `grip()` can't crush things (at a sensible
  `force`).
- `speed` sets the servo's profile velocity and `force` sets its goal current, both as % of the
  limits read from the servo at start-up. So no value can exceed what the servo allows.
- A background task checks the running command every 50 ms:
  - reached the end stop (within 2°) → done. A grip that reaches the closed end stop means there was
    nothing in the jaws.
  - **stalled** (current ≥ 70 % of the force limit and slower than 1 rpm) → a grip keeps holding the
    object, a release/sweep stops pushing and holds where it is.
  - took longer than 2 × the expected travel time + 1 s → stops (jammed).
- **A new command cancels the running one.** The firmware's own commands are non-blocking. That's
  why `gripper.py` waits after each one.

### In `gripper.py`

The firmware doesn't publish when a command is finished. So `gripper.py` waits until the jaws have
**stopped moving** on `/gripper/state`:

1. Call the service, wait 0.6 s (`core.LATENCY`: the command arrives and the servo starts).
2. Wait until the speed stays below 1 rpm for 0.4 s. `sweep()` waits 1 s instead, because the jaws
   briefly stand still at each end stop when they turn around.
3. Give up after 15 s per move (`TIMEOUT`), with a warning.

`grip()` then compares the angle with `closed_deg`: more than 2° away → something is in the jaws.
`is_gripping()` instead uses the current, like the firmware: |current| ≥ 70 % of
`force` % × 1750 mA while not moving.

Measured on the robot at default settings: `grip()` with nothing in the jaws took 2.0 s and
ended at 93.6°. `release()` took 1.8 s. `sweep(2)` took 5.0 s.

The calibration knobs are at the top of `gripper.py`: `CURRENT_LIMIT_MA`, `GRIP_DETECT_RATIO`,
`STILL_RPM`, `MOVE_TOLERANCE`, `TIMEOUT`. They match the firmware's values.

## 4. ROS interface (without Python)

| Name | Type | |
| --- | --- | --- |
| `/gripper/grip` | `std_srvs/srv/Trigger` | start closing |
| `/gripper/release` | `std_srvs/srv/Trigger` | start opening |
| `/gripper/sweep` | `std_srvs/srv/Trigger` | start `gripper_sweep_count` cycles |
| `/gripper/torque` | `std_srvs/srv/SetBool` | `data: false` = limp, `true` = hold |
| `/gripper/state` | `sensor_msgs/msg/JointState`, ~6 Hz | `position` rad, `velocity` rad/s, `effort` = current in mA |

The services return immediately ("grip started"). Speed, force and count come from the parameters.

```bash
./ros.sh ros2 service call /gripper/grip std_srvs/srv/Trigger
./ros.sh ros2 service call /gripper/torque std_srvs/srv/SetBool "{data: false}"
./ros.sh ros2 param set /Edurob gripper_force_pct 30.0
./ros.sh ros2 topic echo /gripper/state
```

## 5. Things to know

- **After a robot reboot the gripper isn't ready at once.** The ROS services appear first. Then the
  firmware searches for the servo (up to ~20 s if none answers) and does a start-up sweep: 2 cycles,
  then open. Commands in that time fail with "servo not ready", or cancel the start-up sweep.
- **Settings reset on every reboot** (see 2.).
- If the servo isn't found (no 5 V, wiring, wrong DIR pin), every command fails with "servo not
  ready". The cause is in the robot's serial log ("No servo answered at any baud rate").
- The servo has its own serial port (UART2, GPIO 16/15, DIR GPIO 3). It doesn't share pins with the
  lidar (UART1, GPIO 43/44).
