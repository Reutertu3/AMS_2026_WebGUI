# Notes

All commands are run from the `ws` folder.

## Files

| File | What it is |
|---|---|
| `scripts/*.py` | My programs (movement scripts), one file each. The web page lists them. |
| `motion.py` | All movement helpers (`from motion import *`): forward … orbit … goto, plus delay, print_pose, calibrate_gyro |
| `edurobot.py` | The ROS plumbing behind it (connection, odometry, send limit, delay compensation), normally don't touch |
| `webui.py` / `webui.html` | Web control: drive by hand, run scripts, odometry + lidar view |
| `ros.sh` | Runs commands inside the ROS 2 Docker container (domain 12, `--turtle` = turtlesim) |
| `start.sh` / `stop.sh` | Check the network, start/stop everything in the background (agent, lidar, SLAM, web control) |
| `lidar_watchdog.sh` | Restarts the lidar driver when it stops sending scans (e.g. after a robot reboot) |
| `odom_fix.py` | Re-stamps the robot odometry for SLAM (the robot's timestamps are broken) |
| `slam_params.yaml` | slam_toolbox settings (map resolution etc.) |
| `slam.Dockerfile` | Image with slam_toolbox for ROS kilted (built by `start.sh --slam`) |
| `maps/` | Saved maps (`.pgm` picture + `.yaml` info) |
| `explore.py` | Planning for autonomous mapping: frontiers, A* path around walls (numpy, no ROS, testable offline) |
| `scripts/mapping.py` | Autonomous mapping: drives to unexplored edges until the map is complete, then saves it |
| `.gitignore` | Build output, `__pycache__`, `.venv`, private `scripts/_*.py`, plan pictures |
| `src/edurob_interfaces` | Robot message definitions (gyro calibration), built once with colcon |
| `legacy_cpp/` | The old C version (`main.c` + `edurobot.cpp`), not used anymore |
| `README.md` | Setup, troubleshooting, all findings in detail |

## Write a script

A new file in `scripts/`, e.g. `scripts/my_test.py`:

```python
from motion import *     # all movement helpers; connects to the robot, waits for odometry

forward(0.2, 0.5)        # runs top to bottom, like setup() on Arduino
rotate_left(90, 30)
print_pose()
```

No compiling: Python runs the file directly. Save, then press Run in the web page (⟳ reloads the list).
Use `print()` like `Serial.print()` on Arduino, the output appears in the web page.
`while True:` gives you an Arduino `loop()`.

## Run

On the real robot:

```bash
./start.sh                           # prints the web page address → pick script → Run
./start.sh --slam                    # same, and build a map while driving
./ros.sh python3 scripts/square.py   # or directly in the terminal (don't drive in the page meanwhile)
./stop.sh                            # stops everything (an unsaved map is lost!)
```

`start.sh` options (can be combined): `--slam` (mapping), `--local` (page only on this laptop),
`PORT=8081`, `DOMAIN=13`. Running it again is safe: it only starts what isn't running and shows the status.

What `start.sh` checks, in this order (✓ ok, ● fine but worth knowing, ✗ problem + hint):
1. **Robot pingable?** No → aborts, nothing is started (robot off / other WLAN).
2. **Laptop IP**, and whether the robot's settings page has exactly this IP as Agent IP and domain 12.
   Wrong → aborts and says what to enter.
3. **Services:** micro-ROS agent, lidar driver, SLAM (with `--slam`), web control. If the laptop got
   a new IP since a container was started, that container is restarted (except the agent).
4. **Robot data:** waits up to 30 s for odometry, lidar and (with `--slam`) the map.

If *Odometry* is ✗: reboot the robot (http://10.0.109.58/ → Reboot ESP32). It stops looking for the
agent after a while, and after the agent restarts (`./stop.sh`) it has to be rebooted too.
The lidar comes back by itself ~20 s after a robot reboot (watchdog).

On the simulated turtle (turtlesim window opens, the real robot can't move):

```bash
./ros.sh --turtle python3 webui.py
./ros.sh --turtle python3 scripts/square.py
```

The script ends at its last line. Ctrl+C (terminal) or STOP / Stop (web page) ends it early.
The robot/turtle is always stopped at the end, also after an error.

## Robot and network facts

| What | Value |
|---|---|
| Robot IP | `10.0.109.58`, settings page http://10.0.109.58/configuration, reboot button on http://10.0.109.58/ |
| Robot settings | Agent IP = **laptop IP** (DHCP, can change! `start.sh` shows it), Agent Port `8888`, DomainID `12` |
| WLAN | "Blockwoche WLAN", ping to the robot 100–350 ms |
| ROS domain | 12 everywhere (older files used 1 or 2: wrong) |
| Topics from the robot | `/cmd_vel` (in), `/tf` odom → base_link (5 Hz), `/tf_static`, `/imu` (5 Hz, gyro + accel, no orientation), service `/calibrategyro` |
| Lidar | RPLIDAR S1 over TCP `10.0.109.58:44227`, `/scan` ~8 Hz, 1080 points, 0.05–12 m, mounted **not rotated** (measured) |
| Max speed | commanded 0.25 m/s → drives ~0.16 m/s |
| Wheel limit | a wheel does max ~0.16 m/s. Mecanum wheel speed = forward ± sideways ± 0.25 m × rotation (rad/s). If one wheel would need more, the robot **stalls** for seconds. `edurobot.py` and the web page scale every command so the fastest wheel stays ≤ 0.16 m/s (`WHEEL_MAX`, `WHEEL_LEVER`) |
| Commands | at most **2 per second**, more pile up and run late (see Problems) |
| Delay | command → movement → odometry: ~0.6 s together |
| Without commands | the robot stops by itself after ~2 s |

## Web control

`./start.sh` starts it; address is printed at the end (http://<laptop IP>:8080). Everything at once:

- **STOP** / `Space`: stops driving and a running script.
- **Scripts**: pick a file from `scripts/`, Run, output below. While it runs: no manual driving,
  no calibration, no second script.
- **Drive**: hold the buttons or `W`/`S` forward/back, `A`/`D` strafe, `Q`/`E` rotate (combinable).
  Letting go stops. If the browser goes quiet for 0.8 s (tab closed, WiFi), the robot is stopped.
- **Sliders**: speed, rotation speed, ramp (0 = full speed at once).
- **Map view**: trail (blue), robot (orange, arrow = front), lidar (red), SLAM map (grey free, white walls).
  Wheel = zoom, drag = move, Fit, Clear trail. Zoomed out, walls and small obstacles are drawn at
  least 3 px so e.g. a 7 cm barrel doesn't vanish.
- **Gyro**: Calibrate gyro (robot must stand still).
- **Map (SLAM)**: name → Save map. *New map* deletes the current map and starts a new one (asks first).
- **ROS domain**: Switch restarts the server on another domain.
- Log of every start/stop with reason: `docker logs -f edurob-webui`.

## Mapping (SLAM)

```bash
./start.sh --slam    # first time builds the slam image (~2 min)
```

Then drive around in the web page; the map grows under the lidar view. Save it with a name →
`maps/<name>.pgm` + `.yaml` (nav2 format). Start over: *New map* button (asks first, starts at the robot's
current position).
**`./stop.sh` or restarting SLAM loses an unsaved map.**

Tips: slow (≤ 0.15 m/s), gentle turns, no strafing (mecanum wheels slip sideways), calibrate the gyro
first, come back to known places (loop closure fixes drift). Small objects: drive close
(at 0.5 m a 7 cm barrel gets ~24 lidar points, at 2 m only ~6) and around them.

How it fits together:
```
robot /tf (odom -> base_link, broken stamps) --odom_fix.py--> odom_fix -> base_fix -> laser_fix
lidar /scan (frame laser_fix) -----------------------------------------------------┘
slam_toolbox: scans + odom_fix tree -> /map + map -> odom_fix
web page: /map + map -> base_fix (robot position on the map)
```

**Map resolution** (`resolution` in `slam_params.yaml`, restart SLAM after changing):

| Cell size | Result |
|---|---|
| 5 cm (default) | 7 cm barrel = 1–2 cells, hard to see |
| 2 cm | barrel = ring of ~5 × 6 cells (10–12 cm: barrel + lidar spread), `maps/barrel.pgm`, `maps/before_1cm.pgm` |
| 1 cm (now, test) | 4× as many cells. The lidar itself is only accurate to ~2–3 cm, so walls get thicker/noisier rather than sharper. Map updates every 3 s; the web server gzips it (1.5 MB → ~12 KB) |

Odometry and IMU can't raise the resolution: they only say *where* a scan was taken, the lidar
decides how precise it is. The IMU (5 Hz, gyro + accel only, same broken timestamps) is used inside
the robot, but not for the published odometry. Firmware (`Edurob_Firmware/`, drive code in
`../edurobv2-feature-ROSDriveHAL/components/DriveHAL/DriveHAL.cpp`): `/odom` heading comes only from
the wheel encoders (wheel radius + wheel positions). A second pose with the gyro's heading (once the
gyro is calibrated) is only the feedback for the robot's own speed controller. CALIBRATE.md has more.

## Autonomous mapping (mapping.py)

Goal: robot in a room (~8 × 20 m) with an obstacle course of walls, map it outside and inside by itself.
Approach: **frontier exploration**. A frontier is the edge between known free space and unknown space.
Repeat: wait for the map → find frontiers → nearest reachable one → A* path around walls → drive the
first 3 corners → plan again with the newer map. Done when no reachable frontier is left; then the map
is saved as `maps/mapping_<date>.pgm`.

```bash
./start.sh --slam
# web page -> mapping.py -> Run   (STOP at hand!)
```

Settings at the top of `scripts/mapping.py`: `DRIVE = False` only plans and prints (try that first),
`SPEED` 0.12 m/s, `MAX_RANGE` 11 m (frontiers farther from the start are ignored, so it doesn't
leave the room through a door), `WAYPOINTS_PER_PLAN`, `MAX_ROUNDS`.

Why not Nav2 + explore_lite (the standard ROS way)? Nav2's controller wants to send commands 10–20×
per second with little delay; this robot takes 2 per second, has ~0.6 s delay and stalls when a wheel
is overloaded. Own code on top of `motion.py` already handles all of that.

**New in `motion.py`** (need `./start.sh --slam`):
```python
get_map_pose()                 # (x, y, yaw deg) on the SLAM map, drift-corrected; None without SLAM
goto_map(vel, x, y)            # straight there on the map, strafing, no turning. Stops if the lidar sees
                               # something closer than 0.4 m in the driving direction. True = arrived
goto_map(vel, x, y, forward_only=True)   # turn on the spot first, stop + wait 0.6 s, then drive forward
                               # only (least slip = least map error). mapping.py uses this
# collision check with the lidar and the robot's footprint (ROBOT_LENGTH/WIDTH in edurobot.py):
#   driving: stop if a point is in the strip the robot sweeps in the next look_ahead m (+ clearance each side)
#   turning: don't turn if a point is inside the corner circle (+ turn_clearance); back up 10 cm, try once more
```
Map coordinates = the ones of the web page map and `maps/*.yaml`. The map pose arrives ~2–5 s after a
program starts; both functions wait up to 8 s for it.

Explained for outsiders, with diagrams: [MAPPING.md](MAPPING.md).

**Planning (`explore.py`)**, works on a 5 cm grid (1 cm map of 8 × 20 m = 1.6 M cells, too slow for A*):
- robot size and clearances are variables at the top of `scripts/mapping.py` (`ROBOT_LENGTH` 0.26,
  `ROBOT_WIDTH` 0.18, `LIDAR_X` 0, `PLAN_MARGIN`, `TURN_CLEARANCE`, `DRIVE_CLEARANCE`, `LOOK_AHEAD`); it
  hands them to `explore.py` and `edurobot.py`
- exact distance to the nearest wall per spot (scipy distance transform on the 1 cm map, after removing
  single noise points): driving needs 9 + 6 cm → passages ≥ 0.30 m; turning needs the 16 cm corner circle
  + 2 cm → path corners and viewpoints only where that fits
- frontiers don't have to be reachable: for each one a **viewpoint** is searched (reachable, clear line of
  sight, ≤ `VIEW_DIST` 2.5 m). The robot drives to the viewpoint; a frontier still there after 2 visits is skipped
- paths within 0.5 m of a wall cost 3× more, so the robot stays in the middle of passages
- frontiers shorter than 0.3 m are ignored (noise between scan rays)
- unknown space is never planned through

Test offline on a saved map, no robot needed:
```bash
./ros.sh python3 explore.py maps/barrel.yaml            # start = map (0, 0)
./ros.sh python3 explore.py maps/barrel.yaml 1.4 0.2    # start somewhere else
```
→ `maps/barrel_plan.png`: white free, black wall, pink too close to a wall, grey unknown,
blue frontiers, green chosen target, red path, orange start.

Tested so far: planning offline on all saved maps (0.01–0.06 s per plan), `goto_map` on the robot
(strafing: 30 cm, 3 cm error; forward_only: turn + 40 cm and back, 2.6–7.3 cm, heading within ~6°;
refuses to drive towards an obstacle 15 cm away), `mapping.py` with `DRIVE = False` on the live map
(sensible 5-corner path around the obstacles).
**Not yet tested: a full autonomous run with `DRIVE = True`.**

For later (barrel): find ~7 cm circles in the lidar points (cluster + circle fit), report their map position.

## Movement functions

```python
# ramp is optional (default True): True = soft start/stop, False = full speed right away
forward(vel, distance, ramp)        # vel in m/s, distance in m
backward(vel, distance, ramp)
strafe_left(vel, distance, ramp)    # slide sideways
strafe_right(vel, distance, ramp)
move_direction(vel, distance, angle, ramp)  # any direction: 0 fwd, 90 left, -90 right, 45 front-left

rotate_left(degree, vel, ramp)      # turn BY degree, vel in degrees/s
rotate_right(degree, vel, ramp)
turn_to(degree, vel, ramp)          # turn TO a heading (0 = start direction)

curve_left(vel, radius, degree, ramp)   # drive along a circle: m/s, m, how many degrees of it
curve_right(vel, radius, degree, ramp)
orbit_left(vel, radius, degree, ramp)   # circle around a point in front, always facing it
orbit_right(vel, radius, degree, ramp)
rotate_around(dx, dy, degree, vel, ramp) # pivot around point (dx in front, dy left)

goto_xy(vel, x, y, ramp)            # straight to a point (from start), without turning
goto_pose(vel, x, y, yaw, ramp)     # to a point and heading at the same time

move_spinning(vel, distance, angle, spin, ramp)  # straight line while spinning at spin degrees/s

drive(x, y, turn, ms)    # all three speeds at once, for ms milliseconds (no odometry)
stop()
delay(ms)
millis()

calibrate_gyro(seconds)  # calibrate the IMU, robot must stand still; False without robot (turtlesim)

print_pose()             # prints "x=0.512 m  y=0.000 m  yaw=-90.7 deg"
get_x(); get_y()         # m, relative to where the script started (forward = +x, left = +y)
get_yaw()                # degrees, -180..180, left = +
```

Examples:

```python
forward(0.1, 0.5)                     # 50 cm forward, soft start/stop
forward(0.1, 0.5, False)              # same, full speed right away
move_direction(0.1, 0.3, 45)          # 30 cm diagonally front-left, still facing forward
turn_to(0, 30)                        # face the start direction again
goto_xy(0.1, 0, 0)                    # back to the start point
goto_pose(0.1, 1.0, 0, 90)            # 1 m forward while turning to face left
orbit_left(0.1, 0.5, 360)             # circle once around an object 50 cm in front
rotate_around(0.2, 0, 90, 30)         # pivot 90 degrees left around a point 20 cm in front
move_spinning(0.1, 1.0, 0, 45)        # 1 m straight ahead while spinning at 45 degrees/s
```

Distances and angles are measured with odometry: the robot keeps driving until it has
really moved that far (real robot: `odom → base_link` on /tf, turtle: `/turtle1/pose`).
The script waits up to 10 s for odometry at startup (`from motion import *`) and quits if there is none.
If a move doesn't finish within 2 × the expected time + 5 s (robot stuck), it gives up and
prints `Timeout`.
`drive()` is the exception: it is still time based.

## How the movement functions work

### The basics: three speeds

The robot only ever gets three numbers, sent as a `Twist` message on `cmd_vel`:

| Speed | Meaning | Unit |
|---|---|---|
| x | forward (+) / backward (−) | m/s |
| y | left (+) / right (−), sideways | m/s |
| turn | left (+) / right (−), counterclockwise is positive | rad/s (the library converts from degrees/s) |

The robot's own controller turns these into wheel speeds (mecanum wheels can do all three at
once). Every movement function is just a choice of these three numbers plus a rule for when to stop.

The robot keeps its last speed until told otherwise (it stops by itself after ~2 s without commands),
so every function repeats the speeds while moving and always ends with `stop()` (all speeds 0).
The real robot can only handle about 2 commands per second over WiFi, so a new speed is sent at most
every 0.5 s (`CMD_INTERVAL`), while the odometry is still checked every 50 ms (see README.md).

### The odometry loop

All functions except `drive()` use the same pattern:

```
remember where the robot is now
repeat every 50 ms:
    read the odometry
    if moved far enough      → done
    if it took far too long  → print "Timeout", give up
    send the speeds
stop()
```

Because the position is only checked every 50 ms, the robot overshoots a little:
up to speed × 50 ms, e.g. up to 1 cm at 0.2 m/s or 1.5° at 30°/s. Slower = more exact,
and ramps fix most of it (see below).

On the real robot there is a bigger delay: the odometry arrives late over WiFi and commands take
effect late (~0.6 s together). So "read the odometry" uses a prediction: measured position + last
speed × `LATENCY` (where the robot will be once the delay has passed), and `stop()` waits `LATENCY`
until the robot has really stopped. Overshoots → increase `LATENCY` in `edurobot.py`, stops short →
decrease it (`scripts/test_accuracy.py` helps). Not used for the turtle.

### Speed ramps (`ramp = true`)

Without a ramp the robot jumps to full speed and stops hard. With `ramp = true`:

- **Speed up:** the speed grows with the distance already covered, `v = sqrt(2 · a · done)`
- **Slow down:** the speed shrinks with the distance left, `v = sqrt(2 · a · remaining)`
- The robot uses the smallest of: full speed, speed-up limit, slow-down limit.
- There is a minimum speed, otherwise it would start at 0 and never move.

The values are fixed at the top of `motion.py`:

| Constant | Value | Meaning |
|---|---|---|
| `LINEAR_ACCEL` | 0.5 m/s² | reaches 0.1 m/s after 1 cm, 0.3 m/s after 9 cm |
| `ANGULAR_ACCEL` | 90 °/s² | reaches 30 °/s after 5° |
| `MIN_LINEAR` | 0.02 m/s | slowest driving speed |
| `MIN_ANGULAR` | 5 °/s | slowest turning speed |

Why it helps: mecanum rollers slip on sudden speed changes (bad for odometry), and arriving
slowly means the 50 ms overshoot is tiny. In a test, errors went from ~1 cm / ~2° to ~3 mm / ~0.3°.
Moves take a little longer (e.g. 50 cm at 0.3 m/s: 1.7 s → 2.3 s).

For moves that drive and turn at the same time (curves, orbit, rotate_around), one factor scales
all speeds together, so the radius stays the same while ramping.

### move_direction / forward / backward / strafe_left / strafe_right

`move_direction(vel, d, angle)` splits the speed into forward and sideways parts:
x = vel · cos(angle), y = vel · sin(angle), turn = 0.

The others are just fixed angles:

| Function | angle | x | y |
|---|---|---|---|
| `forward` | 0° | +vel | 0 |
| `backward` | 180° | −vel | 0 |
| `strafe_left` | 90° | 0 | +vel |
| `strafe_right` | −90° | 0 | −vel |

Stop rule: straight-line distance from the start point ≥ d,
i.e. `sqrt((x − x_start)² + (y − y_start)²) ≥ d`.
Since this measures distance in any direction, the same check works for all of them.

### rotate_left / rotate_right

x = 0, y = 0, turn = ±vel (degrees/s → converted to rad/s). The robot turns on the spot.

Stop rule: the heading has changed by `degree`.
The heading from odometry only goes from −180° to +180°, so it jumps from 180° to −180°
when turning past it. To handle that, the loop adds up the small change every 50 ms
(each step is only a few degrees, so a jump like 179° → −179° is correctly read as +2°).
That's why turns of 360° or more work.

### curve_left / curve_right

Drive forward and turn at the same time:

- x = vel
- turn = vel ÷ radius (rad/s), + for left, − for right

On a circle, turning speed = forward speed ÷ radius. That ratio is what makes the path a circle
with the wanted radius.

Example: `curve_left(0.1, 0.5, 90)`
- turn = 0.1 ÷ 0.5 = 0.2 rad/s ≈ 11.5°/s
- 90° takes 90 ÷ 11.5 ≈ 7.9 s
- distance driven = radius × angle (in rad) = 0.5 × π/2 ≈ 0.79 m

Stop rule: same as rotate, the heading has changed by `degree`. The radius itself is never
measured, only set through the speed ratio. If the wheels slip, the robot still ends facing the
right way, but the circle may be a bit wider or tighter.

Small radius = fast turning: 0.1 m/s with a 5 cm radius is already ~115°/s.

### turn_to

Works out the shortest turn from the current heading to the target heading (e.g. from 170° to −170°
is +20°, not −340°), then does a normal rotate by that amount.

Difference to `rotate_*`: `rotate_right(90)` four times adds up each overshoot (4 × 1° = 4° off).
`turn_to` looks at the absolute heading every time, so errors don't add up.

### orbit_left / orbit_right

The sideways version of a curve: no forward speed, strafe at vel and turn at vel ÷ radius.

- y = ±vel
- turn = ∓vel ÷ radius (opposite sign)

The centre is `radius` in front of the robot. Strafing left around it means going round it
clockwise, so the robot has to turn right to keep facing it. That's why the signs are opposite.

Stop rule: heading changed by `degree` (= how far round the centre it went).

### rotate_around(dx, dy, degree, vel)

Rotate around any point P = (dx, dy) in the robot's frame (dx in front, dy left) instead of the centre.

When the robot turns at ω (rad/s), a point P on it moves at (−ω · dy, ω · dx).
To keep P still, the robot drives the opposite way at the same time:

- x = ω · dy
- y = −ω · dx
- turn = ω

The further away P is, the faster the robot drives: P 0.3 m away at 30°/s → 0.16 m/s.

Stop rule: heading changed by `degree`.

### goto_xy / goto_pose

Target given in the start frame (like `get_x`/`get_y`). Every 50 ms:

1. Get the error to the target: ex = x_target − x, ey = y_target − y
2. Direction to the target in the room: atan2(ey, ex)
3. Seen from the robot: subtract the robot's heading
4. Drive that way at vel: x = vel · cos(direction), y = vel · sin(direction)

Because the direction is recalculated every step, drift is corrected on the way.

`goto_pose` also turns at the same time. The turn speed is chosen so turning finishes when the
robot arrives: turn speed = heading error ÷ (distance ÷ vel), at most 90 °/s
(30 °/s if it only has to turn, not drive). Step 3 is what makes driving straight while turning
work: the robot's heading changes, so the direction "seen from the robot" changes too.

Stop rule: within 5 mm of the target (or just went past it) and within 0.5° of the heading
(or just went past it).

### move_spinning(vel, distance, angle, spin)

At the start, the drive direction is fixed in the room (robot heading + angle).
Every 50 ms, it is converted to the robot's view (room direction − current heading), like goto.
The robot keeps turning during the next 50 ms, so it aims for the heading halfway through that time
(otherwise the line bends a little in the spin direction).

Stop rule: straight-line distance from the start ≥ distance. The final heading is wherever the
spinning got to.

### drive(x, y, turn, ms)

Sends the three speeds as given for `ms` milliseconds, then stops. No odometry, purely time based.
Useful for combinations the other functions don't cover, e.g. moving diagonally while turning.

### Odometry: get_x / get_y / get_yaw / print_pose

- **Where it comes from:** the real robot calculates its position from the wheel encoders
  (`odom → base_link` on /tf). The turtle reports its exact position (`/turtle1/pose`).
- **Relative to the start:** the values are shifted and rotated so the start of the program
  is x = 0, y = 0, yaw = 0, with forward = +x and left = +y.
- **Not the true position on the real robot:** wheel encoders measure how far the wheels turned,
  not where the robot actually is. Wheel slip (a lot with mecanum wheels, especially strafing)
  makes the odometry drift from reality. The turtle has no slip, so its position is always exact.

## Problems found and how they were solved

1. **"Can't reach the robot"**: no micro-ROS agent was running on the laptop (old container exited).
   The robot only appears in ROS through the agent. → `start.sh` starts it.
2. **"Timeout, target not reached"**: not friction. The robot only handles ~2 commands/s over WiFi;
   the old code sent 20/s. They queued up: 1 s forward drove 0.58 m instead of 0.2 m, a stop arrived
   2.4 s late. Measured: 20/s, 5/s, 3.3/s all lag, 2/s works. → commands at most every 0.5 s
   (`CMD_INTERVAL`), odometry still checked every 50 ms, stop always immediately.
3. **Overshoot** (90° turn → 123°): ~0.6 s delay between command, movement and odometry.
   → pose predicted `LATENCY` = 0.6 s ahead with the last command, `stop()` waits that long.
   Result: 93.4° and 0.30 m for 90° / 0.3 m. Tune `LATENCY` in `edurobot.py` with `scripts/test_accuracy.py`.
4. **C++ → Python**: `motion.py` (movement helpers) + `edurobot.py` (plumbing) replace `edurobot.cpp`,
   scripts in `scripts/` start with `from motion import *`, run from the web page.
5. **Lidar froze after a robot reboot**: the driver keeps running but sends nothing.
   → `lidar_watchdog.sh` restarts it after 10 s without scans.
6. **Robot timestamps are broken** (`/tf` and `/imu`): seconds right, nanoseconds barely change
   (+0.2 ms per message instead of +200 ms, sometimes > 10⁹). Poses look up to 0.7 s old.
   → `odom_fix.py` re-stamps the odometry with the laptop clock for SLAM. Real fix: robot firmware.
7. **Old SLAM image** (`../ROS-Laser-SLAM`, ROS lyrical): its `/tf` and `/map` never reached the
   kilted containers. → `slam.Dockerfile` (slam_toolbox on kilted). Also it had `use_sim_time:=true`
   (waits forever for `/clock` on a real robot).
8. **Lidar mount angle**: measured with a 30 cm drive, 0° fits (4 cm mismatch), 90/180/270° don't
   (18–30 cm). The 90° in the old `../docker-compose.yaml` was wrong.
9. **Laptop IP changed (WLAN reconnect)**: containers started before keep the old address and get
   no data; the robot sends to the old Agent IP. → `start.sh` checks the robot settings, labels each
   container with the laptop IP and restarts it when the IP changed.
10. **Typing W/A/S/D in a text field drove the robot** → keys are ignored while typing.
11. **Orbits stalled / didn't finish in one go**: `orbit_left(0.5, 0.4, 180)` moved a few cm, then
    "Timeout, angle not reached". `orbit_left(0.3, 0.3, 360)` drove ~5 s, stopped ~13 s, drove again...
    Pure rotation (23 °/s) and pure strafing (0.12 m/s) ran smoothly, WiFi was fine. Cause: a mecanum
    wheel turns at forward ± sideways ± lever × rotation, so in an orbit (strafe + turn) two diagonal
    wheels have to go about double speed, more than the ~0.16 m/s a wheel can do, and the robot stalls.
    → every command is scaled so the fastest wheel stays ≤ `WHEEL_MAX` (all three speeds by the same factor,
    so the orbit keeps its radius, just slower; one-time note in the output; the timeout grows to match).
    Now `orbit_left(0.3, 0.3, 360)` runs a full circle in one go (~21 s, 16.6 °/s, ends 2 cm / 4° from start).
    `WHEEL_LEVER` = 0.25 m is an estimate (half length + half width); if it still stalls, raise it or lower `WHEEL_MAX`.
12. **"Address already in use"**: a second `webui.py` on port 8080. → `docker ps`, stop the old one.

13. **Planner found no path while the robot stood next to an obstacle** (start inside the safety
    margin). → around the robot only real walls count as blocked, so it can drive away.

14. **Turn-then-drive overshot by ~30°**: switching from turning to driving waited for the next
    command slot (0.5 s) while the robot kept turning, plus 0.6 s delay. → stop immediately at the end
    of a turn, wait `LATENCY`, then drive. Repeated stops are rate-limited too (no 20 Hz stop flood).
15. **Map pose jumped 16° / 30 cm while driving** on an old map that had grown over many tests. Measured
    with a turn on the spot: lidar and odometry agree (164.8° vs. 164.0°) and are only 0.04 s apart
    (`odom_fix.py` `DELAY` 0.1 → 0.04). On a fresh map SLAM and odometry agree exactly. → start
    mapping runs with a fresh map (*New map*).

16. **First autonomous run (`maps/1stmap.pgm`) didn't look into the course**: frontiers were only
    kept where the robot itself could stand (outside the safety margin), so narrow spots and the course
    entrance were dropped → viewpoints. Robot size was overestimated (0.25 → 0.16 m, 26 × 18 cm);
    `goto_map` stop distance 0.4 → 0.3 m.

17. **Second run drove into obstacles / between chairs**: the lidar stop only looked at a cone in front of
    the centre (blind to the robot's width) and turning wasn't checked; the planning margin (5 cm around
    a circle) was below the steering accuracy. → footprint check (strip / turning circle), backing up
    when too tight to turn, exact wall distances instead of coarse cell growing (the 40 cm course
    channel had shrunk to nothing), noise filter, viewpoints prefer a view across known floor.

Still open: holding W sometimes seemed to stop after ~1 s. Recordings showed continuous driving
and a normal key release; `docker logs -f edurob-webui` shows why each stop happened.

## Things to remember

- **Speeds:** 0.1 m/s is right for the real robot but barely moves the turtle
  (its world is only about 11 × 11 units). Use ~1.0 for the turtle.
- **What `--turtle` does:**
  - runs everything on ROS domain 42 instead of 12, so the real robot ignores it
  - starts turtlesim in holonomic mode (it can strafe)
  - shows the turtlesim window at double size (1000 × 1000 px, the world is still 11 × 11).
    Other size: `TURTLE_SCALE=1.5 ./ros.sh --turtle ...` (1 = original 500 × 500)
  - uses `turtle.png` (in the `ws` folder) as the turtle picture, if there is one
  - reroutes `cmd_vel` to `/turtle1/cmd_vel` and `turtle_pose` to `/turtle1/pose`
- **`./ros.sh --turtle` without a command** opens a shell with turtlesim running, but nothing is
  rerouted. Add `--ros-args -r cmd_vel:=/turtle1/cmd_vel -r turtle_pose:=/turtle1/pose` to commands run in there.
- **Own turtle picture:** put a `turtle.png` into the `ws` folder. `ros.sh --turtle` scales it to
  fit 45 × 45 px (keeps the shape, transparent edges) and uses it instead of the random ROS turtle.
  The top of the picture is the front: it points in the driving direction. Use a PNG with a
  transparent background, otherwise the turtle is drawn as a square.
  Delete or rename `turtle.png` to get the normal turtles back.
- **The turtle can strafe.** `ros.sh` starts turtlesim with `-p holonomic:=true`, so it uses the
  sideways speed like the mecanum robot and all movement functions work on it. (Without that
  parameter turtlesim ignores sideways speed, and everything that moves sideways times out.)
- **Only one `--turtle` session at a time.** All of them use domain 42, so two running at once
  mix up each other's commands and positions.
