# EduRob Controller

Drive the EduRob (mecanum robot, ESP32 with micro-ROS) from a laptop:

- **Web control** (`webui.py`): drive by hand, run movement scripts, live odometry trail,
  lidar view, gyro calibration, switch the ROS domain.
- **Movement scripts** (`scripts/*.py`): Arduino-style Python with `forward()`, `goto_xy()`,
  `orbit_left()`, `calibrate_gyro()`, ... from `motion.py`.
- **Turtlesim**: test everything without the robot.

Everything runs in Docker (ROS 2 Kilted), so nothing ROS-related has to be installed on the laptop.
[NOTES.md](NOTES.md) explains all movement functions and how they work.

## What is where

| File | Purpose |
|---|---|
| `start.sh` / `stop.sh` | Start/stop everything in the background: micro-ROS agent, lidar driver, web control. Safe to run twice. |
| `lidar_watchdog.sh` | Restarts the lidar driver when it stops delivering scans (used by `start.sh`). |
| `odom_fix.py`, `slam_params.yaml`, `slam.Dockerfile` | Mapping with slam_toolbox (`./start.sh --slam`), see [Mapping](#mapping-slam). |
| `maps/` | Saved maps (`.pgm` + `.yaml`). |
| `explore.py`, `scripts/mapping.py` | Autonomous mapping (frontier exploration), see NOTES.md. |
| `ros.sh` | Runs a command in the ROS 2 container. `--turtle` uses turtlesim instead of the robot. |
| `webui.py` / `webui.html` | Web control (server + page). |
| `motion.py` | All movement helpers for scripts (`from motion import *`). |
| `edurobot.py` | ROS plumbing behind `motion.py` (connection, odometry, send limit, `LATENCY`). |
| `scripts/` | Movement scripts. The web page lists every `*.py` in it (except names starting with `_`). |
| `src/edurob_interfaces` | Message definitions of the robot (needed for the gyro calibration). |
| `legacy_cpp/` | The earlier C/C++ version (`main.c` + `edurobot.cpp`). Not used anymore. |
| `simple.py`, `simple_forward.py`, `example.py`, `minimal_edurob_node.py` | Older standalone examples, see [notes](#other-notes). |

## How it fits together

```
 Laptop (Docker, ROS_DOMAIN_ID=12)                          EduRob (10.0.109.58)
 ┌──────────────────────────────┐   UDP 8888 (micro-ROS)   ┌─────────────────────────┐
 │ micro-ros-agent  ◄───────────┼──────────────────────────┤ ESP32: motors, encoders, │
 │   cmd_vel ─► robot           │                          │ IMU, /calibrategyro      │
 │   /tf odom, /imu ◄─ robot    │                          │                          │
 │ rplidar driver ◄─────────────┼── TCP 44227 ─────────────┤ lidar (TCP bridge)       │
 │   /scan                      │                          └─────────────────────────┘
 │ webui.py ──starts──► script  │
 │   (motion.py)                │
 └──────────────────────────────┘
```

The robot does **not** appear in ROS by itself. It connects to a micro-ROS agent at the IP address
in its settings. Without a running agent, nothing from the robot shows up in ROS.

## One-time setup

1. **Docker images** (if not present yet):
   ```bash
   docker pull osrf/ros:kilted-desktop
   docker pull microros/micro-ros-agent:kilted
   docker pull husarion/rplidar:jazzy-1.0.1-20250901
   ```
2. **Robot settings**: open http://10.0.109.58/ → *Go to Configuration* and check:
   - **Agent IP** = your laptop's IP in the WLAN (`ip -br addr show wlan0`, currently `10.0.109.251`)
   - **Agent Port** = `8888`
   - **DomainID** = `12`

   The laptop gets its IP from DHCP. If it changes, the robot can't find the agent anymore.
3. **Robot message definitions** (for `calibrate_gyro`): `./start.sh` builds them the first time
   (~1 min, step "Robot messages"). By hand:
   ```bash
   ./ros.sh colcon build --base-paths src --packages-select edurob_interfaces
   ```
   This creates `build/`, `install/` and `log/` (not in git). `ros.sh` loads `install/` automatically.
   Without them everything else still works, only the gyro calibration says it isn't available.
   If you build by hand while the web page is running, restart it (`docker stop edurob-webui`,
   `./start.sh`): scripts started from the page only see the messages after that.

There is nothing to compile: scripts are run directly.

## Every session

```bash
./start.sh          # agent, lidar driver and web control in the background
./stop.sh           # when done (stops the robot and a running script first)
```

`start.sh` checks the network first and **aborts without starting anything** if the robot doesn't
answer pings, or if the robot's settings (Agent IP, DomainID) don't match this laptop. Then it
starts what isn't running yet, waits for robot data and prints the address (in colour in a terminal):

```
EduRob · ROS domain 12 · robot 10.0.109.58

Network
  Robot reachable (ping)              ✓ yes
  Laptop IP                           10.0.109.251
  Robot settings (Agent IP, domain)   ✓ 10.0.109.251, domain 12

Services
  micro-ROS agent (UDP 8888)          ✓ started
  Lidar driver (TCP 44227)            ✓ started
  Web control (port 8080)             ● already running

Robot data
  Waiting for odometry and lidar      ...
  Odometry                            ✓ ok
  Lidar                               ✓ ok

Web control  http://10.0.109.251:8080  (this laptop: http://localhost:8080)
```

✓ = ok, ● = fine but worth knowing (already running, restarted), ✗ = problem, with a hint below.

If *Odometry* says `✗ none`: reboot the robot (http://10.0.109.58/ → Reboot ESP32). It stops
looking for the agent after a while, and doesn't reconnect after `./stop.sh` / `./start.sh`.
Then reload the page, no need to run `start.sh` again.

Options: `./start.sh --slam` (build a map, see [Mapping](#mapping-slam)), `./start.sh --local` (page only on this laptop),
`PORT=8081 ./start.sh`, `DOMAIN=13 ./start.sh`. Options can be combined.
Logs: `docker logs -f edurob-webui` (also shows errors of the web server), `micro-ros-agent`, `rplidar`.

Scripts also run without the web page: `./ros.sh python3 scripts/square.py`.
Don't run one in a terminal while the web page drives at the same time. Both would send `cmd_vel`.

## Web control

`./start.sh` runs it in the background. By hand, in a terminal (stop the background one first,
`docker stop edurob-webui`, both want port 8080):

```bash
./ros.sh python3 webui.py                    # http://localhost:8080, only from this laptop
./ros.sh python3 webui.py --host 0.0.0.0     # also from phone/other PCs (anyone in the WLAN can drive!)
./ros.sh python3 webui.py --port 8081        # other port
./ros.sh --turtle python3 webui.py           # turtlesim instead of the robot
```

- **STOP** (or `Space`): stops manual driving **and** a running script.
- **Scripts**: pick a file from `scripts/`, *▶ Run*. Its output (`print`, errors) appears below.
  While it runs, the button becomes *■ Stop*, manual driving and calibration are blocked, and a
  second script can't be started. ⟳ reloads the list after adding a file. Each script runs as its
  own process (with the turtlesim remaps in `--turtle` mode), so Stop always works. The robot is
  stopped when the script ends, fails or is stopped.
- **Driving**: hold the arrow buttons, or use the keyboard: `W`/`S` forward/back, `A`/`D` strafe,
  `Q`/`E` rotate, arrow keys. Keys can be combined (`W`+`Q` = curve). Releasing everything stops.
- **Sliders**: linear velocity (m/s), rotation velocity (°/s) and ramp (acceleration in m/s²;
  0 = full speed at once; rotation uses the same ratio as `motion.py`: 0.5 m/s² ↔ 90 °/s²).
  The settings are remembered in the browser.
- **Map**: grid (10 cm / 1 m), odometry trail (blue), robot (orange, arrow = front), lidar (red).
  Mouse wheel = zoom, drag = move (turns off *Follow*), *Fit* = show the whole trail,
  *Clear trail* = start over. *Keep points* accumulates lidar scans into a rough map (drifts with
  the odometry). *Mount angle offset* rotates the lidar points, in case the lidar is mounted turned.
- **Gyro (IMU)**: *Calibrate gyro* stops the robot and calls `/calibrategyro` for the given seconds.
- **ROS domain**: *Switch* restarts the server on another domain (a ROS program can't change its
  domain while running). The robot and any script are stopped first, the page reconnects by itself.
  The domain has to match the robot's settings page. The lidar driver uses the domain `start.sh`
  was started with (`DOMAIN=13 ./start.sh`). Default everywhere: 12, also `DOMAIN=13 ./ros.sh ...`.
- **Safety**: the browser repeats the command 4×/s while a button is held. If nothing arrives for
  0.8 s (tab closed, WiFi gone), the server slows the robot to 0. Ctrl+C on the server sends a stop.

## Mapping (SLAM)

```bash
./start.sh --slam     # also starts slam_toolbox (first time: builds the image, ~2 min)
```

Open the web page and drive around by hand. The map (grey = free, white = walls) grows under the
lidar view, and the robot is shown at its position on the map (*robot (SLAM map)* in the top left).
Then *Map (SLAM)* → name → *Save map* writes `maps/<name>.pgm` + `.yaml` (nav2 map_server format).
*New map* throws the current map away and starts again at the robot's position (asks first).

Tips for a clean map:
- Drive slowly (≤ 0.15 m/s) and turn gently (≤ 30 °/s). Avoid strafing: mecanum wheels slip
  sideways a lot, and bad odometry smears the map.
- Come back to places you've already mapped (loop closure): slam_toolbox then corrects the drift.
- Map resolution: 1 cm per cell (test; 2 cm is enough for a 7 cm barrel. `resolution` in `slam_params.yaml`,
  restart SLAM after changing it). The web server gzips the map (1 cm: 1.5 MB → ~12 KB per update).
  Small objects (e.g. a 7 cm barrel) show up as a few white cells; the live lidar points (red) show them best.
- Start over: *New map* in the page (asks first; starts where the robot is now). `./stop.sh` also
  stops SLAM, and **an unsaved map is lost**.

How it works:

```
robot /tf (odom -> base_link, broken stamps) --odom_fix.py--> odom_fix -> base_fix -> laser_fix
lidar driver /scan (frame laser_fix) ----------------------------------------------┘
slam_toolbox: scans + odom_fix tree -> /map and map -> odom_fix
webui.py: /map and map -> base_fix -> map, robot and lidar in the page
```

RViz works too: `./ros.sh rviz2`, Fixed Frame `map`, add *Map* (`/map`), *LaserScan* (`/scan`)
and *TF*.

## Autonomous mapping

`scripts/mapping.py` explores by itself: it drives to the nearest unexplored edge of the SLAM map
(frontier), plans around walls with A* (`explore.py`), and repeats until nothing reachable is left,
then saves the map. Needs `./start.sh --slam`. Try `DRIVE = False` first (plans only). Planning can be
tested offline on a saved map: `./ros.sh python3 explore.py maps/<name>.yaml` → `maps/<name>_plan.png`.
Explained with diagrams for people new to the project: [MAPPING.md](MAPPING.md).
Settings and test status: [NOTES.md](NOTES.md#autonomous-mapping-mappingpy).

## Writing scripts

```python
# scripts/my_test.py
from motion import *       # all movement helpers; connects to ROS, waits up to 10 s for odometry

calibrate_gyro(5)          # optional, robot stands still for 5 s
forward(0.2, 0.5)          # 0.2 m/s, 50 cm (ramp=True is the default: soft start/stop)
rotate_left(90, 30)        # 90 degrees at 30 degrees/s
goto_xy(0.2, 0, 0)         # back to the start point
print_pose()               # "x=0.002 m  y=-0.001 m  yaw=90.3 deg", shown in the web page
```

All functions and how they work: [NOTES.md](NOTES.md). Examples in `scripts/`:

| Script | What it does |
|---|---|
| `square.py` | 1 m square, prints the pose after each step |
| `curved_square.py` | Square with rounded corners, `move_spinning`, `orbit_left` |
| `haus_vom_nikolaus.py` | Draws the "Haus vom Nikolaus" with `goto_xy` (needs ~2 × 3 m) |
| `test_accuracy.py` | One 90° turn + 30 cm, to tune `LATENCY` |
| `calibrate.py` | Only the gyro calibration |

The gyro calibration in a terminal, without a script:
`./ros.sh ros2 service call /calibrategyro edurob_interfaces/srv/CalibrateGyro "{time: 5}"`

## Findings: why it didn't work, and what was changed

### 1. "Can't reach the robot": no micro-ROS agent was running

The robot was fine (WiFi, settings, domain 12), but nothing on the laptop was running the agent
(the old `edurob-controller-agent-1` container had exited weeks before). Without the agent, nothing
from the robot appears in ROS. **Fix:** `start.sh` starts the agent (and the lidar driver).
If the robot was switched on before the agent, it may have stopped trying to connect. Reboot it.

The domain numbers in the older files disagree (`../docker-compose.yaml` and `../Befehle.txt` use 1/2,
the old agent container used 2). The robot is set to **12**, so everything here uses 12.

### 2. "Timeout, target not reached": the robot executed commands seconds late

Not friction. Measured by sending fixed commands and recording the odometry:

| Commands sent | Result |
|---|---|
| 20/s (old library), forward 1 s at 0.2 m/s | drove **0.58 m** instead of 0.2 m, kept going > 2 s after stop |
| 5/s, turn 2 s at 30 °/s, then stop | kept turning **2.4 s** after stop |
| ~3.3/s (300 ms) | still ~1.5 s too long |
| **2/s (500 ms)** | turned 58° (60° expected), stopped within 0.5 s ✓ |

The robot (micro-ROS over WiFi) processes only about 2 commands per second. Commands sent faster
pile up in a queue, and the robot works through them later and later, so the stop arrives
seconds too late. The odometry loop then steered with outdated values until it ran into the timeout.
Changing the command to exactly 0 or a tiny value made no difference. Only the rate matters.
Without new commands, the robot stops by itself after about 2 s.

**Fix:** odometry is still checked every 50 ms, but speed commands go out at most every
`CMD_INTERVAL` = 0.5 s. Stop commands always go out immediately. Both `edurobot.py` (used by `motion.py`) and `webui.py`
do this.

### 3. Overshoot: odometry and commands are delayed

Even without the queue, there is about 0.5 s delay between command and movement, plus
odometry arriving late (5 Hz over WiFi). A 90° turn at 30°/s ended at **123°**, and 0.3 m forward
became 0.43 m.

**Fix:** on the real robot, the measured pose is predicted `LATENCY` ahead with the last command
(where the robot *will* be once the delay has passed), and `stop()` waits `LATENCY` so the next
move starts from a robot that has really stopped. Not used for turtlesim.

| `LATENCY` | `rotate_left(90, 30)` | `forward(0.2, 0.3)` |
|---|---|---|
| 0 (before, C++) | 122.8° | 0.43 m |
| 1.0 s (C++) | 70.3° | 0.26 m |
| 0.6 s (C++) | 83.2° | 0.32 m |
| **0.6 s (Python, now)** | **93.4°** | **0.30 m** |

`LATENCY` is a calibration constant at the top of `edurobot.py`: if the robot overshoots, increase
it; if it stops short, decrease it (`scripts/test_accuracy.py`). It depends on the WiFi quality.
`turn_to()` and `goto_xy()` correct leftover errors themselves, because they look at the absolute
position.

### 4. Mapping: robot timestamps are broken, old SLAM image doesn't talk to the rest

- **Timestamps:** the robot's `/tf` stamps have correct seconds, but the nanoseconds barely change
  (+0.2 ms per message instead of +200 ms; occasionally even > 10⁹). All poses within a second
  carry nearly the same time and look up to 0.7 s old. slam_toolbox needs the pose at each scan's
  exact time. **Fix:** `odom_fix.py` republishes the odometry with the laptop clock as its own
  frame tree (`odom_fix` → `base_fix` → `laser_fix`), and slam_toolbox uses that. The real fix
  would be in the robot firmware.
- **Old image:** the `slam-toolbox` image from `../ROS-Laser-SLAM` is ROS lyrical. Its `/tf` and
  `/map` never arrived at the kilted containers (web page, RViz). **Fix:** `slam.Dockerfile`
  installs slam_toolbox on kilted.
- **Lidar mount angle:** measured by comparing scans before and after driving 30 cm forward: 0°
  fits (4.4 cm median mismatch), 90°/180°/270° don't (18–30 cm). So the lidar isn't rotated.
  The 90° in the old `../docker-compose.yaml` was wrong.
- `../ROS-Laser-SLAM/docker-compose.yaml` starts slam_toolbox with `use_sim_time:=true`. On a real
  robot there is no `/clock`, so it would wait forever. Here it's `false`.

### 5. Orbits stalled the robot

`orbit_left(0.5, 0.4, 180)` moved a few cm, then "Timeout, angle not reached"; `orbit_left(0.3, 0.3, 360)`
drove ~5 s, stopped ~13 s, drove again. Pure rotation and pure strafing at the same speeds ran smoothly,
and the WiFi was fine. Cause: a mecanum wheel turns at `forward ± sideways ± lever × rotation`, so in an
orbit two diagonal wheels need about double speed, more than the ~0.16 m/s a wheel can do, and the robot
stalls instead of just going slower. **Fix:** `edurobot.py` (and the web page for manual driving) scale
every command so that `|x| + |y| + WHEEL_LEVER × |turn| ≤ WHEEL_MAX` (0.25 m, 0.16 m/s). All three speeds
get the same factor, so curves and orbits keep their shape, just slower; the timeout grows accordingly.
Result: `orbit_left(0.3, 0.3, 360)` runs a full circle in one go (~21 s). Not used for turtlesim.

### Other notes

- `simple.py` / `simple_forward.py` / `example.py` send at 20/s. On the real robot they have
  the same delay problem. Use `motion.py` scripts instead, or set their `SEND_INTERVAL` to 0.5.
- The lidar driver doesn't reconnect after a robot reboot: it keeps running but publishes nothing.
  `lidar_watchdog.sh` (runs inside the `rplidar` container) restarts it when no `/scan` arrives for
  10 s, so the lidar comes back by itself ~20 s after a reboot. The page shows
  *lidar: no data for X s* meanwhile instead of an old scan.
- The lidar container shows `(unhealthy)` in `docker ps`. That is the image's own health check;
  `/scan` still arrives (~8 Hz, 1080 points, up to 12 m).
- `ros2 topic list` on domain 12 should show `/cmd_vel`, `/imu`, `/tf`, `/tf_static`, `/scan`.
- `legacy_cpp/` builds with
  `./ros.sh bash -c "cmake -S legacy_cpp -B legacy_cpp/build_cpp && cmake --build legacy_cpp/build_cpp"`
  (needs step 3 of the setup) and runs with `./ros.sh ./legacy_cpp/build_cpp/edurobot`.

## Troubleshooting

| Symptom | Check |
|---|---|
| `No odometry received` / page says *waiting for odometry* / `start.sh` says `NONE` | `./start.sh` run? Reboot the robot. Agent IP in the robot settings = laptop IP? Same domain everywhere? |
| Web page doesn't load | `docker logs edurob-webui`. Port taken (`Address already in use`): another `webui.py` still running, `docker ps`. |
| Robot connected (agent log shows `session established` / `create_topic`), but page/SLAM get nothing | The laptop got a new IP while things were running (WLAN reconnect). `./start.sh` restarts the affected containers by itself (not the agent). If the laptop IP changed, also update the Agent IP on the robot settings page and reboot the robot. |
| Robot reachable at all? | `ping 10.0.109.58` (100–350 ms over this WLAN is normal) |
| No lidar on the page | Wait ~20 s (the watchdog restarts the driver). `docker logs rplidar`: `[watchdog] ... restarting` over and over = lidar not powered / TCP bridge not answering. `SL_RESULT_OPERATION_TIMEOUT` = same. Lidar on another domain than the page? |
| `calibrate_gyro: no /calibrategyro service` | Robot not connected, or you're on turtlesim. |
| `edurob_interfaces not built` | Step 3 of the one-time setup. |
| Script not in the list | File in `scripts/`, ends in `.py`, doesn't start with `_`? Press ⟳. |
| `ModuleNotFoundError: motion` in a terminal | Run it through `./ros.sh` (it sets `PYTHONPATH`). |
| Robot drives on after a script | It stops by itself after ~2 s without commands. If not, something else still sends `cmd_vel` (another `ros.sh` / `webui.py`). |
| No map in the page | Started with `./start.sh --slam`? `docker logs edurob-slam`: `Registering sensor` = it works; the map appears ~2 s later. |
| Map smeared / walls doubled | Drive slower, less strafing, turn gently. Start a new map (see Mapping). |
| Robot still overshoots / stops short | Adjust `LATENCY` in `edurobot.py` (`scripts/test_accuracy.py`). |
