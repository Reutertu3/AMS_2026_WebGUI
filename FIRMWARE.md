# EduRob firmware: what it really does

What we found reading the robot's firmware (ESP32-S3, ESP-IDF, micro-ROS), and how it explains the
robot's behaviour we measured. Read on 2026-11-24, nothing was changed on the robot.

## Where the code is

| What | Where |
| --- | --- |
| App: WiFi, ROS, parameters, web page | `Edurob_Firmware/main/` (`main.cpp`, `ros.cpp`, `parameters.cpp`) |
| Drive: kinematics, odometry, controllers | `../edurobv2-feature-ROSDriveHAL/components/DriveHAL/` (`DriveHAL.cpp`, `Motor.cpp`) |
| Sensors: IMU (gyro), lidar bridge | `../edurobv2-feature-ROSDriveHAL/components/SensorHAL/` (`IMUHAL.cpp`, `TCPLaser.cpp`) |

`Edurob_Firmware/` doesn't contain the drive and sensor code. It's in the sibling project
`edurobv2-feature-ROSDriveHAL`, whose `main/` folder is identical to `Edurob_Firmware/main/`
(`main.cpp`, `ros.cpp`, `parameters.cpp` compared byte by byte).

## What runs, and how often

The firmware is a set of loops (FreeRTOS tasks), each on its own clock:

| Loop | Every | Does |
| --- | --- | --- |
| Motor control (`Motor.cpp`) | 50 ms | measures each motor's speed from its encoder, sets the PWM |
| IMU (`IMUHAL.cpp`) | 20 ms | reads the gyro (chip runs at 104 Hz), adds up the heading |
| Drive (`DriveHAL::WorkerTask`) | 100 ms | odometry, pose controller, wheel speeds |
| ROS publish (`ros.cpp`) | 200 ms | publishes `/tf` (odom) and `/imu`, so **5 Hz** |
| ROS receive (`ros.cpp`) | 400 ms | handles incoming messages (`cmd_vel`, parameters, gyro service) |

The last row matters most: **the robot only looks at incoming commands every 400 ms**, so at most
about 2.5 per second. Commands sent faster pile up and run late. That's why `edurobot.py` sends at
most one command every 0.5 s (`CMD_INTERVAL`).

## Driving: from `cmd_vel` to the wheels

1. **`cmd_vel` arrives** (`ros.cpp`, `cmd_vel` callback):
   - all zero → mode **Off**: the motor driver is switched off (GPIO 40). The robot rolls out and
     doesn't hold its position.
   - anything else → mode **Manual** with that speed.
2. **Pose controller** (`DriveHAL::WorkerTask`, every 100 ms): in Manual mode the robot keeps a
   *target pose*. Every step it moves that target on by the commanded speed (e.g. 0.1 m/s → 1 cm
   per step), then drives towards it:
   ```
   speed = commanded speed + 1.0/s × (target pose − measured pose)
   ```
   So it doesn't just set wheel speeds, it **catches up**: if a wheel slipped or was held back, the
   robot speeds up until it's back on the target path.
3. **Safety cut-off** (`OmniControll`): if the measured pose falls more than **0.3 m** (x or y) or
   **0.5 rad = 29°** behind the target, the controller sends **0** to the wheels. The target keeps
   moving, so the gap only grows: the robot **stands still until the next zero command**, which
   resets the target to where the robot really is.
4. **Wheel speeds** (`DriveHAL::SetVelocities`): the mecanum formula turns robot speed into wheel
   speed (see below), in whole **wheel RPM**, then × 100 (gearbox) for the motor.
5. **Motor control** (`Motor.cpp`, every 50 ms): a simple P-controller,
   `motor setpoint + 0.28 × (setpoint − measured)`, then a measured straight line from RPM to PWM:
   `PWM = 0.1973 × RPM + 245.3`, at most 2000 ticks.

There is **no timeout** on `cmd_vel` in the firmware: a Manual speed stays until the next command.
(The robot stopping by itself ~2 s after the last command, which we measured earlier, doesn't come
from this code. It's unexplained so far.)

### The mecanum formula

Wheels (from `main.cpp` and the parameters): 0 = front right, 1 = front left, 2 = rear left,
3 = rear right, at x = ±0.100 m, y = ±0.0865 m from the centre, radius `r`.

From robot speed (vx forward, vy left, ω turning) to the speed of wheel *i* (rad/s), in
`DriveHAL::MecanumKinematic`:

```
wheel 0:  (vx + vy + (x₀ − y₀) · ω) / r₀
wheel 1:  (vx − vy − (x₁ + y₁) · ω) / r₁
wheel 2:  (vx + vy − (−x₂ + y₂) · ω) / r₂
wheel 3:  (vx − vy + (−x₃ − y₃) · ω) / r₃
```

With the robot's numbers, every wheel's turning term is ± 0.1865 m × ω: the **lever** `|x| + |y|`.
The odometry uses the reverse formula (the average over the 4 wheels):

```
vx = r/4 × ( w₀ + w₁ + w₂ + w₃)
vy = r/4 × ( w₀ − w₁ + w₂ − w₃)
ω  = r/4 × ( w₀ − w₁ − w₂ + w₃) / lever
```

The wheels on the left side (positive y: 1 and 2) are mounted mirrored, so the firmware flips the
sign of their encoders and motor speeds.

## Odometry: two positions

`DriveHAL::WorkerTask` (lines 290–296) updates **two** positions every 100 ms, from the same wheel
movements:

| Position | Heading from | Used for |
| --- | --- | --- |
| `OdoPose` | **wheels only** | **published as `/odom` → `/base_link`**: what `motion.py`, the web page and SLAM see |
| `GyroOdoPose` | the gyro, once it's calibrated (wheels before that) | only inside the robot: the pose controller above |

So the gyro does **not** affect the odometry we get. It affects how the robot drives: once
calibrated, the robot holds its heading by the gyro, while `/odom` reports the heading by the wheels.
The two can slowly drift apart.

Each odometry step moves by the wheel movement turned into the world frame, using the heading
halfway through the step (`DriveHAL::Odometry`): a good way to keep curves accurate.

**Encoder resolution:** 28 counts per motor turn × 100 gearbox = 2800 per wheel turn. With a 24.1 mm
radius, that's 0.054 mm of wheel travel per count.

## Gyro (IMU)

- Chip: LSM6DSOX, gyro range **±125°/s** (`IMUHAL.cpp:68`). Turning faster than that saturates it
  and the gyro heading falls behind. `motion.py` limits `goto_pose` to 90°/s and the web page's
  slider goes up to 120°/s, just below. `rotate_left(…, vel)` with `vel` > 125 would saturate it.
- **Not calibrated at boot.** `/calibrategyro` (web page, `calibrate_gyro()`) measures the gyro's
  zero offset while the robot stands still. Only after that does the robot's controller use the gyro
  heading. So calibrating the gyro changes how the robot drives, not what `/odom` says.
- The heading is added up from **−z** of the chip (`IMUHAL.cpp:273`): the chip is mounted upside
  down. `/imu` publishes the chip's raw z, so its `angular_velocity.z` is probably **negative when
  turning left**, against the ROS convention. Not checked on the robot yet.

## Parameters (`ros2 param list /Edurob`)

12 parameters: `wheel_radius_0..3`, `wheel_pos_x_0..3`, `wheel_pos_y_0..3`.

- A change takes effect **at once** (`Drive->SetDriveConfig`, `ros.cpp`).
- It's also **saved in flash** (`parameters.cpp`, NVS) and loaded at boot: calibrated values
  survive a reboot.
- Saved as whole numbers × 10000, cut off (not rounded) to **0.1 mm**: 24.106 mm is saved as
  24.1 mm (0.02 %).

`scripts/calibrate_odometry.py` and `scripts/calibrate_turning.py` set these with the lidar,
see CALIBRATE.md.

## ROS interface

| Name | Type | Notes |
| --- | --- | --- |
| `/cmd_vel` | subscribed | handled every 400 ms (see above) |
| `/tf` | published, 5 Hz | `odom → base_link` from `OdoPose` |
| `/tf_static` | published, every 400 ms | `base_link → laser`, `base_link → imu` |
| `/imu` | published, 5 Hz | gyro + accelerometer, no orientation |
| `/calibrategyro` | service | `edurob_interfaces/srv/CalibrateGyro`, robot must stand still |
| lidar | TCP port 44227 | not ROS: raw lidar data, the `rplidar` container turns it into `/scan` |

The node is called `Edurob`. Other: web page with the settings (WiFi agent IP, domain), and
the robot announces itself as `edurob-xxxxxx.local` (mDNS, last 6 digits of its MAC address).

## Bugs and surprises

| What | Where | Effect |
| --- | --- | --- |
| **Broken timestamps.** `nanosec = millis × 1000` overflows the 32-bit field. Correct: `(millis % 1000) × 1 000 000` | `ros.cpp:124` (and 3 more stamps) | Each 200 ms step adds only 0.2 ms to the time. That's exactly what `odom_fix.py` works around for SLAM. |
| **IMU array overrun.** Loop runs `i < 4` over 3-element arrays | `IMUHAL.cpp:162` (`getIMUData`) | Writes past the end of two arrays on the ROS task's stack, every 200 ms. Undefined behaviour: may corrupt other variables or crash. Should be `i < 3`. |
| **IMU frame name set from the wrong message.** `IMUData.header.frame_id = micro_ros_string_utilities_set(tfStaticData[0].header.frame_id, "/imu")`, copy-paste slip: should start from `IMUData.header.frame_id` | `ros.cpp:183` | Reallocates the static tf's string buffer. Harmless while the buffer stays in place, otherwise `/tf_static` uses freed memory. |
| **Wheel speed cut to whole RPM** before × 100 gearbox | `DriveHAL::SetVelocities` | Speed steps of 2.5 mm/s per wheel, always rounded down: at 2 cm/s up to 12 % too slow. The pose controller makes up for it over time. |
| **Robot freezes when it falls behind** | `OmniControll`, `MaxPoseErr` | More than 0.3 m / 29° behind target → wheels 0, until the next stop command. |
| **No `cmd_vel` timeout** | `ros.cpp` | If the laptop disappears mid-drive, the robot keeps the last speed (until the freeze above). |
| Only mecanum implemented | `DriveHAL.cpp` | The omni and differential drive functions are empty. |

## How this explains what we measured

| We saw (README.md, NOTES.md) | Cause in the firmware |
| --- | --- |
| Only ~2 commands/s work, faster ones run seconds late | incoming messages handled every 400 ms |
| `/tf` and `/imu` come at 5 Hz | publish loop every 200 ms |
| Timestamps barely move (+0.2 ms per message) | `nanosec = millis × 1000` overflow |
| Robot stalls for seconds in orbits (strafe + turn) | wheels can't reach the speed (PWM limit, ~0.22 m/s per wheel calculated without load, ~0.16 m/s measured), the robot falls behind its target and the 0.3 m / 29° cut-off stops it until the next stop command. `WHEEL_MAX` in `edurobot.py` prevents this. |
| Odometry heading isn't affected by the gyro calibration | `/odom` heading comes from the wheels only |
