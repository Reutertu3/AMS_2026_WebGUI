# Odometry calibration with the lidar

`scripts/calibrate_odometry.py` finds the robot's real wheel radius by comparing two measurements
of the same movement:

- the **odometry**: how far the robot *thinks* it drove, from counting wheel turns,
- the **lidar**: how much closer to a wall it *really* got.

If they don't agree, the wheel radius the robot uses is wrong, and the script corrects it.

**How to run it:** put the robot about 1 m in front of a straight, flat wall, roughly facing it.
Then start `calibrate_odometry.py` in the web page (Scripts). Run it a second time to check: the
ratio should now be about 1.000.

---

## 1. Why the wheel radius matters

The robot can't see where it is. It counts how far its wheels have turned and works out the
distance from that:

```
distance = wheel turns × wheel circumference
         = wheel turns × 2 × π × radius
```

The robot measures the wheel turns well (encoders). The **radius** it uses is just a number it was
given (`wheel_radius_0` … `wheel_radius_3`, 0.023 m = 23 mm). Real wheels differ from that number:
tyres wear, rubber squashes under the robot's weight, the manufacturer's value is rounded.

If the real wheel is 1 % bigger than the number, every distance the robot drives is 1 % longer than
the odometry says. After 5 m that's already 5 cm.

## 2. The correction is one multiplication

The wheel turns are the same in both cases, only the radius differs:

```
odometry distance = wheel turns × 2π × radius the robot uses
real distance     = wheel turns × 2π × real radius
```

Divide the two lines and the wheel turns (and the 2π) cancel out:

```
real distance       real radius
----------------- = ----------------------
odometry distance   radius the robot uses
```

So:

```
new radius = old radius × (real distance / odometry distance)
```

**Example:** the odometry says 0.600 m, the lidar says the robot really moved 0.612 m.
Ratio = 0.612 / 0.600 = 1.020: the wheels are 2 % bigger than the robot thinks.
New radius = 23.00 mm × 1.020 = 23.46 mm.

All four wheels are multiplied by the same ratio, so if they had slightly different values
before, they keep those differences.

The rest of the script is about measuring the "real distance" accurately.

## 3. Measuring the real distance with the lidar

### From lidar rays to points

The lidar sends rays all around and measures how far each one goes. A ray at angle `a` that hits
something at distance `r` gives a point in front of the robot:

```
x = r × cos(a)     (forward)
y = r × sin(a)     (to the left)
```

The script only uses the rays within 20° left and right of straight ahead. On a wall about 1 m
away, these are about 120 points (the lidar has 3 rays per degree) along roughly 70 cm of wall.

### A straight line through the points

The points on a wall lie on a straight line. The script fits the line that runs as close as
possible to all of them (a "least squares" fit: the line where the squared distances of the
points from it add up to the smallest total):

```
x = a + b × y
```

- `a` is how far ahead the line is, straight in front of the robot (at y = 0).
- `b` is how much the wall slants. `b = 0` means the robot faces the wall squarely. Positive `b`
  means the wall is further away on the left, so the robot is turned a bit to the left.

The angle the robot is turned by is `atan(b)`.

```
        wall (b = 0, square)              wall (b > 0, robot turned left)
   ─────────────────────────                              ╱
            ↑                                          ╱
            │ a = distance                          ╱
            │                                    ╱  ← a is measured straight ahead,
          [robot]                             ╱       the shortest way to the wall
                                            ╱          is a bit shorter
                                     [robot]
```

### The shortest distance to the wall

If the robot is turned, `a` (straight ahead) is a bit longer than the shortest way to the wall
(at a right angle to it). The script uses the shortest way:

```
distance = a / √(1 + b²)
```

This is the same as `a × cos(angle)`, because `cos(atan(b)) = 1 / √(1 + b²)`. It's the usual
right-triangle rule: the slanted side is longer than the straight one.

Why the shortest distance? **It doesn't change when the robot turns on the spot.** If the robot's
heading wobbles by half a degree between two measurements, `a` would change, but the shortest
distance stays the same. Only real movement towards the wall changes it.

### Ignoring things that aren't the wall

A chair leg or a box in front of the wall would pull the line towards it. So the line is fitted
three times:

1. a first line through all points,
2. a line through the 70 % of points closest to the first line (the leg is far off it and drops out),
3. the final line through only the points within 3 cm of the second line.

If fewer than half of the points end up on the line, there is no clear wall in front, and the
script stops with a message.

### Averaging out noise

A single lidar scan is a bit noisy: every ray is off by a few millimetres. The script takes
**10 scans** and uses the **median** of the 10 distances (sort them, take the middle one). The
median isn't thrown off by one bad scan the way an average would be.

## 4. One test run

1. Measure the wall distance (lidar) and read the position (odometry).
2. Drive 0.60 m forward at 0.10 m/s. Slow, so the wheels don't slip.
3. Wait 1 s until the robot stands still.
4. Measure both again.

```
odometry distance = how far the odometry position moved
lidar distance    = wall distance before − wall distance after
ratio             = lidar distance / odometry distance
```

The robot drives forward to the wall and back again, twice. That gives four ratios, and their
**average** is the result. The runs in both directions also even out effects that only happen in
one direction.

### Driving slightly crooked

Before the runs, the robot turns until it faces the wall within 1°. If it still drives at a small
angle θ to the wall, it gets closer to the wall a bit less than it actually moves:

```
change of wall distance = distance moved × cos(θ)
```

So the script divides by `cos(θ)`. At 1°, `cos(1°) = 0.99985`, a correction of 0.015 %. It hardly
matters, but it costs nothing.

## 5. How accurate is it?

Every run uses two wall measurements (before and after). If each one were off by 1 mm, the lidar
distance could be off by 2 mm. On 600 mm that's 0.3 %. That's why:

- the runs are long (0.6 m): the same millimetre error is a smaller share of a longer distance,
- each measurement is the median of 10 scans, and the result the average of 4 runs.

The script applies the result only if it makes sense:

| Check | Limit | Why |
| --- | --- | --- |
| Change too small | under 0.2 % | Already accurate, the difference is noise |
| Change too big | over 15 % | Something went wrong (no flat wall, wheels slipping) |
| Runs disagree | over 2 % apart | A run was disturbed, the average can't be trusted |

## 6. Turning: `calibrate_turning.py`

Run it after `calibrate_odometry.py`, with the same setup (wall about 1 m in front, room to turn
on the spot).

### Which numbers decide turning?

The robot exposes 12 parameters (`ros2 param list /Edurob`): the four wheel radii, plus where each
wheel sits relative to the robot's centre:

| Wheel | `wheel_pos_x` (forward) | `wheel_pos_y` (left) |
| --- | --- | --- |
| 0 | +0.100 m | −0.0865 m |
| 1 | +0.100 m | +0.0865 m |
| 2 | −0.100 m | +0.0865 m |
| 3 | −0.100 m | −0.0865 m |

When a mecanum robot turns on the spot, each wheel rolls along a circle. How far the wheels have to
roll for a given turn depends on the **lever** `|x| + |y|` = 0.100 + 0.0865 = 0.1865 m:

```
wheel roll distance = turn angle (in radians) × lever
```

The robot goes the other way round to work out how far it turned:

```
turn angle = wheel roll distance / lever
```

That's the same situation as with the wheel radius: if the lever the robot uses is wrong, every
turn is off by the same factor. Here the lever is in the denominator, so the correction is a
division:

```
new lever = old lever / (real turn / odometry turn)
```

The script scales all eight `wheel_pos` values by the same factor. This changes the lever but keeps
the robot's shape (the ratio of length to width stays the same).

Why does the real lever differ from the measured wheel positions? Mecanum wheels touch the floor
through small rollers, the contact point isn't exactly below the wheel's centre, and the rollers
slip a little when turning. The "effective" lever includes all of that.

### Measuring a turn with the wall

The script uses the same wall line as above (section 3), but this time the **angle** of the wall
matters, not its distance:

1. Line up with the wall and measure the wall angle (e.g. +0.3°) and the odometry heading.
2. Turn left a full 360° by odometry, at 30°/s.
3. Measure the wall angle again (e.g. +5.4°) and the odometry heading.

The wall looks turned by 5.4° − 0.3° = 5.1°. The robot turned a full circle, plus those 5.1°:

```
real turn     = 360° + (wall angle after − wall angle before) = 365.1°
odometry turn = 360° + what's left over of the odometry heading change   (e.g. 360.7°)
ratio         = real turn / odometry turn = 365.1 / 360.7 = 1.0122
```

So the robot turns 1.2 % more than it thinks. New lever = 186.5 mm / 1.0122 = 184.3 mm.

The robot does this twice to the left and twice to the right (for a right turn, both turns are
−360° plus the leftover). The average of the four ratios is the result.

**Accuracy:** the wall angle comes from a line through about 120 points, so it's good to a fraction
of a degree. Over a full circle, 0.2° is 0.06 %. Turning a whole circle instead of e.g. 90° makes
the same angle error four times smaller.

### Wheels or gyro?

The robot also has a gyro (IMU). The firmware (`Edurob_Firmware/`, the drive code is in
`DriveHAL.cpp` of the `edurobv2-feature-ROSDriveHAL` project) keeps **two** positions, both updated
every control step from the same wheel movements:

| Position | Heading from | Used for |
| --- | --- | --- |
| `OdoPose` | wheels only (radius + wheel positions) | published as `/odom` → `/base_link`: what `motion.py`, the web page and these scripts see |
| `GyroOdoPose` | the gyro, once it's calibrated | only inside the robot: feedback for its own speed controller |

So the odometry our scripts use takes its heading **only from the wheels**, and changing the wheel
positions does correct it. Converting wheel turns into robot movement works like this in the firmware:

```
robot turn = sum over the 4 wheels of (± wheel radius / lever × wheel turn) / 4
```

So the turning also gets the wheel radius from section 2. That's why `calibrate_odometry.py` comes
first: it fixes the radius, then this script fixes the lever that's left.

The gyro still matters for driving: when you command 30°/s, the robot's controller checks its
turning against the gyro. But when to stop (360° reached) is decided by the odometry, the wheels.

After setting the new values, the script turns once more each way as a check. If the ratio didn't
get much closer to 1, the result was noise (wheels slipping, uneven floor) and it puts the old
values back.

| Check | Limit |
| --- | --- |
| Already accurate | within 0.3 % (about 1° per full turn) |
| Change too big | over 15 % |
| Turns disagree | over 1 % apart (3.6° per turn) |

## 7. What it doesn't calibrate

- **Driving sideways (strafing):** mecanum wheels roll on small rollers and slip more sideways.
  A sideways run would need its own factor.
- **The gyro:** there is no parameter for its scale. It only affects the robot's own speed
  controller, not the odometry (section 6).

**After a reboot** the robot keeps the values: the firmware saves every change in its flash
(`parameters.cpp`), to 0.1 mm (24.106 mm is stored as 24.1 mm, a difference of 0.02 %). The scripts
also print `ros2 param set` commands at the end, e.g. to copy the values to another robot from the
web page's ROS 2 console.
