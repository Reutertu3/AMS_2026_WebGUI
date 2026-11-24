# Turning calibration with the lidar: does the robot really turn as far as its odometry says?
#
# Idea: the robot turns a full 360 degrees by its odometry. Facing a wall before and after, the
# lidar measures the wall's angle: if the robot really turned 365 degrees, the wall now looks
# turned by 5 degrees. The robot works out turning from the wheels and how far they sit from the
# centre (wheel_pos_x / wheel_pos_y, the "lever"). Scaling those corrects the turning:
#     new wheel positions = old wheel positions / (real turn / odometry turn)
# The published odometry (/odom) takes its heading only from the wheels (firmware DriveHAL.cpp), the
# gyro is only used by the robot's own speed controller. After changing the values the script checks
# with an extra turn that it helped, otherwise it puts the old values back. CALIBRATE.md explains it.
#
# Setup: like calibrate_odometry.py (run that one first): robot ~1 m in front of a straight wall,
#        roughly facing it, room to turn on the spot.
# Run:   web page -> Scripts -> calibrate_turning.py -> Run
import math

import numpy as np

import edurobot as core
from calibrate_odometry import face_wall, get_params, measure_wall, print_set_commands, set_params
from motion import delay, rotate_left, rotate_right

TURN_SPEED = 30       # degrees/s, slow = less wheel slip
TURNS_PER_SIDE = 2    # full turns to the left and to the right
APPLY = True          # False: only measure and print, don't change the robot

GOOD_ENOUGH = 0.003   # within 0.3 % (about 1 degree per full turn): nothing to change
MAX_CHANGE = 0.15     # don't apply a change of more than 15 % (something must be wrong)
MAX_SPREAD = 0.01     # don't apply if the single turns differ by more than 1 % (3.6 degrees)

POS_PARAMS = [f'wheel_pos_{axis}_{i}' for axis in 'xy' for i in range(4)]


def odometry_yaw():
    """Odometry heading in degrees. The robot has to stand still."""
    return math.degrees(core._update_pose()[2])


def wrap(degrees):
    """Angle to -180..180."""
    return (degrees + 180) % 360 - 180


def measure_turn(direction):
    """One full turn on the spot, direction +1 = left, -1 = right. Returns real turn / odometry turn."""
    _, wall_before = measure_wall()  # degrees the robot is turned away from facing the wall
    yaw_before = odometry_yaw()

    if direction > 0:
        rotate_left(360, TURN_SPEED)
    else:
        rotate_right(360, TURN_SPEED)
    delay(1000)  # stand still, so lidar and odometry are up to date

    _, wall_after = measure_wall()
    yaw_after = odometry_yaw()

    # Both turns are close to a full circle: 360 degrees plus the small part left over
    odometry_turn = direction * 360 + wrap(yaw_after - yaw_before)
    real_turn = direction * 360 + (wall_after - wall_before)
    ratio = real_turn / odometry_turn
    name = 'left' if direction > 0 else 'right'
    print(f'  {name:<6} odometry {odometry_turn:+7.1f} deg   lidar {real_turn:+7.1f} deg   ratio {ratio:.4f}')
    return ratio


def main():
    print('Turning calibration with the lidar')
    old_pos = get_params(POS_PARAMS)
    lever = abs(old_pos[0]) + abs(old_pos[4])  # wheel 0: |x| + |y|
    print(f'Wheel positions now: lever |x| + |y| = {lever * 1000:.1f} mm')

    print('Lining up with the wall...')
    face_wall()

    print(f'Turning a full circle, {TURNS_PER_SIDE} times each way...')
    ratios = []
    for _ in range(TURNS_PER_SIDE):
        ratios.append(measure_turn(+1))
        ratios.append(measure_turn(-1))

    ratio = float(np.mean(ratios))
    spread = max(ratios) - min(ratios)
    new_pos = [p / ratio for p in old_pos]
    print()
    print(f'Result: the robot really turns {ratio:.4f} x what the odometry says '
          f'(turns differ by {spread * 100:.1f} %)')
    print(f'New lever: {lever / ratio * 1000:.1f} mm')

    if abs(ratio - 1) < GOOD_ENOUGH:
        print('Already accurate (within 0.3 %), nothing changed.')
        return
    if abs(ratio - 1) > MAX_CHANGE:
        print(f'Not applied: change of {(ratio - 1) * 100:+.1f} % is too big to be right. '
              f'Wall straight? Wheels slipping?')
        return
    if spread > MAX_SPREAD:
        print(f'Not applied: the turns differ too much ({spread * 100:.1f} %). Wheels slipping? Try again.')
        return
    if not APPLY:
        print('Not applied (APPLY = False).')
        return

    set_params(POS_PARAMS, new_pos)
    print('Set on the robot. Checking with one more turn each way...')
    check = float(np.mean([measure_turn(+1), measure_turn(-1)]))

    if abs(check - 1) < abs(ratio - 1) / 2:
        print(f'Better: ratio now {check:.4f} (was {ratio:.4f}).')
        print_set_commands(POS_PARAMS, new_pos)
    else:
        set_params(POS_PARAMS, old_pos)
        print(f'Didn\'t help (ratio now {check:.4f}, was {ratio:.4f}), old values restored. The turns differ')
        print('from one run to the next (wheels slipping? uneven floor?): try again, or turn slower (TURN_SPEED).')


if __name__ == '__main__':
    main()
