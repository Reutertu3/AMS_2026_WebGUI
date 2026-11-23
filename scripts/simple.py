# Example movements: strafing, diagonals, orbiting and pivoting, then back to the start.
# Needs about 1.5 x 1.5 m of free space around the robot.
#
# Run: web page -> simple.py -> Run
#      or ./ros.sh python3 scripts/simple.py
#      turtlesim: ./ros.sh --turtle python3 scripts/simple.py
#
# Every movement waits until it is done (measured with odometry), then the next line runs.
from motion import *

SPEED = 0.5  # m/s, good for the real robot (slow on the turtle)
TURN = 30    # degrees/s

print('Start:          ', end='')
print_pose()

# # 1. Strafe: slide sideways without turning (mecanum wheels)
# strafe_left(SPEED, 0.3)
# strafe_right(SPEED, 0.6)
# strafe_left(SPEED, 0.3)        # back to the middle
# print('After strafing: ', end='')
# print_pose()

# # 2. Diamond: four diagonal strafes, the robot keeps facing forward
# for angle in (45, 135, -135, -45):   # front-left, back-left, back-right, front-right
#     move_direction(SPEED, 0.3, angle)
# print('After diamond:  ', end='')
# print_pose()

# 3. Orbit: circle an (imaginary) object 40 cm in front, always facing itorbit_left(0.5, 0.4, 90)
orbit_left(0.3, 0.3, 360)
print('After orbits:   ', end='')
print_pose()

# # 4. Pivot around a point 20 cm in front of the robot, 90 degrees there and back
# rotate_around(0.2, 0, 90, TURN)
# rotate_around(0.2, 0, -90, TURN)
# print('After pivots:   ', end='')
# print_pose()

# # 5. Back to the start: position and heading (corrects what the moves above drifted)
# goto_pose(SPEED, 0, 0, 0)
# print('End:            ', end='')
# print_pose()
