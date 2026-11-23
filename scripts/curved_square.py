# Square with rounded corners (from the old main.c run()).
import math

from motion import *

print('Start:            ', end='')
print_pose()

goto_xy(0.5, 1, 2)
move_spinning(0.5, 1, 30, 20)
for i in range(1, 5):
    forward(0.5, 2.0)
    delay(100)
    print(f'{i} after forward: ', end='')
    print_pose()

    curve_right(0.2, 0.5, 90)
    delay(100)
    print(f'{i} after rotate:  ', end='')
    print_pose()

orbit_left(0.5, 1, 180)

print(f'Back at start? Off by {math.hypot(get_x(), get_y()):.3f} m')
