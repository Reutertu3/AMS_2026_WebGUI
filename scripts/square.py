# Drive a 1 m square and print the position after every step.
from motion import *

for i in range(1, 5):
    forward(0.2, 1.0)
    print(f'{i} after forward: ', end='')
    print_pose()

    rotate_right(90, 30)
    print(f'{i} after rotate:  ', end='')
    print_pose()
