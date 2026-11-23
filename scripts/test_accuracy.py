# Short test for tuning LATENCY in edurobot.py: one 90 degree turn and 30 cm forward.
# Needs ~1 m free space in front. Robot overshoots -> increase LATENCY, stops short -> decrease.
from motion import *

print('start:              ', end='')
print_pose()
rotate_left(90, 30)
delay(1000)
print('after rotate 90:    ', end='')
print_pose()
forward(0.2, 0.3)
delay(1000)
print('after forward 0.3:  ', end='')
print_pose()
