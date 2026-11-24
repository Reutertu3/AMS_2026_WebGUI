# Gripper demo: open, grab what is put between the jaws, drive a bit, put it down again.
# GRIPPER.md explains the functions.
#
# Run: web page -> gripper_demo.py -> Run
#      or ./ros.sh python3 scripts/gripper_demo.py
from motion import *
from gripper import *

print(get_gripper_settings())
release()
print_gripper()

print('Put something between the jaws, grabbing in 3 s...')
delay(3000)
if grip():
    print(f'Holding it at {get_gripper_angle():.1f} deg, {get_gripper_current():.0f} mA')
    forward(0.1, 0.2)
    backward(0.1, 0.2)
else:
    print('Nothing in the jaws')
release()
print_gripper()
