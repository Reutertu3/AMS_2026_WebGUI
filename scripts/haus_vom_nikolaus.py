# Haus vom Nikolaus (from the old main.c). Coordinates in m from the start point, x = forward, y = left.
from motion import *

# Outer frame
rotate_left(90, 30)
goto_xy(0.5, 0, 1.5)
rotate_right(45, 25)
goto_xy(0.5, 1, 3)
rotate_right(90, 25)
goto_xy(0.5, 2, 1.5)
rotate_right(45, 25)
goto_xy(0.5, 2, 0)
rotate_right(90, 25)
goto_xy(0.5, 0, 0)
rotate_right(90, 25)

# Inner construction
goto_xy(0.5, 2, 1.5)
goto_xy(0.5, 0, 1.5)
goto_xy(0.5, 2, 0)

print_pose()
