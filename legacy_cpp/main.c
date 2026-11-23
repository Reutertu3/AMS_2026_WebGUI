// main.c - your program, Arduino style.
// Press Ctrl+C to stop. The robot is stopped automatically.

#include <math.h>   // sqrt()
#include <stdio.h>  // printf(), like Serial.print() on Arduino

#include "edurobot.h"

void setup(void)
{
  delay(1000);  // give ROS a moment to connect before the first command
}

// void loop(void)
// {
//   forward(0.1, 0.1);      // speed 0.1 m/s, distance 0.1 m (10 cm)
//   delay(250);
//   rotate_right(90, 30, true);   // 90 degrees, speed 30 degrees/s
//   delay(250);
//   forward(0.1, 0.1);

// }

void haus_vom_nikolaus(void)
{
  // Outer Frame
  rotate_left(90, 30, true);
  goto_xy(0.5, 0, 1.5, true);
  rotate_right(45, 25, true);
  goto_xy(0.5, 1, 3, true);
  rotate_right(90, 25, true);
  goto_xy(0.5, 2, 1.5, true);
  rotate_right(45, 25, true);
  goto_xy(0.5, 2, 0, true);
  rotate_right(90, 25, true);
  goto_xy(0.5, 0, 0, true);
  rotate_right(90, 25, true);

  // Inner Construction
  goto_xy(0.5, 2, 1.5, true);
  goto_xy(0.5, 0, 1.5, true);
  goto_xy(0.5, 2, 0, true);

  delay(2000);
}

void run(void)
{
  printf("Start:            ");
  print_pose();

//#### Move in a square
  // for (int i = 1; i <= 4; i++)
  // {
  //   forward(0.2, 1.5, true);
  //   delay(250);
  //   printf("%d after forward: ", i);
  //   print_pose();

  //   rotate_right(90, 30, true);
  //   delay(250);
  //   printf("%d after rotate:  ", i);
  //   print_pose();
  // }
//#### Drive a curved square
goto_xy(0.5,1,2,true);
move_spinning(0.5,1,30,20,true);
  for (int i = 1; i <= 4; i++)
  {
    forward(0.5, 2.0, true);
    delay(100);
    printf("%d after forward: ", i);
    print_pose();

    curve_right(0.2, 0.5, 90, true);
    delay(100);
    printf("%d after rotate:  ", i);
    print_pose();
  }
  orbit_left(0.5, 1, 180, true);
  
  // The single values, if you want to use them yourself:
  printf("Back at start? Off by %.3f m\n", sqrt(get_x() * get_x() + get_y() * get_y()));
}  // end of run(): main() continues with edurobot_end(), which stops the robot and closes ROS


int main(int argc, char **argv)
{
  edurobot_begin(argc, argv);
  setup();
  // while (edurobot_running()) {
  //   loop();
  // }
  // run();          // called once
  haus_vom_nikolaus();
  edurobot_end();
  return 0;
}
