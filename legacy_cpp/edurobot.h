// edurobot.h - Arduino-style movement library for the edurob.
//
// The functions are written in C++ (edurobot.cpp) but can be called from plain C (main.c).
//
// Movements use odometry (the robot's measured position), see the movement section below.
// Every movement function blocks until it is done and then stops the robot, like delay().

#ifndef EDUROBOT_H
#define EDUROBOT_H

#include <stdbool.h>  // bool, true, false in C
#include <stdint.h>

#ifdef __cplusplus
extern "C" {  // lets main.c (C) call the C++ functions
#endif

// ---------- start / stop the connection ----------
void edurobot_begin(int argc, char **argv);  // call once at the start of main()
int edurobot_running(void);                  // 1 until Ctrl+C is pressed
void edurobot_end(void);                     // stops the robot, call once at the end

// ---------- Arduino-like helpers ----------
void delay(uint32_t ms);  // wait ms milliseconds
uint32_t millis(void);    // milliseconds since edurobot_begin()

// Calibrate the robot's gyro (IMU): measures its drift for `seconds` while the robot stands
// still. Stops the robot first, don't touch it meanwhile. Blocks until done, prints the offsets.
// Returns false if it failed (no robot, e.g. turtlesim). Good place: setup().
bool calibrate_gyro(uint8_t seconds);

// ---------- movement ----------
// All movements use odometry: they keep going until the robot has really moved that far.
// Every one blocks until it is done and then stops the robot.
//
// ramp: true  = speed up gently at the start and slow down before the goal
//               (less wheel slip, less overshoot, takes a bit longer)
//       false = full speed right away, hard stop at the goal
//
// Units: speeds in m/s (driving) or degrees/s (turning), distances in m, angles in degrees.
// Positive angles = left / counterclockwise, like everywhere in ROS.

// --- straight lines, the robot doesn't turn ---
void forward(float vel, float distance, bool ramp);
void backward(float vel, float distance, bool ramp);
void strafe_left(float vel, float distance, bool ramp);   // slide sideways
void strafe_right(float vel, float distance, bool ramp);
// Any direction: angle 0 = forward, 90 = left, -90 = right, 180 = backward, 45 = front-left
void move_direction(float vel, float distance, float angle, bool ramp);

// --- turning on the spot ---
void rotate_left(float degree, float vel, bool ramp);   // turn BY degree, counterclockwise
void rotate_right(float degree, float vel, bool ramp);  // turn BY degree, clockwise
// Turn TO a heading, measured from the start of the program (0 = start direction, 90 = left of it).
// Takes the shorter way round. Doesn't add up errors like repeated rotate_* calls.
void turn_to(float degree, float vel, bool ramp);

// --- circles ---
// Drive forward along a circle with `radius` m at `vel` m/s, until the robot has turned `degree`
// degrees (90 = quarter circle, 360 = full circle). The arc is radius * degree * pi / 180 m long.
void curve_left(float vel, float radius, float degree, bool ramp);   // counterclockwise
void curve_right(float vel, float radius, float degree, bool ramp);  // clockwise
// Circle around a point `radius` m in front of the robot, always facing it (e.g. to look at an
// object from all sides). The robot strafes left/right at `vel` m/s. degree = how far round.
void orbit_left(float vel, float radius, float degree, bool ramp);
void orbit_right(float vel, float radius, float degree, bool ramp);
// Rotate by `degree` around the point (dx, dy) instead of the robot's centre.
// dx = m in front of the robot, dy = m to the left. degree > 0 = counterclockwise. vel in degrees/s.
// e.g. rotate_around(0.2, 0, 90, 30, true): pivot 90 degrees left around a point 20 cm in front
void rotate_around(float dx, float dy, float degree, float vel, bool ramp);

// --- go to a position (x, y measured from the start of the program, like get_x/get_y) ---
// Drives in a straight line without turning, and steers towards the target the whole way.
void goto_xy(float vel, float x, float y, bool ramp);
// Same, and turns to heading `yaw` (degrees) at the same time, arriving at both together.
void goto_pose(float vel, float x, float y, float yaw, bool ramp);

// --- straight line while spinning ---
// Drive `distance` m in direction `angle` (as in move_direction, fixed when the call starts)
// while turning at `spin` degrees/s (> 0 = counterclockwise). The path stays straight.
void move_spinning(float vel, float distance, float angle, float spin, bool ramp);

// --- direct control ---
// Set all three speeds directly and keep them for ms milliseconds, then stop. No odometry.
//   x     forward (+) / backward (-)  in m/s
//   y     left (+)    / right (-)     in m/s
//   turn  left (+)    / right (-)     in degrees/s
void drive(float x, float y, float turn, uint32_t ms);

void stop(void);  // all speeds to 0

// ---------- odometry (where the robot is) ----------
// Measured from where the robot (or turtle) was started.
float get_x(void);    // m, forward at start = +x
float get_y(void);    // m, left at start = +y
float get_yaw(void);  // degrees, -180..180, positive = turned left (counterclockwise)
void print_pose(void);  // prints e.g. "x=0.100 m  y=0.000 m  yaw=-90.0 deg"

#ifdef __cplusplus
}
#endif

#endif
