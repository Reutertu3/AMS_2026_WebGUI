// edurobot.cpp - ROS plumbing behind edurobot.h. Like the Arduino core: you normally don't touch this.
//
// How the robot is driven:
//   We publish a geometry_msgs/Twist message on the topic "cmd_vel" (at most twice per second,
//   see CMD_INTERVAL_MS). The robot keeps its last speed until told otherwise (or ~2 s without
//   commands), so we always send a stop at the end.
//
// How we know where the robot is (odometry):
//   Real robot: the odom -> base_link transform on /tf.
//   Turtlesim:  the turtlesim_msgs/Pose topic "turtle_pose" (ros.sh --turtle remaps it to /turtle1/pose).
//   Movements keep driving until the measured distance / angle is reached.

#include "edurobot.h"

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <memory>
#include <thread>

#include <edurob_interfaces/srv/calibrate_gyro.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <turtlesim_msgs/msg/pose.hpp>

static const uint32_t SEND_INTERVAL_MS = 50;  // how often the odometry is checked
// The real robot (micro-ROS over WiFi) handles only ~2 commands/s. Faster ones pile up and are
// executed seconds late (it keeps driving after stop). Measured: 500 ms ok, 300 ms already lags.
static const uint32_t CMD_INTERVAL_MS = 500;  // speed commands are sent at most this often (stop always)
// Real robot: odometry arrives late and commands take effect late (~0.6 s together, measured).
// The pose is predicted this far ahead with the last command, and stop() waits this long.
// Robot overshoots → increase, stops short → decrease. Not used for the turtle (no delay there).
static const uint32_t LATENCY_MS = 600;
static const uint32_t WAIT_FOR_POSE_MS = 10000;

// Fixed speed ramps (used when a movement is called with ramp = true)
static const double LINEAR_ACCEL = 0.5;    // m/s², speed up / slow down this fast
static const double ANGULAR_ACCEL = 90.0;  // degrees/s²
static const double MIN_LINEAR = 0.02;     // m/s, never slower than this, or it would never arrive
static const double MIN_ANGULAR = 5.0;     // degrees/s

// goto_xy / goto_pose / turn_to
static const double POS_TOLERANCE = 0.005;  // m, close enough to the target point
static const double YAW_TOLERANCE = 0.5;    // degrees
static const double DEFAULT_TURN = 30.0;    // degrees/s, goto_pose when only turning
static const double MAX_TURN = 90.0;        // degrees/s, goto_pose turning limit

struct Pose {
  double x;    // m
  double y;    // m
  double yaw;  // rad, positive = counterclockwise
};

static rclcpp::Node::SharedPtr node;
static rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
static std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
static std::unique_ptr<tf2_ros::Buffer> tf_buffer;
static std::shared_ptr<tf2_ros::TransformListener> tf_listener;
static rclcpp::Subscription<turtlesim_msgs::msg::Pose>::SharedPtr turtle_sub;

static Pose pose;        // last known position
static Pose start_pose;  // position at edurobot_begin(), get_x() etc. are relative to it
static Pose turtle_pose;
static bool have_turtle_pose = false;
static bool real_robot = false;        // odometry comes from /tf (not the turtle)
static Pose last_cmd = {0, 0, 0};      // last sent command: m/s, m/s, rad/s

static std::chrono::steady_clock::time_point start_time;
static volatile std::sig_atomic_t running = 1;  // set to 0 by Ctrl+C

static void on_ctrl_c(int)
{
  running = 0;
}

static void publish(float x, float y, float turn_deg)
{
  static auto last = std::chrono::steady_clock::time_point{};
  auto now = std::chrono::steady_clock::now();
  bool is_stop = x == 0 && y == 0 && turn_deg == 0;
  if (!is_stop && now - last < std::chrono::milliseconds(CMD_INTERVAL_MS)) {
    return;  // the robot still drives the last command, see CMD_INTERVAL_MS
  }
  last = now;
  last_cmd = {x, y, turn_deg * M_PI / 180.0};
  geometry_msgs::msg::Twist msg;
  msg.linear.x = x;
  msg.linear.y = y;
  msg.angular.z = turn_deg * M_PI / 180.0;  // ROS wants rad/s
  publisher->publish(msg);
}

// Wrap an angle in radians to the range [-pi, pi]
static double wrap_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// ---------- odometry ----------

// Read the current position. Returns false if none is available (yet).
static bool read_pose(Pose &out)
{
  try {
    auto t = tf_buffer->lookupTransform("odom", "base_link", tf2::TimePointZero);
    auto q = t.transform.rotation;
    out.x = t.transform.translation.x;
    out.y = t.transform.translation.y;
    out.yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    real_robot = true;
    // Predict where the robot will be once the delay has passed (see LATENCY_MS)
    double l = LATENCY_MS / 1000.0;
    out.x += (last_cmd.x * std::cos(out.yaw) - last_cmd.y * std::sin(out.yaw)) * l;
    out.y += (last_cmd.x * std::sin(out.yaw) + last_cmd.y * std::cos(out.yaw)) * l;
    out.yaw += last_cmd.yaw * l;
    return true;
  } catch (const tf2::TransformException &) {
  }
  if (have_turtle_pose) {
    out = turtle_pose;
    return true;
  }
  return false;
}

// Process incoming messages and refresh `pose` (keeps the last known pose if reading fails)
static void update_pose(void)
{
  executor->spin_some();
  Pose p;
  if (read_pose(p)) {
    pose = p;
  }
}

// Current pose seen from the start pose (start = 0, 0, 0 facing +x), yaw in radians
static Pose relative_pose(void)
{
  update_pose();
  double dx = pose.x - start_pose.x;
  double dy = pose.y - start_pose.y;
  double c = std::cos(start_pose.yaw);
  double s = std::sin(start_pose.yaw);
  return {c * dx + s * dy, -s * dx + c * dy, wrap_angle(pose.yaw - start_pose.yaw)};
}

float get_x(void)   { return relative_pose().x; }
float get_y(void)   { return relative_pose().y; }
float get_yaw(void) { return relative_pose().yaw * 180.0 / M_PI; }

void print_pose(void)
{
  Pose p = relative_pose();
  std::printf("x=%.3f m  y=%.3f m  yaw=%.1f deg\n", p.x, p.y, p.yaw * 180.0 / M_PI);
  std::fflush(stdout);
}

static void wait_for_pose(void)
{
  uint32_t deadline = millis() + WAIT_FOR_POSE_MS;
  Pose p;
  while (running && millis() < deadline) {
    executor->spin_some();
    if (read_pose(p)) {
      pose = p;
      return;
    }
    delay(100);
  }
  std::fprintf(stderr, "No odometry received. Is the robot (or turtlesim) running?\n");
  edurobot_end();
  std::exit(1);
}

// ---------- start / stop the connection ----------

void edurobot_begin(int argc, char **argv)
{
  // SignalHandlerOptions::None: ROS must not shut down on Ctrl+C, otherwise the final stop
  // command could not be sent. We catch Ctrl+C ourselves instead.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, on_ctrl_c);
  start_time = std::chrono::steady_clock::now();

  node = rclcpp::Node::make_shared("edurobot");
  publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  // Real robot odometry (tf runs in its own background thread)
  tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node);

  // Turtlesim odometry
  turtle_sub = node->create_subscription<turtlesim_msgs::msg::Pose>(
    "turtle_pose", 1, [](const turtlesim_msgs::msg::Pose &msg) {
      turtle_pose = {msg.x, msg.y, msg.theta};
      have_turtle_pose = true;
    });

  executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);

  wait_for_pose();
  start_pose = pose;
}

int edurobot_running(void)
{
  return running;
}

void edurobot_end(void)
{
  stop();
  executor.reset();
  turtle_sub.reset();
  tf_listener.reset();
  tf_buffer.reset();
  publisher.reset();
  node.reset();
  rclcpp::shutdown();
}

// ---------- Arduino-like helpers ----------

bool calibrate_gyro(uint8_t seconds)
{
  using CalibrateGyro = edurob_interfaces::srv::CalibrateGyro;
  stop();
  auto client = node->create_client<CalibrateGyro>("calibrategyro");
  if (!client->wait_for_service(std::chrono::seconds(3))) {
    std::fprintf(stderr, "calibrate_gyro: no /calibrategyro service (real robot connected?)\n");
    return false;
  }
  auto request = std::make_shared<CalibrateGyro::Request>();
  request->time = seconds;
  std::printf("Calibrating gyro for %u s, keep the robot still...\n", seconds);
  std::fflush(stdout);
  auto future = client->async_send_request(request).future.share();
  uint32_t deadline = millis() + seconds * 1000u + 5000;
  while (running && millis() < deadline) {
    executor->spin_some();
    if (future.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
      auto o = future.get()->offsets;
      std::printf("Gyro calibrated, offsets x=%.5f y=%.5f z=%.5f\n", o.x, o.y, o.z);
      std::fflush(stdout);
      return true;
    }
  }
  std::fprintf(stderr, "calibrate_gyro: no answer from the robot\n");
  return false;
}

uint32_t millis(void)
{
  auto elapsed = std::chrono::steady_clock::now() - start_time;
  return std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
}

void delay(uint32_t ms)
{
  // Sleep in small steps so Ctrl+C is noticed quickly
  uint32_t end = millis() + ms;
  while (running && millis() < end) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

// ---------- movement ----------

static double deg(double rad) { return rad * 180.0 / M_PI; }
static double rad(double deg) { return deg * M_PI / 180.0; }

// Safety limit: give up after twice the expected time + 5 s (e.g. robot stuck)
static uint32_t timeout_ms(double amount, double vel)
{
  if (vel == 0) {
    return 5000;
  }
  return std::fabs(amount) / std::fabs(vel) * 2000 + 5000;
}

// Speed factor 0..1 for a ramp: speed up after the start, slow down before the goal.
//   done, remaining  how far the move has come / still has to go (m or degrees)
//   speed            the full speed (m/s or degrees/s)
// The highest speed reachable after `done` (or stoppable within `remaining`) is sqrt(2 * accel * distance).
static double ramp_factor(double done, double remaining, double speed, double accel, double min_speed)
{
  if (speed <= 0) {
    return 1;
  }
  double v = std::min({speed, std::sqrt(2 * accel * std::max(done, 0.0)),
                       std::sqrt(2 * accel * std::max(remaining, 0.0))});
  v = std::max(v, std::min(min_speed, speed));
  return v / speed;
}

// What one step of a movement wants to send
struct Command {
  double x;     // m/s
  double y;     // m/s
  double turn;  // degrees/s
  bool done;
};
static const Command DONE = {0, 0, 0, true};

// The odometry loop shared by all movements: every 50 ms read the pose, ask step() what to send
static void run_move(const char *what, uint32_t timeout, const std::function<Command()> &step)
{
  uint32_t deadline = millis() + timeout;
  while (running) {
    update_pose();
    Command c = step();
    if (c.done) {
      break;
    }
    if (millis() > deadline) {
      std::fprintf(stderr, "Timeout, %s not reached\n", what);
      break;
    }
    publish(c.x, c.y, c.turn);
    delay(SEND_INTERVAL_MS);
  }
  stop();
}

void drive(float x, float y, float turn, uint32_t ms)
{
  uint32_t end = millis() + ms;
  while (running && millis() < end) {
    publish(x, y, turn);
    delay(SEND_INTERVAL_MS);
  }
  stop();
}

void stop(void)
{
  publish(0, 0, 0);
  if (real_robot) {
    delay(LATENCY_MS);  // let the robot really stop and the odometry catch up
  }
}

// ----- straight lines -----

void move_direction(float vel, float distance, float angle, bool ramp)
{
  double speed = std::fabs(vel);
  double goal = std::fabs(distance);
  double x = speed * std::cos(rad(angle));
  double y = speed * std::sin(rad(angle));
  update_pose();
  Pose start = pose;

  run_move("distance", timeout_ms(goal, speed), [&]() -> Command {
    double done = std::hypot(pose.x - start.x, pose.y - start.y);
    if (done >= goal) {
      return DONE;
    }
    double f = ramp ? ramp_factor(done, goal - done, speed, LINEAR_ACCEL, MIN_LINEAR) : 1;
    return {x * f, y * f, 0, false};
  });
}

void forward(float vel, float distance, bool ramp)      { move_direction(vel, distance, 0, ramp); }
void backward(float vel, float distance, bool ramp)     { move_direction(vel, distance, 180, ramp); }
void strafe_left(float vel, float distance, bool ramp)  { move_direction(vel, distance, 90, ramp); }
void strafe_right(float vel, float distance, bool ramp) { move_direction(vel, distance, -90, ramp); }

// ----- constant speeds, stop after turning `degree` -----

// Send (x, y, turn) until the heading has changed by `degree` degrees.
// Used for rotate (x = y = 0), curve (x), orbit (y) and rotate_around (x and y).
static void move_angle(double x, double y, double turn, double degree, bool ramp)
{
  double goal = std::fabs(degree);
  double turn_speed = std::fabs(turn);
  double linear_speed = std::hypot(x, y);
  double last_yaw = (update_pose(), pose.yaw);
  double turned = 0;  // degrees, summed up so turns larger than 180 degrees work too

  run_move("angle", timeout_ms(goal, turn_speed), [&]() -> Command {
    turned += deg(wrap_angle(pose.yaw - last_yaw));
    last_yaw = pose.yaw;
    double done = std::fabs(turned);
    if (done >= goal) {
      return DONE;
    }
    double f = 1;
    if (ramp) {
      f = ramp_factor(done, goal - done, turn_speed, ANGULAR_ACCEL, MIN_ANGULAR);
      if (linear_speed > 0) {
        // Same ramp for the driving part: meters driven = degrees turned * linear / turn speed
        double m_per_deg = linear_speed / turn_speed;
        f = std::min(f, ramp_factor(done * m_per_deg, (goal - done) * m_per_deg, linear_speed,
                                    LINEAR_ACCEL, MIN_LINEAR));
      }
    }
    // One factor for all three speeds, so curves keep their radius while ramping
    return {x * f, y * f, turn * f, false};
  });
}

void rotate_left(float degree, float vel, bool ramp)  { move_angle(0, 0, std::fabs(vel), degree, ramp); }
void rotate_right(float degree, float vel, bool ramp) { move_angle(0, 0, -std::fabs(vel), degree, ramp); }

// Turning speed for a circle: omega = v / r (rad/s), converted to degrees/s
static double circle_turn(float vel, float radius)
{
  return deg(std::fabs(vel) / std::fabs(radius));
}

void curve_left(float vel, float radius, float degree, bool ramp)
{
  move_angle(std::fabs(vel), 0, circle_turn(vel, radius), degree, ramp);
}

void curve_right(float vel, float radius, float degree, bool ramp)
{
  move_angle(std::fabs(vel), 0, -circle_turn(vel, radius), degree, ramp);
}

// Strafing left around a centre in front means circling it clockwise, so the robot turns right
void orbit_left(float vel, float radius, float degree, bool ramp)
{
  move_angle(0, std::fabs(vel), -circle_turn(vel, radius), degree, ramp);
}

void orbit_right(float vel, float radius, float degree, bool ramp)
{
  move_angle(0, -std::fabs(vel), circle_turn(vel, radius), degree, ramp);
}

// Keep the point (dx, dy) still: its speed is (x - w*dy, y + w*dx), which must be 0
void rotate_around(float dx, float dy, float degree, float vel, bool ramp)
{
  double turn = std::copysign(std::fabs(vel), degree);  // degrees/s
  double w = rad(turn);                                 // rad/s
  move_angle(w * dy, -w * dx, turn, degree, ramp);
}

void turn_to(float degree, float vel, bool ramp)
{
  double delta = deg(wrap_angle(rad(degree) - relative_pose().yaw));  // shortest way round
  if (std::fabs(delta) < YAW_TOLERANCE) {
    return;
  }
  move_angle(0, 0, std::copysign(std::fabs(vel), delta), delta, ramp);
}

// ----- go to a target (steering towards it every step) -----

// Drive straight to (tx, ty) (relative to the program start). If use_yaw, turn to tyaw at the same time.
static void move_to(float vel, float tx, float ty, bool use_yaw, float tyaw, bool ramp)
{
  double speed = std::fabs(vel);
  Pose p = relative_pose();
  double dist0 = std::hypot(tx - p.x, ty - p.y);
  double yaw_err0 = use_yaw ? deg(wrap_angle(rad(tyaw) - p.yaw)) : 0;

  // Turn just fast enough to finish turning when the robot arrives
  double turn_speed = dist0 > POS_TOLERANCE ? std::fabs(yaw_err0) / (dist0 / speed) : DEFAULT_TURN;
  turn_speed = std::min(turn_speed, MAX_TURN);

  bool pos_done = dist0 <= POS_TOLERANCE;
  bool yaw_done = std::fabs(yaw_err0) <= YAW_TOLERANCE;
  double last_dist = dist0;
  uint32_t timeout = std::max(timeout_ms(dist0, speed),
                              turn_speed > 0 ? timeout_ms(yaw_err0, turn_speed) : 0);

  run_move("target", timeout, [&]() -> Command {
    Pose p = relative_pose();
    double ex = tx - p.x;
    double ey = ty - p.y;
    double dist = std::hypot(ex, ey);
    double yaw_err = deg(wrap_angle(rad(tyaw) - p.yaw));

    // Done when close enough, or when it just went past (distance growing again / error changed sign)
    if (dist <= POS_TOLERANCE || (dist > last_dist && dist < speed * 0.1 + POS_TOLERANCE)) {
      pos_done = true;
    }
    last_dist = dist;
    if (!use_yaw || std::fabs(yaw_err) <= YAW_TOLERANCE || yaw_err * yaw_err0 < 0) {
      yaw_done = true;
    }
    if (pos_done && yaw_done) {
      return DONE;
    }

    Command c = {0, 0, 0, false};
    if (!pos_done) {
      double f = ramp ? ramp_factor(dist0 - dist, dist, speed, LINEAR_ACCEL, MIN_LINEAR) : 1;
      double dir = std::atan2(ey, ex) - p.yaw;  // direction to the target, seen from the robot
      c.x = speed * f * std::cos(dir);
      c.y = speed * f * std::sin(dir);
    }
    if (!yaw_done) {
      double done = std::fabs(yaw_err0) - std::fabs(yaw_err);
      double f = ramp ? ramp_factor(done, std::fabs(yaw_err), turn_speed, ANGULAR_ACCEL, MIN_ANGULAR) : 1;
      c.turn = std::copysign(turn_speed * f, yaw_err);
    }
    return c;
  });
}

void goto_xy(float vel, float x, float y, bool ramp)              { move_to(vel, x, y, false, 0, ramp); }
void goto_pose(float vel, float x, float y, float yaw, bool ramp) { move_to(vel, x, y, true, yaw, ramp); }

// ----- straight line while spinning -----

void move_spinning(float vel, float distance, float angle, float spin, bool ramp)
{
  double speed = std::fabs(vel);
  double goal = std::fabs(distance);
  update_pose();
  Pose start = pose;
  double world_dir = start.yaw + rad(angle);  // fixed direction in the room

  run_move("distance", timeout_ms(goal, speed), [&]() -> Command {
    double done = std::hypot(pose.x - start.x, pose.y - start.y);
    if (done >= goal) {
      return DONE;
    }
    double f = ramp ? ramp_factor(done, goal - done, speed, LINEAR_ACCEL, MIN_LINEAR) : 1;
    // The robot keeps this command until the next one, so aim for its heading halfway through
    double yaw = pose.yaw + rad(spin * f) * CMD_INTERVAL_MS / 2000.0;
    double dir = world_dir - yaw;  // room direction, seen from the robot
    return {speed * f * std::cos(dir), speed * f * std::sin(dir), spin * f, false};
  });
}
