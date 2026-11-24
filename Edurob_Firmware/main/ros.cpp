#include "SensorHAL.h"
// ROS Libs
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include "uxr/client/config.h"

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rclc_parameter/rclc_parameter.h>

#include "edurob_interfaces/srv/calibrate_gyro.h"
#include <std_srvs/srv/trigger.h>
#include <std_srvs/srv/set_bool.h>
#include <sensor_msgs/msg/joint_state.h>

#include "ros.h"
#include "parameters.h"
#include "servo.h"

#define TAG "EduRobROS"

static EduRobV2::Parameters::ParameterStore *Params = nullptr;

// Eigen Library
#include <eigen3/Eigen/Eigen>
using namespace Eigen; // Eigen related statement; simplifies syntax for declaration of matrices

// ROS Messages
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/transform.h>
#include <sensor_msgs/msg/imu.h>
#include <tf2_msgs/msg/tf_message.h>

#include "esp_log.h"

// include the parameters file
//#include "include/parameters.h"

// Convert Eulerdegrees to quaternion
static void euler_to_quat(float x, float y, float z, float *q)
{
  float c1 = cos(y / 2.0);
  float c2 = cos(z / 2.0);
  float c3 = cos(x / 2.0);

  float s1 = sin(y / 2.0);
  float s2 = sin(z / 2.0);
  float s3 = sin(x / 2.0);

  q[0] = c1 * c2 * c3 - s1 * s2 * s3;
  q[1] = s1 * s2 * c3 + c1 * c2 * s3;
  q[2] = s1 * c2 * c3 + c1 * s2 * s3;
  q[3] = c1 * s2 * c3 - s1 * c2 * s3;
}

#define RCCHECK(fn)                                                                 \
  {                                                                                 \
    rcl_ret_t temp_rc = fn;                                                         \
    if ((temp_rc != RCL_RET_OK)) {                                                  \
      printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int) temp_rc); \
      vTaskDelete(NULL);                                                            \
    }                                                                               \
  }
#define RCSOFTCHECK(fn)                                                               \
  {                                                                                   \
    rcl_ret_t temp_rc = fn;                                                           \
    if ((temp_rc != RCL_RET_OK)) {                                                    \
      printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int) temp_rc); \
    }                                                                                 \
  }

static rclc_support_t support;
static rcl_allocator_t allocator;
static rclc_executor_t executor;
static rcl_node_t node;

static geometry_msgs__msg__TransformStamped tfData[1];
static geometry_msgs__msg__TransformStamped tfStaticData[2];
static rcl_subscription_t twistSubscriber;
static geometry_msgs__msg__Twist twistMessage;

static tf2_msgs__msg__TFMessage messageTf;
static rcl_publisher_t publisherTf;
static tf2_msgs__msg__TFMessage messageTfStatic;
static rcl_publisher_t publisherTfStatic;
static sensor_msgs__msg__Imu IMUData;
static rcl_publisher_t publisherIMU;

static rclc_parameter_server_t param_server;

// Gripper
static rcl_publisher_t publisherGripper;
static sensor_msgs__msg__JointState GripperState;
static rcl_service_t GripService, ReleaseService, SweepService, TorqueService;
static std_srvs__srv__Trigger_Request GripReq, ReleaseReq, SweepReq;
static std_srvs__srv__Trigger_Response GripRes, ReleaseRes, SweepRes;
static std_srvs__srv__SetBool_Request TorqueReq;
static std_srvs__srv__SetBool_Response TorqueRes;

static DriveHAL *Drive;

static void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *) msgin;

  if (msg->linear.x == 0 && msg->linear.y == 0 && msg->angular.z == 0) {
    printf("Set drive off!\n");

    Drive->SetMode(PoseControlMode::Off);
  } else {
    Drive->SetMode(PoseControlMode::Manual);
  }
  Drive->SetRobotVel({float(msg->linear.x), float(msg->linear.y), float(msg->angular.z)});
  printf("Received Twist: Linear: %f, %f, %f. Angular: %f, %f, %f\n", msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
}

static void setTfData()
{
  float rosQuaternion[4];
  Pose OdoPose;
  Drive->GetOdomPose(OdoPose);

  tfData[0].header.frame_id = micro_ros_string_utilities_set(tfData[0].header.frame_id, "/odom");
  tfData[0].child_frame_id = micro_ros_string_utilities_set(tfData[0].child_frame_id, "/base_link");
  tfData[0].transform.translation.x = OdoPose.x;
  tfData[0].transform.translation.y = OdoPose.y;
  euler_to_quat(0, 0, float(OdoPose.theta), rosQuaternion);
  tfData[0].transform.rotation.x = (double) rosQuaternion[1];
  tfData[0].transform.rotation.y = (double) rosQuaternion[2];
  tfData[0].transform.rotation.z = (double) rosQuaternion[3];
  tfData[0].transform.rotation.w = (double) rosQuaternion[0];
  tfData[0].header.stamp.nanosec = rmw_uros_epoch_millis() * 1000;
  tfData[0].header.stamp.sec = rmw_uros_epoch_millis() / 1000;

  messageTf.transforms.size = 1;
  messageTf.transforms.capacity = 1;
  messageTf.transforms.data = tfData;
}

// Set static tf data
static void setTfStaticData()
{
  float rosQuaternion[4];

  tfStaticData[0].header.frame_id = micro_ros_string_utilities_set(tfStaticData[0].header.frame_id, "/base_link");
  tfStaticData[0].child_frame_id = micro_ros_string_utilities_set(tfStaticData[0].child_frame_id, "/laser");
  tfStaticData[0].transform.translation.x = 0;
  tfStaticData[0].transform.translation.y = 0;
  tfStaticData[0].transform.translation.z = 0.08;
  tfStaticData[0].transform.rotation.x = 0;
  tfStaticData[0].transform.rotation.y = 0;
  tfStaticData[0].transform.rotation.z = 0;
  tfStaticData[0].transform.rotation.w = 1;
  tfStaticData[0].header.stamp.nanosec = rmw_uros_epoch_millis() * 1000;
  tfStaticData[0].header.stamp.sec = rmw_uros_epoch_millis() / 1000;

  tfStaticData[1].header.frame_id = micro_ros_string_utilities_set(tfStaticData[1].header.frame_id, "/base_link");
  tfStaticData[1].child_frame_id = micro_ros_string_utilities_set(tfStaticData[1].child_frame_id, "/imu");
  tfStaticData[1].transform.translation.x = 0;
  tfStaticData[1].transform.translation.y = 0;
  tfStaticData[1].transform.translation.z = 0.037;
  euler_to_quat(std::numbers::pi_v<float>, 0, 0, rosQuaternion);
  tfStaticData[1].transform.rotation.x = (double) rosQuaternion[1];
  tfStaticData[1].transform.rotation.y = (double) rosQuaternion[2];
  tfStaticData[1].transform.rotation.z = (double) rosQuaternion[3];
  tfStaticData[1].transform.rotation.w = (double) rosQuaternion[0];
  tfStaticData[1].header.stamp.nanosec = rmw_uros_epoch_millis() * 1000;
  tfStaticData[1].header.stamp.sec = rmw_uros_epoch_millis() / 1000;

  messageTfStatic.transforms.size = 2;
  messageTfStatic.transforms.data = tfStaticData;
  messageTfStatic.transforms.capacity = 2;
}

// Set IMU data
static void setImuData()
{
  // Get IMU
  IMUHAL& imu = IMUHAL::getInstance();

  float Accel[3], Gyro[3];

  // Get current IMU data
  if (imu.getIMUData(Accel, Gyro) == -1) {
    ESP_LOGE(TAG, "Can not get current IMU data");

    return;
  }

  // Set data
  IMUData.header.frame_id = micro_ros_string_utilities_set(tfStaticData[0].header.frame_id, "/imu");
  IMUData.header.stamp.nanosec = rmw_uros_epoch_millis() * 1000;
  IMUData.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
  IMUData.orientation_covariance[0] = -1;
  IMUData.angular_velocity.x = Gyro[0];
  IMUData.angular_velocity.y = Gyro[1];
  IMUData.angular_velocity.z = Gyro[2];
  IMUData.linear_acceleration.x = Accel[0];
  IMUData.linear_acceleration.y = Accel[1];
  IMUData.linear_acceleration.z = Accel[2];
}

static std::string ROS_IP;
static std::string ROS_Port;
static size_t ROS_DomainID;

static bool on_parameter_changed(const Parameter * old_param, const Parameter * new_param, void * context)
{
  (void) context;

  // Check errors and cases we do not handle
  if (old_param == NULL && new_param == NULL) {
    ESP_LOGE(TAG, "Callback error, both parameters are NULL");

    return false;
  } else if (old_param == NULL) {
    ESP_LOGE(TAG, "Error: Trying to create new parameter %s\n", new_param->name.data);

    return false;
  } else if (new_param == NULL) {
    ESP_LOGE(TAG, "Error: Trying to delete parameter %s\n", old_param->name.data);

    return false;
  }

  // Gripper parameters: handled before the startup skip below, which only works for
  // doubles (an int parameter never has a double value). Re-applying a default is harmless.
  {
    std::string Name(new_param->name.data);
    const double d = new_param->value.double_value;

    if (Name == "gripper_open_deg" || Name == "gripper_closed_deg") {
      if (d < 0.0 || d > 360.0) {
        ESP_LOGE(TAG, "%s must be 0..360 deg, got %f", Name.c_str(), d);
        return false;
      }
      if (Name == "gripper_open_deg") {
        EduRobV2::Servo::SetOpenDeg(d);
      } else {
        EduRobV2::Servo::SetClosedDeg(d);
      }
      ESP_LOGI(TAG, "%s = %.1f deg", Name.c_str(), d);
      return true;
    }

    // Speed, force and sweep count are read by the services at call time, only validate here
    if (Name == "gripper_speed_pct" || Name == "gripper_force_pct") {
      if (d <= 0.0 || d > 100.0) {
        ESP_LOGE(TAG, "%s must be >0..100 %%, got %f", Name.c_str(), d);
        return false;
      }
      return true;
    }

    if (Name == "gripper_sweep_count") {
      if (new_param->value.integer_value < 1) {
        ESP_LOGE(TAG, "gripper_sweep_count must be >= 1, got %lld", new_param->value.integer_value);
        return false;
      }
      return true;
    }
  }

  // Skip the updates from startup
  if (old_param->value.double_value == 0.0) {
    return true;
  }

  // Log and type check
  switch (old_param->value.type) {
    case RCLC_PARAMETER_DOUBLE:
      ESP_LOGI(TAG, "Parameter %s modified: Old value: %f, New value: %f (double)", old_param->name.data, old_param->value.double_value, new_param->value.double_value);
      break;
    default:
      ESP_LOGE(TAG, "Unknown parameter %s modified: type %d", old_param->name.data, old_param->value.type);

      return false;
      break;
  }

  DriveConfig DriveConf = Drive->GetDriveConfig();
  // Check if we have radius or pos update
  std::string ParamName(old_param->name.data);

  if (ParamName.starts_with("wheel_pos_x_")) {
    uint8_t WheelNr = old_param->name.data[12] - '0';
    ESP_LOGI(TAG, "Update wheel parameter x for wheel %hhu to %f", WheelNr, new_param->value.double_value);

    // Set new value to param store
    Params->SetWheelPosX(WheelNr, new_param->value.double_value);

    // Update DriveConf
    DriveConf.Wheels[WheelNr].Position.x = new_param->value.double_value;
  } else if (ParamName.starts_with("wheel_pos_y_")) {
    uint8_t WheelNr = old_param->name.data[12] - '0';
    ESP_LOGI(TAG, "Update wheel parameter y for wheel %hhu to %f", WheelNr, new_param->value.double_value);

    // Set new value to param store
    Params->SetWheelPosY(WheelNr, new_param->value.double_value);

    // Update DriveConf
    DriveConf.Wheels[WheelNr].Position.y = new_param->value.double_value;
  } else if (ParamName.starts_with("wheel_radius_")) {
    uint8_t WheelNr = old_param->name.data[13] - '0';
    ESP_LOGI(TAG, "Update wheel parameter radius for wheel %hhu to %f", WheelNr, new_param->value.double_value);

    // Set new value to param store
    Params->SetWheelRadius(WheelNr, new_param->value.double_value);

    // Update DriveConf
    DriveConf.Wheels[WheelNr].Radius = new_param->value.double_value;
  } else {
    ESP_LOGE(TAG, "Can not parse parameter name %s", old_param->name.data);

    return false;
  }

  // Activate new DriveConf
  Drive->SetDriveConfig(DriveConf);

  return true;
}

static void CalibrateGyro_service_callback(const void * req, void * res)
{
  edurob_interfaces__srv__CalibrateGyro_Request * req_in = (edurob_interfaces__srv__CalibrateGyro_Request *) req;
  edurob_interfaces__srv__CalibrateGyro_Response * res_in = (edurob_interfaces__srv__CalibrateGyro_Response *) res;

  // Param
  uint8_t time = (uint8_t) req_in->time;

  ESP_LOGI(TAG, "CalibrateGyro started: Time %hu", time);

  // Get IMU
  IMUHAL& imu = IMUHAL::getInstance();

  // Start a calibration
  imu.StartCalibration();

  // Give calibration some time
  vTaskDelay(pdMS_TO_TICKS(time*1000));

  // Stop calibration
  imu.StopCalibration();

  // Wait for calibration to be done
  while (imu.isCalibrating()) {
    // Give calibration some time
    vTaskDelay(pdMS_TO_TICKS(20));
  }

  // Get offsets
  float offsets[3];
  imu.getGyroOffsets(offsets);

  res_in->offsets.x = offsets[0];
  res_in->offsets.y = offsets[1];
  res_in->offsets.z = offsets[2];
}

// --- Gripper ---

// Points a ROS string at a static C string (only read during serialization)
static void SetRosString(rosidl_runtime_c__String &Str, const char *Text)
{
  Str.data = const_cast<char *>(Text);
  Str.size = strlen(Text);
  Str.capacity = Str.size + 1;
}

static void GetGripperSpeedForce(double &Speed, double &Force)
{
  Speed = EduRobV2::Servo::DefaultSpeedPct;
  Force = EduRobV2::Servo::DefaultForcePct;
  rclc_parameter_get_double(&param_server, "gripper_speed_pct", &Speed);
  rclc_parameter_get_double(&param_server, "gripper_force_pct", &Force);
}

static void SetTriggerResult(void *res, bool Ok, const char *Started)
{
  std_srvs__srv__Trigger_Response *Res = (std_srvs__srv__Trigger_Response *) res;
  Res->success = Ok;
  SetRosString(Res->message, Ok ? Started : "failed, servo not ready (see serial log)");
}

static void GripCallback(const void *, void *res)
{
  double Speed, Force;
  GetGripperSpeedForce(Speed, Force);
  SetTriggerResult(res, EduRobV2::Servo::Grip(Speed, Force), "grip started");
}

static void ReleaseCallback(const void *, void *res)
{
  double Speed, Force;
  GetGripperSpeedForce(Speed, Force);
  SetTriggerResult(res, EduRobV2::Servo::Release(Speed, Force), "release started");
}

static void SweepCallback(const void *, void *res)
{
  double Speed, Force;
  int64_t Count = EduRobV2::Servo::DefaultSweepCount;
  GetGripperSpeedForce(Speed, Force);
  rclc_parameter_get_int(&param_server, "gripper_sweep_count", &Count);
  SetTriggerResult(res, EduRobV2::Servo::Sweep(Count, Speed, Force), "sweep started");
}

static void TorqueCallback(const void *req, void *res)
{
  const std_srvs__srv__SetBool_Request *Req = (const std_srvs__srv__SetBool_Request *) req;
  std_srvs__srv__SetBool_Response *Res = (std_srvs__srv__SetBool_Response *) res;

  Res->success = EduRobV2::Servo::SetTorque(Req->data);
  SetRosString(Res->message, !Res->success ? "failed, servo not ready (see serial log)" : Req->data ? "torque on" : "torque off");
}

// Position in rad, velocity in rad/s, effort in mA (motor current, proportional to grip force)
static bool setGripperData()
{
  static rosidl_runtime_c__String Name;
  static double Position, Velocity, Effort;

  EduRobV2::Servo::State S;
  if (!EduRobV2::Servo::GetState(S)) {
    return false;
  }

  Position = S.PositionDeg * std::numbers::pi / 180.0;
  Velocity = S.VelocityRpm * 2.0 * std::numbers::pi / 60.0;
  Effort = S.CurrentmA;

  const int64_t Ms = rmw_uros_epoch_millis();
  GripperState.header.stamp.sec = Ms / 1000;
  GripperState.header.stamp.nanosec = (Ms % 1000) * 1000000;
  SetRosString(GripperState.header.frame_id, "");

  SetRosString(Name, "gripper");
  GripperState.name.data = &Name;
  GripperState.name.size = GripperState.name.capacity = 1;
  GripperState.position.data = &Position;
  GripperState.position.size = GripperState.position.capacity = 1;
  GripperState.velocity.data = &Velocity;
  GripperState.velocity.size = GripperState.velocity.capacity = 1;
  GripperState.effort.data = &Effort;
  GripperState.effort.size = GripperState.effort.capacity = 1;

  return true;
}

static void micro_ros_task(void *arg)
{
  uint8_t LoopCount = 0;

  // Get EduRob's parameter store
  Params = &EduRobV2::Parameters::ParameterStore::getInstance();

  allocator = rcl_get_default_allocator();

  // Create init_options.
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));

  // Set the ROS Domain ID to use
  RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DomainID));

  // Iinit the middleware
  rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
  // Use static agent IP and port.
  RCCHECK(rmw_uros_options_set_udp_address(ROS_IP.c_str(), ROS_Port.c_str(), rmw_options));

  // Setup support structure.
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // create node
  node = rcl_get_zero_initialized_node();
  RCCHECK(rclc_node_init_default(&node, "Edurob", "", &support));
  RCCHECK(rclc_publisher_init_default(
    &publisherTf,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "/tf"));

  // create tf publisher
  // Set publisher QoS
  rmw_qos_profile_t rmw_qos_profile_tfstatic = {RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10U, RMW_QOS_POLICY_RELIABILITY_RELIABLE, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL, {0ULL, 0ULL}, {0ULL, 0ULL}, RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT, {0ULL, 0ULL}, false};

  RCCHECK(rclc_publisher_init(
    &publisherTfStatic,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "/tf_static",
    &rmw_qos_profile_tfstatic));

  RCCHECK(rclc_publisher_init_default(
    &publisherIMU,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu"));

  RCCHECK(rclc_publisher_init_default(
    &publisherGripper,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/gripper/state"));

  // create CMD_VEL Subscriber
  RCCHECK(rclc_subscription_init_default(
    &twistSubscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // Initialize parameter server options
  const rclc_parameter_options_t options = {
    .notify_changed_over_dds = true,      // Notify changes over DDS
    .max_params = 17,                     // Maximum number of parameters (12 wheel + 5 gripper)
    .allow_undeclared_parameters = false, // Allow undeclared parameters
    .low_mem_mode = false                 // Disable low memory mode
  };

  // Initialize parameter server with options
  rclc_parameter_server_init_with_option(&param_server, &node, &options);

  // create executor
  // Handles: cmd_vel, calibrategyro, 4 gripper services
  RCCHECK(rclc_executor_init(&executor, &support.context, RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 6, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &twistSubscriber, &twistMessage, &subscription_callback, ON_NEW_DATA));

  // Add parameter server to executor
  RCCHECK(rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed));

  // Add parameters to the server
  rclc_add_parameter(&param_server, "wheel_radius_0", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "wheel_radius_1", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "wheel_radius_2", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "wheel_radius_3", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "wheel_pos_x_0", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "wheel_pos_x_1", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "wheel_pos_x_2", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "wheel_pos_x_3", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "wheel_pos_y_0", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "wheel_pos_y_1", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "wheel_pos_y_2", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "wheel_pos_y_3", RCLC_PARAMETER_DOUBLE);

  // Set current values
  DriveConfig DriveConf = Drive->GetDriveConfig();
  rclc_parameter_set_double(&param_server, "wheel_radius_0", DriveConf.Wheels[0].Radius);
  rclc_parameter_set_double(&param_server, "wheel_radius_1", DriveConf.Wheels[1].Radius);
  rclc_parameter_set_double(&param_server, "wheel_radius_2", DriveConf.Wheels[2].Radius);
  rclc_parameter_set_double(&param_server, "wheel_radius_3", DriveConf.Wheels[3].Radius);
  rclc_parameter_set_double(&param_server, "wheel_pos_x_0", DriveConf.Wheels[0].Position.x);
  rclc_parameter_set_double(&param_server, "wheel_pos_x_1", DriveConf.Wheels[1].Position.x);
  rclc_parameter_set_double(&param_server, "wheel_pos_x_2", DriveConf.Wheels[2].Position.x);
  rclc_parameter_set_double(&param_server, "wheel_pos_x_3", DriveConf.Wheels[3].Position.x);
  rclc_parameter_set_double(&param_server, "wheel_pos_y_0", DriveConf.Wheels[0].Position.y);
  rclc_parameter_set_double(&param_server, "wheel_pos_y_1", DriveConf.Wheels[1].Position.y);
  rclc_parameter_set_double(&param_server, "wheel_pos_y_2", DriveConf.Wheels[2].Position.y);
  rclc_parameter_set_double(&param_server, "wheel_pos_y_3", DriveConf.Wheels[3].Position.y);

  // Gripper parameters, RAM only: firmware defaults after every reboot
  rclc_add_parameter(&param_server, "gripper_open_deg", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "gripper_closed_deg", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "gripper_speed_pct", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "gripper_force_pct", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "gripper_sweep_count", RCLC_PARAMETER_INT);
  rclc_parameter_set_double(&param_server, "gripper_open_deg", EduRobV2::Servo::DefaultOpenDeg);
  rclc_parameter_set_double(&param_server, "gripper_closed_deg", EduRobV2::Servo::DefaultClosedDeg);
  rclc_parameter_set_double(&param_server, "gripper_speed_pct", EduRobV2::Servo::DefaultSpeedPct);
  rclc_parameter_set_double(&param_server, "gripper_force_pct", EduRobV2::Servo::DefaultForcePct);
  rclc_parameter_set_int(&param_server, "gripper_sweep_count", EduRobV2::Servo::DefaultSweepCount);

  // create service
  rcl_service_t service;
  RCCHECK(rclc_service_init_default(&service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(edurob_interfaces, srv, CalibrateGyro), "/calibrategyro"));

  // Add service to executor
  edurob_interfaces__srv__CalibrateGyro_Request req;
  edurob_interfaces__srv__CalibrateGyro_Response res;
  RCCHECK(rclc_executor_add_service(&executor, &service, &req, &res, CalibrateGyro_service_callback));

  // Gripper services: non-blocking, speed/force/count come from the gripper_* parameters
  RCCHECK(rclc_service_init_default(&GripService, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "/gripper/grip"));
  RCCHECK(rclc_service_init_default(&ReleaseService, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "/gripper/release"));
  RCCHECK(rclc_service_init_default(&SweepService, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "/gripper/sweep"));
  RCCHECK(rclc_service_init_default(&TorqueService, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool), "/gripper/torque"));
  RCCHECK(rclc_executor_add_service(&executor, &GripService, &GripReq, &GripRes, GripCallback));
  RCCHECK(rclc_executor_add_service(&executor, &ReleaseService, &ReleaseReq, &ReleaseRes, ReleaseCallback));
  RCCHECK(rclc_executor_add_service(&executor, &SweepService, &SweepReq, &SweepRes, SweepCallback));
  RCCHECK(rclc_executor_add_service(&executor, &TorqueService, &TorqueReq, &TorqueRes, TorqueCallback));

  // Sync time from agent
  rmw_uros_sync_session(100);

  // Same the last time the loop has run to get periodic execution
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Spin forever
  while (1) {
    // Wait for the next execution period on a tight schedule and log
    // if the last run exceeded the period
    if (!xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(200))) {
      ESP_LOGE(TAG, "Execution time exceeded!");
    }

    // Sync time every 256th loop
    if (++LoopCount == 0) {
      // Sync time from agent
      rmw_uros_sync_session(100);
    }

    if (rmw_uros_epoch_synchronized()) {
      setTfData();
      setImuData();
      RCSOFTCHECK(rcl_publish(&publisherTf, &messageTf, NULL));
      RCSOFTCHECK(rcl_publish(&publisherIMU, &IMUData, NULL));

      if (setGripperData()) {
        RCSOFTCHECK(rcl_publish(&publisherGripper, &GripperState, NULL));
      }
    }

    // Only execute callbacks every other loop
    if (LoopCount % 2) {
      setTfStaticData();
      RCSOFTCHECK(rcl_publish(&publisherTfStatic, &messageTfStatic, NULL));
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
  }
}

void micro_ros_init(std::string IP, std::string Port, size_t DomainID, DriveHAL &DHAL)
{
  // Save paramams for later use
  ROS_IP = IP;
  ROS_Port = Port;
  ROS_DomainID = DomainID;
  Drive = &DHAL;

  // Init networking
  //ESP_ERROR_CHECK(uros_network_interface_initialize());

  // Start ROS task and pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
  xTaskCreate(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}
