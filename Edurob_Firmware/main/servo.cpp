#include <atomic>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "servo.h"
#include "dynamixel2espressif.h"
#include "esp_log.h"

#define TAG "Servo"

#define DXL_UART UART_NUM_2 // UART1 is used by TCPLaser

// Calibration knobs, tune on the real gripper
#define GRIP_DETECT_RATIO       0.7f   // stalled when |current| >= ratio * force limit ...
#define GRIP_STALL_RPM          1.0f   // ... and |velocity| below this
#define MOVE_TOLERANCE_DEG      2.0f   // end stop counts as reached within this
#define MOVE_STALL_GRACE_MS     200    // ignore the current spike when a move starts
#define MOVE_TIMEOUT_MARGIN_MS  1000   // timeout = 2x expected travel time + margin
#define POLL_MS                 50     // job task period; each register read is ~5 ms at 57600 baud

// Fallbacks if the limits can not be read from the servo (XL330 factory values)
#define VELOCITY_LIMIT_RAW_DEFAULT 445    // x 0.229 rpm
#define CURRENT_LIMIT_mA_DEFAULT   1750
#define VELOCITY_UNIT_RPM          0.229f

// The library toggles the DIR pin on every transfer (ESP_ERROR_CHECK aborts on -1)
static_assert(CONFIG_DXL_DIR_PIN >= 0, "Set CONFIG_DXL_DIR_PIN to the DXL_TXEN GPIO in menuconfig");

enum class Job { Idle, Grip, Release, Sweep };

static Dynamixel2Espressif *Dxl = nullptr;
static SemaphoreHandle_t Lock = nullptr; // guards the bus and everything below
static std::atomic<bool> Ready{false};
static int ServoID = -1;
static int32_t VelocityLimitRaw = VELOCITY_LIMIT_RAW_DEFAULT;
static float CurrentLimitmA = CURRENT_LIMIT_mA_DEFAULT;
static bool TorqueOn = false;

// End stops, written by the ROS parameter callback possibly before Init
static std::atomic<float> OpenDeg{EduRobV2::Servo::DefaultOpenDeg};
static std::atomic<float> ClosedDeg{EduRobV2::Servo::DefaultClosedDeg};

// Running job
static Job CurJob = Job::Idle;
static bool Closing = false;     // current move goes to the closed end stop
static int SweepLeft = 0;
static float Goal = 0.0f;
static float ForcemA = 0.0f;     // force limit of the running job, for stall detection
static float SpeedPct = EduRobV2::Servo::DefaultSpeedPct;
static TickType_t MoveStart = 0;
static TickType_t MoveTimeout = 0;

// Scans all XL330 baud rates and IDs, keeps the first servo found
static bool FindServo()
{
  // ponytail: 9600 skipped, a status packet there outlasts the library's 10 ms ping timeout
  const uint32_t Bauds[] = {57600, 1000000, 115200, 2000000, 3000000, 4000000};

  for (uint32_t Baud : Bauds) {
    ESP_ERROR_CHECK(uart_set_baudrate(DXL_UART, Baud));
    ESP_LOGI(TAG, "Scanning IDs 0-252 at %lu baud", Baud);

    for (uint8_t id = 0; id < DXL_BROADCAST_ID - 1; id++) {
      if (Dxl->ping(id)) {
        ServoID = id;
        ESP_LOGI(TAG, "Found ID %u at %lu baud, model number %u", id, Baud, Dxl->getModelNumber(id));
        return true;
      }
    }
  }

  ESP_LOGE(TAG, "No servo answered at any baud rate (check 5V power, TXD/RXD wiring, DXL_TXEN)");
  return false;
}

// --- Helpers below are called with Lock held ---

static float ReadPosition()
{
  return Dxl->getPresentPosition(ServoID, UNIT_DEGREE);
}

static bool Stalled()
{
  return std::fabs(Dxl->getPresentCurrent(ServoID, UNIT_MILLI_AMPERE)) >= GRIP_DETECT_RATIO * ForcemA
      && std::fabs(Dxl->getPresentVelocity(ServoID, UNIT_RPM)) < GRIP_STALL_RPM;
}

static float ClampPct(float Pct)
{
  return std::fmin(std::fmax(Pct, 1.0f), 100.0f);
}

// Speed -> profile velocity, force -> goal current. Both are RAM items, writable with torque on.
static bool ApplySpeedForce(float Speed, float Force)
{
  SpeedPct = ClampPct(Speed);
  ForcemA = ClampPct(Force) / 100.0f * CurrentLimitmA;

  // Profile velocity 0 means "unlimited", so never go below 1
  int32_t ProfileRaw = std::lround(SpeedPct / 100.0f * VelocityLimitRaw);
  if (ProfileRaw < 1) {
    ProfileRaw = 1;
  }

  // setProfileVelocity() is #if 0'd out in the library, so write the register directly
  return Dxl->writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, ServoID, ProfileRaw)
      && Dxl->setGoalCurrent(ServoID, ForcemA, UNIT_MILLI_AMPERE);
}

static bool EnableTorque()
{
  // Goal = present position, so enabling torque does not move the gripper
  TorqueOn = Dxl->setGoalPosition(ServoID, ReadPosition(), UNIT_DEGREE) && Dxl->torqueOn(ServoID);
  return TorqueOn;
}

static bool StartMove(bool ToClosed)
{
  const float Open = OpenDeg, Closed = ClosedDeg;
  const float Present = ReadPosition();

  Closing = ToClosed;
  Goal = ToClosed ? Closed : Open;

  // Timeout from the expected travel time at the requested speed
  const float DegPerS = SpeedPct / 100.0f * VelocityLimitRaw * VELOCITY_UNIT_RPM * 6.0f;
  const float TravelMs = std::fabs(Goal - Present) / DegPerS * 1000.0f;
  MoveTimeout = pdMS_TO_TICKS(MOVE_TIMEOUT_MARGIN_MS + 2.0f * TravelMs);
  MoveStart = xTaskGetTickCount();

  return Dxl->setGoalPosition(ServoID, Goal, UNIT_DEGREE);
}

// Stop pushing: hold wherever the jaws are now
static void Hold()
{
  Dxl->setGoalPosition(ServoID, ReadPosition(), UNIT_DEGREE);
  CurJob = Job::Idle;
}

static bool StartJob(Job NewJob, int Count, float Speed, float Force)
{
  if (!Ready) {
    ESP_LOGE(TAG, "Not ready, Init() failed or not called yet");
    return false;
  }

  xSemaphoreTake(Lock, portMAX_DELAY);

  CurJob = Job::Idle; // cancel whatever runs
  bool ok = ApplySpeedForce(Speed, Force) && (TorqueOn || EnableTorque());

  if (ok) {
    SweepLeft = Count;
    // Grip and Sweep end closed, so a sweep starts by opening
    ok = StartMove(NewJob == Job::Grip);
  }

  if (ok) {
    CurJob = NewJob;
    ESP_LOGI(TAG, "Start %s to %.1f deg, speed %.0f %%, force %.0f mA",
             NewJob == Job::Grip ? "grip" : NewJob == Job::Release ? "release" : "sweep", Goal, SpeedPct, ForcemA);
  } else {
    ESP_LOGE(TAG, "Could not start job");
  }

  xSemaphoreGive(Lock);
  return ok;
}

// Background task: watches the running move, chains sweep moves, detects stalls
static void JobTask(void *)
{
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    xSemaphoreTake(Lock, portMAX_DELAY);

    if (CurJob != Job::Idle) {
      const float Present = ReadPosition();
      const TickType_t Elapsed = xTaskGetTickCount() - MoveStart;

      if (std::fabs(Present - Goal) <= MOVE_TOLERANCE_DEG) {
        if (CurJob == Job::Grip) {
          ESP_LOGW(TAG, "Grip: closed at %.1f deg without an object", Present);
          CurJob = Job::Idle;
        } else if (CurJob == Job::Release) {
          ESP_LOGI(TAG, "Released at %.1f deg", Present);
          CurJob = Job::Idle;
        } else if (!Closing) {
          StartMove(true);
        } else if (--SweepLeft > 0) {
          ESP_LOGI(TAG, "Sweep: %d cycles left", SweepLeft);
          StartMove(false);
        } else {
          ESP_LOGI(TAG, "Sweep done");
          CurJob = Job::Idle;
        }
      } else if (Elapsed >= pdMS_TO_TICKS(MOVE_STALL_GRACE_MS) && Stalled()) {
        if (CurJob == Job::Grip) {
          // Keep the goal: current-based position mode now holds the object with the force limit
          ESP_LOGI(TAG, "Grip: object held at %.1f deg", Present);
          CurJob = Job::Idle;
        } else {
          ESP_LOGW(TAG, "Stalled at %.1f deg on the way to %.1f deg, stopped", Present, Goal);
          Hold();
        }
      } else if (Elapsed > MoveTimeout) {
        ESP_LOGW(TAG, "Timeout at %.1f deg on the way to %.1f deg, stopped", Present, Goal);
        Hold();
      }
    }

    xSemaphoreGive(Lock);
  }
}

namespace EduRobV2 {
  namespace Servo {
    bool Init()
    {
      if (Ready) {
        return true;
      }

      // begin() installs the UART driver, which only works once
      if (Dxl == nullptr) {
        Dxl = new Dynamixel2Espressif(DXL_UART, (gpio_num_t)CONFIG_DXL_DIR_PIN);
        Lock = xSemaphoreCreateMutex();
        esp_log_level_set("PORT HANDLER", ESP_LOG_WARN); // library logs every single write
        Dxl->begin(57600);
        Dxl->setPortProtocolVersion(2.0);
      }

      if (ServoID < 0 && !FindServo()) {
        return false;
      }

      // Limits the percentages refer to. readControlTableItem returns 0 on failure.
      if (int32_t v = Dxl->readControlTableItem(ControlTableItem::VELOCITY_LIMIT, ServoID); v > 0) {
        VelocityLimitRaw = v;
      }
      if (int32_t c = Dxl->readControlTableItem(ControlTableItem::CURRENT_LIMIT, ServoID); c > 0) {
        CurrentLimitmA = c; // XL330: 1 mA per unit
      }

      // Operating mode is in EEPROM, which is only writable with torque off
      if (!Dxl->torqueOff(ServoID)
          || !Dxl->setOperatingMode(ServoID, OP_CURRENT_BASED_POSITION)
          || !ApplySpeedForce(DefaultSpeedPct, DefaultForcePct)
          || !EnableTorque()) {
        ESP_LOGE(TAG, "Could not configure ID %d", ServoID);
        return false;
      }

      xTaskCreate(JobTask, "servo", 4096, nullptr, 5, nullptr);
      Ready = true;

      ESP_LOGI(TAG, "ID %d ready, holding %.1f deg (velocity limit %.1f rpm, current limit %.0f mA)",
               ServoID, ReadPosition(), VelocityLimitRaw * VELOCITY_UNIT_RPM, CurrentLimitmA);
      return true;
    }

    bool Grip(float SpeedPct, float ForcePct)
    {
      return StartJob(Job::Grip, 0, SpeedPct, ForcePct);
    }

    bool Release(float SpeedPct, float ForcePct)
    {
      return StartJob(Job::Release, 0, SpeedPct, ForcePct);
    }

    bool Sweep(int Count, float SpeedPct, float ForcePct)
    {
      if (Count < 1) {
        ESP_LOGE(TAG, "Sweep count must be >= 1, got %d", Count);
        return false;
      }

      return StartJob(Job::Sweep, Count, SpeedPct, ForcePct);
    }

    bool SetTorque(bool On)
    {
      if (!Ready) {
        return false;
      }

      xSemaphoreTake(Lock, portMAX_DELAY);
      CurJob = Job::Idle;

      bool ok;
      if (On) {
        ok = EnableTorque();
      } else {
        ok = Dxl->torqueOff(ServoID);
        TorqueOn = !ok;
      }

      xSemaphoreGive(Lock);
      ESP_LOGI(TAG, "Torque %s%s", On ? "on" : "off", ok ? "" : " FAILED");
      return ok;
    }

    bool WaitIdle(uint32_t TimeoutMs)
    {
      if (!Ready) {
        return false;
      }

      const TickType_t Start = xTaskGetTickCount();

      while (true) {
        xSemaphoreTake(Lock, portMAX_DELAY);
        const bool Busy = CurJob != Job::Idle;
        xSemaphoreGive(Lock);

        if (!Busy) {
          return true;
        }
        if (xTaskGetTickCount() - Start >= pdMS_TO_TICKS(TimeoutMs)) {
          ESP_LOGW(TAG, "WaitIdle: still busy after %lu ms", TimeoutMs);
          return false;
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
      }
    }

    void SetOpenDeg(float Degree)
    {
      OpenDeg = Degree;
    }

    void SetClosedDeg(float Degree)
    {
      ClosedDeg = Degree;
    }

    bool GetState(State &S)
    {
      if (!Ready || xSemaphoreTake(Lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
      }

      S.PositionDeg = ReadPosition();
      S.VelocityRpm = Dxl->getPresentVelocity(ServoID, UNIT_RPM);
      S.CurrentmA = Dxl->getPresentCurrent(ServoID, UNIT_MILLI_AMPERE);
      S.Gripping = TorqueOn && std::fabs(S.CurrentmA) >= GRIP_DETECT_RATIO * ForcemA
                && std::fabs(S.VelocityRpm) < GRIP_STALL_RPM;
      S.Busy = CurJob != Job::Idle;

      xSemaphoreGive(Lock);
      return true;
    }
  }
}
