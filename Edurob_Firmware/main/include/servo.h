#pragma once

#ifndef EDUROB_V2_SERVO_H
#define EDUROB_V2_SERVO_H

#include <cstdint>

// Gripper on a single Dynamixel XL330 (protocol 2.0, UART2, pins from menuconfig).
// Runs in current-based position mode: the servo moves to a goal position but never
// exceeds the force limit (goal current), so closing on an object stalls with limited force.
// Force is not measured directly, motor current is proportional to torque.
//
// Grip/Release/Sweep are non-blocking: they start a job that a background task runs.
// A new command (or SetTorque) cancels the running job.
//
// Speed and force are in percent of the servo's own Velocity Limit / Current Limit
// (read from the servo at Init), so they can never exceed what the servo allows.
namespace EduRobV2 {
  namespace Servo {
    // Firmware defaults (also the initial ROS parameter values)
    constexpr float DefaultOpenDeg   = 180.0f; // fully open
    constexpr float DefaultClosedDeg = 92.0f;  // fully closed, no object
    constexpr float DefaultSpeedPct  = 20.0f;
    constexpr float DefaultForcePct  = 17.0f;  // ~300 mA with the XL330's 1750 mA limit
    constexpr int   DefaultSweepCount = 3;

    struct State {
      float PositionDeg;
      float VelocityRpm;
      float CurrentmA;   // sign depends on direction
      bool  Gripping;    // pushing against something: current near the force limit while not moving
      bool  Busy;        // a Grip/Release/Sweep job is running
    };

    // Finds the servo (scans baud rates and IDs), sets current-based position mode,
    // holds the present position, enables torque and starts the job task.
    // Takes up to ~20 s if no servo answers.
    bool Init();

    // Close to the closed end stop. Stops on an object and holds it with the force limit.
    bool Grip(float SpeedPct, float ForcePct);

    // Move to the open end stop.
    bool Release(float SpeedPct, float ForcePct);

    // Count cycles of open -> closed. Aborts on a stall (object/jam) or timeout and stops pushing.
    bool Sweep(int Count, float SpeedPct, float ForcePct);

    // Torque off: servo goes limp and can be moved by hand.
    // Torque on: holds the present position (no jump back to an old goal).
    // Both cancel a running job.
    bool SetTorque(bool On);

    // Blocks until the running Grip/Release/Sweep job is finished (or aborted).
    // Use it to chain commands, otherwise the next command cancels the running one.
    // Returns false on timeout or if not ready.
    bool WaitIdle(uint32_t TimeoutMs);

    // End stops in degrees, RAM only (firmware defaults after reboot). Callable before Init.
    void SetOpenDeg(float Degree);
    void SetClosedDeg(float Degree);

    // Reads position, velocity and current from the servo. False if not ready or read failed.
    bool GetState(State &S);
  }
}

#endif
