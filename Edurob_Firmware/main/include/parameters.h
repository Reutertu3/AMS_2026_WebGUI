#pragma once

#ifndef EDUROB_V2_PARAMETERS_H
#define EDUROB_V2_PARAMETERS_H

#include "nvs_flash.h"
#include "DriveHAL.h"

namespace EduRobV2 {
  namespace Parameters {

    class ParameterStore {
      public:
        ~ParameterStore();

        // Singleton of this class
        ParameterStore(const ParameterStore &) = delete;
        ParameterStore &operator=(const ParameterStore &) = delete;
        static ParameterStore &getInstance()
        {
          static ParameterStore instance;
          return instance;
        }

        // Get and Set parameters
        HAL::DriveSystem GetDriveSystem();
        esp_err_t SetDriveSystem(HAL::DriveSystem);
        float GetWheelRadius(uint8_t WheelNr);
        esp_err_t SetWheelRadius(uint8_t WheelNr, float Radius);
        float GetWheelPosX(uint8_t WheelNr);
        float GetWheelPosY(uint8_t WheelNr);
        esp_err_t SetWheelPosX(uint8_t WheelNr, float Pos);
        esp_err_t SetWheelPosY(uint8_t WheelNr, float Pos);
        void PrintAll();
        uint32_t GetMicroROSAgentIP();
        esp_err_t SetMicroROSAgentIP(uint32_t IP);
        uint16_t GetMicroROSAgentPort();
        esp_err_t SetMicroROSAgentPort(uint16_t Port);
        uint8_t GetROSDomainID();
        esp_err_t SetROSDomainID(uint8_t ID);

      private:
        ParameterStore();
        nvs_handle_t flash_handle;
        HAL::DriveSystem DriveType;
        float WheelRadii[4];
        float WheelPos[4][2];
        uint32_t MicroROSAgentIP;
        uint16_t MicroROSAgentPort;
        uint8_t ROSDomainID;
    };

  } // end namespace Parameters
} // end namespace EduRobV2

#endif /* EDUROB_V2_PARAMETERS_H */
