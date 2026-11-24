#include "parameters.h"
// Flash Memory
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_netif.h"

using namespace EduRobV2::Parameters;

#define TAG "ParameterStore"

ParameterStore::ParameterStore()
{
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }

  ret = nvs_open("storage", NVS_READWRITE, &flash_handle);
  ESP_LOGD(TAG, "NVS Opened!");
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(ret));

    nvs_flash_deinit();
    return;
  }

  // Read the EdruobData from the NVS
  // Drive type
  uint8_t DriveType;
  ret = nvs_get_u8(flash_handle, "DriveType", &DriveType);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) reading DriveType! Setting default Mecnaum", esp_err_to_name(ret));

    DriveType = (uint8_t) EduRobV2::HAL::DriveSystem::Mecanum;
    ret = nvs_set_u8(flash_handle, "DriveType", DriveType);

    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Error (%s) setting default DriveType!", esp_err_to_name(ret));
    }
  }
  this->DriveType = (EduRobV2::HAL::DriveSystem) DriveType;

  // Four wheel radii in .1 mm
  for (int i = 0; i < 4; i++) {
    uint16_t Radius;
    ret = nvs_get_u16(flash_handle, std::format("WheelRadius{}", i).c_str(), &Radius);

    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Error (%s) reading WheelRadius for wheel %d! Setting default 0.023 m", esp_err_to_name(ret), i);

      // .1 mm
      Radius = 230;
      ret = nvs_set_u16(flash_handle, std::format("WheelRadius{}", i).c_str(), Radius);

      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) setting default WheelRadius for wheel %d!", esp_err_to_name(ret), i);
      }
    }
    this->WheelRadii[i] = Radius / float(10000.0);
  }

  // Four wheel positions in .1 mm
  for (int i = 0; i < 4; i++) {
    for (char j = 'x'; j < 'z'; j++) {
      int16_t Pos;
      ret = nvs_get_i16(flash_handle, std::format("WheelPos{}{}", i, j).c_str(), &Pos);

      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) reading WheelPos %c for wheel %d! Setting default", esp_err_to_name(ret), j, i);

        // .1 mm
        Pos = (j-'x' == 0)?(i == 0 || i == 1)?1000:-1000:(i == 0 || i == 3)?-865:865;
        ret = nvs_set_i16(flash_handle, std::format("WheelPos{}{}", i, j).c_str(), Pos);

        if (ret != ESP_OK) {
          ESP_LOGE(TAG, "Error (%s) setting default WheelPos %c for wheel %d!", esp_err_to_name(ret), j, i);
        }
      }
      this->WheelPos[i][j-'x'] = Pos / float(10000.0);
    }
  }

  // IPv4 IP of the MicroROS agent
  uint32_t AgentIP;
  ret = nvs_get_u32(flash_handle, "MicroROSAgentIP", &AgentIP);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) reading MicroROSAgentIP! Setting default from sdkconfig", esp_err_to_name(ret));

    AgentIP = esp_ip4addr_aton(CONFIG_MICRO_ROS_AGENT_IP);
    ret = nvs_set_u32(flash_handle, "MicroROSAgentIP", AgentIP);

    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Error (%s) setting default MicroROSAgentIP!", esp_err_to_name(ret));
    }
  }
  this->MicroROSAgentIP = AgentIP;

  // TCP port of the MicroROS agent
  uint16_t AgentPort;
  ret = nvs_get_u16(flash_handle, "MicroROSAgentPt", &AgentPort);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) reading MicroROSAgentPt! Setting default from sdkconfig", esp_err_to_name(ret));

    AgentPort = atoi(CONFIG_MICRO_ROS_AGENT_PORT);
    ret = nvs_set_u16(flash_handle, "MicroROSAgentPt", AgentPort);

    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Error (%s) setting default MicroROSAgentPt!", esp_err_to_name(ret));
    }
  }
  this->MicroROSAgentPort = AgentPort;

  // ROS DomainID
  uint8_t DomainID;
  ret = nvs_get_u8(flash_handle, "ROSDomainID", &DomainID);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) reading ROSDomainID! Setting default from sdkconfig", esp_err_to_name(ret));

    DomainID = CONFIG_MICRO_ROS_DOMAIN_ID;
    ret = nvs_set_u8(flash_handle, "ROSDomainID", DomainID);

    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Error (%s) setting default ROSDomainID!", esp_err_to_name(ret));
    }
  }
  this->ROSDomainID = DomainID;
}

ParameterStore::~ParameterStore()
{
  nvs_close(flash_handle);

  nvs_flash_deinit();
}

EduRobV2::HAL::DriveSystem ParameterStore::GetDriveSystem()
{
  return DriveType;
}

esp_err_t ParameterStore::SetDriveSystem(HAL::DriveSystem drive)
{
  this->DriveType = drive;

  uint8_t DriveType = (uint8_t) drive;

  esp_err_t ret = nvs_set_u8(flash_handle, "DriveType", DriveType);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) setting DriveType!", esp_err_to_name(ret));

    return ret;
  }

  return ESP_OK;
}

float ParameterStore::GetWheelRadius(uint8_t WheelNr)
{
  if (WheelNr >= 4) {
    ESP_LOGE(TAG, "GetWheelRadius(%hu): EduRob has maximum for four wheels", WheelNr);

    return NAN;
  }

  return WheelRadii[WheelNr];
}

esp_err_t ParameterStore::SetWheelRadius(uint8_t WheelNr, float NewRadius)
{
  if (WheelNr >= 4) {
    ESP_LOGE(TAG, "SetWheelRadius(%hu): EduRob has maximum for four wheels", WheelNr);

    return ESP_ERR_INVALID_ARG;
  }

  // .1 mm
  uint16_t Radius = NewRadius * 10000;
  esp_err_t ret = nvs_set_u16(flash_handle, std::format("WheelRadius{}", WheelNr).c_str(), Radius);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) setting WheelRadius for wheel %hu!", esp_err_to_name(ret), WheelNr);

    return ret;
  }

  WheelRadii[WheelNr] = NewRadius;

  return ESP_OK;
}

float ParameterStore::GetWheelPosX(uint8_t WheelNr)
{
  if (WheelNr >= 4) {
    ESP_LOGE(TAG, "GetWheelPosX(%hu): EduRob has maximum for four wheels", WheelNr);

    return NAN;
  }

  return WheelPos[WheelNr][0];
}

float ParameterStore::GetWheelPosY(uint8_t WheelNr)
{
  if (WheelNr >= 4) {
    ESP_LOGE(TAG, "GetWheelPosY(%hu): EduRob has maximum for four wheels", WheelNr);

    return NAN;
  }

  return WheelPos[WheelNr][1];
}

esp_err_t ParameterStore::SetWheelPosX(uint8_t WheelNr, float NewPos)
{
  if (WheelNr >= 4) {
    ESP_LOGE(TAG, "SetWheelPosX(%hu): EduRob has maximum for four wheels", WheelNr);

    return ESP_ERR_INVALID_ARG;
  }

  // .1 mm
  int16_t Pos = NewPos * 10000;
  esp_err_t ret = nvs_set_i16(flash_handle, std::format("WheelPos{}x", WheelNr).c_str(), Pos);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) setting WheelPosX for wheel %hu!", esp_err_to_name(ret), WheelNr);

    return ret;
  }

  WheelPos[WheelNr][0] = NewPos;

  return ESP_OK;
}

esp_err_t ParameterStore::SetWheelPosY(uint8_t WheelNr, float NewPos)
{
  if (WheelNr >= 4) {
    ESP_LOGE(TAG, "SetWheelPosY(%hu): EduRob has maximum for four wheels", WheelNr);

    return ESP_ERR_INVALID_ARG;
  }

  // .1 mm
  int16_t Pos = NewPos * 10000;
  esp_err_t ret = nvs_set_i16(flash_handle, std::format("WheelPos{}y", WheelNr).c_str(), Pos);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) setting WheelPosY for wheel %hu!", esp_err_to_name(ret), WheelNr);

    return ret;
  }

  WheelPos[WheelNr][1] = NewPos;

  return ESP_OK;
}

void ParameterStore::PrintAll()
{
  nvs_iterator_t it = NULL;
  esp_err_t res = nvs_entry_find_in_handle(flash_handle, NVS_TYPE_ANY, &it);
  while (res == ESP_OK) {
    nvs_entry_info_t info;
    nvs_entry_info(it, &info); // Can omit error check if parameters are guaranteed to be non-NULL

    switch (info.type) {
      case NVS_TYPE_U8: {
        uint8_t data;
        nvs_get_u8(flash_handle, info.key, &data);
        ESP_LOGI(TAG, "key '%s', type '%d', value '%hhu'", info.key, info.type, data);
        break;
      }
      case NVS_TYPE_U16: {
        uint16_t data;
        nvs_get_u16(flash_handle, info.key, &data);
        ESP_LOGI(TAG, "key '%s', type '%d', value '%hu'", info.key, info.type, data);
        break;
      }
      case NVS_TYPE_I16: {
        int16_t data;
        nvs_get_i16(flash_handle, info.key, &data);
        ESP_LOGI(TAG, "key '%s', type '%d', value '%hd'", info.key, info.type, data);
        break;
      }
      case NVS_TYPE_U32: {
        uint32_t data;
        nvs_get_u32(flash_handle, info.key, &data);
        ESP_LOGI(TAG, "key '%s', type '%d', value '%u'", info.key, info.type, data);
        break;
      }
      default:
        ESP_LOGI(TAG, "key '%s', type '%d'", info.key, info.type);
        break;
    }

    res = nvs_entry_next(&it);
  }
  nvs_release_iterator(it);
}

uint32_t ParameterStore::GetMicroROSAgentIP()
{
  return MicroROSAgentIP;
}

esp_err_t ParameterStore::SetMicroROSAgentIP(uint32_t IP)
{
  esp_err_t ret = nvs_set_u32(flash_handle, "MicroROSAgentIP", IP);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) setting default MicroROSAgentIP!", esp_err_to_name(ret));

    return ret;
  }

  MicroROSAgentIP = IP;

  return ESP_OK;
}

uint16_t ParameterStore::GetMicroROSAgentPort()
{
  return MicroROSAgentPort;
}

esp_err_t ParameterStore::SetMicroROSAgentPort(uint16_t Port)
{
  esp_err_t ret = nvs_set_u16(flash_handle, "MicroROSAgentPt", Port);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) setting default MicroROSAgentPt!", esp_err_to_name(ret));

    return ret;
  }

  MicroROSAgentPort = Port;

  return ESP_OK;
}

uint8_t ParameterStore::GetROSDomainID()
{
  return ROSDomainID;
}

esp_err_t ParameterStore::SetROSDomainID(uint8_t ID)
{
  esp_err_t ret = nvs_set_u8(flash_handle, "ROSDomainID", ID);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) setting ROSDomainID!", esp_err_to_name(ret));

    return ret;
  }

  ROSDomainID = ID;

  return ESP_OK;
}
