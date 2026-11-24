#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "DriveHAL.h"
#include "SensorHAL.h"
#include "ros.h"
#include "parameters.h"
#include "webserver.h"
#include "mdns.h"
#include "esp_mac.h"

#define TAG "Main"

using namespace EduRobV2::HAL;
using namespace EduRobV2::Parameters;

static esp_netif_t *WifiNetif = NULL;

static esp_err_t WifiConnect(void)
{
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
  esp_netif_config.if_desc = "Network WIFI";
  esp_netif_config.route_prio = 128;
  WifiNetif = esp_netif_create_wifi(WIFI_IF_STA, &esp_netif_config);
  esp_wifi_set_default_wifi_sta_handlers();

  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());

  wifi_config_t wifi_config = {
    .sta = {
      .ssid = CONFIG_ESP_WIFI_SSID,
      .password = CONFIG_ESP_WIFI_PASSWORD,
      .scan_method = WIFI_ALL_CHANNEL_SCAN,
      .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
      .threshold = {
        .rssi = -127,
        .authmode = WIFI_AUTH_WPA2_PSK,
      },
    },
  };

  ESP_LOGI(TAG, "Connecting to %s...", wifi_config.sta.ssid);
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  esp_err_t ret = esp_wifi_connect();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "WiFi connect failed! ret:%x", ret);
    return ret;
  }

  // initialize mDNS
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);

  // hostname for mDNS
  char hostname[32];
  // generate hostname "edurob-<6 character of mac address>"
  snprintf(hostname, sizeof(hostname), "edurob-%02x%02x%02x", mac[3], mac[4], mac[5]);

  ESP_ERROR_CHECK(mdns_init());
  ESP_ERROR_CHECK(mdns_hostname_set(hostname));
  ESP_ERROR_CHECK(mdns_instance_name_set("EduRob mDNS instance"));
  ESP_LOGI(TAG, "mDNS started. Edurob is reachable under: %s.local\n", hostname);

  return ESP_OK;
}

extern "C" void app_main(void)
{
  Pose OdoPose;
  ParameterStore& Params = ParameterStore::getInstance();

  // Init network and event loop
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  ESP_ERROR_CHECK(WifiConnect());

  // Start TCPLaser on port 44227
  TCPLaser Laser(44227);

  IMUHAL& imu = IMUHAL::getInstance();

  // Start webserver
  ESP_ERROR_CHECK(SetupWebserver());

  // Wait for calibration to be done
  while (imu.isCalibrating()) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // Setup EduRob V2 motors
  DriveConfig DriveCfg = {Params.GetDriveSystem(), {
    {{GPIO_NUM_14, GPIO_NUM_18}, { GPIO_NUM_4,  GPIO_NUM_5}, false, 28, 100},
    {{GPIO_NUM_47, GPIO_NUM_39}, { GPIO_NUM_2,  GPIO_NUM_1},  true, 28, 100},
    {{GPIO_NUM_38, GPIO_NUM_48}, {GPIO_NUM_41, GPIO_NUM_42}, false, 28, 100},
    {{GPIO_NUM_17, GPIO_NUM_21}, {GPIO_NUM_7, GPIO_NUM_6}, false, 28, 100},
    }, {
    {Params.GetWheelRadius(0), Params.GetWheelPosX(0), Params.GetWheelPosY(0), 100, 28},
    {Params.GetWheelRadius(1), Params.GetWheelPosX(1), Params.GetWheelPosY(1), 100, 28},
    {Params.GetWheelRadius(2), Params.GetWheelPosX(2), Params.GetWheelPosY(2), 100, 28},
    {Params.GetWheelRadius(3), Params.GetWheelPosX(3), Params.GetWheelPosY(3), 100, 28}}};

  DriveHAL Drive(DriveCfg);

  esp_ip4_addr AgentIP = {Params.GetMicroROSAgentIP()};
  char IPBuffer[16];
  micro_ros_init(std::string(esp_ip4addr_ntoa(&AgentIP, IPBuffer, sizeof(IPBuffer))), std::to_string(Params.GetMicroROSAgentPort()), Params.GetROSDomainID(), Drive);

  //Drive->SetMode(PoseControlMode::Manual);
  //Drive->SetRobotVel({0.1, 0.0, 0.0});

  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true) {
    xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    Drive.GetGyroOdomPose(OdoPose);
    printf("OdoPose: %f, %f, %f, %f\n", OdoPose.x, OdoPose.y, float(OdoPose.theta), imu.getGyroTheta());
  }
}
