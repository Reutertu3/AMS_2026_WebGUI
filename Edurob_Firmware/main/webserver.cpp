#include "include/webserver.h"
#include "esp_spiffs.h"
#include <sys/stat.h>
#include "matrix_utils.h"
#include "request_utils.cpp"
#include "esp_log.h"
#include <esp_http_server.h>
#include "esp_netif.h"
#include "parameters.h"
#include <string>
#include <regex>

static const char *TAG = "Webserver";

using namespace EduRobV2::Parameters;
static ParameterStore *Params = nullptr;

using namespace std;

/* Prototypes */
static esp_err_t reboot_handler(httpd_req_t *req);
static esp_err_t get_req_handler(httpd_req_t *req);
static esp_err_t config_req_handler(httpd_req_t *req);
static esp_err_t styles_req_handler(httpd_req_t *req);
static esp_err_t setros_handler(httpd_req_t *req);

httpd_uri_t uri_get = {
  .uri = "/",
  .method = HTTP_GET,
  .handler = get_req_handler,
  .user_ctx = NULL};

httpd_uri_t uri_config = {
  .uri = "/configuration",
  .method = HTTP_GET,
  .handler = config_req_handler,
  .user_ctx = NULL};

httpd_uri_t uri_styles = {
  .uri = "/styles.css",
  .method = HTTP_GET,
  .handler = styles_req_handler,
  .user_ctx = NULL};

httpd_uri_t uri_reboot = {
  .uri = "/reboot",
  .method = HTTP_GET,
  .handler = reboot_handler,
  .user_ctx = NULL};

httpd_uri_t uri_setros = {
  .uri = "/set-ros",
  .method = HTTP_GET,
  .handler = setros_handler,
  .user_ctx = NULL};

// HTTP server
static httpd_handle_t server = NULL;

/* HTML data */
#define INDEX_HTML_PATH "/spiffs/index.html"
static char *indexHTML = NULL;
#define CONFIG_HTML_PATH "/spiffs/config.html"
static char *configHTML = NULL;
#define STYLES_CSS_PATH "/spiffs/styles.css"
static char *stylesHTML = NULL;

esp_err_t reboot_handler(httpd_req_t *req)
{
  /* Send an HTTP response to redirect to the main page */
  get_req_handler(req);

  // Log the reboot action
  ESP_LOGI(TAG, "Reboot initiated by user.");

  // Delay a little to ensure the response is sent before rebooting
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // Reboot the ESP32
  esp_restart();
  return ESP_OK;
}

esp_err_t get_req_handler(httpd_req_t *req)
{
  EduRobV2::HAL::DriveSystem Drive = Params->GetDriveSystem();

  string Drivename("Unknown");

  string HTML(indexHTML);

  switch (Drive) {
    case EduRobV2::HAL::DriveSystem::Mecanum:
      Drivename = "Mecanum";
      break;
    case EduRobV2::HAL::DriveSystem::Omni3:
      Drivename = "3 Omni Wheels";
      break;
    case EduRobV2::HAL::DriveSystem::Omni4:
      Drivename = "4 Omni Wheels";
      break;
    case EduRobV2::HAL::DriveSystem::Diff:
      Drivename = "Differential Drive";
      break;
  }

  HTML = std::regex_replace(HTML, std::regex(R"(<KINEMATIC>)"), Drivename);

  // Send the HTML
  return httpd_resp_send(req, HTML.c_str(), HTTPD_RESP_USE_STRLEN);
}

esp_err_t config_req_handler(httpd_req_t *req)
{
  esp_ip4_addr AgentIP = {Params->GetMicroROSAgentIP()};
  char IPBuffer[16];
  esp_ip4addr_ntoa(&AgentIP, IPBuffer, sizeof(IPBuffer));
  uint16_t AgentPort = Params->GetMicroROSAgentPort();
  uint8_t DomainID = Params->GetROSDomainID();

  string HTML(configHTML);

  HTML = std::regex_replace(HTML, std::regex(R"(<AGENTIP>)"), IPBuffer);
  HTML = std::regex_replace(HTML, std::regex(R"(<AGENTPORT>)"), std::to_string(AgentPort));
  HTML = std::regex_replace(HTML, std::regex(R"(<DOMAINID>)"), std::to_string(DomainID));

  return httpd_resp_send(req, HTML.c_str(), HTTPD_RESP_USE_STRLEN);
}

esp_err_t styles_req_handler(httpd_req_t *req)
{
  httpd_resp_set_type(req, "text/css");
  return httpd_resp_send(req, stylesHTML, HTTPD_RESP_USE_STRLEN);
}

esp_err_t setros_handler(httpd_req_t *req)
{
  // Get memory for parsing parameters
  size_t buf_len = httpd_req_get_url_query_len(req)+1;

  // Check if there where parameters at all
  if (buf_len <= 1) {
    ESP_LOGE(TAG, "Set-ROS: No parameters provided");

    return config_req_handler(req);
  }

  char *buf = (char *) malloc(buf_len);
  if (buf == NULL) {
   return ESP_ERR_INVALID_SIZE;
  }

  // Get the query string
  esp_err_t ret = httpd_req_get_url_query_str(req, buf, buf_len);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Set-ROS: Can not get query string");

    free(buf);

    return ret;
  }

  // Get the Parameters
  char AgentIP[16];
  ret = httpd_query_key_value(buf, "AgentIP", AgentIP, sizeof(AgentIP));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Set-ROS: Can not get AgentIP");

    free(buf);

    return ret;
  }

  uint32_t IP = esp_ip4addr_aton(AgentIP);

  // Set IP in params
  ret = Params->SetMicroROSAgentIP(IP);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Set-ROS: Can not set AgentIP");

    free(buf);

    return ret;
  }

  char AgentPort[6];
  ret = httpd_query_key_value(buf, "AgentPort", AgentPort, sizeof(AgentPort));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Set-ROS: Can not get AgentPort");

    free(buf);

    return ret;
  }

  uint16_t Port;
  if ((Port = strtoul(AgentPort, NULL, 10)) == ULONG_MAX) {
    ESP_LOGE(TAG, "Set-ROS: Can not parse AgentPort");

    free(buf);

    return ret;
  }

  // Set Port in params
  ret = Params->SetMicroROSAgentPort(Port);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Set-ROS: Can not set AgentPort");

    free(buf);

    return ret;
  }

  char DomainID[4];
  ret = httpd_query_key_value(buf, "DomainID", DomainID, sizeof(DomainID));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Set-ROS: Can not get DomainID");

    free(buf);

    return ret;
  }

  uint8_t ROSDomainID;
  if ((ROSDomainID = strtoul(DomainID, NULL, 10)) == ULONG_MAX) {
    ESP_LOGE(TAG, "Set-ROS: Can not parse DomainID");

    free(buf);

    return ret;
  }

  // Set Port in params
  ret = Params->SetROSDomainID(ROSDomainID);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Set-ROS: Can not set ROSDomainID");

    free(buf);

    return ret;
  }

  // Free the buffer
  free(buf);

  return config_req_handler(req);
}

esp_err_t SetupWebserver()
{
  // Get parameter store
  Params = &ParameterStore::getInstance();

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 9;

  /* Read the html data */
  esp_vfs_spiffs_conf_t conf = {
    .base_path = "/spiffs",
    .partition_label = NULL,
    .max_files = 5,
    .format_if_mount_failed = true};
  ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

  /* Read index.html from SPIFFS */
  struct stat st;
  if (stat(INDEX_HTML_PATH, &st)) {
    ESP_LOGE(TAG, "index.html not found");

    return ESP_ERR_NOT_FOUND;
  }

  /* Alloc memory */
  indexHTML = (char *) malloc(st.st_size + 1);
  if (indexHTML == NULL) {
    ESP_LOGE(TAG, "Can not allocate memory: %m");

    return ESP_ERR_NOT_FOUND;
  }

  memset((void *) indexHTML, 0, st.st_size + 1);

  FILE *fp = fopen(INDEX_HTML_PATH, "r");
  if (fread(indexHTML, st.st_size, 1, fp) == 0) {
    ESP_LOGE(TAG, "fread failed");
  }
  fclose(fp);

  /* Read config.html from SPIFFS */
  if (stat(CONFIG_HTML_PATH, &st)) {
    ESP_LOGE(TAG, "config.html not found");

    return ESP_ERR_NOT_FOUND;
  }

  /* Alloc memory */
  configHTML = (char *) malloc(st.st_size + 1);
  if (configHTML == NULL) {
    ESP_LOGE(TAG, "Can not allocate memory: %m");

    return ESP_ERR_NOT_FOUND;
  }

  memset((void *) configHTML, 0, st.st_size + 1);

  fp = fopen(CONFIG_HTML_PATH, "r");
  if (fread(configHTML, st.st_size, 1, fp) == 0) {
    ESP_LOGE(TAG, "fread failed");
  }
  fclose(fp);

  /* Read styles.css from SPIFFS */
  if (stat(STYLES_CSS_PATH, &st)) {
    ESP_LOGE(TAG, "styles.css not found");

    return ESP_ERR_NOT_FOUND;
  }

  /* Alloc memory */
  stylesHTML = (char *) malloc(st.st_size + 1);
  if (stylesHTML == NULL) {
    ESP_LOGE(TAG, "Can not allocate memory: %m");

    return ESP_ERR_NOT_FOUND;
  }

  memset((void *) stylesHTML, 0, st.st_size + 1);

  fp = fopen(STYLES_CSS_PATH, "r");
  if (fread(stylesHTML, st.st_size, 1, fp) == 0) {
    ESP_LOGE(TAG, "fread failed");
  }
  fclose(fp);

  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_register_uri_handler(server, &uri_get);
    httpd_register_uri_handler(server, &uri_config);
    httpd_register_uri_handler(server, &uri_styles);
    httpd_register_uri_handler(server, &uri_reboot);
    httpd_register_uri_handler(server, &uri_setros);
  }

  return ESP_OK;
}
