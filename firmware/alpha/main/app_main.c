/* Solution Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <dirent.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/timers.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <driver/ledc.h>
#include <driver/gpio.h>

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include <nvs_flash.h>
#include <esp_spiffs.h>
#include <esp_event_base.h>
#include <esp_event.h>
#include "esp_mac.h"
#include <esp_https_ota.h>
#include <lwip/sockets.h>
#include <esp_http_server.h>

#include <wifi_provisioning/manager.h>
#include "wifi_prov.h"

#include <espnow.h>
#include <initiator.h>
#include <espnow_prov.h>
#include <espnow_security.h>
#include <espnow_security_handshake.h>
#include <espnow_storage.h>
#include <espnow_utils.h>
#include <espnow_ctrl.h>

#include <iot_button.h>

#include "led_indicator.h"
#include "net_logging.h"

#include <cJSON.h>

#include <vscp.h>
#include <vscp-firmware-helper.h>
#include <vscp_class.h>
#include <vscp_type.h>

#include "websrv.h"
#include "mqtt.h"
#include "tcpsrv.h"

#include "alpha.h"

static const char *TAG = "app";
static const char *POP = VSCP_PROJDEF_ESPNOW_SESSION_POP;

#define KEYSTR                                                                                                         \
  "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x%02x:%02x:%02x:%02x:%02x:%02x:%02x:%" \
  "02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x"
#define KEY2STR(a)                                                                                                     \
  (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5], (a)[6], (a)[7], (a)[8], (a)[9], (a)[10], (a)[11], (a)[12], (a)[13],  \
    (a)[14], (a)[15], (a)[16], (a)[17], (a)[18], (a)[19], (a)[20], (a)[21], (a)[22], (a)[23], (a)[24], (a)[25],        \
    (a)[26], (a)[27], (a)[28], (a)[29], (a)[30], (a)[31]

// All the default GPIOs are based on ESP32 series DevKitC boards, for other boards, please modify them accordingly.
#ifdef CONFIG_IDF_TARGET_ESP32C2
#define LED_RED_GPIO   GPIO_NUM_0
#define LED_GREEN_GPIO GPIO_NUM_1
#define LED_BLUE_GPIO  GPIO_NUM_8
#elif CONFIG_IDF_TARGET_ESP32C3
#define LED_STRIP_GPIO GPIO_NUM_8
#elif CONFIG_IDF_TARGET_ESP32
// There is not LED module in ESP32 DevKitC board, so you need to connect one by yourself.
#define LED_STRIP_GPIO GPIO_NUM_18
#elif CONFIG_IDF_TARGET_ESP32S2
#define LED_STRIP_GPIO GPIO_NUM_18
#elif CONFIG_IDF_TARGET_ESP32S3
// For old version board, the number is 48.
#define LED_STRIP_GPIO GPIO_NUM_38
#endif

led_indicator_handle_t s_led_handle_red;
led_indicator_handle_t s_led_handle_green;

const blink_step_t alpha_blink[] = {
  { LED_BLINK_HOLD, LED_STATE_ON, 50 },   // step1: turn on LED 50 ms
  { LED_BLINK_HOLD, LED_STATE_OFF, 100 }, // step2: turn off LED 100 ms
  { LED_BLINK_HOLD, LED_STATE_ON, 150 },  // step3: turn on LED 150 ms
  { LED_BLINK_HOLD, LED_STATE_OFF, 100 }, // step4: turn off LED 100 ms
  { LED_BLINK_STOP, 0, 0 },               // step5: stop blink (off)
};


// Signal Wi-Fi events on this event-group
const int WIFI_CONNECTED_BIT = BIT0;
static EventGroupHandle_t g_wifi_event_group;

/* 
  Used to count the time factory default storage button is down
  If > 10 seconds defaults will be restored.
*/
static uint32_t s_restore_defaults_timer; 

///////////////////////////////////////////////////////////
//                   P E R S I S T A N T
///////////////////////////////////////////////////////////

// Set default configuration

node_persistent_config_t g_persistent = {

  // General
  .nodeName   = VSCP_PROJDEF_DEVICE_NAME,
  .pmk        = { 0 },
  .nodeGuid   = { 0 }, // GUID for unit
  .startDelay = 2,
  .bootCnt    = 0,
  .queueSize  = CONFIG_APP_ESPNOW_QUEUE_SIZE,

  // Logging
  .logwrite2Stdout = 1,
  .logLevel        = ESP_LOG_INFO,
  .logType         = ALPHA_LOG_UDP,
  .logRetries      = 5,
  .logUrl          = "255.255.255.255",
  .logPort         = 6789,
  .logMqttTopic    = "%guid/log",

  // Web server
  .webEnable   = true,
  .webPort     = 80,
  .webUsername = "vscp",
  .webPassword = "secret",

  // VSCP tcp/ip Link
  .vscplinkEnable   = true,
  .vscplinkUrl      = { 0 },
  .vscplinkPort     = VSCP_DEFAULT_TCP_PORT,
  .vscplinkUsername = "vscp",
  .vscplinkPassword = "secret",
  .vscpLinkKey      = { 0 }, // VSCP_DEFAULT_KEY32,

  // MQTT
  .mqttEnable       = true,
  .mqttUrl          = { 0 },
  .mqttPort         = 1883,
  .mqttClientid     = "{{node}}-{{guid}}",
  .mqttUsername     = "vscp",
  .mqttPassword     = "secret",
  .mqttQos          = 0,
  .mqttRetain       = 0,
  .mqttSub          = "vscp/{{guid}}/pub/#",
  .mqttPub          = "vscp/{{guid}}/{{class}}/{{type}}/{{index}}",
  .mqttVerification = { 0 },
  .mqttLwTopic      = { 0 },
  .mqttLwMessage    = { 0 },
  .mqttLwQos        = 0,
  .mqttLwRetain     = false,

  // Droplet
  .dropletEnable                = true,
  .dropletLongRange             = false,
  .dropletChannel               = 0, // Use wifi channel
  .dropletTtl                   = 32,
  .dropletSizeQueue             = 32,                     // Size fo input queue
  .dropletForwardEnable         = true,                   // Forward when packets are received
  .dropletEncryption            = VSCP_ENCRYPTION_AES128, // 0=no encryption, 1=AES-128, 2=AES-192, 3=AES-256
  .dropletFilterAdjacentChannel = true,                   // Don't receive if from other channel
  .dropletForwardSwitchChannel  = false,                  // Allow switchin gchannel on forward
  .dropletFilterWeakSignal      = -100,                   // Filter onm RSSI (zero is no rssi filtering)
};

#if CONFIG_IDF_TARGET_ESP32C2 || CONFIG_IDF_TARGET_ESP32C3
#define CONTROL_KEY_GPIO GPIO_NUM_9
#else
#define CONTROL_KEY_GPIO GPIO_NUM_0

typedef enum { APP_ESPNOW_CTRL_INIT, APP_ESPNOW_CTRL_BOUND, APP_ESPNOW_CTRL_MAX } app_espnow_ctrl_status_t;

static app_espnow_ctrl_status_t s_espnow_ctrl_status = APP_ESPNOW_CTRL_INIT;
#endif

#define WIFI_PROV_KEY_GPIO GPIO_NUM_0

typedef enum {
  APP_WIFI_PROV_INIT,
  APP_WIFI_PROV_START,
  APP_WIFI_PROV_SUCCESS,
  APP_WIFI_PROV_MAX
} app_wifi_prov_status_t;

static app_wifi_prov_status_t s_wifi_prov_status = APP_WIFI_PROV_INIT;

// Handle for nvs storage
nvs_handle_t g_nvsHandle = 0;

///////////////////////////////////////////////////////////////////////////////
// read_onboard_temperature
//

float
read_onboard_temperature(void)
{
  // TODO
  return 0;
}

///////////////////////////////////////////////////////////////////////////////
// getMilliSeconds
//

uint32_t
getMilliSeconds(void)
{
  return (esp_timer_get_time() / 1000);
};

///////////////////////////////////////////////////////////////////////////////
// get_device_guid
//

bool
get_device_guid(uint8_t *pguid)
{
  esp_err_t rv;
  size_t length = 16;

  // Check pointer
  if (NULL == pguid) {
    return false;
  }

  rv = nvs_get_blob(g_nvsHandle, "guid", pguid, &length);
  switch (rv) {

    case ESP_OK:
      break;

    case ESP_ERR_NVS_NOT_FOUND:
      printf("GUID not found in nvs\n");
      return false;

    default:
      printf("Error (%s) reading GUID from nvs!\n", esp_err_to_name(rv));
      return false;
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////
// app_led_init
//

static void
app_led_init(void)
{
  // Initialize LED indicator
  led_indicator_config_t indicator_config_green = {
    .off_level = 0, // if zero, attach led positive side to esp32 gpio pin
    .mode      = LED_GPIO_MODE,
  };
  
  s_led_handle_green = led_indicator_create(PRJDEF_INDICATOR_LED_PIN_GREEN, &indicator_config_green);
  if (NULL == s_led_handle_green) {
    ESP_LOGE(TAG, "Failed to create LED indicator green");
  }

  // Initialize LED indicator
  led_indicator_config_t indicator_config_red = {
    .off_level = 0, // if zero, attach led positive side to esp32 gpio pin
    .mode      = LED_GPIO_MODE,
  };
  
  s_led_handle_red = led_indicator_create(PRJDEF_INDICATOR_LED_PIN_RED, &indicator_config_red);
  if (NULL == s_led_handle_red) {
    ESP_LOGE(TAG, "Failed to create LED indicator green");
  }

}

static esp_err_t
app_espnow_prov_recv_cb(uint8_t *src_addr, void *data, size_t size, wifi_pkt_rx_ctrl_t *rx_ctrl)
{
  ESP_PARAM_CHECK(src_addr);
  ESP_PARAM_CHECK(data);
  ESP_PARAM_CHECK(size);
  ESP_PARAM_CHECK(rx_ctrl);

  espnow_prov_initiator_t *initiator_info = (espnow_prov_initiator_t *) data;
  /**
   * @brief Authenticate the device through the information of the initiator
   */
  ESP_LOGI(TAG,
           "MAC: " MACSTR ", Channel: %d, RSSI: %d, Product_id: %s, Device Name: %s, Auth Mode: %d, device_secret: %s",
           MAC2STR(src_addr),
           rx_ctrl->channel,
           rx_ctrl->rssi,
           initiator_info->product_id,
           initiator_info->device_name,
           initiator_info->auth_mode,
           initiator_info->device_secret);

  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// app_sec_initiator
//

static esp_err_t
app_sec_initiator()
{
  esp_err_t ret;
  uint8_t key_info[APP_KEY_LEN];

  if (espnow_get_key(key_info) != ESP_OK) {
    esp_fill_random(key_info, APP_KEY_LEN);
  }
  espnow_set_key(key_info);

  uint32_t start_time1                  = xTaskGetTickCount();
  espnow_sec_result_t espnow_sec_result = { 0 };
  espnow_sec_responder_t *info_list     = NULL;
  size_t num                            = 0;
  ret                                   = espnow_sec_initiator_scan(&info_list, &num, pdMS_TO_TICKS(3000));
  ESP_LOGI(TAG, "-----> Nodes waiting for security params: %u", num);

  if (num == 0) {
    ESP_FREE(info_list);
    return ESP_ERR_INVALID_SIZE;
  }

  espnow_addr_t *dest_addr_list = ESP_MALLOC(num * ESPNOW_ADDR_LEN);

  for (size_t i = 0; i < num; i++) {
    ESP_LOGI(TAG, "sec node %d - " MACSTR " ", i, MAC2STR(info_list[i].mac));
    memcpy(dest_addr_list[i], info_list[i].mac, ESPNOW_ADDR_LEN);
  }

  espnow_sec_initiator_scan_result_free();

  uint32_t start_time2 = xTaskGetTickCount();
  ret                  = espnow_sec_initiator_start(key_info, POP, dest_addr_list, num, &espnow_sec_result);
  ESP_ERROR_GOTO(ret != ESP_OK, EXIT, "<%s> espnow_sec_initiator_start", esp_err_to_name(ret));

  ESP_LOGI(TAG,
           "App key is sent to the device to complete, Spend time: %" PRId32 "ms, Scan time: %" PRId32 "ms",
           (xTaskGetTickCount() - start_time1) * portTICK_PERIOD_MS,
           (start_time2 - start_time1) * portTICK_PERIOD_MS);
  ESP_LOGI(TAG,
           "Devices security completed, successed_num: %u, unfinished_num: %u",
           espnow_sec_result.successed_num,
           espnow_sec_result.unfinished_num);

EXIT:
  ESP_FREE(dest_addr_list);
  return espnow_sec_initiator_result_free(&espnow_sec_result);
}

///////////////////////////////////////////////////////////////////////////////
// app_prov_responder_init
//
// Provisioning wifi for other nodes.
//

static esp_err_t
app_prov_responder_init()
{
  esp_err_t ret                          = ESP_OK;
  espnow_prov_responder_t responder_info = { .product_id = "responder_test" };
  espnow_prov_wifi_t wifi_config = {
        .sta = {
            .ssid = "ssid test",
            .password = "password test",
        },
    };

  ret = espnow_prov_responder_start(&responder_info, pdMS_TO_TICKS(30 * 1000), &wifi_config, app_espnow_prov_recv_cb);
  ESP_ERROR_RETURN(ret != ESP_OK, ret, "espnow_prov_responder_beacon");

  return ESP_OK;
}


///////////////////////////////////////////////////////////////////////////////
// app_wifi_prov_over_espnow_start_press_cb
//

static void
app_wifi_prov_over_espnow_start_press_cb(void *arg, void *usr_data)
{
  ESP_ERROR_CHECK(!(BUTTON_SINGLE_CLICK == iot_button_get_event(arg)));

  if (s_wifi_prov_status == APP_WIFI_PROV_SUCCESS) {
    bool enabled;

    espnow_get_config_for_data_type(ESPNOW_DATA_TYPE_PROV, &enabled);

    if (enabled) {
      ESP_LOGI(TAG, "WiFi provisioning over ESP-NOW is started");
    }
    else {
      ESP_LOGI(TAG, "Start WiFi provisioning over ESP-NOW on initiator");

      //  Start 30s prov beacon
      // app_espnow_prov_beacon_start(30);
      app_prov_responder_init();
    }
  }
  else if (s_wifi_prov_status == APP_WIFI_PROV_START) {
    ESP_LOGI(TAG, "Please finish WiFi provisioning firstly");
  }
  else {
    ESP_LOGI(TAG, "Please start WiFi provisioning firstly");
  }
}

///////////////////////////////////////////////////////////////////////////////
// app_wifi_prov_start_press_cb
//

static void
app_wifi_prov_start_press_cb(void *arg, void *usr_data)
{
  ESP_ERROR_CHECK(!(BUTTON_DOUBLE_CLICK == iot_button_get_event(arg)));

  if (s_wifi_prov_status == APP_WIFI_PROV_INIT) {

    ESP_LOGI(TAG, "Starting WiFi provisioning on initiator");

    wifi_prov();

    s_wifi_prov_status = APP_WIFI_PROV_START;

    //app_led_set_color(255, 255, 255);
  }
  else if (s_wifi_prov_status == APP_WIFI_PROV_START) {
    ESP_LOGI(TAG, "WiFi provisioning is running");
  }
  else {
    ESP_LOGI(TAG, "Security initiator started.");
    app_sec_initiator();
  }
}

///////////////////////////////////////////////////////////////////////////////
// app_long_press_start_cb
//

static void
app_long_press_start_cb(void *arg, void *usr_data)
{
  ESP_ERROR_CHECK(!(BUTTON_LONG_PRESS_START == iot_button_get_event(arg)));
  s_restore_defaults_timer = getMilliSeconds();
}

///////////////////////////////////////////////////////////////////////////////
// app_factory_reset_press_cb
//

static void
app_factory_reset_press_cb(void *arg, void *usr_data)
{
  ESP_ERROR_CHECK(!(BUTTON_LONG_PRESS_HOLD == iot_button_get_event(arg)));
  ESP_LOGI(TAG,
           "Will restore defaults in %u seconds",
           (int) (10 - ((getMilliSeconds() - s_restore_defaults_timer) / 1000)));
  if ((getMilliSeconds() - s_restore_defaults_timer) > 10000) {
    //vprintf_like_t logFunc = esp_log_set_vprintf(g_stdLogFunc);
    wifi_prov_mgr_reset_provisioning();
    esp_wifi_disconnect();
    esp_restart();
  }

  ESP_LOGI(TAG, "Reset WiFi provisioning information and restart");  
}



///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_data_cb
//

static void
vscp_espnow_data_cb(uint8_t *src_addr, void *data, size_t size, wifi_pkt_rx_ctrl_t *rx_ctrl)
{
  ESP_LOGI(TAG, "Rcv data len=%zd src=" MACSTR, size, MAC2STR(src_addr));
}

///////////////////////////////////////////////////////////////////////////////
// app_wifi_prov_button_init
//

static void
app_wifi_prov_button_init(void)
{
  button_config_t button_config = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = 5000,
        .gpio_button_config = {
            .gpio_num = WIFI_PROV_KEY_GPIO,
            .active_level = 0,
        },
    };

  button_handle_t button_handle = iot_button_create(&button_config);

  iot_button_register_cb(button_handle, BUTTON_SINGLE_CLICK, app_wifi_prov_over_espnow_start_press_cb, NULL);
  iot_button_register_cb(button_handle, BUTTON_DOUBLE_CLICK, app_wifi_prov_start_press_cb, NULL);
  iot_button_register_cb(button_handle, BUTTON_LONG_PRESS_HOLD, app_factory_reset_press_cb, NULL);
  iot_button_register_cb(button_handle, BUTTON_LONG_PRESS_START, app_long_press_start_cb, NULL);
}

///////////////////////////////////////////////////////////////////////////////
// app_wifi_init
//

static void
app_wifi_init()
{
  wifi_prov_init();
}

///////////////////////////////////////////////////////////////////////////////
// readPersistentConfigs
//

static esp_err_t
readPersistentConfigs(void)
{
  esp_err_t rv;
  char buf[80];
  size_t length = sizeof(buf);
  uint8_t val;

  // Set default primary key
  vscp_fwhlp_hex2bin(g_persistent.vscpLinkKey, 16, VSCP_DEFAULT_KEY16);

  // boot counter
  rv = nvs_get_u32(g_nvsHandle, "boot_counter", &g_persistent.bootCnt);
  switch (rv) {

    case ESP_OK:
      ESP_LOGI(TAG, "Boot counter = %d", (int) g_persistent.bootCnt);
      break;

    case ESP_ERR_NVS_NOT_FOUND:
      ESP_LOGE(TAG, "The boot counter is not initialized yet!");
      break;

    default:
      ESP_LOGE(TAG, "Error (%s) reading boot counter!", esp_err_to_name(rv));
      break;
  }

  // Update and write back boot counter
  g_persistent.bootCnt++;
  rv = nvs_set_u32(g_nvsHandle, "boot_counter", &g_persistent.bootCnt);
  if (rv != ESP_OK) {
    ESP_LOGE(TAG, "Failed to update boot counter");
  }

  // Node name
  rv = nvs_get_str(g_nvsHandle, "node_name", buf, &length);
  switch (rv) {
    case ESP_OK:
      strncpy(g_persistent.nodeName, buf, sizeof(g_persistent.nodeName));
      ESP_LOGI(TAG, "Node Name = %s", g_persistent.nodeName);
      break;

    case ESP_ERR_NVS_NOT_FOUND:
      rv = nvs_set_str(g_nvsHandle, "node_name", "Alpha Node");
      if (rv != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update node name");
      }
      break;

    default:
      ESP_LOGE(TAG, "Error (%s) reading 'node_name'!", esp_err_to_name(rv));
      break;
  }

  // Start Delay (seconds)
  rv = nvs_get_u8(g_nvsHandle, "start_delay", &g_persistent.startDelay);
  switch (rv) {

    case ESP_OK:
      ESP_LOGI(TAG, "Start delay = %d", g_persistent.startDelay);
      break;

    case ESP_ERR_NVS_NOT_FOUND:
      rv = nvs_set_u8(g_nvsHandle, "start_delay", 2);
      if (rv != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update start delay");
      }
      break;

    default:
      ESP_LOGE(TAG, "Error (%s) reading!", esp_err_to_name(rv));
      break;
  }

  // pmk (Primary key)
  length = 16;
  rv     = nvs_get_blob(g_nvsHandle, "pmk", g_persistent.pmk, &length);
  if (rv != ESP_OK) {
    const char key[] = VSCP_DEFAULT_KEY16;
    const char *pos  = key;
    for (int i = 0; i < 16; i++) {
      sscanf(pos, "%2hhx", &g_persistent.pmk[i]);
      pos += 2;
    }
    rv = nvs_set_blob(g_nvsHandle, "pmk", g_persistent.pmk, 16);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to write node pmk to nvs. rv=%d", rv);
    }
  }

  // GUID
  length = 16;
  rv     = nvs_get_blob(g_nvsHandle, "guid", g_persistent.nodeGuid, &length);

  if (rv != ESP_OK) {
    // FF:FF:FF:FF:FF:FF:FF:FE:MAC1:MAC2:MAC3:MAC4:MAC5:MAC6:NICKNAME1:NICKNAME2
    memset(g_persistent.nodeGuid, 0xff, 7);
    g_persistent.nodeGuid[7] = 0xfe;
    // rv                       = esp_efuse_mac_get_default(g_persistent.nodeGuid + 8);
    //  ESP_MAC_WIFI_STA
    //  ESP_MAC_WIFI_SOFTAP
    rv = esp_read_mac(g_persistent.nodeGuid + 8, ESP_MAC_WIFI_STA);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "esp_efuse_mac_get_default failed to get GUID. rv=%d", rv);
    }

    rv = nvs_set_blob(g_nvsHandle, "guid", g_persistent.nodeGuid, 16);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to write node GUID to nvs. rv=%d", rv);
    }
  }
  ESP_LOGI(TAG,
           "GUID for node: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
           g_persistent.nodeGuid[0],
           g_persistent.nodeGuid[1],
           g_persistent.nodeGuid[2],
           g_persistent.nodeGuid[3],
           g_persistent.nodeGuid[4],
           g_persistent.nodeGuid[5],
           g_persistent.nodeGuid[6],
           g_persistent.nodeGuid[7],
           g_persistent.nodeGuid[8],
           g_persistent.nodeGuid[9],
           g_persistent.nodeGuid[10],
           g_persistent.nodeGuid[11],
           g_persistent.nodeGuid[12],
           g_persistent.nodeGuid[13],
           g_persistent.nodeGuid[14],
           g_persistent.nodeGuid[15]);

  // espnow queueSize
  rv = nvs_get_u8(g_nvsHandle, "queue_size", &g_persistent.queueSize);
  if (ESP_OK != rv) {
    rv = nvs_set_u8(g_nvsHandle, "queue_size", g_persistent.queueSize);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update queue_size");
    }
  }

  // Logging ------------------------------------------------------------------

  // logwrite2Stdout
  rv = nvs_get_u8(g_nvsHandle, "log_stdout", &g_persistent.logwrite2Stdout);
  if (ESP_OK != rv) {
    rv = nvs_set_u8(g_nvsHandle, "log_stdout", g_persistent.logwrite2Stdout);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update log-stdout");
    }
  }

  // logLevel
  esp_log_level_set("*", ESP_LOG_INFO);
  rv = nvs_get_u8(g_nvsHandle, "log_level", &g_persistent.logLevel);
  if (ESP_OK != rv) {
    rv = nvs_set_u8(g_nvsHandle, "log_level", g_persistent.logLevel);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update log-level");
    }
  }

  // logType
  rv = nvs_get_u8(g_nvsHandle, "log_type", &g_persistent.logType);
  if (ESP_OK != rv) {
    rv = nvs_set_u8(g_nvsHandle, "log_type", g_persistent.logType);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update log-type");
    }
  }

  // logRetries
  rv = nvs_get_u8(g_nvsHandle, "log_retries", &g_persistent.logRetries);
  if (ESP_OK != rv) {
    rv = nvs_set_u8(g_nvsHandle, "log:retries", g_persistent.logRetries);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update log-retries");
    }
  }

  // logUrl
  length = sizeof(g_persistent.logUrl);
  rv     = nvs_get_str(g_nvsHandle, "log_url", g_persistent.logUrl, &length);
  if (rv != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read 'log URL' will be set to default. ret=%d", rv);
    rv = nvs_set_str(g_nvsHandle, "log_url", g_persistent.logUrl);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to save log URL");
    }
  }

  // logPort
  rv = nvs_get_u16(g_nvsHandle, "log_port", &g_persistent.logPort);
  if (ESP_OK != rv) {
    rv = nvs_set_u16(g_nvsHandle, "log_port", g_persistent.logPort);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update log_port");
    }
  }

  // logMqttTopic
  length = sizeof(g_persistent.logMqttTopic);
  rv     = nvs_get_str(g_nvsHandle, "log_mqtt_topic", g_persistent.logMqttTopic, &length);
  if (rv != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read 'log MQTT topic' will be set to default. ret=%d", rv);
    rv = nvs_set_str(g_nvsHandle, "log_mqtt_topic", g_persistent.logMqttTopic);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to save log MQTT topic");
    }
  }

  // VSCP Link ----------------------------------------------------------------

  // VSCP Link enable
  rv = nvs_get_u8(g_nvsHandle, "vscp_enable", &val);
  if (rv != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read 'VSCP link enable' will be set to default. ret=%d", rv);
    val = (uint8_t) g_persistent.vscplinkEnable;
    rv  = nvs_set_u8(g_nvsHandle, "vscp_enable", g_persistent.vscplinkEnable);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to save VSCP link enable");
    }
  }
  else {
    g_persistent.vscplinkEnable = (bool) val;
  }

  // VSCP Link host
  length = sizeof(g_persistent.vscplinkUrl);
  rv     = nvs_get_str(g_nvsHandle, "vscp_url", g_persistent.vscplinkUrl, &length);
  if (rv != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read 'VSCP link host' will be set to default. ret=%d", rv);
    rv = nvs_set_str(g_nvsHandle, "vscp_url", PRJDEF_DEFAULT_TCPIP_USER);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to save VSCP link host");
    }
  }

  // VSCP link port
  rv = nvs_get_u16(g_nvsHandle, "vscp_port", &g_persistent.vscplinkPort);
  if (ESP_OK != rv) {
    rv = nvs_set_u16(g_nvsHandle, "vscp_port", g_persistent.vscplinkPort);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update VSCP link port");
    }
  }

  // VSCP Link key
  length = sizeof(g_persistent.vscpLinkKey);
  rv     = nvs_get_blob(g_nvsHandle, "vscp_key", (char *) g_persistent.vscpLinkKey, &length);
  if (rv != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read 'VSCP link_key' will be set to default. ret=%d", rv);
    rv = nvs_set_blob(g_nvsHandle, "vscp_key", g_persistent.vscpLinkKey, sizeof(g_persistent.vscpLinkKey));
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to save VSCL link key");
    }
  }

  // VSCP Link Username
  length = sizeof(g_persistent.vscplinkUsername);
  rv     = nvs_get_str(g_nvsHandle, "vscp_user", g_persistent.vscplinkUsername, &length);
  if (rv != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read 'VSCP Username' will be set to default. ret=%d", rv);
    rv = nvs_set_str(g_nvsHandle, "vscp_user", PRJDEF_DEFAULT_TCPIP_USER);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to save VSCP username");
    }
  }

  // VSCP Link password
  length = sizeof(g_persistent.vscplinkPassword);
  rv     = nvs_get_str(g_nvsHandle, "vscp_password", g_persistent.vscplinkPassword, &length);
  if (rv != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read 'VSCP password' will be set to default. ret=%d", rv);
    nvs_set_str(g_nvsHandle, "vscp_password", PRJDEF_DEFAULT_TCPIP_PASSWORD);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to save VSCP password");
    }
  }
  // ESP_LOGI(TAG, "VSCP Password: %s", buf);

  // MQTT ----------------------------------------------------------------

  // MQTT enable
  rv = nvs_get_u8(g_nvsHandle, "mqtt_enable", &val);
  if (rv != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read 'mqtt enable' will be set to default. ret=%d", rv);
    val = (uint8_t) g_persistent.mqttEnable;
    rv  = nvs_set_u8(g_nvsHandle, "mqtt_enable", g_persistent.mqttEnable);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to save MQTT enable");
    }
  }
  else {
    g_persistent.mqttEnable = (bool) val;
  }

  // MQTT host
  length = sizeof(g_persistent.mqttUrl);
  rv     = nvs_get_str(g_nvsHandle, "mqtt_url", g_persistent.mqttUrl, &length);
  if (rv != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read 'MQTT host' will be set to default. ret=%d", rv);
    rv = nvs_set_str(g_nvsHandle, "mqtt_url", g_persistent.mqttUrl);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to save MQTT host");
    }
  }

  // MQTT port
  rv = nvs_get_u16(g_nvsHandle, "mqtt_port", &g_persistent.mqttPort);
  if (ESP_OK != rv) {
    rv = nvs_set_u16(g_nvsHandle, "mqtt_port", g_persistent.mqttPort);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update MQTT port");
    }
  }

  // MQTT client
  length = sizeof(g_persistent.mqttClientid);
  rv     = nvs_get_str(g_nvsHandle, "mqtt_cid", g_persistent.mqttClientid, &length);
  if (rv != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read 'MQTT clientid' will be set to default. ret=%d", rv);
    rv = nvs_set_str(g_nvsHandle, "mqtt_cid", g_persistent.mqttClientid);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to save MQTT clientid");
    }
  }

  // MQTT Link Username
  length = sizeof(g_persistent.mqttUsername);
  rv     = nvs_get_str(g_nvsHandle, "mqtt_user", g_persistent.mqttUsername, &length);
  if (rv != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read 'MQTT user' will be set to default. ret=%d", rv);
    rv = nvs_set_str(g_nvsHandle, "mqtt_user", PRJDEF_DEFAULT_TCPIP_USER);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to save MQTT username");
    }
  }

  // MQTT password
  length = sizeof(g_persistent.mqttPassword);
  rv     = nvs_get_str(g_nvsHandle, "mqtt_password", g_persistent.mqttPassword, &length);
  if (rv != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read 'MQTT password' will be set to default. ret=%d", rv);
    nvs_set_str(g_nvsHandle, "mqtt_password", PRJDEF_DEFAULT_TCPIP_PASSWORD);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to save MQTT password");
    }
  }

  // MQTT subscribe
  length = sizeof(g_persistent.mqttSub);
  rv     = nvs_get_str(g_nvsHandle, "mqtt_sub", g_persistent.mqttSub, &length);
  if (rv != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read 'MQTT sub' will be set to default. ret=%d", rv);
    rv = nvs_set_str(g_nvsHandle, "mqtt_sub", g_persistent.mqttSub);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to save MQTT sub");
    }
  }

  // MQTT publish
  length = sizeof(g_persistent.mqttPub);
  rv     = nvs_get_str(g_nvsHandle, "mqtt_pub", g_persistent.mqttPub, &length);
  if (rv != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read 'MQTT pub' will be set to default. ret=%d", rv);
    rv = nvs_set_str(g_nvsHandle, "mqtt_pub", g_persistent.mqttPub);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to save MQTT pub");
    }
  }

  // WEB server ----------------------------------------------------------------

  // WEB enable
  rv = nvs_get_u8(g_nvsHandle, "web_enable", &val);
  if (rv != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read 'web enable' will be set to default. ret=%d", rv);
    val = (uint8_t) g_persistent.webEnable;
    rv  = nvs_set_u8(g_nvsHandle, "web_enable", g_persistent.webEnable);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to save web enable");
    }
  }
  else {
    g_persistent.webEnable = (bool) val;
  }

  // WEB port
  rv = nvs_get_u16(g_nvsHandle, "web_port", &g_persistent.webPort);
  if (ESP_OK != rv) {
    rv = nvs_set_u16(g_nvsHandle, "web_port", g_persistent.webPort);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update Web server port");
    }
  }

  // WEB Username
  length = sizeof(g_persistent.webUsername);
  rv     = nvs_get_str(g_nvsHandle, "web_user", g_persistent.webUsername, &length);
  if (rv != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read 'Web server user' will be set to default. ret=%d", rv);
    rv = nvs_set_str(g_nvsHandle, "web_user", PRJDEF_DEFAULT_TCPIP_USER);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to save Web Server username");
    }
  }

  // WEB password
  length = sizeof(g_persistent.webPassword);
  rv     = nvs_get_str(g_nvsHandle, "web_password", g_persistent.webPassword, &length);
  if (rv != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read 'Web server password' will be set to default. ret=%d", rv);
    nvs_set_str(g_nvsHandle, "web_password", PRJDEF_DEFAULT_TCPIP_PASSWORD);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to save Web server password");
    }
  }

  // Droplet ----------------------------------------------------------------

  // VSCP Link enable
  rv = nvs_get_u8(g_nvsHandle, "drop_enable", &val);
  if (rv != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read 'Droplet enable' will be set to default. ret=%d", rv);
    val = (uint8_t) g_persistent.dropletEnable;
    rv  = nvs_set_u8(g_nvsHandle, "drop_enable", g_persistent.dropletEnable);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to save VSCP link enable");
    }
  }
  else {
    g_persistent.dropletEnable = (bool) val;
  }

  // Long Range
  rv = nvs_get_u8(g_nvsHandle, "drop_lr", &val);
  if (ESP_OK != rv) {
    val = (uint8_t) g_persistent.dropletLongRange;
    rv  = nvs_set_u8(g_nvsHandle, "drop_lr", g_persistent.dropletLongRange);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update droplet long range");
    }
  }
  else {
    g_persistent.dropletLongRange = (bool) val;
  }

  // Channel
  rv = nvs_get_u8(g_nvsHandle, "drop_ch", &g_persistent.dropletChannel);
  if (ESP_OK != rv) {
    rv = nvs_set_u8(g_nvsHandle, "drop_ch", g_persistent.dropletChannel);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update droplet channel");
    }
  }

  // Default queue size
  rv = nvs_get_u8(g_nvsHandle, "drop_qsize", &g_persistent.dropletSizeQueue);
  if (ESP_OK != rv) {
    rv = nvs_set_u8(g_nvsHandle, "drop_qsize", g_persistent.dropletSizeQueue);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update droplet queue size");
    }
  }

  // Default ttl
  rv = nvs_get_u8(g_nvsHandle, "drop_ttl", &g_persistent.dropletTtl);
  if (ESP_OK != rv) {
    rv = nvs_set_u8(g_nvsHandle, "drop_ttl", g_persistent.dropletTtl);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update droplet ttl");
    }
  }

  // Forward
  rv = nvs_get_u8(g_nvsHandle, "drop_fw", &val);
  if (ESP_OK != rv) {
    val = (uint8_t) g_persistent.dropletForwardEnable;
    rv  = nvs_set_u8(g_nvsHandle, "drop_fw", g_persistent.dropletForwardEnable);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update droplet forward");
    }
  }
  else {
    g_persistent.dropletForwardEnable = (bool) val;
  }

  // Encryption
  rv = nvs_get_u8(g_nvsHandle, "drop_enc", &g_persistent.dropletEncryption);
  if (ESP_OK != rv) {
    rv = nvs_set_u8(g_nvsHandle, "drop_enc", g_persistent.dropletEncryption);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update droplet encryption");
    }
  }

  // Adj filter channel
  rv = nvs_get_u8(g_nvsHandle, "drop_filt", &val);
  if (ESP_OK != rv) {
    val = (uint8_t) g_persistent.dropletFilterAdjacentChannel;
    rv  = nvs_set_u8(g_nvsHandle, "drop_filt", g_persistent.dropletFilterAdjacentChannel);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update droplet adj channel filter");
    }
  }
  else {
    g_persistent.dropletFilterAdjacentChannel = (bool) val;
  }

  // Allow switching channel on forward
  rv = nvs_get_u8(g_nvsHandle, "drop_swchf", &val);
  if (ESP_OK != rv) {
    val = (uint8_t) g_persistent.dropletForwardSwitchChannel;
    rv  = nvs_set_u8(g_nvsHandle, "drop_swchf", g_persistent.dropletForwardSwitchChannel);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update droplet shitch channel on forward");
    }
  }
  else {
    g_persistent.dropletFilterAdjacentChannel = (bool) val;
  }

  // RSSI limit
  rv = nvs_get_i8(g_nvsHandle, "drop_rssi", &g_persistent.dropletFilterWeakSignal);
  if (ESP_OK != rv) {
    rv = nvs_set_u8(g_nvsHandle, "drop_rssi", g_persistent.dropletFilterWeakSignal);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update droplet RSSI");
    }
  }

  rv = nvs_commit(g_nvsHandle);
  if (rv != ESP_OK) {
    ESP_LOGI(TAG, "Failed to commit updates to nvs\n");
  }

  return ESP_OK;
}

// ///////////////////////////////////////////////////////////////////////////////
// // alpha_event_handler
// //
// // Event handler for catching system events
// //

// static void
// alpha_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
// {
//   if (event_base == ALPHA_EVENT) {
//     if (event_id == ALPHA_START_CLIENT_PROVISIONING) {
//       ESP_LOGI(TAG, "* * * Start client provisioning");
//     }
//     else if (event_id == ALPHA_STOP_CLIENT_PROVISIONING) {
//       ESP_LOGI(TAG, "* * * Stop client provisioning");
//     }
//     else if (event_id == ALPHA_GET_IP_ADDRESS_START) {
//       ESP_LOGI(TAG, "* * * Waiting for IP-address");
//     }
//     else if (event_id == ALPHA_GET_IP_ADDRESS_STOP) {
//       ESP_LOGI(TAG, "* * * IP-address received");
//     }
//   }
// }

///////////////////////////////////////////////////////////////////////////////
// system_event_handler
//
// Event handler for catching system events
//

static void
system_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  static bool s_ap_staconnected_flag = false;
  static bool s_sta_connected_flag   = false;
  static int retries;

  if (event_base == WIFI_PROV_EVENT) {

    switch (event_id) {

      case WIFI_PROV_START:
        ESP_LOGI(TAG, "Provisioning started");
        break;

      case WIFI_PROV_CRED_RECV: {
        wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *) event_data;
        ESP_LOGI(TAG,
                 "Received Wi-Fi credentials"
                 "\n\tSSID     : %s\n\tPassword : %s",
                 (const char *) wifi_sta_cfg->ssid,
                 (const char *) wifi_sta_cfg->password);
        break;
      }

      case WIFI_PROV_CRED_FAIL: {
        wifi_prov_sta_fail_reason_t *reason = (wifi_prov_sta_fail_reason_t *) event_data;
        ESP_LOGE(TAG,
                 "Provisioning failed!\n\tReason : %s"
                 "\n\tPlease reset to factory and retry provisioning",
                 (*reason == WIFI_PROV_STA_AUTH_ERROR) ? "Wi-Fi station authentication failed"
                                                       : "Wi-Fi access-point not found");
        retries++;
        // if (retries >= PROV_MGR_MAX_RETRY_CNT) {
        //   ESP_LOGI(TAG,
        //            "Failed to connect with provisioned AP, reseting "
        //            "provisioned credentials");
        //   wifi_prov_mgr_reset_sm_state_on_failure();
        //   retries = 0;
        // }
        break;
      }

      case WIFI_PROV_CRED_SUCCESS:
        ESP_LOGI(TAG, "Provisioning successful");
        retries = 0;
        break;

      case WIFI_PROV_END:
        // De-initialize manager once provisioning is finished
        wifi_prov_mgr_deinit();
        ESP_LOGI(TAG, "Provisioning manager released");
        break;

      default:
        break;
    }
  }
  else if (event_base == WIFI_EVENT) {

    switch (event_id) {

      case WIFI_EVENT_WIFI_READY: {
        // Set channel
        // ESP_ERROR_CHECK(esp_wifi_set_channel(PRJDEF_DROPLET_CHANNEL, WIFI_SECOND_CHAN_NONE));
      } break;

      case WIFI_EVENT_STA_START: {
        ESP_LOGI(TAG, "Connecting........");
        // esp_wifi_connect();
      }

      case WIFI_EVENT_AP_STACONNECTED: {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *) event_data;
        // ESP_LOGI(TAG, "station " MACSTR " join, AID=%d", MAC2STR(event->mac), (int)event->aid);
        s_ap_staconnected_flag = true;
        break;
      }

      case WIFI_EVENT_AP_STADISCONNECTED: {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *) event_data;
        // ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d", MAC2STR(event->mac), ((int)event->aid);
        s_ap_staconnected_flag = false;
        break;
      }

      case WIFI_EVENT_STA_CONNECTED: {
        wifi_event_sta_connected_t *event = (wifi_event_sta_connected_t *) event_data;
        ESP_LOGI(TAG,
                 "Connected to %s (BSSID: " MACSTR ", Channel: %d)",
                 event->ssid,
                 MAC2STR(event->bssid),
                 event->channel);
        s_sta_connected_flag = true;
        break;
      }

      case WIFI_EVENT_STA_DISCONNECTED: {
        ESP_LOGI(TAG, "sta disconnect");
        // esp_restart();
        // esp_wifi_stop();
        // esp_wifi_start();
        //  if (!s_sta_connected_flag) {
        //    s_sta_connected_flag = false;
        //    ESP_LOGI(TAG, "Disconnected. Connecting to the AP again...");
        //    g_state = MAIN_STATE_INIT;
        //    //esp_wifi_disconnect();
        //    esp_err_t ret = esp_wifi_connect();
        //    if (ESP_OK != ret) {
        //      ESP_LOGE(TAG,"Failed to connect %X", ret);
        //    }
        //  }
        //app_led_set_color(255, 0, 0); // RED
        break;
      }
      default:
        break;
    }
  }
  // Post 5.0 stable
  // ---------------
  else if (event_base == ESP_HTTPS_OTA_EVENT) {
    switch (event_id) {

      case ESP_HTTPS_OTA_START: {
        ESP_LOGI(TAG, "OTA https start");
      } break;

      case ESP_HTTPS_OTA_CONNECTED: {
        ESP_LOGI(TAG, "OTA https connected");
      } break;

      case ESP_HTTPS_OTA_GET_IMG_DESC: {
        ESP_LOGI(TAG, "OTA https get image description");
      } break;

      case ESP_HTTPS_OTA_VERIFY_CHIP_ID: {
        ESP_LOGI(TAG, "Verifying chip id of new image: %d", *(esp_chip_id_t *) event_data);
      } break;

      case ESP_HTTPS_OTA_DECRYPT_CB: {
        ESP_LOGI(TAG, "OTA https decrypt callback");
      } break;

      case ESP_HTTPS_OTA_WRITE_FLASH: {
        ESP_LOGI(TAG, "Writing to flash: %d written", *(int *) event_data);
      } break;

      case ESP_HTTPS_OTA_UPDATE_BOOT_PARTITION: {
        ESP_LOGI(TAG, "Boot partition updated. Next Partition: %d", *(esp_partition_subtype_t *) event_data);
      } break;

      case ESP_HTTPS_OTA_FINISH: {
        ESP_LOGI(TAG, "OTA https finish");
      } break;

      case ESP_HTTPS_OTA_ABORT: {
        ESP_LOGI(TAG, "OTA https abort");
      } break;
    }
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    ESP_LOGI(TAG, "Connected with IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
    s_wifi_prov_status = APP_WIFI_PROV_SUCCESS;
    if (led_indicator_start(s_led_handle_green, BLINK_CONNECTED) != ESP_OK) {
      ESP_LOGE(TAG, "Failed to start indicator light");
    }
    // Signal main application to continue execution
    xEventGroupSetBits(g_wifi_event_group, WIFI_CONNECTED_BIT);
  }
  // else if (event_base == ALPHA_EVENT) {
  //   ESP_LOGI(TAG, "Alpha event -----------------------------------------------------------> id=%ld", event_id);
  // }
}

///////////////////////////////////////////////////////////////////////////////
// app_main
//

void
app_main()
{
  esp_err_t ret;
  espnow_storage_init();

  // Set default parameters for espnow
  //     Set here due to persistent writing
  espnow_config_t espnow_config = ESPNOW_INIT_CONFIG_DEFAULT();

  // ----------------------------------------------------------------------------
  //                         Initiate message queues
  // ----------------------------------------------------------------------------

  // g_tr_mqtt.msg_queue = xQueueCreate(10, VSCP_ESPNOW_MAX_FRAME); // MQTT empties

  // ----------------------------------------------------------------------------
  //                        NVS - Persistent storage
  // ----------------------------------------------------------------------------

  // Init persistent storage
  ESP_LOGI(TAG, "Read persistent storage ... ");

  ret = nvs_open("config", NVS_READWRITE, &g_nvsHandle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(ret));
  }
  else {
    // Read (or set to defaults) persistent values
    readPersistentConfigs();
  }

  g_wifi_event_group = xEventGroupCreate();

  app_wifi_init();

  // memcpy(espnow_config.pmk, g_persistent.pmk, 16);
  espnow_config.qsize                  = g_persistent.queueSize; // CONFIG_APP_ESPNOW_QUEUE_SIZE;
  espnow_config.sec_enable             = 1;
  espnow_config.forward_enable         = 1;
  espnow_config.forward_switch_channel = 1;

  espnow_init(&espnow_config);

  // uint8_t mac[6] = { 0xA4, 0xA8, 0x6F, 0x7D, 0x7E, 0x11 };
  // espnow_add_peer(mac, g_persistent.pmk);

  // Register our event handler for Wi-Fi, IP and Provisioning related events
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &system_event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &system_event_handler, NULL));
  // ESP_ERROR_CHECK(esp_event_handler_register(ALPHA_EVENT, ESP_EVENT_ANY_ID, &system_event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(ESP_HTTPS_OTA_EVENT, ESP_EVENT_ANY_ID, &system_event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &system_event_handler, NULL));

  app_led_init();

  if (led_indicator_start(s_led_handle_green, BLINK_CONNECTING) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start indicator light");
  }

  app_wifi_prov_button_init();
  app_espnow_initiator_register();
  // app_control_button_init();
  app_espnow_initiator();

  // ----------------------------------------------------------------------------
  //                                   Spiffs
  // ----------------------------------------------------------------------------

  // Initialize Spiffs for web pages
  ESP_LOGI(TAG, "Initializing SPIFFS");

  esp_vfs_spiffs_conf_t spiffsconf = { .base_path              = "/www",
                                       .partition_label        = "web",
                                       .max_files              = 50,
                                       .format_if_mount_failed = true };

  // Initialize and mount SPIFFS filesystem.
  ret = esp_vfs_spiffs_register(&spiffsconf);

  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      ESP_LOGE(TAG, "Failed to mount or format web filesystem");
    }
    else if (ret == ESP_ERR_NOT_FOUND) {
      ESP_LOGE(TAG, "Failed to find SPIFFS partition for web ");
    }
    else {
      ESP_LOGE(TAG, "Failed to initialize SPIFFS for web (%s)", esp_err_to_name(ret));
    }
    return;
  }

  ESP_LOGI(TAG, "Performing SPIFFS_check().");
  ret = esp_spiffs_check(spiffsconf.partition_label);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "SPIFFS_check() failed (%s)", esp_err_to_name(ret));
    return;
  }
  else {
    ESP_LOGI(TAG, "SPIFFS_check() successful");
  }

  ESP_LOGI(TAG, "SPIFFS for web initialized");

  size_t total = 0, used = 0;
  ret = esp_spiffs_info(spiffsconf.partition_label, &total, &used);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s). Formatting...", esp_err_to_name(ret));
    esp_spiffs_format(spiffsconf.partition_label);
    return;
  }
  else {
    ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
  }

  DIR *dir = opendir("/spiffs");
  if (dir == NULL) {
    return;
  }

  while (true) {

    struct dirent *de = readdir(dir);
    if (!de) {
      break;
    }

    printf("Found file: %s\n", de->d_name);
  }

  closedir(dir);

  // --------------------------------------------------------------------------
  //                     Wait for Wi-Fi connection
  // --------------------------------------------------------------------------

  ESP_LOGI(TAG, "Wait for wifi connection...");
  {
    EventBits_t ret;
    uint8_t cnt = 20; // 20 seconds until reboot due to no IP address
    while (!xEventGroupWaitBits(g_wifi_event_group, WIFI_CONNECTED_BIT, false, true, 1000 / portTICK_PERIOD_MS)) {
      if (--cnt == 0) {
        // esp_wifi_disconnect();
        // esp_restart();
        vTaskDelay(2000 / portTICK_PERIOD_MS);
      }
      // ESP_LOGI(TAG, "Waiting for IP address. %d", cnt);
    }
  }

  // ----------------------------------------------------------------------------
  //                                  Logging
  // ----------------------------------------------------------------------------

  switch (g_persistent.logType) {

    case ALPHA_LOG_NONE:
      break;

    case ALPHA_LOG_UDP:
      // ESP_ERROR_CHECK(udp_logging_init(g_persistent.logUrl, g_persistent.logPort, g_persistent.logwrite2Stdout));
      break;

    case ALPHA_LOG_TCP:
      ESP_ERROR_CHECK(tcp_logging_init(g_persistent.logUrl, g_persistent.logPort, g_persistent.logwrite2Stdout));
      break;

    case ALPHA_LOG_HTTP:
      ESP_ERROR_CHECK(http_logging_init(g_persistent.logUrl, g_persistent.logwrite2Stdout));
      break;

    case ALPHA_LOG_MQTT:
      ESP_ERROR_CHECK(mqtt_logging_init(g_persistent.logUrl, g_persistent.logMqttTopic, g_persistent.logwrite2Stdout));
      break;

    case ALPHA_LOG_VSCP:
      // ESP_ERROR_CHECK(mqtt_logging_init( CONFIG_LOG_MQTT_SERVER_URL, CONFIG_LOG_MQTT_PUB_TOPIC,
      // g_persistent.logwrite2Stdout ));
      break;

    case ALPHA_LOG_STD:
    default:
      break;
  }

  uint8_t key_info[APP_KEY_LEN];

  if (espnow_get_key(key_info) != ESP_OK) {
    ESP_LOGI(TAG, "Generate new security key");
    esp_fill_random(key_info, APP_KEY_LEN);
  }

  ESP_LOGI(TAG, "Security Key: " KEYSTR, KEY2STR(key_info));
  espnow_set_key(key_info);

  // Setup VSCP esp-now

  espnow_set_config_for_data_type(ESPNOW_DATA_TYPE_DATA, true, vscp_espnow_data_cb);
  if (ESP_OK != ret) {
    ESP_LOGE(TAG, "Failed to set VSCP event callback");
  }

  // Start heartbeat task vscp_heartbeat_task
  xTaskCreate(&vscp_espnow_heartbeat_task, "vscp_espnow_heartbeat_task", 1024 * 3, NULL, 1, NULL);

  // Start web server
  httpd_handle_t server;
  if (g_persistent.webEnable) {
    server = start_webserver();
  }

  // Start MQTT client
  if (g_persistent.mqttEnable) {
    // mqtt_start();
  }

  // Start the VSCP Link Protocol Server
  if (g_persistent.vscplinkEnable) {
#ifdef PRJDEF_IPV6
    // xTaskCreate(&tcpsrv_task, "vscp_tcpsrv_task", 4096, (void *) AF_INET6, 5, NULL);
#else
    xTaskCreate(&tcpsrv_task, "vscp_tcpsrv_task", 4096, (void *) AF_INET, 5, NULL);
#endif
  }

  ESP_LOGI(TAG, "Going to work now");

  uint8_t addr[] = { 0xcc, 0x50, 0xe3, 0x80, 0x10, 0xbc };
  while (1) {
    // esp_task_wdt_reset();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    taskYIELD();

    espnow_send(ESPNOW_DATA_TYPE_DATA,
                addr,
                "This is the test of all tests and 123456789",
                43,
                NULL,
                1000 / portTICK_PERIOD_MS);
  }

  // Unmount web spiffs partition and disable SPIFFS
  esp_vfs_spiffs_unregister(spiffsconf.partition_label);
  ESP_LOGI(TAG, "web SPIFFS unmounted");

  // Clean up
  // iot_button_delete(g_btns[0]);

  // Close
  nvs_close(g_nvsHandle);
}
