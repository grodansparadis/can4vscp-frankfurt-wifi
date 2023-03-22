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
#include "esp_mac.h"
#include <wifi_provisioning/manager.h>

#include <espnow.h>
#include <espnow_prov.h>
#include <espnow_security.h>
#include <espnow_security_handshake.h>
#include <espnow_storage.h>
#include <espnow_utils.h>
#include <espnow_ctrl.h>

#include "espnow_console.h"
#include "espnow_log.h"

#include "espnow_ota.h"

#include <iot_button.h>

#include "led_indicator.h" // https://docs.espressif.com/projects/espressif-esp-iot-solution/en/latest/display/led_indicator.html

#include <vscp.h>
#include <vscp-firmware-helper.h>
#include <vscp_class.h>
#include <vscp_type.h>

#include "beta.h"

#ifndef CONFIG_ESPNOW_VERSION
#define ESPNOW_VERSION 2
#else
#define ESPNOW_VERSION CONFIG_ESPNOW_VERSION
#endif

static espnow_addr_t ESPNOW_ADDR_SELF = { 0 };

static const char *TAG = "app";

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

static TaskHandle_t s_prov_task;

typedef enum {
  APP_WIFI_PROV_INIT,
  APP_WIFI_PROV_START,
  APP_WIFI_PROV_SUCCESS,
  APP_WIFI_PROV_MAX
} app_wifi_prov_status_t;

static app_wifi_prov_status_t s_wifi_prov_status = APP_WIFI_PROV_INIT;

/*
 * May connecting network by debug command or espnow provisioning
 */
static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

const char *pop_data = VSCP_PROJDEF_ESPNOW_SESSION_POP;
static TaskHandle_t s_sec_task;
static TaskHandle_t s_prov_task;

///////////////////////////////////////////////////////////
//                   P E R S I S T A N T
///////////////////////////////////////////////////////////

// Set default configuration

node_persistent_config_t g_persistent = {

  // General
  .nodeName   = "Beta Node",
  .nodeGuid   = { 0 }, // GUID for unit
  .startDelay = 2,
  .bootCnt    = 0,
  .queueSize  = CONFIG_APP_ESPNOW_QUEUE_SIZE,

  // espnow
  .espnowLongRange             = false,
  .espnowChannel               = 0, // Use wifi channel
  .espnowTtl                   = 32,
  .espnowSizeQueue             = 32,    // Size fo input queue
  .espnowForwardEnable         = true,  // Forward when packets are received
  .espnowFilterAdjacentChannel = true,  // Don't receive if from other channel
  .espnowForwardSwitchChannel  = false, // Allow switchin gchannel on forward
  .espnowFilterWeakSignal      = -55,   // Filter onm RSSI (zero is no rssi filtering)
};

#define CONTROL_KEY_GPIO GPIO_NUM_0

typedef enum { APP_ESPNOW_CTRL_INIT, APP_ESPNOW_CTRL_BOUND, APP_ESPNOW_CTRL_MAX } app_espnow_ctrl_status_t;

static app_espnow_ctrl_status_t s_espnow_ctrl_status = APP_ESPNOW_CTRL_INIT;

#define WIFI_PROV_KEY_GPIO GPIO_NUM_0

// Handle for nvs storage
nvs_handle_t g_nvsHandle = 0;

///////////////////////////////////////////////////////////////////////////////
// setPersistenDefaults
//

static void
setPersistenDefaults(void)
{
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
  vscp_fwhlp_hex2bin(g_persistent.pmk, 16, VSCP_DEFAULT_KEY16);

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
  rv = nvs_set_u32(g_nvsHandle, "boot_counter", g_persistent.bootCnt);
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

  // Long Range
  rv = nvs_get_u8(g_nvsHandle, "longr", &val);
  if (ESP_OK != rv) {
    val = (uint8_t) g_persistent.espnowLongRange;
    rv  = nvs_set_u8(g_nvsHandle, "longr", g_persistent.espnowLongRange);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update espnow long range");
    }
  }
  else {
    g_persistent.espnowLongRange = (bool) val;
  }

  // Channel
  rv = nvs_get_u8(g_nvsHandle, "channel", &g_persistent.espnowChannel);
  if (ESP_OK != rv) {
    rv = nvs_set_u8(g_nvsHandle, "channel", g_persistent.espnowChannel);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update espnow channel");
    }
  }

  // Default ttl
  rv = nvs_get_u8(g_nvsHandle, "ttl", &g_persistent.espnowTtl);
  if (ESP_OK != rv) {
    rv = nvs_set_u8(g_nvsHandle, "ttl", g_persistent.espnowTtl);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update espnow ttl");
    }
  }

  // Forward
  rv = nvs_get_u8(g_nvsHandle, "fw", &val);
  if (ESP_OK != rv) {
    val = (uint8_t) g_persistent.espnowForwardEnable;
    rv  = nvs_set_u8(g_nvsHandle, "fw", g_persistent.espnowForwardEnable);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update espnow forward");
    }
  }
  else {
    g_persistent.espnowForwardEnable = (bool) val;
  }

  // Adj filter channel
  rv = nvs_get_u8(g_nvsHandle, "adjchfilt", &val);
  if (ESP_OK != rv) {
    val = (uint8_t) g_persistent.espnowFilterAdjacentChannel;
    rv  = nvs_set_u8(g_nvsHandle, "adjchfilt", g_persistent.espnowFilterAdjacentChannel);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update espnow adj channel filter");
    }
  }
  else {
    g_persistent.espnowFilterAdjacentChannel = (bool) val;
  }

  // Allow switching channel on forward
  rv = nvs_get_u8(g_nvsHandle, "swchf", &val);
  if (ESP_OK != rv) {
    val = (uint8_t) g_persistent.espnowForwardSwitchChannel;
    rv  = nvs_set_u8(g_nvsHandle, "swchf", g_persistent.espnowForwardSwitchChannel);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update espnow switch channel on forward");
    }
  }
  else {
    g_persistent.espnowFilterAdjacentChannel = (bool) val;
  }

  // RSSI limit
  rv = nvs_get_i8(g_nvsHandle, "rssi", &g_persistent.espnowFilterWeakSignal);
  if (ESP_OK != rv) {
    rv = nvs_set_u8(g_nvsHandle, "rssi", g_persistent.espnowFilterWeakSignal);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update espnow RSSI");
    }
  }

  rv = nvs_commit(g_nvsHandle);
  if (rv != ESP_OK) {
    ESP_LOGI(TAG, "Failed to commit updates to nvs\n");
  }

  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// app_wifi_event_handler
//

static void
app_wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    // // app_led_set_color(255, 0, 0);
    ESP_LOGI(TAG, "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    // s_wifi_prov_status = APP_WIFI_PROV_SUCCESS;
    //  app_led_set_color(0, 255, 0);
    ESP_LOGI(TAG, "*********************************************");
  }
}

///////////////////////////////////////////////////////////////////////////////
// app_espnow_event_handler
//

static void
app_espnow_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
  if (base != ESP_EVENT_ESPNOW) {
    return;
  }

  switch (id) {
    case ESP_EVENT_ESPNOW_CTRL_BIND: {
      espnow_ctrl_bind_info_t *info = (espnow_ctrl_bind_info_t *) event_data;
      ESP_LOGI(TAG, "bind, uuid: " MACSTR ", initiator_type: %d", MAC2STR(info->mac), info->initiator_attribute);

      // app_led_set_color(0, 255, 0);
      break;
    }

    case ESP_EVENT_ESPNOW_CTRL_UNBIND: {
      espnow_ctrl_bind_info_t *info = (espnow_ctrl_bind_info_t *) event_data;
      ESP_LOGI(TAG, "unbind, uuid: " MACSTR ", initiator_type: %d", MAC2STR(info->mac), info->initiator_attribute);

      // app_led_set_color(255, 0, 0);
      break;
    }

    default:
      break;
  }
}

///////////////////////////////////////////////////////////////////////////////
// app_responder_ctrl_data_cb
//

static void
app_responder_ctrl_data_cb(espnow_attribute_t initiator_attribute,
                           espnow_attribute_t responder_attribute,
                           uint32_t status)
{
  ESP_LOGI(TAG,
           "app_responder_ctrl_data_cb, initiator_attribute: %d, responder_attribute: %d, value: %" PRIu32 "",
           initiator_attribute,
           responder_attribute,
           status);

  if (status) {
    // app_led_set_color(255, 255, 255);
  }
  else {
    // app_led_set_color(0, 0, 0);
  }
}

/////////////////////////////////////////////////////////////////////////////
// app_espnow_prov_initiator_recv_cb
//

static esp_err_t
app_espnow_prov_initiator_recv_cb(uint8_t *src_addr, void *data, size_t size, wifi_pkt_rx_ctrl_t *rx_ctrl)
{
  esp_err_t ret;

  ESP_PARAM_CHECK(src_addr);
  ESP_PARAM_CHECK(data);
  ESP_PARAM_CHECK(size);
  ESP_PARAM_CHECK(rx_ctrl);

  ESP_LOGI(TAG,
           "Provisioning data received. size=%zd (%d) channel=%d",
           size,
           size - sizeof(espnow_prov_wifi_t),
           rx_ctrl->channel);

  espnow_prov_wifi_t *wifi_config = (espnow_prov_wifi_t *) data;
  if (wifi_config->custom_size != sizeof(vscp_espnow_prov_data_t)) {
    ESP_LOGE(TAG,
             "Provisioning custom data have wrong size %zd (should be %zd)",
             wifi_config->custom_size,
             sizeof(vscp_espnow_prov_data_t));
  }
  vscp_espnow_prov_data_t *prov_data = (vscp_espnow_prov_data_t *) wifi_config->custom_data;
  ESP_LOG_BUFFER_HEX(TAG, data, size);
  ESP_LOG_BUFFER_HEX(TAG, wifi_config->custom_data, sizeof(vscp_espnow_prov_data_t));

  g_persistent.espnowChannel = prov_data->espnowChannel;

  // Save channel to persistent storage
  ret = nvs_set_u8(g_nvsHandle, "channel", g_persistent.espnowChannel);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to update espnow channel %X", ret);
  }

  ESP_LOGI(TAG,
           "MAC: " MACSTR ", Channel: %d, RSSI: %d, queue: %d, ttl: %d, forward: %d, wifi_mode: %d, ssid: %s, "
           "password: %s, token: %s",
           MAC2STR(src_addr),
           rx_ctrl->channel,
           rx_ctrl->rssi,
           prov_data->espnowSizeQueue,
           prov_data->espnowTtl,
           prov_data->espnowForwardEnable,
           wifi_config->mode,
           wifi_config->sta.ssid,
           wifi_config->sta.password,
           wifi_config->token);

  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, (wifi_config_t *) &wifi_config->sta));
  // ESP_ERROR_CHECK(esp_wifi_connect());
  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// app_espnow_prov_initiator_init
//

static void
app_espnow_prov_initiator_init(void *arg)
{
  esp_err_t ret                          = ESP_OK;
  wifi_pkt_rx_ctrl_t rx_ctrl             = { 0 };
  espnow_prov_initiator_t initiator_info = {
    .product_id = VSCP_PROJDEF_PROVISIONING_PRODUCT_ID,
  };
  espnow_addr_t responder_addr           = { 0 };
  espnow_prov_responder_t responder_info = { 0 };

  for (;;) {

    uint8_t key_info[APP_KEY_LEN];

    // Security key is not ready
    if (espnow_get_key(key_info) != ESP_OK) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    ret = espnow_prov_initiator_scan(responder_addr, &responder_info, &rx_ctrl, pdMS_TO_TICKS(3 * 1000));
    ESP_ERROR_CONTINUE(ret != ESP_OK, "");

    ESP_LOGI(TAG,
             "MAC: " MACSTR ", Channel: %d, RSSI: %d, Product_id: %s, Device Name: %s",
             MAC2STR(responder_addr),
             rx_ctrl.channel,
             rx_ctrl.rssi,
             responder_info.product_id,
             responder_info.device_name);

    ret = espnow_prov_initiator_send(responder_addr,
                                     &initiator_info,
                                     app_espnow_prov_initiator_recv_cb,
                                     pdMS_TO_TICKS(3 * 1000));
    ESP_ERROR_CONTINUE(ret != ESP_OK, "<%s> espnow_prov_responder_add", esp_err_to_name(ret));

    break;
  }

  ESP_LOGI(TAG, "provisioning initiator exit");
  vTaskDelete(NULL);
  s_prov_task = NULL;
}

static esp_err_t
app_espnow_prov_responder_recv_cb(uint8_t *src_addr, void *data, size_t size, wifi_pkt_rx_ctrl_t *rx_ctrl)
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

esp_err_t
app_espnow_prov_beacon_start(int32_t sec)
{
  esp_err_t ret;

  /* clang-format off */
  wifi_config_t wifi_config              = { 0 };
  espnow_prov_wifi_t prov_wifi_config    = { 0 };
  espnow_prov_responder_t responder_info = { 
      .product_id = "responder_test" 
  };
  /* clang-format on */

  // esp_err_t ret = esp_wifi_get_config(WIFI_IF_STA, &wifi_config);
  // if (ret != ESP_OK) {
  //   ESP_LOGE(TAG, "Get wifi config failed, %d", ret);
  //   return ret;
  // }

  // if (strlen((const char *) wifi_config.sta.ssid) == 0) {
  //   ESP_LOGW(TAG, "WiFi not configured");
  //   return ESP_FAIL;
  // }

  // memcpy(&prov_wifi_config.sta, &wifi_config.sta, sizeof(wifi_sta_config_t));

  ret = espnow_prov_responder_start(&responder_info,
                                    pdMS_TO_TICKS(sec * 1000),
                                    &prov_wifi_config,
                                    app_espnow_prov_responder_recv_cb);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "espnow_prov_responder_start failed, %d", ret);
  }

  return ret;
}

static void
app_espnow_prov_responder_task(void *arg)
{
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits =
      xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
      ESP_LOGD(TAG, "connected to ap");
    }
    else if (bits & WIFI_FAIL_BIT) {
      ESP_LOGD(TAG, "Failed to connect to ap");
      continue;
    }
    else {
      ESP_LOGE(TAG, "UNEXPECTED EVENT");
      continue;
    }

    if (app_espnow_prov_beacon_start(30) != ESP_OK) {
      break;
    }

    // Clear WIFI_CONNECTED_BIT to restart responder when reconnection
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }

  ESP_LOGI(TAG, "provisioning responder exit");
  vTaskDelete(NULL);
}

///////////////////////////////////////////////////////////////////////////////
// app_start_beacon_press_cb
//

static void
app_start_beacon_press_cb(void *arg, void *usr_data)
{
  ESP_ERROR_CHECK(!(BUTTON_SINGLE_CLICK == iot_button_get_event(arg)));
  ESP_LOGI(TAG, "Starting provision beacon");
  // app_espnow_responder();
  // app_espnow_prov_beacon_start(30);
  xTaskCreate(app_espnow_prov_initiator_init, "PROV_init", 3072, NULL, tskIDLE_PRIORITY + 1, &s_prov_task);
}

///////////////////////////////////////////////////////////////////////////////
// app_wifi_prov_start_press_cb
//

static void
app_wifi_prov_start_press_cb(void *arg, void *usr_data)
{
  ESP_ERROR_CHECK(!(BUTTON_DOUBLE_CLICK == iot_button_get_event(arg)));

  // app_espnow_prov_initiator_init()

  // ESP_LOGI(TAG, "Unbound the device");
  // espnow_erase_key(); // Unbound device

  // esp_wifi_disconnect();
  // esp_restart();

  if (s_wifi_prov_status == APP_WIFI_PROV_INIT) {

    if (!s_prov_task) {
      ESP_LOGI(TAG, "Start WiFi provisioning over ESP-NOW on responder");
      xTaskCreate(app_espnow_prov_initiator_init, "PROV_init", 3072, NULL, tskIDLE_PRIORITY + 1, &s_prov_task);
    }

    s_wifi_prov_status = APP_WIFI_PROV_START;

    // app_led_set_color(255, 255, 255);
  }
  else if (s_wifi_prov_status == APP_WIFI_PROV_START) {
    ESP_LOGI(TAG, "WiFi provisioning is already started");
  }
  else {
    ESP_LOGI(TAG, "WiFi is already provisioned");
  }
}

///////////////////////////////////////////////////////////////////////////////
// app_wifi_prov_reset_press_cb
//

static void
app_wifi_prov_reset_press_cb(void *arg, void *usr_data)
{
  ESP_ERROR_CHECK(!(BUTTON_LONG_PRESS_START == iot_button_get_event(arg)));

  espnow_erase_key();     // Unbound device

  setPersistenDefaults(); // set defaults

  // Erase all settings
  nvs_flash_erase_partition("config");

  // Disconnect from wifi
  esp_wifi_disconnect();

  // Restart system (set defaults)
  esp_restart();
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_data_cb
//

static void
vscp_espnow_data_cb(uint8_t *src_addr, void *data, size_t size, wifi_pkt_rx_ctrl_t *rx_ctrl)
{
  esp_err_t ret;
  uint8_t ch                = 0;
  wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
  if (ESP_OK != (ret = esp_wifi_get_channel(&ch, &second))) {
    ESP_LOGE(TAG, "Failed to get wifi channel, rv = %X", ret);
  }
  ESP_LOGI(TAG, "|| esp-now data received. len=%zd ch=%d (%d) " MACSTR, size, MAC2STR(src_addr), ch, second);
}

//-----------------------------------------------------------------------------
//                                  ESPNOW
//-----------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////
// app_log_read_task
//

/**
 * @brief http_client task creation
 *      You can modify the http address according to your needs,
 *      parameters CONFIG_FLASH_LOG_POST_URL if change.and parameters
 *      transport_type if change.
 */

// // * * * BETA * * *

// static void
// app_log_read_task(void *arg)
// {
//   esp_err_t ret                   = ESP_OK;
//   char *log_data                  = ESP_MALLOC(ESPNOW_DATA_LEN);
//   size_t log_size                 = 0;
//   esp_http_client_handle_t client = NULL;
//   esp_http_client_config_t config = {
//     .url            = "192.168.1.7/espnow-debug", // TODO CONFIG_APP_FLASH_LOG_POST_URL,
//     .transport_type = HTTP_TRANSPORT_UNKNOWN,
//   };

//   client = esp_http_client_init(&config);
//   esp_http_client_set_method(client, HTTP_METHOD_POST);

//   for (;;) {
//     vTaskDelay(pdMS_TO_TICKS(1000));

//     /*
//       Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
//       number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above)
//     */
//     EventBits_t bits =
//       xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT , pdFALSE, pdFALSE, portMAX_DELAY);

//     /*
//       xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
//       happened.
//     */
//     if (bits & WIFI_CONNECTED_BIT) {
//       ESP_LOGD(TAG, "connected to ap");
//     }
//     else if (bits & WIFI_FAIL_BIT) {
//       ESP_LOGD(TAG, "Failed to connect to ap");
//       continue;
//     }
//     else {
//       ESP_LOGE(TAG, "UNEXPECTED EVENT");
//       continue;
//     }

//     log_size = espnow_log_flash_size();
//     ESP_ERROR_CONTINUE(!log_size, "");

//     ret = esp_http_client_open(client, log_size);
//     if (ret != ESP_OK) {
//       ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(ret));
//       esp_http_client_close(client);
//       continue;
//     }

//     /**
//      * @brief Read the log data and transfer the data out via http server
//      */
//     for (size_t size = MIN(ESPNOW_DATA_LEN, log_size); size > 0 && espnow_log_flash_read(log_data, &size) == ESP_OK;
//          log_size -= size, size = MIN(ESPNOW_DATA_LEN, log_size)) {
//       ret = esp_http_client_write(client, log_data, size);
//       ESP_ERROR_BREAK(ret < 0, "<%s> Failed to write HTTP data", esp_err_to_name(ret));
//     }

//     esp_http_client_close(client);
//   }

//   ESP_FREE(log_data);
//   esp_http_client_cleanup(client);
//   vTaskDelete(NULL);
// }

///////////////////////////////////////////////////////////////////////////////
// app_espnow_initiator
//

// void
// app_espnow_initiator()
// {
//   if (!s_sec_task) {
//     xTaskCreate(app_espnow_initiator_sec_task, "sec", 3072, NULL, tskIDLE_PRIORITY + 1, &s_sec_task);
//   }

//   /* clang-format off */
//   espnow_console_config_t console_config = {
//     .monitor_command.uart = true,
//     .store_history        = {
//       .base_path = "/spiffs",
//       .partition_label = "console" },
//   };
//   /* clang-format on */

//   espnow_console_init(&console_config);
//   espnow_console_commands_register();

//   // Receive esp-now data from other device
//   // espnow_set_config_for_data_type(ESPNOW_DATA_TYPE_DEBUG_LOG, true, app_espnow_debug_recv_process);

//   // OTA with command
//   // ...

//   xTaskCreate(app_espnow_prov_responder_task, "prov_resp", 1024 * 3, NULL, tskIDLE_PRIORITY + 1, NULL);
// }

///////////////////////////////////////////////////////////////////////////////
// app_espnow_start
//

// static void
// app_espnow_start()
// {
//   const char *pop_data = VSCP_PROJDEF_ESPNOW_SESSION_POP;
//   uint8_t key_info[APP_KEY_LEN];

//   // If espnow_set_key succeed, sending and receiving will be in security mode
//   if (espnow_get_key(key_info) == ESP_OK) {
//     espnow_set_key(key_info);
//   }

//   // If responder handshake with initiator succeed, espnow_set_key will be executed again.
//   espnow_sec_responder_start(pop_data);

//   // Initialize time synchronization
//   espnow_timesync_start();

//   /* clang-format off */
//   espnow_console_config_t console_config = {
//     .monitor_command.uart   = true,
//     .monitor_command.espnow = true,
//   };
//   espnow_log_config_t log_config = {
//     .log_level_uart   = ESP_LOG_INFO,
//     .log_level_espnow = ESP_LOG_INFO,
//     .log_level_flash  = ESP_LOG_INFO,
//   };
//   /* clang-format on */

//   espnow_console_init(&console_config);
//   espnow_console_commands_register();
//   espnow_log_init(&log_config);
//   esp_log_level_set("*", ESP_LOG_INFO);

//   espnow_ota_config_t ota_config = {
//     .skip_version_check       = true,
//     .progress_report_interval = 10,
//   };

//   espnow_ota_responder_start(&ota_config);
// }

//-----------------------------------------------------------------------------
//                                  Wifi
//-----------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////
// app_wifi_init
//

static void
app_wifi_init()
{
  ESP_ERROR_CHECK(esp_netif_init());

  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_ERROR_CHECK(esp_wifi_start());
}

///////////////////////////////////////////////////////////////////////////////
// app_espnow_responder
//

void
app_espnow_responder()
{
  const char *pop_data = VSCP_PROJDEF_ESPNOW_SESSION_POP;
  uint8_t key_info[APP_KEY_LEN];

  // If espnow_set_key succeed, sending and receiving will be in security mode
  if (espnow_get_key(key_info) == ESP_OK) {
    espnow_set_key(key_info);
  }

  ESP_LOGI(TAG, "Starting sec");

  // If responder handshake with initiator succeed, espnow_set_key will be executed again.
  espnow_sec_responder_start(pop_data);

  ESP_LOGI(TAG, "Starting time sync");

  // Initialize time synchronization
  espnow_timesync_start();

  ESP_LOGI(TAG, "Starting log");

  espnow_console_config_t console_config = {
    .monitor_command.uart   = true,
    .monitor_command.espnow = true,
  };

  espnow_log_config_t log_config = {
    .log_level_uart   = ESP_LOG_INFO,
    .log_level_espnow = ESP_LOG_INFO,
    .log_level_flash  = ESP_LOG_INFO,
  };

  espnow_console_init(&console_config);
  espnow_console_commands_register();
  espnow_log_init(&log_config);
  esp_log_level_set("*", ESP_LOG_INFO);

  espnow_ota_config_t ota_config = {
    .skip_version_check       = true,
    .progress_report_interval = 10,
  };

  ESP_LOGI(TAG, "Starting ota");
  espnow_ota_responder_start(&ota_config);
}

///////////////////////////////////////////////////////////////////////////////
// app_main
//

void
app_main()
{
  esp_err_t ret;

  s_wifi_event_group = xEventGroupCreate();

  espnow_storage_init();
  app_wifi_init();

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

  // Set the provisioned channel
  esp_wifi_set_channel(g_persistent.espnowChannel, WIFI_SECOND_CHAN_NONE);

  espnow_config_t espnow_config = ESPNOW_INIT_CONFIG_DEFAULT();
  espnow_config.qsize           = CONFIG_APP_ESPNOW_QUEUE_SIZE;
  espnow_config.sec_enable      = 1;
  espnow_init(&espnow_config);

  button_config_t button_config = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num = WIFI_PROV_KEY_GPIO,
            .active_level = 0,
        },
    };

  button_handle_t button_handle = iot_button_create(&button_config);

  iot_button_register_cb(button_handle, BUTTON_SINGLE_CLICK, app_start_beacon_press_cb, NULL);
  iot_button_register_cb(button_handle, BUTTON_DOUBLE_CLICK, app_wifi_prov_start_press_cb, NULL);
  iot_button_register_cb(button_handle, BUTTON_LONG_PRESS_START, app_wifi_prov_reset_press_cb, NULL);

  // Set default primary key
  vscp_fwhlp_hex2bin(g_persistent.pmk, 16, VSCP_DEFAULT_KEY16);
  memcpy((uint8_t *) espnow_config.pmk, g_persistent.pmk, 16);
  espnow_config.qsize          = g_persistent.queueSize; // CONFIG_APP_ESPNOW_QUEUE_SIZE;
  espnow_config.sec_enable     = 1;
  espnow_config.forward_enable = 1;

  uint8_t key_info[APP_KEY_LEN];
  if (espnow_get_key(key_info) == ESP_OK) {
    espnow_set_key(key_info);
  }
  else {
    esp_fill_random(key_info, 32);
    // vscp_fwhlp_hex2bin(key, 32, "a3ea2c7811f37e29ed6a677da9d9bdedef64f3541dd0c27dd17ea6c127027efe");
    espnow_set_key(key_info);
  }

  esp_event_handler_register(ESP_EVENT_ESPNOW, ESP_EVENT_ANY_ID, app_espnow_event_handler, NULL);
  esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, app_wifi_event_handler, NULL);
  esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, app_wifi_event_handler, NULL);

  esp_event_handler_register(ESP_EVENT_ESPNOW, ESP_EVENT_ESPNOW_CTRL_BIND, app_espnow_event_handler, NULL);
  esp_event_handler_register(ESP_EVENT_ESPNOW, ESP_EVENT_ESPNOW_CTRL_UNBIND, app_espnow_event_handler, NULL);

  // Controller bind
  // ESP_ERROR_CHECK(espnow_ctrl_responder_bind(30 * 1000, -55, NULL));
  // espnow_ctrl_responder_data(app_responder_ctrl_data_cb);

  app_espnow_responder();

  // Setup VSCP esp-now

  espnow_set_config_for_data_type(ESPNOW_DATA_TYPE_DATA, true, vscp_espnow_data_cb);
  if (ESP_OK != ret) {
    ESP_LOGE(TAG, "Failed to set VSCP event callback");
  }

  // Start heartbeat task vscp_heartbeat_task
  xTaskCreate(&vscp_espnow_heartbeat_task, "vscp_espnow_heartbeat_task", 1024 * 3, NULL, tskIDLE_PRIORITY + 1, NULL);

  esp_wifi_get_mac(ESP_IF_WIFI_STA, ESPNOW_ADDR_SELF);
  ESP_LOGI(TAG, "mac: " MACSTR ", version: %d", MAC2STR(ESPNOW_ADDR_SELF), ESPNOW_VERSION);

  while (1) {
    // esp_task_wdt_reset();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    taskYIELD();
  }
}
