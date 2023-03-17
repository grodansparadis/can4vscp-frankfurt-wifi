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

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include <nvs_flash.h>
#include "esp_mac.h"

#include "espnow.h"
#include "espnow_utils.h"
#include "espnow_ctrl.h"
#include "iot_button.h"

#ifdef VSCP_PROJDEF_LED_STRIP
#include "led_strip.h"
#endif

#include <wifi_provisioning/manager.h>

#include "responder.h"

#include "led_indicator.h"

#include <vscp.h>
#include <vscp-firmware-helper.h>
#include <vscp_class.h>
#include <vscp_type.h>

#include "beta.h"

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

///////////////////////////////////////////////////////////
//                   P E R S I S T A N T
///////////////////////////////////////////////////////////

// Set default configuration

node_persistent_config_t g_persistent = {

  // General
  .nodeName   = "Alpha Node Beta",
  .pmk        = { 0 },
  .nodeGuid   = { 0 }, // GUID for unit
  .startDelay = 2,
  .bootCnt    = 0,
  .queueSize  = CONFIG_APP_ESPNOW_QUEUE_SIZE,

  // espnow
  .enLongRange             = false,
  .enChannel               = 0, // Use wifi channel
  .enTtl                   = 32,
  .enSizeQueue             = 32,    // Size fo input queue
  .enForwardEnable         = true,  // Forward when packets are received
  .enFilterAdjacentChannel = true,  // Don't receive if from other channel
  .enForwardSwitchChannel  = false, // Allow switchin gchannel on forward
  .enFilterWeakSignal      = -100,  // Filter onm RSSI (zero is no rssi filtering)
};

#ifdef VSCP_PROJDEF_LED_STRIP
#if !CONFIG_IDF_TARGET_ESP32C2
static led_strip_handle_t g_strip_handle = NULL;
#endif
#endif

#define CONTROL_KEY_GPIO GPIO_NUM_0

typedef enum { APP_ESPNOW_CTRL_INIT, APP_ESPNOW_CTRL_BOUND, APP_ESPNOW_CTRL_MAX } app_espnow_ctrl_status_t;

static app_espnow_ctrl_status_t s_espnow_ctrl_status = APP_ESPNOW_CTRL_INIT;

#define WIFI_PROV_KEY_GPIO GPIO_NUM_0

// Handle for nvs storage
nvs_handle_t g_nvsHandle = 0;

typedef enum {
  APP_WIFI_PROV_INIT,
  APP_WIFI_PROV_START,
  APP_WIFI_PROV_SUCCESS,
  APP_WIFI_PROV_MAX
} app_wifi_prov_status_t;

static app_wifi_prov_status_t s_wifi_prov_status = APP_WIFI_PROV_INIT;

///////////////////////////////////////////////////////////////////////////////
// app_led_init
//

static void
app_led_init(void)
{
#ifdef VSCP_PROJDEF_LED_STRIP
#if CONFIG_IDF_TARGET_ESP32C2
  gpio_reset_pin(LED_RED_GPIO);
  gpio_reset_pin(LED_GREEN_GPIO);
  gpio_reset_pin(LED_BLUE_GPIO);

  /* Set the GPIO as a push/pull output */
  gpio_set_direction(LED_RED_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED_GREEN_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED_BLUE_GPIO, GPIO_MODE_OUTPUT);

  gpio_set_level(LED_RED_GPIO, 1);
  gpio_set_level(LED_GREEN_GPIO, 1);
  gpio_set_level(LED_BLUE_GPIO, 1);
#else
  /* LED strip initialization with the GPIO and pixels number*/
  led_strip_config_t strip_config = {
    .strip_gpio_num = LED_STRIP_GPIO,
    .max_leds       = 1, // at least one LED on board
  };
  led_strip_rmt_config_t rmt_config = {
    .resolution_hz = 10 * 1000 * 1000, // 10MHz
  };
  ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &g_strip_handle));
  /* Set all LED off to clear all pixels */
  led_strip_clear(g_strip_handle);
#endif
#endif
}

///////////////////////////////////////////////////////////////////////////////
// app_wifi_event_handler
//

static void
app_wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    // // app_led_set_color(255, 0, 0);
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
#if defined(CONFIG_APP_WIFI_PROVISION) || defined(CONFIG_APP_ESPNOW_PROVISION)
    s_wifi_prov_status = APP_WIFI_PROV_SUCCESS;
#endif

    // app_led_set_color(0, 255, 0);
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

///////////////////////////////////////////////////////////////////////////////
// app_control_responder_init
//

static void
app_control_responder_init(void)
{
  esp_event_handler_register(ESP_EVENT_ESPNOW, ESP_EVENT_ESPNOW_CTRL_BIND, app_espnow_event_handler, NULL);
  esp_event_handler_register(ESP_EVENT_ESPNOW, ESP_EVENT_ESPNOW_CTRL_UNBIND, app_espnow_event_handler, NULL);

  ESP_ERROR_CHECK(espnow_ctrl_responder_bind(30 * 1000, -55, NULL));
  espnow_ctrl_responder_data(app_responder_ctrl_data_cb);
}

///////////////////////////////////////////////////////////////////////////////
// app_wifi_prov_start_press_cb
//

static void
app_wifi_prov_start_press_cb(void *arg, void *usr_data)
{
  ESP_ERROR_CHECK(!(BUTTON_DOUBLE_CLICK == iot_button_get_event(arg)));

  if (s_wifi_prov_status == APP_WIFI_PROV_INIT) {

    ESP_LOGI(TAG, "Start WiFi provisioning over ESP-NOW on responder");

    app_espnow_prov_responder_start();

    s_wifi_prov_status = APP_WIFI_PROV_START;

    // app_led_set_color(255, 255, 255);
  }
  else if (s_wifi_prov_status == APP_WIFI_PROV_START) {
    ESP_LOGI(TAG, "WiFi provisioning is started");
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

  ESP_LOGI(TAG, "Reset WiFi provisioning information and restart");

  wifi_prov_mgr_reset_provisioning();

  esp_wifi_disconnect();
  esp_restart();
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_data_cb
//

static void
vscp_espnow_data_cb(uint8_t *src_addr, void *data, size_t size, wifi_pkt_rx_ctrl_t *rx_ctrl)
{
  ESP_LOGI(TAG, "|| esp-now data received. len=%zd " MACSTR, size, MAC2STR(src_addr));
}

///////////////////////////////////////////////////////////////////////////////
// app_wifi_prov_button_init
//

static void
app_wifi_prov_button_init(void)
{
  button_config_t button_config = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num = WIFI_PROV_KEY_GPIO,
            .active_level = 0,
        },
    };

  esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, app_wifi_event_handler, NULL);
  esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, app_wifi_event_handler, NULL);

  button_handle_t button_handle = iot_button_create(&button_config);

  iot_button_register_cb(button_handle, BUTTON_DOUBLE_CLICK, app_wifi_prov_start_press_cb, NULL);
  iot_button_register_cb(button_handle, BUTTON_LONG_PRESS_START, app_wifi_prov_reset_press_cb, NULL);
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
    const char key[] = { VSCP_DEFAULT_KEY16 };
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

  // Long Range
  rv = nvs_get_u8(g_nvsHandle, "longr", &val);
  if (ESP_OK != rv) {
    val = (uint8_t) g_persistent.enLongRange;
    rv  = nvs_set_u8(g_nvsHandle, "longr", g_persistent.enLongRange);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update espnow long range");
    }
  }
  else {
    g_persistent.enLongRange = (bool) val;
  }

  // Channel
  rv = nvs_get_u8(g_nvsHandle, "channel", &g_persistent.enChannel);
  if (ESP_OK != rv) {
    rv = nvs_set_u8(g_nvsHandle, "channel", g_persistent.enChannel);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update espnow channel");
    }
  }

  // Default ttl
  rv = nvs_get_u8(g_nvsHandle, "ttl", &g_persistent.enTtl);
  if (ESP_OK != rv) {
    rv = nvs_set_u8(g_nvsHandle, "ttl", g_persistent.enTtl);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update espnow ttl");
    }
  }

  // Forward
  rv = nvs_get_u8(g_nvsHandle, "fw", &val);
  if (ESP_OK != rv) {
    val = (uint8_t) g_persistent.enForwardEnable;
    rv  = nvs_set_u8(g_nvsHandle, "fw", g_persistent.enForwardEnable);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update espnow forward");
    }
  }
  else {
    g_persistent.enForwardEnable = (bool) val;
  }

  // Adj filter channel
  rv = nvs_get_u8(g_nvsHandle, "adjchfilt", &val);
  if (ESP_OK != rv) {
    val = (uint8_t) g_persistent.enFilterAdjacentChannel;
    rv  = nvs_set_u8(g_nvsHandle, "adjchfilt", g_persistent.enFilterAdjacentChannel);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update espnow adj channel filter");
    }
  }
  else {
    g_persistent.enFilterAdjacentChannel = (bool) val;
  }

  // Allow switching channel on forward
  rv = nvs_get_u8(g_nvsHandle, "swchf", &val);
  if (ESP_OK != rv) {
    val = (uint8_t) g_persistent.enForwardSwitchChannel;
    rv  = nvs_set_u8(g_nvsHandle, "swchf", g_persistent.enForwardSwitchChannel);
    if (rv != ESP_OK) {
      ESP_LOGE(TAG, "Failed to update espnow switch channel on forward");
    }
  }
  else {
    g_persistent.enFilterAdjacentChannel = (bool) val;
  }

  // RSSI limit
  rv = nvs_get_i8(g_nvsHandle, "rssi", &g_persistent.enFilterWeakSignal);
  if (ESP_OK != rv) {
    rv = nvs_set_u8(g_nvsHandle, "rssi", g_persistent.enFilterWeakSignal);
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
  ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
}
///////////////////////////////////////////////////////////////////////////////
// app_main
//

void
app_main()
{
  esp_err_t ret;

  espnow_storage_init();
  app_wifi_init();

  espnow_config_t espnow_config = ESPNOW_INIT_CONFIG_DEFAULT();
  espnow_config.qsize           = CONFIG_APP_ESPNOW_QUEUE_SIZE;
  espnow_config.sec_enable      = 1;

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

  // Set default primary key
  vscp_fwhlp_hex2bin(g_persistent.pmk, 16, VSCP_DEFAULT_KEY16);
  memcpy(espnow_config.pmk, g_persistent.pmk, 16);
  espnow_config.qsize          = g_persistent.queueSize; // CONFIG_APP_ESPNOW_QUEUE_SIZE;
  espnow_config.sec_enable     = 1;
  espnow_config.forward_enable = 1;

  espnow_init(&espnow_config);

  uint8_t key[32];
  //esp_fill_random(key, 32);
  vscp_fwhlp_hex2bin(key, 32, "a3ea2c7811f37e29ed6a677da9d9bdedef64f3541dd0c27dd17ea6c127027efe");
  espnow_set_key(key);

  app_led_init();
  app_wifi_prov_button_init();
  app_espnow_responder_register();
  app_control_responder_init();
  app_espnow_responder();

  // Setup VSCP esp-now

  espnow_set_config_for_data_type(ESPNOW_DATA_TYPE_DATA, true, vscp_espnow_data_cb);
  if (ESP_OK != ret) {
    ESP_LOGE(TAG, "Failed to set VSCP event callback");
  }

  // Start heartbeat task vscp_heartbeat_task
  xTaskCreate(&vscp_espnow_heartbeat_task, "vscp_espnow_heartbeat_task", 1024 * 3, NULL, 1, NULL);

  //esp_wifi_get_mac(ESP_IF_WIFI_STA, ESPNOW_ADDR_SELF);
  //ESP_LOGI(TAG, "mac: " MACSTR ", version: %d", MAC2STR(ESPNOW_ADDR_SELF), ESPNOW_VERSION);

  while (1) {
    // esp_task_wdt_reset();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    taskYIELD();
  }
}
