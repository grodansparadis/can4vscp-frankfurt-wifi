/* Solution Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"

#include "esp_mac.h"

#include "espnow.h"
#include "espnow_utils.h"
#include "espnow_ctrl.h"
#include "iot_button.h"

#include "led_strip.h"

#include <wifi_provisioning/manager.h>

#include "responder.h"

static const char *TAG = "app";


// All the default GPIOs are based on ESP32 series DevKitC boards, for other boards, please modify them accordingly.
#ifdef CONFIG_IDF_TARGET_ESP32C2
#define LED_RED_GPIO          GPIO_NUM_0
#define LED_GREEN_GPIO        GPIO_NUM_1
#define LED_BLUE_GPIO         GPIO_NUM_8
#elif CONFIG_IDF_TARGET_ESP32C3
#define LED_STRIP_GPIO        GPIO_NUM_8
#elif CONFIG_IDF_TARGET_ESP32
// There is not LED module in ESP32 DevKitC board, so you need to connect one by yourself.
#define LED_STRIP_GPIO        GPIO_NUM_18
#elif CONFIG_IDF_TARGET_ESP32S2
#define LED_STRIP_GPIO        GPIO_NUM_18
#elif CONFIG_IDF_TARGET_ESP32S3
// For old version board, the number is 48.
#define LED_STRIP_GPIO        GPIO_NUM_38
#endif

#if !CONFIG_IDF_TARGET_ESP32C2
static led_strip_handle_t g_strip_handle = NULL;
#endif

#define CONTROL_KEY_GPIO        GPIO_NUM_0

typedef enum {
    APP_ESPNOW_CTRL_INIT,
    APP_ESPNOW_CTRL_BOUND,
    APP_ESPNOW_CTRL_MAX
} app_espnow_ctrl_status_t;

static app_espnow_ctrl_status_t s_espnow_ctrl_status = APP_ESPNOW_CTRL_INIT;

#define WIFI_PROV_KEY_GPIO      GPIO_NUM_0

typedef enum {
    APP_WIFI_PROV_INIT,
    APP_WIFI_PROV_START,
    APP_WIFI_PROV_SUCCESS,
    APP_WIFI_PROV_MAX
} app_wifi_prov_status_t;

static app_wifi_prov_status_t s_wifi_prov_status = APP_WIFI_PROV_INIT;

///////////////////////////////////////////////////////////////////////////////
// app_wifi_event_handler
//

static void app_wifi_event_handler(void *arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // // app_led_set_color(255, 0, 0);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
#if defined(CONFIG_APP_WIFI_PROVISION) || defined(CONFIG_APP_ESPNOW_PROVISION)
        s_wifi_prov_status = APP_WIFI_PROV_SUCCESS;
#endif

        // app_led_set_color(0, 255, 0);
    }
}


///////////////////////////////////////////////////////////////////////////////
// app_espnow_event_handler
//

static void app_espnow_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    if (base != ESP_EVENT_ESPNOW) {
        return;
    }

    switch (id) {
    case ESP_EVENT_ESPNOW_CTRL_BIND: {
        espnow_ctrl_bind_info_t *info = (espnow_ctrl_bind_info_t *)event_data;
        ESP_LOGI(TAG, "bind, uuid: " MACSTR ", initiator_type: %d", MAC2STR(info->mac), info->initiator_attribute);

        // app_led_set_color(0, 255, 0);
        break;
    }

    case ESP_EVENT_ESPNOW_CTRL_UNBIND: {
        espnow_ctrl_bind_info_t *info = (espnow_ctrl_bind_info_t *)event_data;
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

static void app_responder_ctrl_data_cb(espnow_attribute_t initiator_attribute,
                                       espnow_attribute_t responder_attribute,
                                       uint32_t status)
{
    ESP_LOGI(TAG, "app_responder_ctrl_data_cb, initiator_attribute: %d, responder_attribute: %d, value: %" PRIu32 "",
             initiator_attribute, responder_attribute, status);

    if (status) {
        // app_led_set_color(255, 255, 255);
    } else {
        // app_led_set_color(0, 0, 0);
    }
}

///////////////////////////////////////////////////////////////////////////////
// app_control_responder_init
//

static void app_control_responder_init(void)
{
    esp_event_handler_register(ESP_EVENT_ESPNOW, ESP_EVENT_ESPNOW_CTRL_BIND, app_espnow_event_handler, NULL);
    esp_event_handler_register(ESP_EVENT_ESPNOW, ESP_EVENT_ESPNOW_CTRL_UNBIND, app_espnow_event_handler, NULL);

    ESP_ERROR_CHECK(espnow_ctrl_responder_bind(30 * 1000, -55, NULL));
    espnow_ctrl_responder_data(app_responder_ctrl_data_cb);
}

///////////////////////////////////////////////////////////////////////////////
// app_wifi_prov_start_press_cb
//

static void app_wifi_prov_start_press_cb(void *arg, void *usr_data)
{
    ESP_ERROR_CHECK(!(BUTTON_DOUBLE_CLICK == iot_button_get_event(arg)));

    if (s_wifi_prov_status == APP_WIFI_PROV_INIT) {

        ESP_LOGI(TAG, "Start WiFi provisioning over ESP-NOW on responder");

        app_espnow_prov_responder_start();

        s_wifi_prov_status = APP_WIFI_PROV_START;

        // app_led_set_color(255, 255, 255);
    } else if (s_wifi_prov_status == APP_WIFI_PROV_START) {
        ESP_LOGI(TAG, "WiFi provisioning is started");
    } else {
        ESP_LOGI(TAG, "WiFi is already provisioned");
    }
}

///////////////////////////////////////////////////////////////////////////////
// app_wifi_prov_reset_press_cb
//

static void app_wifi_prov_reset_press_cb(void *arg, void *usr_data)
{
    ESP_ERROR_CHECK(!(BUTTON_LONG_PRESS_START == iot_button_get_event(arg)));

    ESP_LOGI(TAG, "Reset WiFi provisioning information and restart");

    wifi_prov_mgr_reset_provisioning();

    esp_wifi_disconnect();
    esp_restart();
}

///////////////////////////////////////////////////////////////////////////////
// app_wifi_prov_button_init
//

static void app_wifi_prov_button_init(void)
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
// app_wifi_init
//

static void app_wifi_init()
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
// app_main
//

void app_main()
{
    espnow_storage_init();

    app_wifi_init();

    espnow_config_t espnow_config = ESPNOW_INIT_CONFIG_DEFAULT();

    espnow_config.qsize = CONFIG_APP_ESPNOW_QUEUE_SIZE;
    espnow_config.sec_enable = 1;

    espnow_init(&espnow_config);

    //app_led_init();

    app_wifi_prov_button_init();

    app_espnow_responder_register();

    app_control_responder_init();

    app_espnow_responder();
}
