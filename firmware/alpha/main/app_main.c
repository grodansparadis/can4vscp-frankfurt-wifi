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

#include "wifi_prov.h"

#include "initiator.h"

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

#if CONFIG_IDF_TARGET_ESP32C2 || CONFIG_IDF_TARGET_ESP32C3
#define CONTROL_KEY_GPIO        GPIO_NUM_9
#else
#define CONTROL_KEY_GPIO        GPIO_NUM_0

typedef enum {
    APP_ESPNOW_CTRL_INIT,
    APP_ESPNOW_CTRL_BOUND,
    APP_ESPNOW_CTRL_MAX
} app_espnow_ctrl_status_t;

static app_espnow_ctrl_status_t s_espnow_ctrl_status = APP_ESPNOW_CTRL_INIT;
#endif

#define WIFI_PROV_KEY_GPIO      GPIO_NUM_0

typedef enum {
    APP_WIFI_PROV_INIT,
    APP_WIFI_PROV_START,
    APP_WIFI_PROV_SUCCESS,
    APP_WIFI_PROV_MAX
} app_wifi_prov_status_t;

static app_wifi_prov_status_t s_wifi_prov_status = APP_WIFI_PROV_INIT;

static void app_led_init(void)
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
        .max_leds = 1, // at least one LED on board
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

void app_led_set_color(uint8_t red, uint8_t green, uint8_t blue)
{
#ifdef VSCP_PROJDEF_LED_STRIP  
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#if CONFIG_IDF_TARGET_ESP32C2
    gpio_set_level(LED_RED_GPIO, red > 0 ? 0 : 1);
    gpio_set_level(LED_GREEN_GPIO, green > 0 ? 0 : 1);
    gpio_set_level(LED_BLUE_GPIO, blue > 0 ? 0 : 1);
#else
    led_strip_set_pixel(g_strip_handle, 0, red, green, blue);
    led_strip_refresh(g_strip_handle);
#endif
#else
    g_strip_handle->set_pixel(g_strip_handle, 0, red, green, blue);
    g_strip_handle->refresh(g_strip_handle, 100);
#endif
#endif
}

static void app_wifi_event_handler(void *arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        app_led_set_color(255, 0, 0);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
#if defined(CONFIG_APP_WIFI_PROVISION) || defined(CONFIG_APP_ESPNOW_PROVISION)
        s_wifi_prov_status = APP_WIFI_PROV_SUCCESS;
#endif

        app_led_set_color(0, 255, 0);
    }
}


static void app_initiator_send_press_cb(void *arg, void *usr_data)
{
    static bool status = 0;

    ESP_ERROR_CHECK(!(BUTTON_SINGLE_CLICK == iot_button_get_event(arg)));

    if (s_espnow_ctrl_status == APP_ESPNOW_CTRL_BOUND) {
        ESP_LOGI(TAG, "initiator send press");
        espnow_ctrl_initiator_send(ESPNOW_ATTRIBUTE_KEY_1, ESPNOW_ATTRIBUTE_POWER, status);
        status = !status;
    } else {
        ESP_LOGI(TAG, "please double click to bind the devices firstly");
    }
}

static void app_initiator_bind_press_cb(void *arg, void *usr_data)
{
    ESP_ERROR_CHECK(!(BUTTON_DOUBLE_CLICK == iot_button_get_event(arg)));

    if (s_espnow_ctrl_status == APP_ESPNOW_CTRL_INIT) {
        ESP_LOGI(TAG, "initiator bind press");
        espnow_ctrl_initiator_bind(ESPNOW_ATTRIBUTE_KEY_1, true);
        s_espnow_ctrl_status = APP_ESPNOW_CTRL_BOUND;
    } else {
        ESP_LOGI(TAG, "this device is already in bound status");
    }
}

static void app_initiator_unbind_press_cb(void *arg, void *usr_data)
{
    ESP_ERROR_CHECK(!(BUTTON_LONG_PRESS_START == iot_button_get_event(arg)));

    if (s_espnow_ctrl_status == APP_ESPNOW_CTRL_BOUND) {
        ESP_LOGI(TAG, "initiator unbind press");
        espnow_ctrl_initiator_bind(ESPNOW_ATTRIBUTE_KEY_1, false);
        s_espnow_ctrl_status = APP_ESPNOW_CTRL_INIT;
    } else {
        ESP_LOGI(TAG, "this device is not been bound");
    }
}

static void app_control_button_init(void)
{
    button_config_t button_config = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num = CONTROL_KEY_GPIO,
            .active_level = 0,
        },
    };

    button_handle_t button_handle = iot_button_create(&button_config);

    iot_button_register_cb(button_handle, BUTTON_SINGLE_CLICK, app_initiator_send_press_cb, NULL);
    iot_button_register_cb(button_handle, BUTTON_DOUBLE_CLICK, app_initiator_bind_press_cb, NULL);
    iot_button_register_cb(button_handle, BUTTON_LONG_PRESS_START, app_initiator_unbind_press_cb, NULL);
}



static void app_wifi_prov_over_espnow_start_press_cb(void *arg, void *usr_data)
{
    ESP_ERROR_CHECK(!(BUTTON_SINGLE_CLICK == iot_button_get_event(arg)));

    if (s_wifi_prov_status == APP_WIFI_PROV_SUCCESS) {
        bool enabled;

        espnow_get_config_for_data_type(ESPNOW_DATA_TYPE_PROV, &enabled);

        if (enabled) {
            ESP_LOGI(TAG, "WiFi provisioning over ESP-NOW is started");
        } else {
            ESP_LOGI(TAG, "Start WiFi provisioning over ESP-NOW on initiator");

            /*  Start 30s prov beacon */
            app_espnow_prov_beacon_start(30);
        }
    } else if (s_wifi_prov_status == APP_WIFI_PROV_START) {
        ESP_LOGI(TAG, "Please finish WiFi provisioning firstly");
    } else {
        ESP_LOGI(TAG, "Please start WiFi provisioning firstly");
    }
}


static void app_wifi_prov_start_press_cb(void *arg, void *usr_data)
{
    ESP_ERROR_CHECK(!(BUTTON_DOUBLE_CLICK == iot_button_get_event(arg)));

    if (s_wifi_prov_status == APP_WIFI_PROV_INIT) {

        ESP_LOGI(TAG, "Starting WiFi provisioning on initiator");

        wifi_prov();

        s_wifi_prov_status = APP_WIFI_PROV_START;

        app_led_set_color(255, 255, 255);
    } else if (s_wifi_prov_status == APP_WIFI_PROV_START) {
        ESP_LOGI(TAG, "WiFi provisioning is started");
    } else {
        ESP_LOGI(TAG, "WiFi is already provisioned");
    }
}

static void app_wifi_prov_reset_press_cb(void *arg, void *usr_data)
{
    ESP_ERROR_CHECK(!(BUTTON_LONG_PRESS_START == iot_button_get_event(arg)));

    ESP_LOGI(TAG, "Reset WiFi provisioning information and restart");

    wifi_prov_mgr_reset_provisioning();
    esp_wifi_disconnect();
    esp_restart();
}

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

    iot_button_register_cb(button_handle, BUTTON_SINGLE_CLICK, app_wifi_prov_over_espnow_start_press_cb, NULL);
    iot_button_register_cb(button_handle, BUTTON_DOUBLE_CLICK, app_wifi_prov_start_press_cb, NULL);
    iot_button_register_cb(button_handle, BUTTON_LONG_PRESS_START, app_wifi_prov_reset_press_cb, NULL);
}

static void app_wifi_init()
{
    wifi_prov_init();
}

void app_main()
{
    espnow_storage_init();
    app_wifi_init();

    espnow_config_t espnow_config = ESPNOW_INIT_CONFIG_DEFAULT();

    espnow_config.qsize = CONFIG_APP_ESPNOW_QUEUE_SIZE;
    espnow_config.sec_enable = 1;

    espnow_init(&espnow_config);

    app_led_init();
    app_wifi_prov_button_init();
    app_espnow_initiator_register();
    app_control_button_init();
    app_espnow_initiator();
}
