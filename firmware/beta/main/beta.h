/*
  VSCP Beta node

  This file is part of the VSCP (https://www.vscp.org)

  The MIT License (MIT)
  Copyright © 2022-2023 Ake Hedman, the VSCP project <info@vscp.org>

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#ifndef __VSCP_ESP_NOW_BETA_H__
#define __VSCP_ESP_NOW_BETA_H__

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <esp_log.h>
#include "esp_now.h"

#include <vscp.h>
#include <vscp-espnow.h>

#define NODETYPE VSCP_DROPLET_BETA

#define CONNECTED_LED_GPIO_NUM 2
#define ACTIVE_LED_GPIO_NUM    3
#define GPIO_OUTPUT_PIN_SEL    ((1ULL << CONNECTED_LED_GPIO_NUM) | (1ULL << ACTIVE_LED_GPIO_NUM))

#define DEV_BUFFER_LENGTH 64

typedef enum {
  CH_LINK = 0, // tcp/ip link protocol
  CH_CAN,      // CAN
  CH_WS,       // websocket I & II
  CH_UDP,      // UDP
  CH_MULTI,    // Multicast
  CH_MQTT,     // MQTT
  CH_BLE,      // BLE
  CH_UART      // UART
} dev_channel_t;

// All transports use this structure for state

typedef struct {
  union {
    struct {
      uint32_t active : 1;    /**< Transport active if set to one */
      uint32_t open : 1;      /**< Transport open if set to one */
      uint32_t reserved : 30; /**< Reserved bits */
    };
    uint32_t flags; /**< Don't use */
  };
  QueueHandle_t msg_queue; /**< Message queue for transport */
  uint32_t overruns;       /**< Queue overrun counter */

} transport_t;

/*!
  Default values stored in non volatile memory
  on start up.
*/

#define DEFAULT_GUID "" // Empty constructs from MAC, "-" all nills, "xx:yy:..." set GUID

// ----------------------------------------------------------------------------



typedef struct {

  // Module
  bool bProvisioned;    // Node has got channel and pmk from alpha node if true
  char nodeName[32];    // User name for node
  uint8_t startDelay;   // Delay before wifi is enabled (to charge cap)
  uint32_t bootCnt;     // Number of restarts (not editable)

  // Droplet
  bool dropletLongRange;             // Enable long range mode
  uint8_t dropletSizeQueue;          // Input queue size
  uint8_t dropletChannel;            // Channel to use (zero is current)
  uint8_t dropletTtl;                // Default ttl
  bool dropletForwardEnable;         // Forward when packets are received
  uint8_t dropletEncryption;         // 0=no encryption, 1=AES-128, 2=AES-192, 3=AES-256
  bool dropletFilterAdjacentChannel; // Don't receive if from other channel
  bool dropletForwardSwitchChannel;  // Allow switching channel on forward
  int8_t dropletFilterWeakSignal;    // Filter on RSSI (zero is no rssi filtering)
} node_persistent_config_t;

// ----------------------------------------------------------------------------

/*!
  ESP-NOW
*/
#define ESPNOW_SIZE_TX_BUF 10 /*!< Size for transmitt buffer >*/
#define ESPNOW_SIZE_RX_BUF 20 /*!< Size for receive buffer >*/
#define ESPNOW_MAXDELAY 512 // Ticks to wait for send queue access
#define ESPNOW_QUEUE_SIZE 6

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_vscp_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

// Beta node states
// typedef enum {
//   MAIN_STATE_WORK, // Standard working state
//   MAIN_STATE_INIT, // Active state during init until wifi is connected
//   MAIN_STATE_PROV, // Active state during provisioning
//   MAIN_STATE_SET_DEFAULTS
// } beta_node_states_t;

ESP_EVENT_DECLARE_BASE(ALPHA_EVENT); // declaration of the alpha events family

/*!
  Beta events
*/
typedef enum {
  /**
   * Start client provisioning and security transfer.
   * This state is active for 30 seconds.
   */
  BETA_START_CLIENT_PROVISIONING,

  /**
   * Stop client provisioning and security transfer.
   * This event happens 30 seconds after start
   */
  BETA_STOP_CLIENT_PROVISIONING,

  /**
   * Restart system
   */
  BETA_RESTART,

  /**
   * Restore factory default and erase wifi credentials
   */
  BETA_RESTORE_FACTORY_DEFAULTS,

  /**
   * Node is waiting to get IP address
   */
  BETA_GET_IP_ADDRESS_START,

  /**
   * Node have received IP address
   */
  BETA_GET_IP_ADDRESS_STOP,
} beta_cb_event_t;

// ----------------------------------------------------------------------------



/**
 * @brief Read processor on chip temperature
 * @return Temperature as floating point value
 */
float
read_onboard_temperature(void);

/**
 * @fn getMilliSeconds
 * @brief Get system time in Milliseconds
 *
 * @return Systemtime in milliseconds
 */
uint32_t
getMilliSeconds(void);

/**
 * @brief receive callback
 *
 * @param pev Pointer to received event.
 * @param userdata Pointer to user data.
 */
void
receive_cb(const vscpEvent *pev, void *userdata);

#endif