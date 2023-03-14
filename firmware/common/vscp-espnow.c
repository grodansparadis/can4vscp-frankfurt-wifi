/**
 * @brief           VSCP over esp-now code
 * @file            vscp_espnow.h
 * @author          Ake Hedman, The VSCP Project, www.vscp.org
 *
 *********************************************************************/

/* ******************************************************************************
 * VSCP (Very Simple Control Protocol)
 * http://www.vscp.org
 *
 * The MIT License (MIT)
 *
 * Copyright © 2000-2023 Ake Hedman, the VSCP project <info@vscp.org>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *  This file is part of VSCP - Very Simple Control Protocol
 *  http://www.vscp.org
 *
 * ******************************************************************************
 */

#include "vscp-compiler.h"
#include "vscp-projdefs.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>

#include <freertos/FreeRTOS.h>
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <esp_check.h>
#include <esp_crc.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_now.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>

#include <cJSON.h>

#include <vscp-firmware-helper.h>
#include <vscp.h>

#include "vscp-espnow.h"

static const char *TAG = "vscpnow";

typedef struct {
  uint16_t frame_head;
  uint16_t duration;
  uint8_t destination_address[6];
  uint8_t source_address[6];
  uint8_t broadcast_address[6];
  uint16_t sequence_control;

  uint8_t category_code;
  uint8_t organization_identifier[3]; // 0x18fe34
  uint8_t random_values[4];
  struct {
    uint8_t element_id;                 // 0xdd
    uint8_t lenght;                     //
    uint8_t organization_identifier[3]; // 0x18fe34
    uint8_t type;                       // 4
    uint8_t version;
    uint8_t body[0];
  } vendor_specific_content;
} __attribute__((packed)) espnow_frame_format_t;

// ------------------------------------------------

typedef struct {
  unsigned frame_ctrl : 16;
  unsigned duration_id : 16;
  uint8_t addr1[6]; /* receiver address */
  uint8_t addr2[6]; /* sender address */
  uint8_t addr3[6]; /* filtering address */
  unsigned sequence_ctrl : 16;
  uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;

typedef struct {
  wifi_ieee80211_mac_hdr_t hdr;
  uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;

static vscp_espnow_config_t s_vscp_espnow_config = { 0 };
static wifi_country_t g_self_country     = { 0 };

#define VSCP_ESPNOW_MAX_BUFFERED_NUM                                                                                   \
  (CONFIG_ESP32_WIFI_DYNAMIC_TX_BUFFER_NUM / 2) /* Not more than CONFIG_ESP32_WIFI_DYNAMIC_TX_BUFFER_NUM */

// Free running counter that is updated for every sent frame
uint8_t g_vscp_espnow_sendSequence = 0;

static EventGroupHandle_t s_vscp_espnow_event_group = NULL;

// Number of send events in transit
uint32_t g_vscp_espnow_buffered_num = 0;

QueueHandle_t g_vscp_espnow_rcvqueue = NULL;

#define VSCP_ESPNOW_SEND_CB_OK_BIT            BIT0
#define VSCP_ESPNOW_SEND_CB_FAIL_BIT          BIT1
#define VSCP_ESPNOW_PROV_CLIENT_GOT_INIT1_BIT BIT4 // Client new node on-line received
#define VSCP_ESPNOW_PROV_CLIENT_GOT_INIT2_BIT BIT5 // Client probe ack received
#define VSCP_ESPNOW_PROV_SRV_GOT_PMK_BIT      BIT6 // Provisioning key received

static struct {
  uint16_t magic;
} __attribute__((packed)) g_vscp_espnow_magic_cache[VSCP_ESPNOW_MSG_CACHE_SIZE] = { 0 };

static uint8_t g_vscp_espnow_magic_cache_next = 0;

// This mutex protects the espnow_send as it is NOT thread safe
static SemaphoreHandle_t vscp_espnow_send_lock;

/*!
  The discovery cache holds all nodes this node has discovered by there
  heartbeats.
*/
// static struct {
//   uint8_t mac[ESP_NOW_ETH_ALEN];
// } __attribute__((packed)) g_vscp_espnow_discovery_cache[VSCP_ESPNOW_DISCOVERY_CACHE_SIZE] = { 0 };

/**
 * @brief Receive data packet temporarily store in queue
 */
typedef struct __vscp_espnow_rxpkt {
  wifi_pkt_rx_ctrl_t rx_ctrl; /**< metadata header */
  uint8_t src_addr[6];
  uint8_t dst_addr[6];
  // dest_addr is variable in payload for pre 5.0.1
  uint8_t size;
  uint8_t payload[0];
} vscp_espnow_rxpkt_t;

static vscp_espnow_stats_t s_vscp_espnowStats = { 0 };

static uint8_t VSCP_ESPNOW_ADDR_SELF[6] = { 0 };

const uint8_t VSCP_ESPNOW_ADDR_NONE[6]      = { 0 };
const uint8_t VSCP_ESPNOW_ADDR_BROADCAST[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFF };

static uint8_t s_vscp_zero_guid[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// User handler for received vscp_espnow frames/events
static vscp_event_handler_cb_t s_vscp_event_handler_cb = NULL;

// User handler for clinet when attaching to network
static vscp_espnow_attach_network_handler_cb_t s_vscp_espnow_attach_network_handler_cb = NULL;

/*
  State machine state for the vscp_espnow stack
*/
static vscp_espnow_state_t s_stateDroplet = VSCP_ESPNOW_STATE_IDLE;

/*
  Structure that holds data for node that should
  be provisioned.
*/
typedef struct {
  uint8_t mac[6];                    // MAC address for node to provision
  uint8_t keyLocal[16]; // Local key for node
} vscp_espnow_provisioning_t;

/*
  Info about node that is under provisioning
*/
static vscp_espnow_provisioning_t s_provisionNodeInfo = { 0 };

// Forward declarations
static void
vscp_espnow_heartbeat_task(void *pvParameter);

//-----------------------------------------------------------------------------
//                                Droplet
//-----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
//                                  VSCP
// ----------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_build_guid_from_mac
//

int
vscp_espnow_build_guid_from_mac(uint8_t *pguid, const uint8_t *pmac, uint16_t nickname)
{
  uint8_t prebytes[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe };

  // Need a GUID pointer
  if (NULL == pguid) {
    ESP_LOGE(TAG, "Pointer to GUID is NULL");
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Need a mac pointer
  if (NULL == pmac) {
    ESP_LOGE(TAG, "Pointer to mac address is NULL");
    return VSCP_ERROR_INVALID_POINTER;
  }

  memcpy(pguid, prebytes, 8);
  memcpy(pguid + 8, pmac, ESP_NOW_ETH_ALEN);
  pguid[14] = (nickname << 8) & 0xff;
  pguid[15] = nickname & 0xff;

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_build_l1_heartbeat
//

int
vscp_espnow_build_l1_heartbeat(uint8_t *buf, uint8_t len, const uint8_t *pguid)
{
  // Need a buffer
  if (NULL == buf) {
    ESP_LOGE(TAG, "Pointer to buffer is NULL");
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Must have room for frame
  if (len < (VSCP_ESPNOW_MIN_FRAME + 3)) {
    ESP_LOGE(TAG, "Size of buffer is to small to fit event, len:%d", len);
    return VSCP_ERROR_PARAMETER;
  }

  memset(buf, 0, len);

  // Construct VSCP heart beat event

  buf[VSCP_ESPNOW_POS_ID]     = 0x55;
  buf[VSCP_ESPNOW_POS_ID + 1] = 0xAA;

  buf[VSCP_ESPNOW_POS_TYPE_VER] = (s_vscp_espnow_config.nodeType << 4) + VSCP_ESPNOW_VERSION;

  // VSCP Head
  buf[VSCP_ESPNOW_POS_HEAD]     = 0x00;
  buf[VSCP_ESPNOW_POS_HEAD + 1] = 0x00;

  // Nickname
  if (NULL != pguid) {
    buf[VSCP_ESPNOW_POS_NICKNAME]     = (PRJDEF_NODE_NICKNAME >> 8) & 0xff;
    buf[VSCP_ESPNOW_POS_NICKNAME + 1] = PRJDEF_NODE_NICKNAME & 0xff;
  }

  // VSCP Class
  buf[VSCP_ESPNOW_POS_VSCP_CLASS]     = (VSCP_CLASS1_INFORMATION >> 8) & 0xff;
  buf[VSCP_ESPNOW_POS_VSCP_CLASS + 1] = VSCP_CLASS1_INFORMATION & 0xff;

  // VSCP Type
  buf[VSCP_ESPNOW_POS_VSCP_TYPE]     = (VSCP_TYPE_INFORMATION_NODE_HEARTBEAT >> 8) & 0xff;
  buf[VSCP_ESPNOW_POS_VSCP_TYPE + 1] = VSCP_TYPE_INFORMATION_NODE_HEARTBEAT & 0xff;

  buf[VSCP_ESPNOW_POS_SIZE] = 3;

  // Data
  buf[VSCP_ESPNOW_POS_DATA]     = 0;    // User specific
  buf[VSCP_ESPNOW_POS_DATA + 1] = 0xff; // All zones
  buf[VSCP_ESPNOW_POS_DATA + 2] = 0xff; // All subzones

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_build_l2_heartbeat
//

int
vscp_espnow_build_l2_heartbeat(uint8_t *buf, uint8_t len, const uint8_t *pguid, const char *name)
{
  // Need a buffer
  if (NULL == buf) {
    ESP_LOGE(TAG, "Pointer to buffer is NULL");
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Must have room for frame
  if (len < (VSCP_ESPNOW_MIN_FRAME + 3)) {
    ESP_LOGE(TAG, "Size of byffer is to small to fit event, len:%d", len);
    return VSCP_ERROR_PARAMETER;
  }

  memset(buf, 0, len);

  // Construct VSCP heart beat event

  buf[VSCP_ESPNOW_POS_ID]     = 0x55;
  buf[VSCP_ESPNOW_POS_ID + 1] = 0xAA;

  buf[VSCP_ESPNOW_POS_TYPE_VER] = (s_vscp_espnow_config.nodeType << 4) + VSCP_ESPNOW_VERSION;

  // Head
  buf[VSCP_ESPNOW_POS_HEAD]     = 0;
  buf[VSCP_ESPNOW_POS_HEAD + 1] = 0;

  // Nickname
  if (NULL != pguid) {
    buf[VSCP_ESPNOW_POS_NICKNAME]     = pguid[14]; // (g_node_nickname >> 8) & 0xff;
    buf[VSCP_ESPNOW_POS_NICKNAME + 1] = pguid[15]; // g_node_nickname & 0xff;
  }

  // VSCP Class
  buf[VSCP_ESPNOW_POS_VSCP_CLASS]     = (VSCP_CLASS1_INFORMATION >> 8) & 0xff;
  buf[VSCP_ESPNOW_POS_VSCP_CLASS + 1] = VSCP_CLASS1_INFORMATION & 0xff;

  // VSCP Type
  buf[VSCP_ESPNOW_POS_VSCP_TYPE]     = (VSCP_TYPE_INFORMATION_NODE_HEARTBEAT >> 8) & 0xff;
  buf[VSCP_ESPNOW_POS_VSCP_TYPE + 1] = VSCP_TYPE_INFORMATION_NODE_HEARTBEAT & 0xff;

  // Data
  buf[VSCP_ESPNOW_POS_DATA]     = 0;    // User specific
  buf[VSCP_ESPNOW_POS_DATA + 1] = 0xff; // All zones
  buf[VSCP_ESPNOW_POS_DATA + 2] = 0xff; // All subzones

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_getMinBufSizeEv
//

size_t
vscp_espnow_getMinBufSizeEv(vscpEvent *pev)
{
  // Need event pointer
  if (NULL == pev) {
    ESP_LOGE(TAG, "Pointer to event is NULL");
    return 0;
  }

  return (VSCP_ESPNOW_MIN_FRAME + pev->sizeData);
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_getMinBufSizeEx
//

size_t
vscp_espnow_getMinBufSizeEx(vscpEventEx *pex)
{
  // Need event ex pointer
  if (NULL == pex) {
    ESP_LOGE(TAG, "Pointer to event ex is NULL");
    return 0;
  }

  return (VSCP_ESPNOW_MIN_FRAME + pex->sizeData);
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_evToFrame
//

int
vscp_espnow_evToFrame(uint8_t *buf, uint8_t len, const vscpEvent *pev)
{
  // Need a buffer
  if (NULL == buf) {
    ESP_LOGE(TAG, "Pointer to buffer is NULL");
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Need event
  if (NULL == pev) {
    ESP_LOGE(TAG, "Pointer to event is NULL");
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Must have room for frame
  if (len < (VSCP_ESPNOW_MIN_FRAME + pev->sizeData)) {
    ESP_LOGE(TAG, "Size of buffer is to small to fit event, len:%d", len);
    return VSCP_ERROR_PARAMETER;
  }

  memset(buf, 0, len);

  buf[VSCP_ESPNOW_POS_ID] = 0x55;
  buf[VSCP_ESPNOW_POS_ID + 1] = 0xAA;

  buf[VSCP_ESPNOW_POS_TYPE_VER] = (s_vscp_espnow_config.nodeType << 4) + VSCP_ESPNOW_VERSION;

  // head
  buf[VSCP_ESPNOW_POS_HEAD] = (pev->head >> 8) & 0xff;
  buf[VSCP_ESPNOW_POS_HEAD] = pev->head & 0xff;

  // nickname
  buf[VSCP_ESPNOW_POS_NICKNAME]     = pev->GUID[14];
  buf[VSCP_ESPNOW_POS_NICKNAME + 1] = pev->GUID[15];

  // vscp-class
  buf[VSCP_ESPNOW_POS_VSCP_CLASS]     = (pev->vscp_class >> 8) & 0xff;
  buf[VSCP_ESPNOW_POS_VSCP_CLASS + 1] = pev->vscp_class & 0xff;

  // vscp-type
  buf[VSCP_ESPNOW_POS_VSCP_TYPE]     = (pev->vscp_type >> 8) & 0xff;
  buf[VSCP_ESPNOW_POS_VSCP_TYPE + 1] = pev->vscp_type & 0xff;

  // data
  if (pev->sizeData) {
    memcpy((buf + VSCP_ESPNOW_POS_DATA), pev->pdata, pev->sizeData);
  }

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_exToFrame
//

int
vscp_espnow_exToFrame(uint8_t *buf, uint8_t len, const vscpEventEx *pex)
{
  // Need a buffer
  if (NULL == buf) {
    ESP_LOGE(TAG, "Pointer to buffer is NULL");
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Need event
  if (NULL == pex) {
    ESP_LOGE(TAG, "Pointer to event is NULL");
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Must have room for frame
  if (len < (VSCP_ESPNOW_MIN_FRAME + pex->sizeData)) {
    ESP_LOGE(TAG, "Size of buffer is to small to fit event, len:%d", len);
    return VSCP_ERROR_PARAMETER;
  }

  memset(buf, 0, len);

  // head
  buf[VSCP_ESPNOW_POS_HEAD]     = (pex->head >> 8) & 0xff;
  buf[VSCP_ESPNOW_POS_HEAD + 1] = pex->head & 0xff;

  // nickname
  buf[VSCP_ESPNOW_POS_NICKNAME]     = pex->GUID[14];
  buf[VSCP_ESPNOW_POS_NICKNAME + 1] = pex->GUID[15];

  // vscp-class
  buf[VSCP_ESPNOW_POS_VSCP_CLASS]     = (pex->vscp_class >> 8) & 0xff;
  buf[VSCP_ESPNOW_POS_VSCP_CLASS + 1] = pex->vscp_class & 0xff;

  // vscp-type
  buf[VSCP_ESPNOW_POS_VSCP_TYPE]     = (pex->vscp_type >> 8) & 0xff;
  buf[VSCP_ESPNOW_POS_VSCP_TYPE + 1] = pex->vscp_type & 0xff;

  // data
  if (pex->sizeData) {
    memcpy((buf + VSCP_ESPNOW_POS_DATA), pex->data, pex->sizeData);
  }

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_frameToEv
//

int
vscp_espnow_frameToEv(vscpEvent *pev, const uint8_t *buf, uint8_t len, uint32_t timestamp)
{
  // Need event
  if (NULL == pev) {
    ESP_LOGE(TAG, "Pointer to event is NULL");
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Must be at least have min size
  if (len < VSCP_ESPNOW_MIN_FRAME) {
    ESP_LOGE(TAG, "esp-now data is too short, len:%d", len);
    return VSCP_ERROR_MTU;
  }

  // Must have valid paket type byte
  if ((buf[VSCP_ESPNOW_POS_ID] != 0x55) || (buf[VSCP_ESPNOW_POS_ID + 1] != 0xA0)) {
    ESP_LOGE(TAG, "esp-now data is an invalid frame");
    return VSCP_ERROR_INVALID_FRAME;
  }

  memset(pev, 0, sizeof(vscpEvent));

  // Free any allocated event data
  if (NULL != pev->pdata) {
    VSCP_FREE(pev->pdata);
    pev->pdata = NULL;
  }

  // Set VSCP size
  pev->sizeData = len - VSCP_ESPNOW_MIN_FRAME;
  if (pev->sizeData) {
    pev->pdata = VSCP_MALLOC(pev->sizeData);
    if (NULL == pev->pdata) {
      return VSCP_ERROR_MEMORY;
    }
  }

  // Copy in VSCP data
  memcpy(pev->pdata, buf + VSCP_ESPNOW_MIN_FRAME, pev->sizeData);

  // Set timestamp if not set
  if (!timestamp) {
    pev->timestamp = esp_timer_get_time();
  }
  else {
    pev->timestamp = timestamp;
  }

  // Head
  pev->head = (buf[VSCP_ESPNOW_POS_HEAD] << 8) + buf[VSCP_ESPNOW_POS_HEAD + 1];

  // Nickname
  pev->GUID[14] = buf[VSCP_ESPNOW_POS_NICKNAME];
  pev->GUID[15] = buf[VSCP_ESPNOW_POS_NICKNAME + 1];

  // VSCP class
  pev->vscp_class = (buf[VSCP_ESPNOW_POS_VSCP_CLASS] << 8) + buf[VSCP_ESPNOW_POS_VSCP_CLASS + 1];

  // VSCP type
  pev->vscp_type = (buf[VSCP_ESPNOW_POS_VSCP_TYPE] << 8) + buf[VSCP_ESPNOW_POS_VSCP_TYPE + 1];

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_frameToEx
//

int
vscp_espnow_frameToEx(vscpEventEx *pex, const uint8_t *buf, uint8_t len, uint32_t timestamp)
{
  // Need event
  if (NULL == pex) {
    ESP_LOGE(TAG, "Pointer to event is NULL");
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Must at least have min size
  if (len < VSCP_ESPNOW_MIN_FRAME) {
    ESP_LOGE(TAG, "esp-now data is too short, len:%d", len);
    return VSCP_ERROR_MTU;
  }

  // Must have valid first byte
  if ((buf[0] & 0xff) > VSCP_ENCRYPTION_AES256) {
    ESP_LOGE(TAG, "esp-now data is an invalid frame");
    return VSCP_ERROR_MTU;
  }

  memset(pex, 0, sizeof(vscpEventEx));

  // Set VSCP size
  pex->sizeData = len - VSCP_ESPNOW_MIN_FRAME;

  // Copy in VSCP data
  memcpy(pex->data, buf + VSCP_ESPNOW_MIN_FRAME, pex->sizeData);

  // Set timestamp if not set
  if (!timestamp) {
    pex->timestamp = esp_timer_get_time();
  }
  else {
    pex->timestamp = timestamp;
  }

  // Head
  pex->head = (buf[VSCP_ESPNOW_POS_HEAD] << 8) + buf[VSCP_ESPNOW_POS_HEAD + 1];

  // Nickname
  pex->GUID[14] = buf[VSCP_ESPNOW_POS_NICKNAME];
  pex->GUID[15] = buf[VSCP_ESPNOW_POS_NICKNAME + 1];

  // VSCP class
  pex->vscp_class = (buf[VSCP_ESPNOW_POS_VSCP_CLASS] << 8) + buf[VSCP_ESPNOW_POS_VSCP_CLASS + 1];

  // VSCP type
  pex->vscp_type = (buf[VSCP_ESPNOW_POS_VSCP_TYPE] << 8) + buf[VSCP_ESPNOW_POS_VSCP_TYPE + 1];

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_sendEvent
//

esp_err_t
vscp_espnow_sendEvent(const uint8_t *destAddr, const vscpEvent *pev, const uint8_t *pkey, uint32_t wait_ms)
{
  esp_err_t rv;
  uint8_t *pbuf = NULL;

  // Need event
  if (NULL == pev) {
    ESP_LOGE(TAG, "Pointer to event is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  pbuf = VSCP_MALLOC(VSCP_ESPNOW_MIN_FRAME + pev->sizeData);
  if (NULL == pbuf) {
    ESP_LOGE(TAG, "Failed to allocate memory for send buffer-");
    return ESP_ERR_NO_MEM;
  }

  if (VSCP_ERROR_SUCCESS != (rv = vscp_espnow_evToFrame(pbuf, VSCP_ESPNOW_MIN_FRAME + pev->sizeData, pev))) {
    VSCP_FREE(pbuf);
    ESP_LOGE(TAG, "Failed to convert event to frame. rv=%d", rv);
    return rv;
  }

  // ESP_LOGI(TAG, "Send mac: " MACSTR ", version: %d", MAC2STR(destAddr), VSCP_ESPNOW_VERSION);

  if (ESP_OK != (rv = vscp_espnow_send(destAddr, // VSCP_ESPNOW_ADDR_BROADCAST,
                                   false,
                                   s_vscp_espnow_config.nEncryption,
                                   (pkey != NULL) ? pkey : s_vscp_espnow_config.pmk,
                                   s_vscp_espnow_config.ttl,
                                   pbuf,
                                   VSCP_ESPNOW_MIN_FRAME + pev->sizeData,
                                   wait_ms))) {
    ESP_LOGE(TAG, "Failed to send event. rv=%X", rv);
    VSCP_FREE(pbuf);
    return rv;
  }

  VSCP_FREE(pbuf);
  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_sendEventEx
//

esp_err_t
vscp_espnow_sendEventEx(const uint8_t *destAddr, const vscpEventEx *pex, const uint8_t *pkey, uint32_t wait_ms)
{
  esp_err_t rv;
  uint8_t *pbuf;

  ESP_LOGI(TAG, "Send Event");

  // Need event
  if (NULL == pex) {
    ESP_LOGE(TAG, "Pointer to event ex is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  pbuf = VSCP_MALLOC(VSCP_ESPNOW_MIN_FRAME + pex->sizeData);
  if (NULL == pbuf) {
    return ESP_ERR_NO_MEM;
  }

  if (VSCP_ERROR_SUCCESS != (rv = vscp_espnow_exToFrame(pbuf, VSCP_ESPNOW_MIN_FRAME + pex->sizeData, pex))) {
    VSCP_FREE(pbuf);
    ESP_LOGE(TAG, "Failed to convert event to frame. rv=%d", rv);
    return ESP_ERR_INVALID_ARG;
  }

  if (ESP_OK != (rv = vscp_espnow_send(destAddr, //  VSCP_ESPNOW_ADDR_BROADCAST,
                                   false,
                                   s_vscp_espnow_config.nEncryption,
                                   (pkey != NULL) ? pkey : s_vscp_espnow_config.pmk,
                                   s_vscp_espnow_config.ttl,
                                   pbuf,
                                   VSCP_ESPNOW_MIN_FRAME + pex->sizeData,
                                   wait_ms))) {
    ESP_LOGE(TAG, "Failed to send event. rv=%X", rv);
    VSCP_FREE(pbuf);
    return rv;
  }

  VSCP_FREE(pbuf);
  return ESP_OK;
}

//=============================================================================
//                         Droplet Core stuff
//=============================================================================

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_init
//

esp_err_t
vscp_espnow_init(const vscp_espnow_config_t *config)
{
  // void *p;
  //  esp_err_t ret = ESP_FAIL;

  s_stateDroplet = VSCP_ESPNOW_STATE_IDLE;

  // ESP_ERROR_CHECK(config);
  memcpy(&s_vscp_espnow_config, config, sizeof(vscp_espnow_config_t));

  g_vscp_espnow_rcvqueue = xQueueCreate(s_vscp_espnow_config.sizeQueue, sizeof(void *));
  if (NULL == g_vscp_espnow_rcvqueue) {
    ESP_LOGD(TAG, "Create vscp_espnow event queue fail");
    return ESP_FAIL;
  }

  // Event group for vscp_espnow sent cb
  s_vscp_espnow_event_group = xEventGroupCreate();
  ESP_RETURN_ON_ERROR(!s_vscp_espnow_event_group, TAG, "Create event group fail");

  vscp_espnow_send_lock = xSemaphoreCreateMutex();
  ESP_RETURN_ON_ERROR(!vscp_espnow_send_lock, TAG, "Create send semaphore mutex fail");

  esp_wifi_get_country(&g_self_country);
  esp_wifi_get_mac(ESP_IF_WIFI_STA, VSCP_ESPNOW_ADDR_SELF);
  ESP_LOGD(TAG, "mac: " MACSTR ", version: %d", MAC2STR(VSCP_ESPNOW_ADDR_SELF), VSCP_ESPNOW_VERSION);

  // Add broadcast peer information to peer list.
  esp_now_peer_info_t *peer = VSCP_MALLOC(sizeof(esp_now_peer_info_t));
  if (NULL == peer) {
    ESP_LOGE(TAG, "Malloc peer information fail");
    // vSemaphoreDelete(s_vscp_vscp_espnow_event_queue);
    esp_now_deinit();
    return ESP_FAIL;
  }
  memset(peer, 0, sizeof(esp_now_peer_info_t));
  peer->channel = PRJDEF_VSCP_ESPNOW_CHANNEL;
  peer->ifidx   = PRJDEF_VSCP_ESPNOW_WIFI_IF;
  peer->encrypt = false;
  memcpy(peer->peer_addr, VSCP_ESPNOW_ADDR_BROADCAST, ESP_NOW_ETH_ALEN);
  ESP_ERROR_CHECK(esp_now_add_peer(peer));
  VSCP_FREE(peer);

  // Start receive task
  // xTaskCreate(vscp_espnow_rcv_task, "vscp_espnow rcv_task", 1024 * 3, (void *) &s_vscp_espnow_config, 5, NULL);

  // Start heartbeat task vscp_heartbeat_task
  xTaskCreate(&vscp_espnow_heartbeat_task, "vscp_espnow_heartbeat_task", 1024 * 2, (void *) &s_vscp_espnow_config, 1, NULL);

  return ESP_OK;
}


  ///////////////////////////////////////////////////////////////////////////////
  // vscp_espnow_set_vscp_user_handler_cb
  //

  void
  vscp_espnow_set_vscp_user_handler_cb(vscp_event_handler_cb_t cb)
{
  s_vscp_event_handler_cb = cb;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_clear_vscp_handler_cb
//

void
vscp_espnow_clear_vscp_handler_cb(void)
{
  s_vscp_event_handler_cb = NULL;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_set_attach_network_handler_cb
//

void
vscp_espnow_set_attach_network_handler_cb(vscp_espnow_attach_network_handler_cb_t cb)
{
  s_vscp_espnow_attach_network_handler_cb = cb;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_clear_attach_network_handler_cb
//

void
vscp_espnow_clear_attach_network_handler_cb(void)
{
  s_vscp_espnow_attach_network_handler_cb = NULL;
}

int
vscp_espnow_probe(void)
{
  // Add broadcast peer information to peer list.
  esp_now_peer_info_t *peer = VSCP_MALLOC(sizeof(esp_now_peer_info_t));
  if (NULL == peer) {
    ESP_LOGE(TAG, "Malloc peer information fail");
    // vSemaphoreDelete(s_vscp_vscp_espnow_event_queue);
    esp_now_deinit();
    return ESP_FAIL;
  }
  memset(peer, 0, sizeof(esp_now_peer_info_t));
  peer->channel = PRJDEF_VSCP_ESPNOW_CHANNEL;
  peer->ifidx   = PRJDEF_VSCP_ESPNOW_WIFI_IF;
  peer->encrypt = false;
  memcpy(peer->peer_addr, VSCP_ESPNOW_ADDR_BROADCAST, ESP_NOW_ETH_ALEN);
  ESP_ERROR_CHECK(esp_now_add_peer(peer));
  VSCP_FREE(peer);

  return 0;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_heartbeat_task
//
// Sent periodically as a broadcast to all zones/subzones
//

static void
vscp_espnow_heartbeat_task(void *pvParameter)
{
  esp_err_t ret = 0;
  uint8_t buf[VSCP_ESPNOW_MIN_FRAME + 3]; // Three byte data
  size_t size = sizeof(buf);

  vscp_espnow_config_t *pconfig = (vscp_espnow_config_t *) pvParameter;
  if (NULL == pconfig) {
    ESP_LOGE(TAG, "Invalid (NULL) parameter given");
    return;
  }

  // Create Heartbeat event
  if (VSCP_ERROR_SUCCESS != (ret = vscp_espnow_build_l1_heartbeat(buf, size, pconfig->nodeGuid))) {
    ESP_LOGE(TAG, "Could not create heartbeat event, will exit task. VSCP rv %d", ret);
    goto ERROR;
  }

  ESP_LOGI(TAG, "Start sending VSCP heartbeats");

  while (true) {

    if (VSCP_ESPNOW_STATE_IDLE == s_stateDroplet) {

      uint8_t ch     = 0;
      wifi_second_chan_t second = 0;

      if (ESP_OK != (ret = esp_wifi_get_channel(&ch, &second))) {
        ESP_LOGE(TAG, "Failed to get wifi channel, rv = %X", ret);
      }
      ESP_LOGI(TAG, "Sending heartbeat ch=%d (%d).", ch, second);
      ret = vscp_espnow_send(VSCP_ESPNOW_ADDR_BROADCAST,
                         false,
                         VSCP_ENCRYPTION_NONE,
                         s_vscp_espnow_config.pmk,
                         4,
                         buf,
                         VSCP_ESPNOW_MIN_FRAME + 3,
                         1000 / portTICK_PERIOD_MS);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send heartbeat. ret = %X", ret);
      }
    }
    vTaskDelay(VSCP_ESPNOW_HEART_BEAT_INTERVAL / portTICK_PERIOD_MS);
  }

  // ESP_ERROR_CONTINUE(ret != ESP_OK, "<%s>", esp_err_to_name(ret));

ERROR:
  ESP_LOGW(TAG, "Heartbeat task exit %d", ret);
  vTaskDelete(NULL);
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_client_provisioning_task
//
// Just running during client provisioning state on beta and gamma nodes
//

void
vscp_espnow_client_provisioning_task(void *pvParameter)
{
  esp_err_t ret = 0;
  int rv;
  size_t size    = VSCP_ESPNOW_MIN_FRAME + 16;
  uint8_t *pbuf  = NULL;
  vscpEvent *pev = NULL;
  uint8_t nLoops = 0;
  uint8_t origChannel;
  uint8_t channel   = 1; // Start on this channel
  uint8_t intMsgCnt = 0;

  vscp_espnow_config_t *pconfig = (vscp_espnow_config_t *) pvParameter;
  if (NULL == pconfig) {
    ESP_LOGE(TAG, "Invalid (NULL) parameter for vscp_espnow config given");
    goto ERROR;
  }

  // Check that the buffer was allocated correctly
  pbuf = VSCP_MALLOC(size);
  if (NULL == pbuf) {
    ESP_LOGE(TAG, "Unable to allocate storage for event buffer.");
    goto ERROR;
  }

  // Check event pointer
  pev = vscp_fwhlp_newEvent();
  if (NULL == pev) {
    VSCP_FREE(pbuf);
    ESP_LOGE(TAG, "Unable to allocate memory for event");
    goto ERROR;
  }

  // Allocate data for event (our GUID)
  pev->pdata = VSCP_MALLOC(16);
  if (NULL == pev->pdata) {
    VSCP_FREE(pbuf);
    vscp_fwhlp_deleteEvent(&pev);
    ESP_LOGE(TAG, "Unable to allocate memory for event data");
    goto ERROR;
  }

  pev->head       = 0;
  pev->vscp_class = VSCP_CLASS1_PROTOCOL;
  pev->vscp_type  = VSCP_TYPE_PROTOCOL_NEW_NODE_ONLINE;
  pev->timestamp  = esp_timer_get_time();
  pev->sizeData   = 16;
  memcpy(pev->GUID, pconfig->nodeGuid, 16);

  if (VSCP_ERROR_SUCCESS != (rv = vscp_espnow_evToFrame(pbuf, size, pev))) {
    ESP_LOGE(TAG, "Failed to create buffer data from event. ev=%d", rv);
    vscp_fwhlp_deleteEvent(&pev);
    goto ERROR;
  }

  // Event is not needed anymore
  vscp_fwhlp_deleteEvent(&pev);

  // Save channel we use
  esp_wifi_get_channel(&origChannel, NULL);

  ESP_LOGI(TAG, "Start initialization sequency");

  while ((VSCP_ESPNOW_STATE_CLIENT_INIT == s_stateDroplet) && (nLoops < VSCP_ESPNOW_INIT_LOOPS)) {

    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    ESP_LOGI(TAG, "Channel = %d\n", channel);

    ret = vscp_espnow_send(VSCP_ESPNOW_ADDR_BROADCAST,
                       false,
                       VSCP_ENCRYPTION_NONE,
                       s_vscp_espnow_config.lkey,
                       4,
                       pbuf,
                       VSCP_ESPNOW_MIN_FRAME + 2,
                       0 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to send heartbeat. ret = %X", ret);
    }

    vTaskDelay(VSCP_ESPNOW_INIT_HEART_BEAT_INTERVAL / portTICK_PERIOD_MS);

    // We send five time on each channel
    if ((++intMsgCnt) >= 5) {
      intMsgCnt = 0;
      if ((++channel) > 13) {
        channel = 1;
        nLoops++;
      }
    }
  } // while

ERROR:

  // Free allocated frae buffer
  VSCP_FREE(pbuf);

  // Set original channel
  esp_wifi_set_channel(origChannel, WIFI_SECOND_CHAN_NONE);

  // Set idle state
  s_stateDroplet = VSCP_ESPNOW_STATE_IDLE;

  ESP_LOGI(TAG, "End initialization sequency");
  vTaskDelete(NULL);
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_startClientProvisioning
//

int
vscp_espnow_startClientProvisioning()
{
  if (VSCP_ESPNOW_STATE_IDLE == s_stateDroplet) {
    s_stateDroplet = VSCP_ESPNOW_STATE_CLIENT_INIT;
    xTaskCreate(&vscp_espnow_client_provisioning_task, "prov_client_task", 1024 * 4, &s_vscp_espnow_config, 5, NULL);
  }
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_isClientBitInit1Set
//

bool
vscp_espnow_isClientBitInit1Set(void)
{
  // We should wait for a heartbeat event from the node we is provisioning
  xEventGroupClearBits(s_vscp_espnow_event_group, VSCP_ESPNOW_PROV_CLIENT_GOT_INIT1_BIT);

  return (xEventGroupWaitBits(s_vscp_espnow_event_group,
                              VSCP_ESPNOW_PROV_CLIENT_GOT_INIT1_BIT,
                              pdTRUE,
                              pdTRUE,
                              1000 / portTICK_PERIOD_MS) &
          VSCP_ESPNOW_PROV_CLIENT_GOT_INIT1_BIT);
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_isClientInit2Set
//

bool
vscp_espnow_isClientInit2Set(void)
{
  // We should wait for a heartbeat event from the node we is provisioning
  xEventGroupClearBits(s_vscp_espnow_event_group, VSCP_ESPNOW_PROV_CLIENT_GOT_INIT2_BIT);

  return (xEventGroupWaitBits(s_vscp_espnow_event_group,
                              VSCP_ESPNOW_PROV_CLIENT_GOT_INIT1_BIT,
                              pdTRUE,
                              pdTRUE,
                              1000 / portTICK_PERIOD_MS) &
          VSCP_ESPNOW_PROV_CLIENT_GOT_INIT1_BIT);
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_server_provisioning_task
//
// Just running during provisioning provisioning state.
// 1.) Wait for new node online from client node. (unencrypted)
// 2.) Send encryption key to client node encrypted with client nodes key.
// 3.) Wait for encrypted new-node online from client node.
//

static void
vscp_espnow_server_provisioning_task(void *pvParameter)
{
  EventBits_t uxBits;
  esp_err_t ret = 0;

  ESP_LOGI(TAG, "vscp_espnow server provisioning task started");

  vscp_espnow_config_t *pconfig = (vscp_espnow_config_t *) pvParameter;
  if (NULL == pconfig) {
    ESP_LOGE(TAG, "Invalid (NULL) parameter given");
    goto ERROR;
  }

  // Add broadcast peer information to peer list.
  esp_now_peer_info_t *peer = VSCP_MALLOC(sizeof(esp_now_peer_info_t));
  if (NULL == peer) {
    ESP_LOGE(TAG, "Malloc peer information fail");
    goto ERROR;
  }
  memset(peer, 0, sizeof(esp_now_peer_info_t));
  peer->channel = 0;
  peer->ifidx   = PRJDEF_VSCP_ESPNOW_WIFI_IF;
  peer->encrypt = false;
  memcpy(peer->peer_addr, s_provisionNodeInfo.mac, ESP_NOW_ETH_ALEN);
  ESP_ERROR_CHECK(esp_now_add_peer(peer));
  VSCP_FREE(peer);

  // Wait for first event
  s_stateDroplet = VSCP_ESPNOW_STATE_SRV_INIT1;

  // We should wait for heartbeat from the node we is provisioning
  xEventGroupClearBits(s_vscp_espnow_event_group, VSCP_ESPNOW_PROV_CLIENT_GOT_INIT1_BIT);

  // Wait for confirmation of heart beat from client
  uxBits = xEventGroupWaitBits(s_vscp_espnow_event_group,
                               VSCP_ESPNOW_PROV_CLIENT_GOT_INIT1_BIT,
                               pdTRUE,
                               pdTRUE,
                               20000 / portTICK_PERIOD_MS);
  // Bit is set if heart beat event from node received (auto cleared after above return)
  if (uxBits & VSCP_ESPNOW_PROV_CLIENT_GOT_INIT1_BIT) {
    // Send primary key to node
    vscpEvent *pev = vscp_fwhlp_newEvent();
    if (NULL == pev) {
      ESP_LOGE(TAG, "[srvprov] Failed to allocate new event.");
      goto ERROR;
    }

    ESP_LOGI(TAG, "[srvprov] INIT1");

    // Go to next state
    s_stateDroplet = VSCP_ESPNOW_STATE_SRV_INIT2;

    pev->pdata = VSCP_MALLOC(2 + 16 + 32 + 16); // encryption byte + reserved + GUID + key + iv
    if (NULL == pev->pdata) {
      VSCP_FREE(pev);
      ESP_LOGE(TAG, "[srvprov] Failed to allocate new event data.");
      goto ERROR;
    }

    pev->head       = 0;
    pev->vscp_class = 1034;
    pev->vscp_type  = 1;
    pev->sizeData   = 2 + 16 + 32; // encryption + reserved + GUID + key
    pev->pdata[0]   = pconfig->nEncryption;
    // memcpy(pev->pdata + 2, pconfig->nodeGuid, 16);
    //  Build GUID for desitnation node
    memset(pev->pdata + 2, 0xff, 7);
    pev->pdata[2 + 7] = 0xfe;
    memcpy(pev->pdata + 2 + 8, s_provisionNodeInfo.mac, ESP_NOW_ETH_ALEN); // Destination GUID
    memset(pev->pdata + 2 + 14, 0, 2);
    memcpy(pev->pdata + 2 + 16, pconfig->pmk, 32);

    // Send for set key events
    for (int i = 0; i < VSCP_ESPNOW_SRV_SEND_KEY_CNT; i++) {
      if (ESP_OK != (ret = vscp_espnow_sendEvent(s_provisionNodeInfo.mac,
                                             pev,
                                             s_provisionNodeInfo.keyLocal,
                                             VSCP_ESPNOW_SET_KEY_INTERVAL / portTICK_PERIOD_MS))) {
        ESP_LOGE(TAG, "Failed to send provisioning setkey event %d rv=%X", i, ret);
      }
    }

    // Wait for confirmation of new node online from client
    uxBits = xEventGroupWaitBits(s_vscp_espnow_event_group,
                                 VSCP_ESPNOW_PROV_CLIENT_GOT_INIT2_BIT,
                                 pdTRUE,
                                 pdTRUE,
                                 20000 / portTICK_PERIOD_MS);
    // Bit is set if heart beat event from node received (auto cleared after above return)
    if (uxBits & VSCP_ESPNOW_PROV_CLIENT_GOT_INIT2_BIT) {
      ;
    }

    ESP_LOGI(TAG, "[srvprov] INIT2");
  }

ERROR:
  esp_now_del_peer(s_provisionNodeInfo.mac);
  s_stateDroplet = VSCP_ESPNOW_STATE_IDLE;
  ESP_LOGI(TAG, "End initialization sequency");
  vTaskDelete(NULL);
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_startServerProvisioning
//

int
vscp_espnow_startServerProvisioning(const uint8_t *pmac, const uint8_t *pkey)
{
  if ((NULL == pmac) || (NULL == pkey)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Save provision parameters
  memcpy(s_provisionNodeInfo.mac, pmac, ESP_NOW_ETH_ALEN);
  memcpy(s_provisionNodeInfo.keyLocal, pkey, 16);

  ESP_LOG_BUFFER_HEXDUMP(TAG, s_provisionNodeInfo.mac, ESP_NOW_ETH_ALEN, ESP_LOG_INFO);
  ESP_LOG_BUFFER_HEXDUMP(TAG, s_provisionNodeInfo.keyLocal, 32, ESP_LOG_INFO);

  if (VSCP_ESPNOW_STATE_IDLE == s_stateDroplet) {
    s_stateDroplet = VSCP_ESPNOW_STATE_SRV_INIT1;
    xTaskCreate(&vscp_espnow_server_provisioning_task, "prov_server_task", 1024 * 4, &s_vscp_espnow_config, 5, NULL);
  }
  return VSCP_ERROR_SUCCESS;
}