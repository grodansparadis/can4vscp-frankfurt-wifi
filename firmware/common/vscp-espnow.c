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

#include <espnow.h>

#include <cJSON.h>

#include <vscp-firmware-helper.h>
#include <vscp.h>

#include "vscp-espnow.h"

static const char *TAG = "vscpnow";

static vscp_espnow_config_t s_vscp_espnow_config = { 0 };

#define VSCP_ESPNOW_MAX_BUFFERED_NUM                                                                                   \
  (CONFIG_ESP32_WIFI_DYNAMIC_TX_BUFFER_NUM / 2) /* Not more than CONFIG_ESP32_WIFI_DYNAMIC_TX_BUFFER_NUM */

// Free running counter that is updated for every sent frame
uint8_t g_vscp_espnow_sendSequence = 0;

// Number of send events in transit
uint32_t g_vscp_espnow_buffered_num = 0;

QueueHandle_t g_vscp_espnow_rcvqueue = NULL;

#define VSCP_ESPNOW_SEND_CB_OK_BIT            BIT0
#define VSCP_ESPNOW_SEND_CB_FAIL_BIT          BIT1
// #define VSCP_ESPNOW_PROV_CLIENT_GOT_INIT1_BIT BIT4 // Client new node on-line received
// #define VSCP_ESPNOW_PROV_CLIENT_GOT_INIT2_BIT BIT5 // Client probe ack received
// #define VSCP_ESPNOW_PROV_SRV_GOT_PMK_BIT      BIT6 // Provisioning key received

static uint8_t g_vscp_espnow_magic_cache_next = 0;

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
  uint8_t mac[6];       // MAC address for node to provision
  uint8_t keyLocal[16]; // Local key for node
} vscp_espnow_provisioning_t;

/*
  Info about node that is under provisioning
*/
static vscp_espnow_provisioning_t s_provisionNodeInfo = { 0 };

// Forward declarations

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
// vscp_espnow_getMinBufSizeEv
//

size_t
vscp_espnow_getMinBufSizeEv(const vscpEvent *pev)
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
vscp_espnow_getMinBufSizeEx(const vscpEventEx *pex)
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

  buf[VSCP_ESPNOW_POS_ID]     = 0x55;
  buf[VSCP_ESPNOW_POS_ID + 1] = 0xAA;

  buf[VSCP_ESPNOW_POS_TYPE_VER] = (PRJDEF_NODE_TYPE << 4) + VSCP_ESPNOW_VERSION;

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

  buf[VSCP_ESPNOW_POS_ID]     = 0x55;
  buf[VSCP_ESPNOW_POS_ID + 1] = 0xAA;

  buf[VSCP_ESPNOW_POS_TYPE_VER] = (PRJDEF_NODE_TYPE << 4) + VSCP_ESPNOW_VERSION;

  // head
  buf[VSCP_ESPNOW_POS_HEAD] = (pex->head >> 8) & 0xff;
  buf[VSCP_ESPNOW_POS_HEAD] = pex->head & 0xff;

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

int
vscp_espnow_sendEvent(const uint8_t *destAddr, const vscpEvent *pev, uint32_t wait_ms)
{
  esp_err_t rv;
  uint8_t *pbuf = NULL;
  size_t len    = vscp_espnow_getMinBufSizeEv(pev);

  // ESP_ERROR_RETURN(!s_vscp_espnow_config, VSCP_ERROR_INIT_MISSING, "ESPNOW is not initialized");

  // Need event
  if (NULL == pev) {
    ESP_LOGE(TAG, "Pointer to event is NULL");
    return VSCP_ERROR_INVALID_POINTER;
  }

  pbuf = VSCP_MALLOC(len);
  if (NULL == pbuf) {
    ESP_LOGE(TAG, "Failed to allocate memory for send buffer-");
    return VSCP_ERROR_MEMORY;
  }

  if (VSCP_ERROR_SUCCESS != (rv = vscp_espnow_evToFrame(pbuf, len, pev))) {
    VSCP_FREE(pbuf);
    ESP_LOGE(TAG, "Failed to convert event to frame. rv=%d", rv);
    return rv;
  }

  ESP_LOGI(TAG, "Send mac: " MACSTR ", version: %d", MAC2STR(destAddr), VSCP_ESPNOW_VERSION);

  // espnow_frame_head_t espnowhead = ESPNOW_FRAME_CONFIG_DEFAULT();
  // espnowhead.security = false;
  // espnowhead.broadcast = true;

  esp_err_t ret = espnow_send(ESPNOW_DATA_TYPE_DATA, destAddr, pbuf, len, NULL, wait_ms / portTICK_PERIOD_MS);
  if (ESP_OK != ret) {

    // Free the buffer
    VSCP_FREE(pbuf);

    if (ESP_ERR_INVALID_ARG == ret) {
      ESP_LOGE(TAG, "Invalid parameter");
      return VSCP_ERROR_PARAMETER;
    }
    else if (ESP_ERR_TIMEOUT == ret) {
      ESP_LOGE(TAG, "Timeout");
      return VSCP_ERROR_TIMEOUT;
    }
    else if (ESP_ERR_WIFI_TIMEOUT == ret) {
      ESP_LOGE(TAG, "Wifi timeout");
      return VSCP_ERROR_TIMEOUT;
    }
    else {
      ESP_LOGE(TAG, "Unknow error");
      return VSCP_ERROR_ERROR;
    }
  }

  VSCP_FREE(pbuf);
  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_sendEventEx
//

esp_err_t
vscp_espnow_sendEventEx(const uint8_t *destAddr, const vscpEventEx *pex, uint32_t wait_ms)
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

  // if (ESP_OK != (rv = vscp_espnow_send(destAddr, //  VSCP_ESPNOW_ADDR_BROADCAST,
  //                                  false,
  //                                  s_vscp_espnow_config.nEncryption,
  //                                  (pkey != NULL) ? pkey : s_vscp_espnow_config.pmk,
  //                                  s_vscp_espnow_config.ttl,
  //                                  pbuf,
  //                                  VSCP_ESPNOW_MIN_FRAME + pex->sizeData,
  //                                  wait_ms))) {
  //   ESP_LOGE(TAG, "Failed to send event. rv=%X", rv);
  //   VSCP_FREE(pbuf);
  //   return rv;
  // }

  VSCP_FREE(pbuf);
  return ESP_OK;
}

// void
// vscp_espnow_data_cb(uint8_t *src_addr, void *data, size_t size, wifi_pkt_rx_ctrl_t *rx_ctrl)
// {
// }

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_init
//

esp_err_t
vscp_espnow_init(size_t sizeQueue)
{
  s_stateDroplet = VSCP_ESPNOW_STATE_IDLE;

  g_vscp_espnow_rcvqueue = xQueueCreate(sizeQueue, sizeof(void *));
  if (NULL == g_vscp_espnow_rcvqueue) {
    ESP_LOGD(TAG, "Create vscp_espnow event queue fail");
    return ESP_FAIL;
  }

  // esp_err_t ret = espnow_set_config_for_data_type(ESPNOW_DATA_TYPE_DATA, true, vscp_espnow_data_cb);
  // if (ESP_OK != ret) {
  //   ESP_LOGE(TAG, "Failed to set VSCP event callback");
  // }

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
// vscp_espnow_probe
//

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

void
vscp_espnow_heartbeat_task(void *pvParameter)
{
  esp_err_t ret = 0;
  uint8_t buf[VSCP_ESPNOW_MIN_FRAME + 3]; // Three byte data
  size_t size = sizeof(buf);

  // vscp_espnow_config_t *pconfig = (vscp_espnow_config_t *) pvParameter;
  // if (NULL == pconfig) {
  //   ESP_LOGE(TAG, "Invalid (NULL) parameter given");
  //   return;
  // }

  vscpEvent *pev = vscp_fwhlp_newEvent();
  if (NULL == pev) {
    ESP_LOGE(TAG, "Unable to allocate heartbeat event");
    goto ERROR;
  }

  pev->pdata = VSCP_CALLOC(3);
  if (NULL == pev->pdata) {
    ESP_LOGE(TAG, "Unable to allocate heartbeat event data");
    vscp_fwhlp_deleteEvent(&pev);
    goto ERROR;
  }

  pev->vscp_class = VSCP_CLASS1_INFORMATION;
  pev->vscp_type = VSCP_TYPE_INFORMATION_NODE_HEARTBEAT;
  pev->sizeData   = 3;
  pev->pdata[0]   = 0; // index
  pev->pdata[1]   = 0; // zone
  pev->pdata[2]   = 0; // subzone
  pev->timestamp  = esp_timer_get_time();

  ESP_LOGI(TAG, "Start sending VSCP heartbeats");

  while (true) {

    if (VSCP_ESPNOW_STATE_IDLE == s_stateDroplet) {

      uint8_t ch                = 0;
      wifi_second_chan_t second = 0;

      if (ESP_OK != (ret = esp_wifi_get_channel(&ch, &second))) {
        ESP_LOGE(TAG, "Failed to get wifi channel, rv = %X", ret);
      }
      ESP_LOGI(TAG, "Sending heartbeat ch=%d (%d).", ch, second);

      vscp_espnow_sendEvent(ESPNOW_ADDR_BROADCAST, pev, 1000);

      vTaskDelay(VSCP_ESPNOW_HEART_BEAT_INTERVAL / portTICK_PERIOD_MS);
    }
  }

ERROR:
  if (NULL != pev) {
    vscp_fwhlp_deleteEvent(&pev);
  }
  ESP_LOGW(TAG, "Heartbeat task exit %d", ret);
  vTaskDelete(NULL);
}
