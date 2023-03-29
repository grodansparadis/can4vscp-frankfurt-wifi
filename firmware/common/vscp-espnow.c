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

// QueueHandle_t g_vscp_espnow_rcvqueue = NULL;

#define VSCP_ESPNOW_SEND_CB_OK_BIT   BIT0
#define VSCP_ESPNOW_SEND_CB_FAIL_BIT BIT1
// #define VSCP_ESPNOW_PROV_CLIENT_GOT_INIT1_BIT BIT4 // Client new node on-line received
// #define VSCP_ESPNOW_PROV_CLIENT_GOT_INIT2_BIT BIT5 // Client probe ack received
// #define VSCP_ESPNOW_PROV_SRV_GOT_PMK_BIT      BIT6 // Provisioning key received

static uint8_t g_vscp_espnow_magic_cache_next = 0;

static uint8_t VSCP_ESPNOW_ADDR_SELF[6] = { 0 };

const uint8_t VSCP_ESPNOW_ADDR_NONE[6]      = { 0 };
const uint8_t VSCP_ESPNOW_ADDR_BROADCAST[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0Xff };

/*!
  GUID for unassigned node.
*/
static uint8_t s_vscp_zero_guid[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/*!
  GUID used for a node that is uninitialized.
*/
static uint8_t s_vscp_uninit_guid[16] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE,
                                          0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00 };

// User handler for received vscp_espnow frames/events
static vscp_event_handler_cb_t s_vscp_event_handler_cb = NULL;

// User handler for clinet when attaching to network
static vscp_espnow_attach_network_handler_cb_t s_vscp_espnow_attach_network_handler_cb = NULL;

/*
  State machine state for the vscp_espnow stack
*/
static vscp_espnow_state_t s_stateVscpEspNow = VSCP_ESPNOW_STATE_IDLE;

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

// Statistics
static vscp_espnow_stats_t s_vscpEspNowStats;

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

  buf[VSCP_ESPNOW_POS_TYPE_VER] = (PRJDEF_NODE_TYPE << 6) + (VSCP_ESPNOW_VERSION << 4) + VSCP_ENCRYPTION_AES128;

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

  buf[VSCP_ESPNOW_POS_SIZE] = pev->sizeData;

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

  buf[VSCP_ESPNOW_POS_TYPE_VER] = (PRJDEF_NODE_TYPE << 6) + (VSCP_ESPNOW_VERSION << 4) + VSCP_ENCRYPTION_AES128;

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

  buf[VSCP_ESPNOW_POS_SIZE] = pex->sizeData;

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
  if ((buf[VSCP_ESPNOW_POS_ID] != 0x55) || (buf[VSCP_ESPNOW_POS_ID + 1] != 0xAA)) {
    ESP_LOGE(TAG, "esp-now data is an invalid frame");
    return VSCP_ERROR_INVALID_FRAME;
  }

  // To be sure
  memset(pev, 0, sizeof(vscpEvent));

  // Free any allocated event data
  if (NULL != pev->pdata) {
    VSCP_FREE(pev->pdata);
    pev->pdata = NULL;
  }

  // Set VSCP size
  pev->sizeData = MIN(buf[VSCP_ESPNOW_POS_SIZE], len - VSCP_ESPNOW_MIN_FRAME);
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
  pex->sizeData = buf[VSCP_ESPNOW_POS_SIZE];

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
// readRegister
//

int readRegister(uint8_t *dest_addr, uint32_t address, uint8_t value, uint32_t wait_ms)
{
  vscpEvent *pev = vscp_fwhlp_newEvent();
  if (NULL == pev) {
    ESP_LOGE(TAG, "[%s, %d]: Could not ", __func__, __LINE__);
    return VSCP_ERROR_MEMORY;
  }

  VSCP_FREE(pev);
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// writeRegister
//

int writeRegister(uint8_t *dest_addr, uint32_t address, uint8_t *value, uint32_t wait_ms)
{
  vscpEvent *pev = vscp_fwhlp_newEvent();
  if (NULL == pev) {
    return VSCP_ERROR_MEMORY;
  }

  VSCP_FREE(pev);
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_data_cb
//

static void
vscp_espnow_data_cb(uint8_t *src_addr, uint8_t *data, size_t size, wifi_pkt_rx_ctrl_t *rx_ctrl)
{
  int rv;

  if ((src_addr == NULL) || (data == NULL) || (rx_ctrl == NULL) || (size <= 0)) {
    ESP_LOGE(TAG, "Receive cb arg error");
    return;
  }

  ESP_LOGI(TAG,
           "<<< Receive event from " MACSTR " to " MACSTR " , RSSI %d Channel %d, size %zd",
           MAC2STR((src_addr)),
           MAC2STR((src_addr)),
           rx_ctrl->rssi,
           rx_ctrl->channel,
           size);

  // Check that frame length is within limits
  if ((size < VSCP_ESPNOW_MIN_FRAME) || (size > VSCP_ESPNOW_MAX_FRAME) ||
      ((data[VSCP_ESPNOW_POS_TYPE_VER] & 0x0f) > VSCP_ENCRYPTION_AES256)) {
    ESP_LOGE(TAG,
             "[%s, %d]: Frame length/type is invalid len=%d (%d) type=%d",
             __func__,
             __LINE__,
             size,
             VSCP_ESPNOW_MAX_FRAME,
             data[VSCP_ESPNOW_POS_TYPE_VER]);

    ESP_LOG_BUFFER_HEXDUMP(TAG, data, size, ESP_LOG_DEBUG);

    s_vscpEspNowStats.nRecvFrameFault++; // Increase receive frame faults
    return;
  }

  // Check frame id and vscp espnow protocol version
  if ((data[VSCP_ESPNOW_POS_ID] != 0x55) || (data[VSCP_ESPNOW_POS_ID + 1] != 0xAA)) {
    ESP_LOGW(TAG,
             "Frame is invalid. id=%X,  protocol version=%d",
             (data[VSCP_ESPNOW_POS_ID] << 8) + data[VSCP_ESPNOW_POS_ID + 1],
             data[VSCP_ESPNOW_POS_ID + 1] & 0xf);
    return;
  }

  vscpEvent *pev = vscp_fwhlp_newEvent();
  if (NULL == pev) {
    ESP_LOGE(TAG, "[%s, %d]: Could not ", __func__, __LINE__);
    return;
  }

  if (VSCP_ERROR_SUCCESS != vscp_espnow_frameToEv(pev, data, size, rx_ctrl->timestamp)) {
    vscp_fwhlp_deleteEvent(&pev);
    return;
  }

  if ((VSCP_CLASS1_PROTOCOL == pev->vscp_class) && (VSCP_TYPE_PROTOCOL_NEW_NODE_ONLINE == pev->vscp_type) && (16 == pev->sizeData)) {

    ESP_LOGI(TAG, "New node on-line");

    espnow_frame_head_t espnowhead = {
      .security                = false,
      .retransmit_count        = 10,
      .broadcast               = true,
      .filter_adjacent_channel = false,
      .channel                 = rx_ctrl->channel,
      .forward_ttl             = 10,
      .forward_rssi            = -55,
      .filter_weak_signal      = true,
    };

    esp_err_t ret =
      espnow_send(ESPNOW_DATA_TYPE_DATA, ESPNOW_ADDR_BROADCAST, "yes", 3, &espnowhead, pdMS_TO_TICKS(500));
  }

  

  // ESP_LOGD(TAG, "Before decryption\n");
  // ESP_LOG_BUFFER_HEXDUMP(TAG, data, size, ESP_LOG_DEBUG);

  /*
    It is normally not good practice to take time to decrypt the frame here
    but this is also done in the espnow main thread for encrypted
    data.
  */

  // * * * Decrypt frame if needed * * *

  // uint8_t nEncryption = data[VSCP_ESPNOW_POS_TYPE_VER] & 0x0f;

  // Allocate space for data
  // uint8_t *pdata = VSCP_MALLOC(size);
  // if (NULL == pdata) {
  //   ESP_LOGE(TAG, "Unable to allocate data. Terminating");
  //   return;
  // }

  // if (VSCP_ERROR_SUCCESS != (rv = vscp_fwhlp_decryptFrame(pdata + 2,
  //                                                         data + 2,
  //                                                         size - 2,
  //                                                         s_vscp_espnow_config.pmk, // key
  //                                                         NULL,                     // IV  - use embedded
  //                                                         nEncryption))) {
  //   ESP_LOGE(TAG, "Failed to decrypt frame rv = %d", rv);
  //   VSCP_FREE(pdata);
  //   return;
  // }
  // memcpy(pdata, data, size);

  // Restore header
  // pdata[0] = 0x55;
  // pdata[1] = 0xaa;

  // if (nEncryption) {
  //   size -= 16; // no need to send the iv
  // }

  // ESP_LOGD(TAG, "After decrypt\n");
  // ESP_LOG_BUFFER_HEXDUMP(TAG, pdata, size, ESP_LOG_DEBUG);

  // Send to callback (if one is defined)
  // esp_err_t ret;
  // uint8_t ch                = 0;
  // wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
  // if (ESP_OK != (ret = esp_wifi_get_channel(&ch, &second))) {
  //   ESP_LOGE(TAG, "Failed to get wifi channel, rv = %X", ret);

ESP_LOGI(TAG,
         ">> esp-now data received. len=%zd ch=%d src=" MACSTR " rssi=%d",
         size,
         rx_ctrl->channel,
         MAC2STR(src_addr),
         rx_ctrl->rssi);

// vscpEvent *pev = vscp_fwhlp_newEvent();
// if (NULL == pev) {
//   ESP_LOGE(TAG, "Unable to allocate event");
//   VSCP_FREE(pdata);
//   return;
// }

// if (VSCP_ERROR_SUCCESS != (rv = vscp_espnow_frameToEv(pev, pdata, size, rx_ctrl->timestamp))) {
//   ESP_LOGE(TAG, "Failed to convert frame to VSCP event.");
//   goto EXIT;
// }

ESP_LOGI(TAG,
         ">> class=%d, type=%d sizedata=%d timestamp=%lX\n",
         pev->vscp_class,
         pev->vscp_type,
         pev->sizeData,
         pev->timestamp);

//EXIT: 
  vscp_fwhlp_deleteEvent(&pev);
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_sendEvent
//

int
vscp_espnow_sendEvent(const uint8_t *destAddr,
                      const vscpEvent *pev,
                      const uint8_t *pkey,
                      uint8_t nEncryption,
                      uint32_t wait_ms)
{
  esp_err_t rv;
  uint8_t *pbuf    = NULL;
  size_t len       = vscp_espnow_getMinBufSizeEv(pev);
  size_t frame_len = len;

  // ESP_ERROR_RETURN(!s_vscp_espnow_config, VSCP_ERROR_INIT_MISSING, "ESPNOW is not initialized");

  // Need dest address
  if (NULL == destAddr) {
    ESP_LOGE(TAG, "Pointer to destAddr is NULL");
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Need event
  if (NULL == pev) {
    ESP_LOGE(TAG, "Pointer to event is NULL");
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Need key
  if (NULL == pkey) {
    ESP_LOGE(TAG, "Pointer to key is NULL");
    return VSCP_ERROR_INVALID_POINTER;
  }

  pbuf = VSCP_CALLOC(len);
  if (NULL == pbuf) {
    ESP_LOGE(TAG, "Failed to allocate memory for send buffer-");
    return VSCP_ERROR_MEMORY;
  }

  if (VSCP_ERROR_SUCCESS != (rv = vscp_espnow_evToFrame(pbuf, len, pev))) {
    VSCP_FREE(pbuf);
    ESP_LOGE(TAG, "Failed to convert event to frame. rv=%d", rv);
    return rv;
  }

  // Set encryption
  pbuf[VSCP_ESPNOW_POS_TYPE_VER] = (PRJDEF_NODE_TYPE << 6) + (VSCP_ESPNOW_VERSION << 4) + (nEncryption & 0x0f);

  // ESP_LOG_BUFFER_HEXDUMP(TAG, pbuf, len, ESP_LOG_DEBUG);

  ESP_LOGD(TAG, "Send mac: " MACSTR ", version: %d", MAC2STR(destAddr), VSCP_ESPNOW_VERSION);

  uint8_t *outbuf = VSCP_CALLOC(len + (16 - (len % 16)) + 16 +
                                30); // len + padding + iv + head (padding not including type and ver bytes)
  if (NULL == outbuf) {
    VSCP_FREE(pbuf);
    ESP_LOGE(TAG, "Unable to allocate memory for encryption buffer");
    return VSCP_ERROR_MEMORY;
  }

  ESP_LOGD(TAG, "Size of outbuf %d\n", len + (16 - (len % 16)) + 16 + 3);

  // Encrypt frame if requested to do so
  if (0 && (NULL != pkey) && (VSCP_ENCRYPTION_NONE != nEncryption)) {

    // Fill in iv at end of send frame
    uint8_t *piv = VSCP_CALLOC(VSCP_ESPNOW_IV_LEN);
    if (NULL == piv) {
      ESP_LOGE(TAG, "Unable to allocate memory for iv");
      VSCP_FREE(outbuf);
      VSCP_FREE(pbuf);
      return VSCP_ERROR_MEMORY;
    }

    esp_fill_random(piv, VSCP_ESPNOW_IV_LEN);

    ESP_LOGD(TAG, "---> IV:  len = %zd encryption=%d\n", len, nEncryption);
    // ESP_LOG_BUFFER_HEXDUMP(TAG, piv, 16, ESP_LOG_DEBUG);

    // frame_len += frame_len - 1 + (16 - (frame_len % 16)) + 1;
    if (0 == (frame_len = vscp_fwhlp_encryptFrame(outbuf + 2,
                                                  pbuf + 2,
                                                  len - 2,
                                                  pkey, // key
                                                  piv,  // IV
                                                  nEncryption))) {
      ESP_LOGE(TAG, "Failed to encrypt frame");
      VSCP_FREE(pbuf);
      VSCP_FREE(piv);
      return VSCP_ERROR_ERROR;
    }

    outbuf[0] = 0x55;
    outbuf[1] = 0xaa;
    frame_len += 2;

    // ESP_LOG_BUFFER_HEXDUMP(TAG, outbuf, frame_len, ESP_LOG_DEBUG);

    VSCP_FREE(piv);
  }
  else {
    frame_len = len;
    memcpy(outbuf, pbuf, len);
  }

  // Not needed anymore
  VSCP_FREE(pbuf);

  espnow_frame_head_t espnowhead     = ESPNOW_FRAME_CONFIG_DEFAULT();
  espnowhead.security                = true;
  espnowhead.broadcast               = true;
  espnowhead.filter_adjacent_channel = false;
  // espnowhead.channel                 = ESPNOW_CHANNEL_ALL;
  espnowhead.forward_ttl        = 10;
  espnowhead.forward_rssi       = -55;
  espnowhead.filter_weak_signal = true;

  esp_err_t ret = espnow_send(ESPNOW_DATA_TYPE_DATA, destAddr, outbuf, frame_len, &espnowhead, pdMS_TO_TICKS(wait_ms));
  if (ESP_OK != ret) {

    if (ESP_ERR_INVALID_ARG == ret) {
      ESP_LOGE(TAG, "Invalid parameter");
      rv = VSCP_ERROR_PARAMETER;
      goto ERROR;
    }
    else if (ESP_ERR_TIMEOUT == ret) {
      ESP_LOGE(TAG, "Timeout");
      rv = VSCP_ERROR_TIMEOUT;
      goto ERROR;
    }
    else if (ESP_ERR_WIFI_TIMEOUT == ret) {
      ESP_LOGE(TAG, "Wifi timeout");
      rv = VSCP_ERROR_TIMEOUT;
      goto ERROR;
    }
    else {
      ESP_LOGE(TAG, "Unknow error %X", ret);
      rv = VSCP_ERROR_ERROR;
      goto ERROR;
    }
  }

  rv = VSCP_ERROR_SUCCESS;

ERROR:
  VSCP_FREE(outbuf);
  return rv;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_sendEventEx
//

esp_err_t
vscp_espnow_sendEventEx(const uint8_t *destAddr,
                        const vscpEventEx *pex,
                        const uint8_t *pkey,
                        uint8_t nEncryption,
                        uint32_t wait_ms)
{
  esp_err_t rv;
  uint8_t *pbuf;

  ESP_LOGI(TAG, "Send Event");

  // Need event
  if (NULL == pex) {
    ESP_LOGE(TAG, "Pointer to event ex is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  // Need dest address
  if (NULL == destAddr) {
    ESP_LOGE(TAG, "Pointer to destAddr is NULL");
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Need key
  if (NULL == pkey) {
    ESP_LOGE(TAG, "Pointer to key is NULL");
    return VSCP_ERROR_INVALID_POINTER;
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

  // Set encryption
  pbuf[VSCP_ESPNOW_POS_TYPE_VER] = (PRJDEF_NODE_TYPE << 6) + (VSCP_ESPNOW_VERSION << 4) + (nEncryption & 0x0f);

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

static const uint8_t scan_channel_sequence[] = { 1, 6, 11, 1, 6, 11, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13 };
#define RESEND_SCAN_COUNT_MAX (sizeof(scan_channel_sequence) * 2)

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_probe
//

int
vscp_espnow_probe(void)
{
  int rv = VSCP_ERROR_SUCCESS;

  vscpEvent *pev = vscp_fwhlp_newEvent();
  if (NULL == pev) {
    ESP_LOGE(TAG, "Failed to allocate memory for event");
    return VSCP_ERROR_MEMORY;
  }

  // GUID is data
  pev->pdata = VSCP_CALLOC(16);
  if (NULL == pev->pdata) {
    VSCP_FREE(pev);
    ESP_LOGE(TAG, "Failed to allocate memory for event data");
    return VSCP_ERROR_MEMORY;
  }

  pev->head       = 0;
  pev->timestamp  = esp_timer_get_time();
  pev->vscp_class = VSCP_CLASS1_PROTOCOL;
  pev->vscp_type  = VSCP_TYPE_PROTOCOL_NEW_NODE_ONLINE;

  // Set uninitialized
  pev->sizeData = VSCP_SIZE_GUID;
  memcpy(pev->pdata, s_vscp_uninit_guid, VSCP_SIZE_GUID);

  size_t len    = vscp_espnow_getMinBufSizeEv(pev);
  uint8_t *pbuf = VSCP_MALLOC(len);
  if (NULL == pbuf) {
    ESP_LOGE(TAG, "Failed to allocate memory for event buffer");
    vscp_fwhlp_deleteEvent(&pev);
    return VSCP_ERROR_MEMORY;
  }

  if (VSCP_ERROR_SUCCESS != (rv = vscp_espnow_evToFrame(pbuf, len, pev))) {
    ESP_LOGE(TAG, "Failed to convert event to frame");
    return rv;
  }

  for (int i = 0; i < RESEND_SCAN_COUNT_MAX; i++) {

    espnow_frame_head_t espnowhead = {
      .security                = false,
      .broadcast               = true,
      .retransmit_count        = 10,
      .magic                   = esp_random(),
      .ack                     = true,
      .filter_adjacent_channel = true,
      .channel                 = scan_channel_sequence[i % sizeof(scan_channel_sequence)],
      .forward_ttl             = 10,
      .forward_rssi            = -55,
      .filter_weak_signal      = true,
    };

    esp_err_t ret =
      espnow_send(ESPNOW_DATA_TYPE_DATA, ESPNOW_ADDR_BROADCAST, pbuf, len, &espnowhead, pdMS_TO_TICKS(100));
    if (ESP_OK != ret) {

      if (ESP_ERR_INVALID_ARG == ret) {
        ESP_LOGE(TAG, "Invalid parameter sending probe event");
        rv = VSCP_ERROR_PARAMETER;
        goto EXIT;
      }
      else if (ESP_ERR_TIMEOUT == ret) {
        ESP_LOGE(TAG, "Timeout sending probe event");
        rv = VSCP_ERROR_TIMEOUT;
        goto EXIT;
      }
      else if (ESP_ERR_WIFI_TIMEOUT == ret) {
        ESP_LOGE(TAG, "Wifi timeout sending probe event");
        rv = VSCP_ERROR_TIMEOUT;
        goto EXIT;
      }
      else {
        ESP_LOGE(TAG, "Unknow error %X  sending probe event", ret);
        rv = VSCP_ERROR_ERROR;
        goto EXIT;
      }
    }
  }
EXIT:
  vscp_fwhlp_deleteEvent(&pev);
  VSCP_FREE(pbuf);
  return rv;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_heartbeat_task
//
// Sent periodically as a broadcast to all zones/subzones
//

void
vscp_espnow_heartbeat_task(void *pvParameter)
{
  uint8_t buf[VSCP_ESPNOW_MIN_FRAME + 3]; // Three byte data
  size_t size = sizeof(buf);

  // vscp_espnow_config_t *pconfig = (vscp_espnow_config_t *) pvParameter;
  // if (NULL == pconfig) {
  //   ESP_LOGE(TAG, "Invalid (NULL) parameter given");
  //   vTaskDelete(NULL);
  // }

  vscpEvent *pev = vscp_fwhlp_newEvent();
  if (NULL == pev) {
    ESP_LOGE(TAG, "Unable to allocate heartbeat event");
    goto ERROR;
  }

  pev->pdata = VSCP_CALLOC(3);
  if (NULL == pev->pdata) {
    ESP_LOGE(TAG, "Unable to allocate heartbeat event data");
    goto ERROR;
  }

  pev->vscp_class = VSCP_CLASS1_INFORMATION;
  pev->vscp_type  = VSCP_TYPE_INFORMATION_NODE_HEARTBEAT;
  pev->sizeData   = 3;
  pev->pdata[0]   = 0xff; // index
  pev->pdata[1]   = 0xff; // zone
  pev->pdata[2]   = 0xff; // subzone
  pev->timestamp  = esp_timer_get_time();

  ESP_LOGI(TAG, "Start sending VSCP heartbeats");

  while (true) {

    if (VSCP_ESPNOW_STATE_IDLE == s_stateVscpEspNow) {

      esp_err_t ret;
      uint8_t ch                = 0;
      wifi_second_chan_t second = 0;

      if (ESP_OK != (ret = esp_wifi_get_channel(&ch, &second))) {
        ESP_LOGE(TAG, "Failed to get wifi channel, rv = %X", ret);
      }

      ESP_LOGI(TAG, "Sending heartbeat ch=%d (%d).", ch, second);

      vscp_espnow_sendEvent(ESPNOW_ADDR_BROADCAST,
                            pev,
                            s_vscp_espnow_config.pmk,
                            VSCP_ENCRYPTION_AES128,
                            pdMS_TO_TICKS(1000));

      vTaskDelay(pdMS_TO_TICKS(VSCP_ESPNOW_HEART_BEAT_INTERVAL));
    }
  }

ERROR:
  if (NULL != pev) {
    vscp_fwhlp_deleteEvent(&pev);
  }
  ESP_LOGW(TAG, "Heartbeat task exit");
  vTaskDelete(NULL);
}

///////////////////////////////////////////////////////////////////////////////
// vscp_espnow_init
//

esp_err_t
vscp_espnow_init(const vscp_espnow_config_t *pconfig)
{
  esp_err_t ret;
  s_stateVscpEspNow = VSCP_ESPNOW_STATE_IDLE;

  s_vscp_espnow_config.pmk   = pconfig->pmk;
  s_vscp_espnow_config.lmk   = pconfig->lmk;
  s_vscp_espnow_config.pguid = pconfig->pguid;

  ret = espnow_set_config_for_data_type(ESPNOW_DATA_TYPE_DATA, true, vscp_espnow_data_cb);
  if (ESP_OK != ret) {
    ESP_LOGE(TAG, "Failed to set VSCP event callback");
  }

  esp_wifi_get_mac(ESP_IF_WIFI_STA, VSCP_ESPNOW_ADDR_SELF);
  ESP_LOGD(TAG, "mac: " MACSTR ", version: %d", MAC2STR(VSCP_ESPNOW_ADDR_SELF), VSCP_ESPNOW_VERSION);

  // Start heartbeat task vscp_heartbeat_task
  xTaskCreate(&vscp_espnow_heartbeat_task, "vscp_hb", 1024 * 3, NULL, 7 /*tskIDLE_PRIORITY + 1*/, NULL);

  return ESP_OK;
}
