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

#ifndef VSCP_ESPNOW_H
#define VSCP_ESPNOW_H

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <esp_wifi_types.h>

#include <vscp.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VSCP_ESPNOW_VERSION 0

// Frame id

#define VSCP_ESPNOW_ID_MSB 0x55
#define VSCP_ESPNOW_ID_LSB (0xA0 + VSCP_ESPNOW_VERSION)

/**
 * @brief Frame positions for data in the VSCP esp-now frame
 */

// Identify as esp-now frame (0x55/0xAA)  X = esp-now protocol version
#define VSCP_ESPNOW_POS_ID 0

// 0xab where b = esp-now protocol version and a = type (alpha/beta...)
#define VSCP_ESPNOW_POS_TYPE_VER 2

// VSCP content
#define VSCP_ESPNOW_POS_HEAD       3  // VSCP head bytes (2)
#define VSCP_ESPNOW_POS_NICKNAME   4  // Node nickname (2)
#define VSCP_ESPNOW_POS_VSCP_CLASS 5  // VSCP class (2)
#define VSCP_ESPNOW_POS_VSCP_TYPE  6  // VSCP Type (2)
#define VSCP_ESPNOW_POS_SIZE       11 // Data size (needed because of encryption padding) (1)
#define VSCP_ESPNOW_POS_DATA       12 // VSCP data (max 128 bytes)

#define VSCP_ESPNOW_MIN_FRAME VSCP_ESPNOW_POS_DATA // Number of bytes in minimum frame
#define VSCP_ESPNOW_MAX_DATA                                                                                           \
  (ESPNOW_SEC_PACKET_MAX_SIZE - VSCP_ESPNOW_MIN_FRAME) // Max VSCP data (of possible 512 bytes) that a frame can hold
#define VSCP_ESPNOW_MAX_FRAME (ESPNOW_SEC_PACKET_MAX_SIZE - VSCP_ESPNOW_MIN_FRAME)

/*
  Note on  max data size
  ----------------------
  An esp-now frame can hold a payload of max 250 bytes
  IV is 16 bytes
  VSCP frame data is 16 -bytes
  So left for Droplet data is 250-16-15 = 219 bytes
*/

typedef enum {
  VSCP_ESPNOW_ALPHA_NODE = 0,
  VSCP_ESPNOW_BETA_NODE,
  VSCP_ESPNOW_GAMMA_NODE,
} vscp_espnow_node_type_t;

/*
  The idel state is the normal state a node is in. This is where it does all it's
  work if it has been initialized.

  Alpha nodes can only be in the idle or one of the SRV states.
  Beta nodes can be both in one of the SRV states and in one of the CLIENT states and in idle.
  Gamma nodes can only be in CLIENT state and idle.
*/
typedef enum {
  VSCP_ESPNOW_STATE_IDLE,        // Normal state for all nodes, but may be uninitialized.
  VSCP_ESPNOW_STATE_CLIENT_INIT, // Initialization state for Beta/Gamma nodes.
  VSCP_ESPNOW_STATE_SRV_INIT1,   // Server initialization state 1 (Alpha/Beta nodes)  Waiting for heartbeat.
  VSCP_ESPNOW_STATE_SRV_INIT2,   // Server initialization state 2 (Alpha/Beta nodes). Waiting for new node on-line
  VSCP_ESPNOW_STATE_CLIENT_OTA,  // OTA state for all nodes being updated.
  VSCP_ESPNOW_STATE_SRV_OTA      // OTA state for Alpha/Beta/Gamma nodes that serve firmware.
} vscp_espnow_state_t;

/**
 * @brief Initialize the configuration of esp-now
 */
typedef struct {
  vscp_espnow_node_type_t nodeType; // Alpha/Gamma/Beta
  uint8_t channel;              // Channel to use (zero is current)
  uint8_t ttl;                  // Default ttl
  bool bForwardEnable;          // Forward when packets are received
  bool bForwardSwitchChannel;   // Forward data packet with exchange channel
  uint8_t sizeQueue;            // Size of receive queue
  uint8_t nEncryption;          // 0=no encryption, 1=AES-128, 2=AES-192, 3=AES-256
  bool bFilterAdjacentChannel;  // Don't receive if from other channel
  int filterWeakSignal;         // Filter onm RSSI (zero is no rssi filtering)
  uint8_t *lkey;                // Pointer to 32 byte local key (16 (EAS128)/24(AES192)/32(AES256)) (Beta/Gammal nodes)
  uint8_t *pmk;                 // Pointer tp 32 byte primary master key (16 (EAS128)/24(AES192)/32(AES256))
  uint8_t *nodeGuid;            // Pointer to 16 byte GUID for node.
} vscp_espnow_config_t;

/**
 * @brief Send and receive statistics
 *
 */
typedef struct {
  uint32_t nSend;            // # sent frames
  uint32_t nSendFailures;    // Number of send failures
  uint32_t nSendLock;        // Number of send lock give ups
  uint32_t nSendAck;         // # of failed send confirms
  uint32_t nRecv;            // # received frames
  uint32_t nRecvOverruns;    // Number of receive overruns
  uint32_t nRecvFrameFault;  // Frame to big or to small
  uint32_t nRecvAdjChFilter; // Adjacent channel filter
  uint32_t nRecvŔssiFilter;  // RSSI filter stats
  uint32_t nForw;            // # Number of forwarded frames
} vscp_espnow_stats_t;

/**
 * @brief   Droplet type data receive callback function
 *
 * @param[in]  src_addr  peer MAC address
 * @param[in]  data  received data
 * @param[in]  size  length of received data
 * @param[in]  rx_ctrl  received packet radio metadata header
 *
 * @return
 *    - ESP_OK
 *    - ESP_ERR_INVALID_ARG
 */
typedef esp_err_t (*type_handle_t)(uint8_t *src_addr, void *data, size_t size, wifi_pkt_rx_ctrl_t *rx_ctrl);

#define VSCP_ESPNOW_MSG_CACHE_SIZE           32    // Size for magic cache
#define VSCP_ESPNOW_HEART_BEAT_INTERVAL      30000 // Milliseconds between heartbeat events
#define VSCP_ESPNOW_INIT_LOOPS               2     // Number of all channel loops
#define VSCP_ESPNOW_INIT_HEART_BEAT_INTERVAL 200   // Milliseconds between heartbeat probe events
#define VSCP_ESPNOW_SET_KEY_INTERVAL         100   // Provisioning interval in ms between set key events
#define VSCP_ESPNOW_SRV_SEND_KEY_CNT         3

// Control states for esp-now provisioning
typedef enum { VSCP_ESPNOW_CTRL_INIT, VSCP_ESPNOW_CTRL_BOUND, DEOPLET_CTRL_MAX } vscp_espnow_ctrl_status_t;

/**
 * @brief The channel on which the device sends packets
 */
#define VSCP_ESPNOW_CHANNEL_CURRENT 0x0  // Only in the current channel
#define VSCP_ESPNOW_CHANNEL_ALL     0x0f // All supported channels

#define VSCP_ESPNOW_FORWARD_MAX_COUNT 0xff // Maximum number of forwards

#define VSCP_ESPNOW_ADDR_LEN                  (6)
#define VSCP_ESPNOW_DECLARE_COMMON_ADDR(addr) extern const uint8_t addr[6];
#define VSCP_ESPNOW_ADDR_IS_EMPTY(addr)       (((addr)[0] | (addr)[1] | (addr)[2] | (addr)[3] | (addr)[4] | (addr)[5]) == 0x0)
#define VSCP_ESPNOW_ADDR_IS_BROADCAST(addr)                                                                            \
  (((addr)[0] & (addr)[1] & (addr)[2] & (addr)[3] & (addr)[4] & (addr)[5]) == 0xFF)
#define VSCP_ESPNOW_ADDR_IS_SELF(addr)          !memcmp(addr, VSCP_ESPNOW_ADDR_SELF, 6)
#define VSCP_ESPNOW_ADDR_IS_EQUAL(addr1, addr2) !memcmp(addr1, addr2, 6)

// Callback functions

// Callback for esp-now received events
typedef void (*vscp_event_handler_cb_t)(const vscpEvent *pev, void *userdata);

// Callback for client node attach to network
typedef void (*vscp_espnow_attach_network_handler_cb_t)(wifi_pkt_rx_ctrl_t *prxdata, void *userdata);

// ----------------------------------------------------------------------------

/**
 * @brief Initialize VSCP esp-now
 *
 * @param config Pointer to VSCP esp-now configuration
 * @return esp_err_t
 */
esp_err_t
vscp_espnow_init(const vscp_espnow_config_t *config);

/**
 * @brief Send vscp_espnow frame
 *
 * @param dest_addr Pointer to destination mac address. Normally broadcast 0xff,0xff,0xff,0xff,0xff,0xff
 * @param bPreserveHeader Set to true if header is already set in payload and need to be preserved. If false
 *                        ttl , magic etc will be set by the routine.
 * @param nEncrypt  Encryption type. Frame size will increase by 16 as the iv is appended to
 *                  the end of it. Valid values is
 *                  VSCP_ENCRYPTION_NONE           0
 *                  VSCP_ENCRYPTION_AES128         1
 *                  VSCP_ENCRYPTION_AES192         2
 *                  VSCP_ENCRYPTION_AES256         3
 * @param pkey Pointer to 32 bit key used for encryption.
 * @param ttl   Time to live for frame. Will be decrease by one for every hop.
 * @param payload The frame data.
 * @param size  The size of the payload.
 * @param wait_ms Milliseconds to wait for the frame to get sent.
 * @return esp_err_t ESP_OK is returned if all is OK
 */
esp_err_t
vscp_espnow_send(const uint8_t *dest_addr,
             bool bPreserveHeader,
             uint8_t nEncrypt,
             const uint8_t *pkey,
             uint8_t ttl,
             uint8_t *data,
             size_t size,
             uint16_t wait_ms);

/**
 * @brief Build full GUID from mac address
 *
 * @param pguid Pointer to GUID that will get data
 * @param pmac Pointer to six byte mac
 * @param nickname Nickname for node. Set to zero if not used.
 * @return int VSCP_ERROR_SUCCES is returned if all goes well. Otherwise VSCP error code is returned.
 */
int
vscp_espnow_build_guid_from_mac(uint8_t *pguid, const uint8_t *pmac, uint16_t nickname);

/**
 * @brief Construct VSCP level I heartbeat frame
 *
 * @param buf Pointer to buffer that will get the frame data
 * @param len Size of the buffer. Must be at least VSCP_ESPNOW_PACKET_MIN_SIZE + 3
 * @param pguid Pointer to node GUID. Can be NULL in which case the node id will be set to zero.
 * @return int VSCP_ERROR_SUCCES is returned if all goes well. Otherwise VSCP error code is returned.
 *
 * The user defined first byte and zone/subzone is predefined in this call. Zone information is set to 0xff
 * for all zones/subzones and the user defined byte is set to zero.
 */

int
vscp_espnow_build_l1_heartbeat(uint8_t *buf, uint8_t len, const uint8_t *pguid);

/**
 * @brief Construct VSCP level II heartbeat frame
 *
 * @param buf Pointer to buffer that will get the frame data
 * @param len Size of the buffer. Must be at least VSCP_ESPNOW_PACKET_MIN_SIZE + 3
 * @param pguid Pointer to node GUID. Can be NULL in which case the node id will be set to zero.
 * @param pname Pointer to node name or NULL in which case no name is set.
 * @return int VSCP_ERROR_SUCCES is returned if all goes well. Otherwise VSCP error code is returned.
 *
 */

int
vscp_espnow_build_l2_heartbeat(uint8_t *buf, uint8_t len, const uint8_t *pguid, const char *pname);

/**
 * @fn vscp_espnow_sendEvent
 * @brief  Send event on vscp_espnow network
 *
 * @param destAddr Destination address.
 * @param pev Event to send
 * @param wait_ms Time in milliseconds to wait for send
 * @return esp_err_t Error code. ESP_OK if all is OK.
 */

esp_err_t
vscp_espnow_sendEvent(const uint8_t *destAddr, const vscpEvent *pev, const uint8_t *pkey, uint32_t wait_ms);

/**
 * @fn vscp_espnow_sendEventEx
 * @brief Send event ex on vscp_espnow network
 *
 * @param destAddr Destination address.
 * @param pex Pointer to event ex to send.
 * @param pkey Pointer to 32 bit key used for encryption.
 * @param wait_ms Time in milliseconds to wait for send
 * @return esp_err_t Error code. ESP_OK if all is OK.
 */
esp_err_t
vscp_espnow_sendEventEx(const uint8_t *destAddr, const vscpEventEx *pex, const uint8_t *pkey, uint32_t wait_ms);

/**
 * @fn vscp_espnow_getMinBufSizeEv
 * @brief Get minimum buffer size for a VSCP event
 *
 * @param pev Pointer to event
 * @param pkey Pointer to 32 bit key used for encryption.
 * @return size_t Needed buffer size or zero for error (invalid event pointer).
 */
size_t
vscp_espnow_getMinBufSizeEv(vscpEvent *pev);

/**
 * @fn vscp_espnow_getMinBufSizeEx
 * @brief Get minimum buffer size for a VSCP ex event
 *
 * @param pex Pointer to event ex
 * @return size_t Needed buffer size or zero for error (invalid event pointer).
 */
size_t
vscp_espnow_getMinBufSizeEx(vscpEventEx *pex);

/**
 * @brief Construct VSCP ESP-NOW frame form event structure
 *
 * @param buf Pointer to buffer that will get the frame data
 * @param len Size of buffer. The buffer should have room for the frame plus VSCP data so it
 * should have a length that exceeds VSCP_ESPNOW_PACKET_MIN_SIZE + VSCP event data length.
 * @param pev Pointer to VSCP event which will have its content written to the buffer.
 * @return int VSCP_ERROR_SUCCES is returned if all goes well. Otherwise VSCP error code is returned.
 */

int
vscp_espnow_evToFrame(uint8_t *buf, uint8_t len, const vscpEvent *pev);

/**
 * @brief Construct VSCP ESP-NOW frame form event ex structure
 *
 * @param buf Pointer to buffer that will get the frame data
 * @param len Size of buffer. The buffer should have room for the frame plus VSCP data so it
 * should have a length that exceeds VSCP_ESPNOW_PACKET_MIN_SIZE + VSCP event data length.
 * @param pex Pointer to VSCP event ex which will have its content written to the buffer.
 * @return int VSCP_ERROR_SUCCES is returned if all goes well. Otherwise VSCP error code is returned.
 */

int
vscp_espnow_exToFrame(uint8_t *buf, uint8_t len, const vscpEventEx *pex);

/**
 * @brief Fill in Data of VSCP ex event from esp-now frame
 *
 * @param pev Pointer to VSCP event
 * @param buf  Buffer holding esp-now frame data
 * @param len  Len of buffer
 * @param timestamp The event timestamp normally comes from wifi_pkt_rx_ctrl_t in the wifi frame. If
 * set to zero  it will be set from tickcount
 * @return int VSCP_ERROR_SUCCES is returned if all goes well. Otherwise VSCP error code is returned.
 */
int
vscp_espnow_frameToEv(vscpEvent *pev, const uint8_t *buf, uint8_t len, uint32_t timestamp);

/**
 * @brief Fill in Data of VSCP ex event from esp-now frame
 *
 * @param pex Pointer to VSCP ex event
 * @param buf  Buffer holding esp-now frame data
 * @param len  Len of buffer
 * @param timestamp The event timestamp normally comes from wifi_pkt_rx_ctrl_t in the wifi frame. If
 * set to zero  it will be set from tickcount
 * @return int VSCP_ERROR_SUCCES is returned if all goes well. Otherwise VSCP error code is returned.
 */
int
vscp_espnow_frameToEx(vscpEventEx *pex, const uint8_t *buf, uint8_t len, uint32_t timestamp);

/**
 * @fn vscp_espnow_set_vscp_user_handler_cb
 * @brief Set the VSCP event receive handler callback
 *
 * @param cb Callback that can do work when when a VSCP event is received.
 *
 * Set the VSCP event receive handler callback
 *
 */
void
vscp_espnow_set_vscp_user_handler_cb(vscp_event_handler_cb_t cb);

/**
 * @fn vscp_espnow_clear_vscp_handler_cb
 * @brief Clear VSCP event receive handler callback
 *
 */
void
vscp_espnow_clear_vscp_handler_cb(void);

/**
 * @fn vscp_espnow_set_attach_network_handler_cb
 * @brief Set handler callback for network attach
 *
 * @param cb Callback that can do work when network attach occur.
 */

void
vscp_espnow_set_attach_network_handler_cb(vscp_espnow_attach_network_handler_cb_t cb);

/**
 * @fn vscp_espnow_clear_attach_network_handler_cb
 * @brief Clear handler callback for network attach
 *
 */
void
vscp_espnow_clear_attach_network_handler_cb(void);

/**
 * @fn vscp_espnow_parse_vscp_json
 * @brief Convert JSON string to VSCP event
 *
 * @param jsonVscpEventObj1
 * @param pev
 * @return int
 */
int
vscp_espnow_parse_vscp_json(vscpEvent *pev, const char *jsonVscpEventObj);

/**
 * @fn vscp_espnow_create_vscp_json
 * @brief Convert pointer to VSCP event to VSCP JSON string
 *
 * @param strObj String buffer that will get result
 * @param len Size of string buffer
 * @param pev Pointer to event
 * @return int Returns VSCP_ERROR_SUCCESS on OK, error code else.
 */
int
vscp_espnow_create_vscp_json(char *strObj, size_t len, vscpEvent *pev);

void
vscp_espnow_client_provisioning_task(void *pvParameter);

/**
 * @fn vscp_espnow_isClientInit1Set
 * @brief Check if client init event 1 has been received
 *
 * @return EventBits_t
 */

bool
vscp_espnow_isClientInit1Set(void);

/**
 * @fn vscp_espnow_isClientInit2Set
 * @brief Check if client init event 2 has been received
 *
 * @return EventBits_t
 */

bool
vscp_espnow_isClientInit2Set(void);

/**
 * @fn vscp_espnow_startClientProvisioning
 * @brief Start client provisioning task
 *
 * @return int VSCP_ERROR_SUCCESSif OK error code on failure.
 */

int
vscp_espnow_startClientProvisioning(void);

/**
 * @fn vscp_espnow_startServerProvisioning
 * @brief Start server provisioning task
 *
 * @param pmac Pointer to MAC address pf client node.
 * @param pkey Pointer to local key of client node.
 * @return int VSCP_ERROR_SUCCESSif OK error code on failure.
 */

int
vscp_espnow_startServerProvisioning(const uint8_t *pmac, const uint8_t *pkey);

#ifdef __cplusplus
}
#endif /**< _cplusplus */

#endif