/*
 *  Copyright (c) 2020, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file implements the OpenThread platform abstraction for ToBLE communication.
 *
 */

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "platform-nrf5.h"

#include <openthread-core-config.h>
#include <openthread/config.h>

#include <openthread-system.h>
#include <openthread/platform/ble.h>
#include <openthread/platform/toble.h>
#include <openthread/platform/toolchain.h>

#include <common/code_utils.hpp>
#include <common/logging.hpp>
#include <utils/code_utils.h>
#include <openthread/error.h>
#include <openthread/instance.h>

#include "nrf_sdh_ble.h"
#include "softdevice.h"

#include <common/logging.hpp>

#if OPENTHREAD_CONFIG_TOBLE_ENABLE && SOFTDEVICE_PRESENT

/**
 * The maximum number of charactertistics and descriptors returned via service discovery.
 *
 */
#define MAX_CHARACTERISTICS_COUNT 2
#define MAX_DESCRIPTORS_COUNT 2

#define UUID_SERVICE_GAP 0x1800
#define UUID_SERVICE_GATT 0x1801

/// Amount of time for exchanging packets each connection interval [units of 1.25 ms]
#define BLE_DEFAULT_CONNECTION_EVENT_LENGTH 320

/// Default BLE LL PDU size [bytes] (valid range: 27 to 251)
#define BLE_DEFAULT_PDU_SIZE 27

/// Default number of retries when no communication happens [units of connection interval]
#define BLE_DEFAULT_CONNECTION_RETRY 10

/// BLE time unit.
#define BLE_TIME_UNIT_USEC 625
#define USEC_PER_MSEC 1000

#define MS_TO_BLE_TIME_UNIT(ms) (((ms)*USEC_PER_MSEC) / BLE_TIME_UNIT_USEC)

#define BLE_CONNECTION_INTERVAL_UNIT 1250
#define MS_TO_BLE_CONNECTION_INTERVAL_UNIT(ms) (((ms)*USEC_PER_MSEC) / BLE_CONNECTION_INTERVAL_UNIT)

#define BLE_L2CAP_MTU 1000
#define BLE_L2CAP_PSM 0x90

#define BLE_TASK_COUNT 10

/**
 * Macro to find the minimum of two values.
 */
#define MIN(a, b) ((a) < (b) ? (a) : (b))

/**
 * BLE Radio states.
 */
typedef enum
{
    kStateUninitialized,                 ///< BLE radio is uninitialized.
    kStateIdle,                          ///< BLE radio is in idle state.
    kStateAdvertising,                   ///< BLE radio advertising.
    kStateConnectionRequest,             ///< Connection request has been sent, and waiting for connection response.
    kStateGattiDiscoverUuidC1Handle,     ///<
    kStateGattiDiscoverUuidC2Handle,     ///<
    kStateGattiDiscoverUuidC2CccdHandle, ///<
    kStateGattSubscribing,               ///<
} BleState;

/**
 * BLE Control structure.
 */
typedef struct
{
    BleState              mState;                                      ///< Actual state of BLE Radio driver.
    ble_gap_conn_params_t mConnParams;                                 ///< Current connection parameters.
    ble_gap_adv_data_t    mAdvDataInfo;                                ///< GAP advertising data buffers info.
    uint8_t               mAdvData[BLE_GAP_ADV_SET_DATA_SIZE_MAX];     ///< GAP advertising data.
    uint8_t               mScanRspData[BLE_GAP_ADV_SET_DATA_SIZE_MAX]; ///< GAP scan response data.
    uint8_t               mAdvHandle;                                  ///< GAP advertising handle.
    uint8_t               mScanBuff[BLE_GAP_SCAN_BUFFER_MAX];          ///< GAP scanning buffer.
    uint16_t              mAdvInterval;                                ///< Advertising interval.
    otTobleAdvType        mAdvType;

    ble_gattc_handle_range_t mServiceHandleRange;

    uint16_t mC1Handle;
    uint16_t mC2Handle;
    uint16_t mC2CccdHandle;
} BleControl;

/**
 * BLE Peer structure.
 */
typedef struct
{
    bool     mIsUsed;       ///< Indicate if this structure is used.
    bool     mIsPeripheral; ///< Indicate if the peer is a peripheral.
    uint16_t mConnHandle;   ///< BLE GAP connection handle.
    uint16_t mAttMtu;       ///< Attribute MTU size.
    uint16_t mLocalCid;     ///< L2cap local CID.

    otTobleConnectionLinkType mLinkType; ///< Toble Link type.
} BlePeer;

typedef enum
{
    kHandleConnected,
    kHandleDisconnected,
    kHandleScanResponse,
    kHandleAdvertisement,
    kHandleC1Write,
    kHandleC1WriteDone,
    kHandleC2Subscribed,
    kHandleC2Notification,
    kHandleC2NotificationDone,
    kHandleConnectionIsReady,
    kHandleL2CapFrameReceived,
    kHandleL2CapScanSendDone,
} BleTaskType;

typedef struct
{
    BleTaskType        mType;
    otTobleConnection *mConnection;
    uint8_t            mBuffer[255];
    uint16_t           mBufferLength;
    otTobleAdvType     mAdvertisementType;
    otTobleAdvPacket   mAdvertisement;
    bool               mSubscribed;
    uint8_t            mCount;
    otTobleConnectionLinkType mLinkType;
    bool               mPending;
} BlePendingTask;

static BlePendingTask sPendingTasks[BLE_TASK_COUNT];
uint8_t sTaskQueueHead = 0;
uint8_t sTaskQueueTail = 0;

/**
 *  Control BLE structre.
 */
static BleControl sBle;

/**
 *  BLE Peers structre.
 */
static BlePeer sBlePeers[NRF_SDH_BLE_TOTAL_LINK_COUNT];

static BlePeer *sPeripheralPeer;
static bool     sDiagMode;

/**
 * OpenThread instance.
 */
static otInstance *sInstance = NULL;

/**
 * Buffer for L2CAP SDU
 */
static uint8_t sL2capRxBuffer[BLE_DEFAULT_L2CAP_MAX_MTU_SIZE];

/**
 * Characteristic C1 UUID: 18EE2EF5-263D-4559-959F-4F9C429F9D11
 */
uint8_t sBtpUuidC1[] = {OT_PLAT_TOBLE_UUID_C1};

/**
 * Characteristic C2 UUID: 18EE2EF5-263D-4559-959F-4F9C429F9D12
 */
uint8_t sBtpUuidC2[] = {OT_PLAT_TOBLE_UUID_C2};

otPlatBleGattService sBtpServices[] = {
    {.mUuid   = {.mType = OT_BLE_UUID_TYPE_16, .mValue.mUuid16 = OT_PLAT_TOBLE_UUID_THREAD_GROUP},
     .mHandle = 0, ///< Set by bleGattServerServicesRegister
     .mCharacteristics =
         (otPlatBleGattCharacteristic[]){
             {.mUuid        = {.mType = OT_BLE_UUID_TYPE_128, .mValue.mUuid128 = sBtpUuidC1},
              .mHandleValue = 0, ///< Set by bleGattServerServicesRegister
              .mProperties  = OT_BLE_CHAR_PROP_WRITE | OT_BLE_CHAR_PROP_WRITE_NO_RESPONSE},
             {.mUuid        = {.mType = OT_BLE_UUID_TYPE_128, .mValue.mUuid128 = sBtpUuidC2},
              .mHandleValue = 0, ///< Set by bleGattServerServicesRegister
              .mProperties  = OT_BLE_CHAR_PROP_INDICATE | OT_BLE_CHAR_PROP_NOTIFY | OT_BLE_CHAR_PROP_READ},
             {
                 {0},               ///< No more characteristics in this service
                 .mHandleValue = 0, ///< Set by bleGattServerServicesRegister
             }}},
    {
        {0}, // No more services.
        .mHandle = 0,
    }};

static otError bleGattServerServicesRegister(otInstance *aInstance, otPlatBleGattService *aServices);

static void peerInit(BlePeer *aPeer)
{
    memset(aPeer, 0, sizeof(BlePeer));
    aPeer->mConnHandle = BLE_CONN_HANDLE_INVALID;
    aPeer->mAttMtu     = BLE_GATT_ATT_MTU;
}

/**
 * Gets the peer.
 */
static uint16_t peerGet(uint16_t aConnectionId)
{
    uint32_t index;

    for (index = 0; index < NRF_SDH_BLE_TOTAL_LINK_COUNT; index++)
    {
        if (sBlePeers[index].mConnHandle == aConnectionId)
        {
            break;
        }
    }

    return index;
}

/**
 * Returns number of connected peers.
 */
static uint32_t peerGetCount(void)
{
    uint32_t count = 0;

    for (uint32_t index = 0; index < NRF_SDH_BLE_TOTAL_LINK_COUNT; index++)
    {
        if (sBlePeers[index].mConnHandle != BLE_CONN_HANDLE_INVALID)
        {
            count++;
        }
    }

    return count;
}

/**
 * Allocates the new connection handle.
 */
static BlePeer *peerAllocate(void)
{
    uint16_t index = peerGet(BLE_CONN_HANDLE_INVALID);
    assert(index != NRF_SDH_BLE_TOTAL_LINK_COUNT);

    sBlePeers[index].mIsUsed = true;
    sBlePeers[index].mAttMtu = BLE_GATT_ATT_MTU;

    return &sBlePeers[index];
}

static BlePeer *peerFind(uint16_t aConnectionId)
{
    uint32_t index;
    BlePeer *peer = NULL;

    for (index = 0; index < NRF_SDH_BLE_TOTAL_LINK_COUNT; index++)
    {
        if (sBlePeers[index].mConnHandle == aConnectionId)
        {
            peer = &sBlePeers[index];
            break;
        }
    }

    return peer;
}

/**
 * Frees the new connection handle.
 */
static void peerFree(uint16_t aConnectionId)
{
    uint16_t index = peerGet(aConnectionId);
    assert(index != NRF_SDH_BLE_TOTAL_LINK_COUNT);
    peerInit(&sBlePeers[index]);
}

/**
 *  Translates OpenThread's UUID type into SoftDevice's one.
 */
static uint32_t uuidDecode(ble_uuid_t *aSDUuid, const otPlatBleUuid *aOtUuid)
{
    uint32_t retval = NRF_SUCCESS;

    if (aOtUuid->mType == OT_BLE_UUID_TYPE_128)
    {
        otEXPECT_ACTION(aOtUuid->mValue.mUuid128 != NULL, retval = NRF_ERROR_INVALID_PARAM);

        retval = sd_ble_uuid_decode(OT_BLE_UUID_LENGTH, aOtUuid->mValue.mUuid128, aSDUuid);
    }
    else if (aOtUuid->mType == OT_BLE_UUID_TYPE_16)
    {
        aSDUuid->type = BLE_UUID_TYPE_BLE;
        aSDUuid->uuid = aOtUuid->mValue.mUuid16;
    }
    else
    {
        // 32-bit option is not supported.
        otEXPECT_ACTION(false, retval = NRF_ERROR_NOT_SUPPORTED);
    }

exit:
    return retval;
}

/**
 * Recognizes if handle is of CCCD type.
 */
static bool isCccdHandle(uint16_t aHandle)
{
    uint32_t retval;

    ble_uuid_t uuid;

    retval = sd_ble_gatts_attr_get(aHandle, &uuid, NULL);

    if (retval != NRF_SUCCESS)
    {
        otLogCritPlat("[BLE] sd_ble_gatts_attr_get error: 0x%x", retval);
    }
    else
    {
        if (uuid.type == BLE_UUID_TYPE_BLE && uuid.uuid == BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG)
        {
            return true;
        }
    }

    return false;
}

/*******************************************************************************
 * @section Bluetooth Low Energy management.
 *******************************************************************************/

static void initState()
{
    uint32_t index;

    sBle.mState                        = kStateIdle;
    sBle.mConnParams.min_conn_interval = BLE_DEFAULT_CONNECTION_INTERVAL_MIN;
    sBle.mConnParams.max_conn_interval = BLE_DEFAULT_CONNECTION_INTERVAL_MAX;
    sBle.mConnParams.slave_latency     = BLE_DEFAULT_SLAVE_LATENCY;
    sBle.mConnParams.conn_sup_timeout  = BLE_DEFAULT_CONNECTIION_TIMEOUT;
    sBle.mAdvHandle                    = BLE_GAP_ADV_SET_HANDLE_NOT_SET;

    for (index = 0; index < NRF_SDH_BLE_TOTAL_LINK_COUNT; index++)
    {
        peerInit(&sBlePeers[index]);
    }

    for (index = 0; index < sizeof(sPendingTasks) / sizeof(BlePendingTask); index++)
    {
        sPendingTasks[index].mPending = false;
    }

    memset(&sBle.mAdvDataInfo, 0, sizeof(sBle.mAdvDataInfo));

    sPeripheralPeer = NULL;
    sDiagMode       = false;
}

// FIXME @gjc @zhanglongxia: should not init in cli
void otPlatTobleInit(otInstance *aInstance)
{
    uint8_t isEnabled;

    sd_softdevice_is_enabled(&isEnabled);

    otEXPECT(((bool)isEnabled && (sBle.mState != kStateUninitialized)) == false);

    sInstance = aInstance;

    initState();
    bleGattServerServicesRegister(sInstance, sBtpServices);

#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
    sBle.mC1Handle     = sBtpServices[0].mCharacteristics[0].mHandleValue;
    sBle.mC2Handle     = sBtpServices[0].mCharacteristics[1].mHandleValue;
    sBle.mC2CccdHandle = sBtpServices[0].mCharacteristics[1].mHandleCccd;
#endif

exit:
    return;
}

uint8_t otPlatTobleGetL2capPsm(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return 0;
}

otTobleConnectionLinkType otPlatTobleGetConnectionLinkType(const otTobleConnection *aConn)
{
    BlePeer *peer = (BlePeer *)aConn;
    return peer->mLinkType;
}

bool otPlatTobleDiagModeGet(void)
{
    return sDiagMode;
}

void otPlatTobleDiagModeSet(otInstance *aInstance, bool aMode)
{
    OT_UNUSED_VARIABLE(aInstance);
    sDiagMode = aMode;
}

/*******************************************************************************
 * @section Bluetooth Low Energy GAP.
 *******************************************************************************/

otError bleGapAdvStart(otTobleAdvType aType, uint16_t aInterval)
{
    uint32_t             retval;
    ble_gap_adv_params_t params;

    memset(&params, 0, sizeof(ble_gap_adv_params_t));

    if (aType == OT_TOBLE_ADV_IND)
    {
        params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    }
    else if (aType == OT_TOBLE_ADV_DIRECT_IND)
    {
        params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_NONSCANNABLE_DIRECTED_HIGH_DUTY_CYCLE;
    }
    else if (aType == OT_TOBLE_ADV_SCAN_IND)
    {
        params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED;
    }
    else if (aType == OT_TOBLE_ADV_NONCONN_IND)
    {
        params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    }

    params.p_peer_addr   = NULL;
    params.filter_policy = BLE_GAP_ADV_FP_ANY;
    params.interval      = MS_TO_BLE_TIME_UNIT(aInterval);
    params.duration      = BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED;
    params.primary_phy   = BLE_GAP_PHY_1MBPS;

    retval = sd_ble_gap_adv_set_configure(&sBle.mAdvHandle, &sBle.mAdvDataInfo, &params);
    otEXPECT_ACTION(retval == NRF_SUCCESS, otLogCritPlat("[BLE]: sd_ble_gap_adv_set_configure error: 0x%x", retval));

    retval = sd_ble_gap_adv_start(sBle.mAdvHandle, BLE_CFG_TAG);
    otEXPECT_ACTION(retval == NRF_SUCCESS, otLogCritPlat("[BLE]: sd_ble_gap_adv_start error: 0x%x", retval));

    sBle.mAdvInterval = aInterval;
    sBle.mAdvType     = aType;
    sBle.mState       = kStateAdvertising;

exit:
    return nrf5SdErrorToOtError(retval);
}

otError otPlatTobleAdvStart(otInstance *aInstance, const otTobleAdvConfig *aConfig)
{
    OT_UNUSED_VARIABLE(aInstance);

    if (aConfig->mData != NULL)
    {
        memcpy(sBle.mAdvData, aConfig->mData, aConfig->mLength);

        sBle.mAdvDataInfo.adv_data.p_data = sBle.mAdvData;
        sBle.mAdvDataInfo.adv_data.len    = aConfig->mLength;
    }

    if (aConfig->mScanRspData != NULL)
    {
        memcpy(sBle.mScanRspData, aConfig->mScanRspData, aConfig->mScanRspDataLength);

        sBle.mAdvDataInfo.scan_rsp_data.p_data = sBle.mScanRspData;
        sBle.mAdvDataInfo.scan_rsp_data.len    = aConfig->mScanRspDataLength;
    }

    return bleGapAdvStart(aConfig->mType, aConfig->mInterval);
}

otError otPlatTobleAdvStop(otInstance *aInstance)
{
    uint32_t retval = NRF_ERROR_INVALID_STATE;

    OT_UNUSED_VARIABLE(aInstance);

    otEXPECT(sBle.mAdvHandle != BLE_GAP_ADV_SET_HANDLE_NOT_SET);

    retval = sd_ble_gap_adv_stop(sBle.mAdvHandle);
    otEXPECT_ACTION(retval == NRF_SUCCESS, otLogCritPlat("[BLE] sd_ble_gap_adv_stop error: 0x%x", retval));

exit:
    return nrf5SdErrorToOtError(retval);
}

otError otPlatTobleScanStart(otInstance *aInstance, uint16_t aInterval, uint16_t aWindow, bool aActive)
{
    uint32_t              retval;
    ble_gap_scan_params_t params;
    ble_data_t            advData;

    OT_UNUSED_VARIABLE(aInstance);

    memset(&params, 0, sizeof(ble_gap_scan_params_t));
    params.active        = aActive;
    params.filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL;
    params.interval      = MS_TO_BLE_TIME_UNIT(aInterval);
    params.window        = MS_TO_BLE_TIME_UNIT(aWindow);
    params.timeout       = BLE_GAP_SCAN_TIMEOUT_UNLIMITED;
    params.scan_phys     = BLE_GAP_PHY_1MBPS;

    advData.p_data = sBle.mScanBuff;
    advData.len    = sizeof(sBle.mScanBuff);

    if ((retval = sd_ble_gap_scan_start(&params, &advData)) != NRF_SUCCESS)
    {
        otLogCritPlat("[BLE] sd_ble_gap_scan_start error: 0x%x", retval);
    }

    return nrf5SdErrorToOtError(retval);
}

otError otPlatTobleScanStop(otInstance *aInstance)
{
    uint32_t retval;

    OT_UNUSED_VARIABLE(aInstance);

    if ((retval = sd_ble_gap_scan_stop()) != NRF_SUCCESS)
    {
        otLogCritPlat("[BLE] sd_ble_gap_scan_stop error: 0x%x", retval);
    }

    return nrf5SdErrorToOtError(retval);
}

otTobleConnection *otPlatTobleCreateConnection(otInstance *             aInstance,
                                               const otTobleAddress *   aPeerAddress,
                                               otTobleConnectionConfig *aConfig)
{
    uint32_t              retval;
    ble_gap_addr_t        addr;
    ble_gap_scan_params_t scanParams;
    ble_gap_conn_params_t connParams;

    OT_UNUSED_VARIABLE(aInstance);

    otEXPECT_ACTION(sPeripheralPeer == NULL, retval = NRF_ERROR_INVALID_STATE);
    otEXPECT_ACTION(peerGetCount() < NRF_SDH_BLE_TOTAL_LINK_COUNT, retval = NRF_ERROR_INVALID_STATE);

    memset(&scanParams, 0, sizeof(ble_gap_scan_params_t));
    scanParams.scan_phys = BLE_GAP_PHY_1MBPS;
    scanParams.interval  = MS_TO_BLE_TIME_UNIT(aConfig->mScanInterval);
    scanParams.window    = MS_TO_BLE_TIME_UNIT(aConfig->mScanWindow);
    scanParams.timeout   = BLE_GAP_SCAN_TIMEOUT_UNLIMITED;

    connParams.min_conn_interval = MS_TO_BLE_CONNECTION_INTERVAL_UNIT(aConfig->mInterval);
    connParams.max_conn_interval = MS_TO_BLE_CONNECTION_INTERVAL_UNIT(aConfig->mInterval);
    connParams.slave_latency     = BLE_DEFAULT_SLAVE_LATENCY;
    connParams.conn_sup_timeout  = aConfig->mInterval * BLE_DEFAULT_CONNECTION_RETRY;

    memset(&addr, 0, sizeof(ble_gap_addr_t));
    addr.addr_type = aPeerAddress->mType;
    memcpy(addr.addr, aPeerAddress->mAddress, OT_TOBLE_ADDRESS_SIZE);

    retval = sd_ble_gap_connect(&addr, &scanParams, &connParams, BLE_CFG_TAG);

    if (retval == NRF_SUCCESS)
    {
        sPeripheralPeer                = peerAllocate();
        sPeripheralPeer->mLinkType     = aConfig->mLinkType;
        sPeripheralPeer->mIsPeripheral = true;
        sBle.mState                    = kStateConnectionRequest;
    }
    else
    {
        otLogCritPlat("[BLE] sd_ble_gap_connect error: 0x%x", retval);
    }

exit:
    return (otTobleConnection *)sPeripheralPeer;
}

void otPlatTobleDisconnect(otInstance *aInstance, otTobleConnection *aConn)
{
    uint32_t retval;
    BlePeer *peer = (BlePeer *)aConn;

    OT_UNUSED_VARIABLE(aInstance);

    if (sBle.mState == kStateConnectionRequest)
    {
        otEXPECT_ACTION(false, retval = sd_ble_gap_connect_cancel());
    }

    otEXPECT_ACTION(peer->mConnHandle != BLE_CONN_HANDLE_INVALID, retval = NRF_ERROR_INVALID_STATE);

    retval = sd_ble_gap_disconnect(peer->mConnHandle, OT_BLE_HCI_REMOTE_USER_TERMINATED);

    if (retval != NRF_SUCCESS)
    {
        otLogCritPlat("[BLE] sd_ble_gap_disconnect error: 0x%x", retval);
    }

exit:
    return;
}

uint16_t otPlatTobleGetMtu(otInstance *aInstance, otTobleConnection *aConn)
{
    BlePeer *peer = (BlePeer *)aConn;

    OT_UNUSED_VARIABLE(aInstance);
    return peer->mAttMtu;
}

void otPlatTobleC2Subscribe(otInstance *aInstance, otTobleConnection *aConn, bool aSubscribe)
{
    uint32_t                 retval;
    ble_gattc_write_params_t params;
    uint8_t                  cccdVal[2];
    BlePeer *                peer = (BlePeer *)aConn;

    OT_UNUSED_VARIABLE(aInstance);

#if OPENTHREAD_CONFIG_TOBLE_BTP_NO_GATT_ACK
    cccdVal[0] = aSubscribe ? BLE_GATT_HVX_NOTIFICATION : 0;
#else
    cccdVal[0] = aSubscribe ? BLE_GATT_HVX_INDICATION : 0;
#endif

    cccdVal[1] = 0;

    memset(&params, 0, sizeof(ble_gattc_write_params_t));
    params.write_op = BLE_GATT_OP_WRITE_REQ;
    params.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;
    params.handle   = sBle.mC2CccdHandle;
    params.offset   = 0;
    params.len      = sizeof(cccdVal);
    params.p_value  = cccdVal;

    retval = sd_ble_gattc_write(peer->mConnHandle, &params);

    if (retval != NRF_SUCCESS)
    {
        otLogCritPlat("[BLE] sd_ble_gattc_write error: 0x%x", retval);
    }
    else
    {
        sBle.mState = kStateGattSubscribing;
    }
}

otError otPlatTobleC1Write(otInstance *aInstance, otTobleConnection *aConn, const void *aBuffer, uint16_t aLength)
{
    otError                  error = OT_ERROR_NONE;
    uint32_t                 retval;
    ble_gattc_write_params_t params;
    BlePeer *                peer = (BlePeer *)aConn;

    memset(&params, 0, sizeof(ble_gattc_write_params_t));
    params.write_op = BLE_GATT_OP_WRITE_CMD;
    params.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;
    params.handle   = sBle.mC1Handle;
    params.offset   = 0;
    params.p_value  = aBuffer;
    params.len      = aLength;

    otLogInfoPlat("[BLE] otPlatTobleC1Write(ConnectionId=%d)", peer->mConnHandle);
    retval = sd_ble_gattc_write(peer->mConnHandle, &params);

    if (retval != NRF_SUCCESS)
    {
        // otLogCritPlat("[BLE] sd_ble_gattc_write error: 0x%x", retval);
        error = OT_ERROR_FAILED;
    }

    return error;
}

/*******************************************************************************
 * @section Bluetooth Low Energy GATT Server.
 *******************************************************************************/

static otError serviceRegister(const otPlatBleUuid *aUuid, uint16_t *aHandle)
{
    uint32_t   retval;
    ble_uuid_t uuid;

    if (aUuid->mType == OT_BLE_UUID_TYPE_16 &&
        (aUuid->mValue.mUuid16 == UUID_SERVICE_GAP || aUuid->mValue.mUuid16 == UUID_SERVICE_GATT))
    {
        // SoftDevice registers GAP and GATT services by default.
        otEXPECT_ACTION(false, retval = NRF_SUCCESS);
    }

    retval = uuidDecode(&uuid, aUuid);
    otEXPECT(retval == NRF_SUCCESS);

    retval = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &uuid, aHandle);

    if (retval != NRF_SUCCESS)
    {
        otLogCritPlat("[BLE] sd_ble_gatts_service_add error: 0x%x", retval);
    }

exit:
    return nrf5SdErrorToOtError(retval);
}

static otError characteristicRegister(uint16_t aServiceHandle, otPlatBleGattCharacteristic *aChar, bool aCccd)
{
    uint32_t                 retval;
    uint8_t                  uuid_type = 0;
    ble_uuid_t               uuid;
    ble_uuid128_t            uuid128;
    ble_gatts_char_md_t      charMd;
    ble_gatts_attr_md_t      attrMd;
    ble_gatts_attr_md_t      cccdMd;
    ble_gatts_attr_t         attr;
    ble_gatts_char_handles_t handles;

    otEXPECT_ACTION((aChar->mUuid.mType == OT_BLE_UUID_TYPE_128) && (aChar->mUuid.mValue.mUuid128 != NULL),
                    retval = NRF_ERROR_INVALID_PARAM);

    memcpy(uuid128.uuid128, aChar->mUuid.mValue.mUuid128, OT_BLE_UUID_LENGTH);
    retval = sd_ble_uuid_vs_add(&uuid128, &uuid_type);

    if (retval != NRF_SUCCESS)
    {
        otLogCritPlat("[BLE] sd_ble_uuid_vs_add error: 0x%x", retval);
        otEXPECT(false);
    }

    retval = uuidDecode(&uuid, &aChar->mUuid);
    otEXPECT(retval == NRF_SUCCESS);

    memset(&charMd, 0, sizeof(ble_gatts_char_md_t));

    if (aChar->mProperties & OT_BLE_CHAR_PROP_READ)
    {
        charMd.char_props.read = 1;
    }

    if (aChar->mProperties & OT_BLE_CHAR_PROP_WRITE)
    {
        charMd.char_props.write = 1;
    }

    if (aChar->mProperties & OT_BLE_CHAR_PROP_NOTIFY)
    {
        charMd.char_props.notify = 1;
    }

    if (aChar->mProperties & OT_BLE_CHAR_PROP_INDICATE)
    {
        charMd.char_props.indicate = 1;
    }

    if (aCccd)
    {
        memset(&cccdMd, 0, sizeof(ble_gatts_attr_md_t));
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccdMd.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccdMd.write_perm);

        cccdMd.vloc      = BLE_GATTS_VLOC_STACK;
        charMd.p_cccd_md = &cccdMd;
    }

    memset(&attrMd, 0, sizeof(ble_gatts_attr_md_t));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attrMd.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attrMd.write_perm);

    attrMd.vloc    = BLE_GATTS_VLOC_STACK;
    attrMd.rd_auth = 0;
    attrMd.wr_auth = 0;
    attrMd.vlen    = 1;

    memset(&attr, 0, sizeof(ble_gatts_attr_t));
    attr.p_uuid    = &uuid;
    attr.p_attr_md = &attrMd;
    attr.init_len  = sizeof(uint8_t);
    attr.init_offs = 0;
    attr.max_len   = OT_BLE_CHARACTERISTIC_MAX_LENGTH;
    attr.p_value   = NULL;

    retval = sd_ble_gatts_characteristic_add(aServiceHandle, &charMd, &attr, &handles);

    if (retval != NRF_SUCCESS)
    {
        otLogCritPlat("[BLE] sd_ble_gatts_characteristic_add error: 0x%x", retval);
    }
    else
    {
        aChar->mHandleValue = handles.value_handle;
        aChar->mHandleCccd  = handles.cccd_handle;
    }

exit:
    return nrf5SdErrorToOtError(retval);
}

static otError bleGattServerServicesRegister(otInstance *aInstance, otPlatBleGattService *aServices)
{
    (void)aInstance;

    otError                      error = OT_ERROR_NONE;
    otPlatBleGattCharacteristic *chr;

    while (aServices && (aServices->mUuid.mType != OT_BLE_UUID_TYPE_NONE))
    {
        error = serviceRegister(&aServices->mUuid, &aServices->mHandle);
        otEXPECT(error == OT_ERROR_NONE);

        chr = aServices->mCharacteristics;

        while (chr && (chr->mUuid.mType != OT_BLE_UUID_TYPE_NONE))
        {
            error = characteristicRegister(aServices->mHandle, chr, true);
            otEXPECT(error == OT_ERROR_NONE);

            chr++;
        }

        aServices += sizeof(otPlatBleGattService);
    }

exit:
    return nrf5SdErrorToOtError(error);
}

otError otPlatTobleC2Notificate(otInstance *aInstance, otTobleConnection *aConn, const void *aBuffer, uint16_t aLength)
{
    otError                error = OT_ERROR_NONE;
    uint32_t               retval;
    ble_gatts_hvx_params_t params;
    BlePeer *              peer = (BlePeer *)aConn;

    OT_UNUSED_VARIABLE(aInstance);

    // TODO: Gatt Server should also take the connection id.

    memset(&params, 0, sizeof(ble_gatts_hvx_params_t));
    params.handle = sBle.mC2Handle;
    params.p_data = aBuffer;
    params.p_len  = &aLength;
    params.type   = BLE_GATT_HVX_NOTIFICATION;

    otLogInfoPlat("[BLE] otPlatTobleC2Notificate(ConnectionId=%d)", peer->mConnHandle);
    retval = sd_ble_gatts_hvx(peer->mConnHandle, &params);

    if (retval != NRF_SUCCESS)
    {
        // otLogCritPlat("[BLE] sd_ble_gatts_hvx error: 0x%x", retval);
        error = OT_ERROR_FAILED;
    }

    return error;
}

/****************************************************************************
 * @section Bluetooth Low Energy L2CAP Connection Oriented Channels.
 ***************************************************************************/

uint32_t bleL2capSendConnectionRequest(BlePeer *aPeer, uint16_t aPsm, uint16_t aMtu)
{
    uint32_t                    retval;
    ble_l2cap_ch_setup_params_t params;

    otEXPECT_ACTION(aMtu <= BLE_DEFAULT_L2CAP_MAX_MTU_SIZE, retval = NRF_ERROR_INVALID_PARAM);

    memset(&params, 0, sizeof(ble_l2cap_ch_setup_params_t));
    params.le_psm                   = aPsm;
    params.rx_params.rx_mtu         = aMtu;
    params.rx_params.rx_mps         = BLE_DEFAULT_L2CAP_MPS_SIZE;
    params.rx_params.sdu_buf.p_data = NULL;
    params.rx_params.sdu_buf.len    = 0;

    retval = sd_ble_l2cap_ch_setup(aPeer->mConnHandle, &aPeer->mLocalCid, &params);

    if (retval != NRF_SUCCESS)
    {
        otLogCritPlat("[BLE] sd_ble_l2cap_ch_setup error: 0x%x", retval);
    }

exit:
    return retval;
}

uint32_t bleL2capSendConnectionResponse(BlePeer *aPeer, otPlatBleL2capError aError, uint16_t aMtu)
{
    uint32_t                    retval;
    ble_l2cap_ch_setup_params_t params;

    otEXPECT_ACTION(aMtu <= BLE_DEFAULT_L2CAP_MAX_MTU_SIZE, retval = NRF_ERROR_INVALID_PARAM);

    memset(&params, 0, sizeof(ble_l2cap_ch_setup_params_t));
    params.le_psm                   = 0x0055; // TODO: PSM here?
    params.status                   = aError;
    params.rx_params.rx_mtu         = aMtu;
    params.rx_params.rx_mps         = BLE_DEFAULT_L2CAP_MPS_SIZE;
    params.rx_params.sdu_buf.p_data = NULL;
    params.rx_params.sdu_buf.len    = 0;

    retval = sd_ble_l2cap_ch_setup(aPeer->mConnHandle, &aPeer->mLocalCid, &params);

    if (retval != NRF_SUCCESS)
    {
        otLogCritPlat("[BLE] sd_ble_l2cap_ch_setup error: 0x%x", retval);
    }

exit:
    return retval;
}

otError otPlatTobleL2capSend(otInstance *aInstance, otTobleConnection *aConn, const uint8_t *aBuffer, uint16_t aLength)
{
    BlePeer *  peer = (BlePeer *)aConn;
    uint32_t   retval;
    ble_data_t sduBuf;

    sduBuf.p_data = (uint8_t *)aBuffer;
    sduBuf.len    = aLength;

    retval = sd_ble_l2cap_ch_tx(peer->mConnHandle, peer->mLocalCid, &sduBuf);

    if (retval != NRF_SUCCESS)
    {
        otLogCritPlat("[BLE] sd_ble_l2cap_ch_tx error: 0x%x", retval);
    }

    return nrf5SdErrorToOtError(retval);
}

/*******************************************************************************
 * @section SoftDevice events.
 *******************************************************************************/

bool nrf5SoftDeviceBleIsEnabled()
{
    return sBle.mState != kStateUninitialized;
}

static otError advReportTypeConvert(ble_gap_adv_report_type_t aSdAdvType, otTobleAdvType *aTobleAdvType)
{
    otError error = OT_ERROR_NONE;

    if (aSdAdvType.connectable && aSdAdvType.scannable && !aSdAdvType.directed)
    {
        *aTobleAdvType = OT_TOBLE_ADV_IND;
    }
    else if (aSdAdvType.connectable && !aSdAdvType.scannable && aSdAdvType.directed)
    {
        *aTobleAdvType = OT_TOBLE_ADV_DIRECT_IND;
    }
    else if (!aSdAdvType.connectable && aSdAdvType.scannable && !aSdAdvType.directed)
    {
        *aTobleAdvType = OT_TOBLE_ADV_SCAN_IND;
    }
    else if (!aSdAdvType.connectable && !aSdAdvType.scannable && !aSdAdvType.directed)
    {
        *aTobleAdvType = OT_TOBLE_ADV_NONCONN_IND;
    }
    else
    {
        error = OT_ERROR_FAILED;
    }

    return error;
}

uint32_t bleGattcCharValueByUuidRead(uint16_t                  aConnectionId,
                                     ble_gattc_handle_range_t *aRange,
                                     const otPlatBleUuid *     aOtUuid)
{
    ble_uuid_t uuid;

    uuidDecode(&uuid, aOtUuid);
    return sd_ble_gattc_char_value_by_uuid_read(aConnectionId, &uuid, aRange);
}

#if (OPENTHREAD_CONFIG_LOG_LEVEL >= OT_LOG_LEVEL_NOTE) && (OPENTHREAD_CONFIG_LOG_PLATFORM == 1)
static const char *bleEvtToString(int aEvtId)
{
    const char *str = "unknown";

    switch (aEvtId)
    {
    case BLE_GAP_EVT_CONNECTED:
        str = "BLE_GAP_EVT_CONNECTED";
        break;
    case BLE_GAP_EVT_DISCONNECTED:
        str = "BLE_GAP_EVT_DISCONNECTED";
        break;
    case BLE_GAP_EVT_CONN_PARAM_UPDATE:
        str = "BLE_GAP_EVT_CONN_PARAM_UPDATE";
        break;
    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        str = "BLE_GATTS_EVT_SYS_ATTR_MISSING";
        break;
    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        str = "BLE_GAP_EVT_SEC_PARAMS_REQUEST";
        break;
    case BLE_GAP_EVT_ADV_REPORT:
        str = "BLE_GAP_EVT_ADV_REPORT";
        break;
    case BLE_GAP_EVT_TIMEOUT:
        str = "BLE_GAP_EVT_TIMEOUT";
        break;
    case BLE_GAP_EVT_RSSI_CHANGED:
        str = "BLE_GAP_EVT_RSSI_CHANGED";
        break;
    case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
        str = "BLE_GAP_EVT_DATA_LENGTH_UPDATE";
        break;
    case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
        str = "BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST";
        break;
    case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
        str = "BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST";
        break;
    case BLE_GATTS_EVT_WRITE:
        str = "BLE_GATTS_EVT_WRITE";
        break;
    case BLE_GATTS_EVT_HVN_TX_COMPLETE:
        str = "BLE_GATTS_EVT_HVN_TX_COMPLETE";
        break;
    case BLE_GATTS_EVT_HVC:
        str = "BLE_GATTS_EVT_HVC";
        break;
    case BLE_GATTS_EVT_TIMEOUT:
        str = "BLE_GATTS_EVT_TIMEOUT";
        break;
    case BLE_GATTC_EVT_EXCHANGE_MTU_RSP:
        str = "BLE_GATTC_EVT_EXCHANGE_MTU_RSP";
        break;
    case BLE_GATTC_EVT_TIMEOUT:
        str = "BLE_GATTC_EVT_TIMEOUT";
        break;
    case BLE_GATTC_EVT_HVX:
        str = "BLE_GATTC_EVT_HVX";
        break;
    case BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE:
        str = "BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE";
        break;
    case BLE_GATTC_EVT_WRITE_RSP:
        str = "BLE_GATTC_EVT_WRITE_RSP";
        break;
    case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
        str = "BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP";
        break;
    case BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP:
        str = "BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP";
        break;
    }

    return str;
}
#endif //#if (OPENTHREAD_CONFIG_LOG_LEVEL >= OT_LOG_LEVEL_DEBG) && (OPENTHREAD_CONFIG_LOG_PLATFORM == 1)

static void SignalBleTaskPending(BlePendingTask *aTask)
{
    aTask->mPending = true;
    sTaskQueueTail++;
    sTaskQueueTail %= BLE_TASK_COUNT;
    otLogCritPlat("SignalBleTaskPending %d %d", sTaskQueueHead, sTaskQueueTail);
    otSysEventSignalPending();
}

static void ble_evt_handler(ble_evt_t const *aEvent, void *aContext)
{
    uint32_t   retval       = NRF_SUCCESS;
    ble_evt_t *evt          = (ble_evt_t *)aEvent;
    uint16_t   connectionId = evt->evt.common_evt.conn_handle;
    BlePeer *  peer;
    BlePendingTask *task = &sPendingTasks[sTaskQueueTail];

    OT_UNUSED_VARIABLE(aContext);

    otEXPECT(nrf5SoftDeviceBleIsEnabled());

    if ((sTaskQueueTail + 1) % BLE_TASK_COUNT == sTaskQueueHead)
    {
        otLogCritPlat("No free BLE task slot, will skip event");
        ExitNow();
    }

    if ((evt->header.evt_id != BLE_GAP_EVT_ADV_REPORT) && (evt->header.evt_id != BLE_GAP_EVT_RSSI_CHANGED))
    {
        otLogInfoPlat("[BLE] Event: ConnectionId=%d %s", connectionId, bleEvtToString(evt->header.evt_id));
    }

    switch (evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
    {
        sBle.mState = kStateIdle;

        peer = (sPeripheralPeer != NULL) ? sPeripheralPeer : peerAllocate();
        if (peer == NULL)
        {
            sd_ble_gap_disconnect(connectionId, OT_BLE_HCI_REMOTE_USER_TERMINATED);
            break;
        }
        peer->mConnHandle = connectionId;

        task->mType = kHandleConnected;
        task->mConnection = peer;
        SignalBleTaskPending(task);

        if (sPeripheralPeer != NULL)
        {
            if (peer->mLinkType == kConnectionLinkTypeL2Cap)
            {
                bleL2capSendConnectionRequest(peer, BLE_L2CAP_PSM, BLE_L2CAP_MTU);
            }
            else if (peer->mLinkType == kConnectionLinkTypeGatt)
            {
                sd_ble_gattc_exchange_mtu_request(peer->mConnHandle, peer->mAttMtu);
            }
        }

        ble_gap_data_length_params_t bleParams;

        // Clearing the struct will effectivly set members to @ref BLE_GAP_DATA_LENGTH_AUTO
        memset(&bleParams, 0, sizeof(ble_gap_data_length_params_t));
        bleParams.max_tx_octets = BLE_DEFAULT_PDU_SIZE;
        bleParams.max_rx_octets = BLE_DEFAULT_PDU_SIZE;

        otLogInfoPlat("[BLE] Requesting data Length (tx, rx) octets = (%d, %d), time = (%d, %d) us",
                      bleParams.max_tx_octets, bleParams.max_rx_octets, bleParams.max_tx_time_us,
                      bleParams.max_rx_time_us);

        retval = sd_ble_gap_data_length_update(connectionId, &bleParams, NULL);

        if (retval != NRF_SUCCESS)
        {
            otLogInfoPlat("[BLE] sd_ble_gap_data_length_update error: 0x%x", retval);
        }

        // Start reporting RSSI.
        retval = sd_ble_gap_rssi_start(connectionId, BLE_DEFAULT_RSSI_THRESHOLD_CHANGE, BLE_DEFAULT_RSSI_SKIP);

        if (retval != NRF_SUCCESS)
        {
            otLogCritPlat("[BLE] sd_ble_gap_rssi_start error: 0x%x", retval);
        }

        break;
    }

    case BLE_GAP_EVT_DISCONNECTED:

        if ((peer = peerFind(connectionId)) != NULL)
        {
            sPeripheralPeer = NULL;

            task->mType = kHandleDisconnected;
            task->mConnection = peer;
            SignalBleTaskPending(task);
        }

        // Free the BLE Peer slot.
        peerFree(connectionId);

        break;

    case BLE_GAP_EVT_CONN_PARAM_UPDATE:
    {
        ble_gap_conn_params_t *connParams = &evt->evt.gap_evt.params.conn_param_update.conn_params;

        retval = sd_ble_gap_conn_param_update(connectionId, connParams);

        if (retval != NRF_SUCCESS)
        {
            otLogCritPlat("[BLE] sd_ble_gap_conn_param_update error: 0x%x", retval);
        }

        break;
    }

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        // No system attributes have been stored.
        retval = sd_ble_gatts_sys_attr_set(connectionId, NULL, 0, 0);

        break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        // Pairing not supported
        retval = sd_ble_gap_sec_params_reply(connectionId, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);

        if (retval != NRF_SUCCESS)
        {
            otLogCritPlat("[BLE] sd_ble_gap_sec_params_reply error: 0x%x", retval);
        }

        break;

    case BLE_GAP_EVT_ADV_REPORT:
    {
        ble_gap_evt_adv_report_t *advReport = &evt->evt.gap_evt.params.adv_report;
        ble_data_t       advData;

        advData.p_data = sBle.mScanBuff;
        advData.len    = sizeof(sBle.mScanBuff);
        // Continue scanning.
        retval = sd_ble_gap_scan_start(NULL, &advData);
        if (retval != NRF_SUCCESS)
        {
            otLogCritPlat("[BLE] sd_ble_gap_sec_params_reply error: 0x%x", retval);
        }

        task->mAdvertisement.mRssi   = advReport->rssi;
        memcpy(task->mBuffer, advReport->data.p_data, advReport->data.len);
        task->mAdvertisement.mData   = task->mBuffer;
        task->mAdvertisement.mLength = advReport->data.len;
        task->mAdvertisement.mSrcAddress.mType = advReport->peer_addr.addr_type;
        memcpy(task->mAdvertisement.mSrcAddress.mAddress, advReport->peer_addr.addr, OT_TOBLE_ADDRESS_SIZE);

        if (advReport->type.scan_response)
        {
            task->mType = kHandleScanResponse;
        }
        else
        {
            task->mType = kHandleAdvertisement;
            VerifyOrExit(advReportTypeConvert(advReport->type, &task->mAdvertisementType) == OT_ERROR_NONE, OT_NOOP);
        }
        SignalBleTaskPending(task);
        break;
    }

    case BLE_GAP_EVT_TIMEOUT:
        if (sBle.mState == kStateAdvertising)
        {
            bleGapAdvStart(sBle.mAdvType, sBle.mAdvInterval);
        }

        break;

    case BLE_GAP_EVT_RSSI_CHANGED:
        // Ignore.
        break;

    case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
    {
        ble_gap_data_length_params_t *evtMtu = &evt->evt.gap_evt.params.data_length_update.effective_params;
        otLogInfoPlat("[BLE] BLE_GAP_EVT_DATA_LENGTH_UPDATE: (max_tx, max_rx) octets = (%d, %d), time = (%d, %d) us",
                      evtMtu->max_tx_octets, evtMtu->max_rx_octets, evtMtu->max_tx_time_us, evtMtu->max_rx_time_us);
        UNUSED_PARAMETER(evtMtu);
        break;
    }

    case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
    {
        ble_gap_data_length_params_t bleParams;

        // Clearing the struct will effectivly set members to @ref BLE_GAP_DATA_LENGTH_AUTO
        memset(&bleParams, 0, sizeof(ble_gap_data_length_params_t));
        retval = sd_ble_gap_data_length_update(connectionId, &bleParams, NULL);

        otLogDebgPlat(
            "[BLE]: BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST: (max_tx, max_rx) octets = (%d, %d), time = (%d, %d) us",
            bleParams.max_tx_octets, bleParams.max_rx_octets, bleParams.max_tx_time_us, bleParams.max_rx_time_us);

        if (retval != NRF_SUCCESS)
        {
            otLogCritPlat("[BLE] sd_ble_gap_data_length_update error: 0x%x", retval);
        }

        break;
    }

    case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
    {
        uint16_t                              mtuMax = BLE_GATT_ATT_MTU;
        ble_gatts_evt_exchange_mtu_request_t *evtMtu = &evt->evt.gatts_evt.params.exchange_mtu_request;

        otLogDebgPlat("[BLE] BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST: mtuMax=%d, client_rx_mtu=%d", mtuMax,
                      evtMtu->client_rx_mtu);
        if ((peer = peerFind(connectionId)) != NULL)
        {
            peer->mAttMtu = MIN(mtuMax, evtMtu->client_rx_mtu);

            if ((retval = sd_ble_gatts_exchange_mtu_reply(connectionId, peer->mAttMtu)) != NRF_SUCCESS)
            {
                otLogCritPlat("[BLE] sd_ble_gatts_exchange_mtu_reply error: 0x%x", retval);
            }
        }

        break;
    }

    case BLE_GATTS_EVT_WRITE:
    {
        ble_gatts_evt_write_t *evtWrite = &evt->evt.gatts_evt.params.write;
        otBleRadioPacket       packet;
        uint8_t                channel_index;

        retval = sd_ble_gap_rssi_get(connectionId, &packet.mPower, &channel_index);

        if (retval != NRF_SUCCESS)
        {
            otLogCritPlat("[BLE] sd_ble_gap_rssi_get error: 0x%x", retval);
            packet.mPower = 0;
        }

        packet.mValue  = evtWrite->data;
        packet.mLength = evtWrite->len;

        if ((peer = peerFind(connectionId)) != NULL)
        {
            peer->mLinkType = kConnectionLinkTypeGatt;

            if (packet.mLength == 2 && isCccdHandle(evtWrite->handle))
            {
                uint16_t cccd = packet.mValue[0] | (packet.mValue[1] << 8);

#if OPENTHREAD_CONFIG_TOBLE_BTP_NO_GATT_ACK
                task->mSubscribed = (cccd & BLE_GATT_HVX_NOTIFICATION) > 0;
#else
                task->mSubscribed = (cccd & BLE_GATT_HVX_INDICATION) > 0;
#endif
                task->mConnection = peer;
                task->mType = kHandleC2Subscribed;
                SignalBleTaskPending(task);
            }
            else
            {
                memcpy(task->mBuffer, evtWrite->data, evtWrite->len);
                task->mBufferLength = evtWrite->len;
                task->mConnection = peer;
                task->mType = kHandleC1Write;
                SignalBleTaskPending(task);
            }
        }

        break;
    }

    case BLE_GATTS_EVT_HVN_TX_COMPLETE:
    {
        ble_gatts_evt_hvn_tx_complete_t *evtHvn = &evt->evt.gatts_evt.params.hvn_tx_complete;
        otLogDebgPlat("[BLE] BLE_GATTS_EVT_HVN_TX_COMPLETE: count=%d", evtHvn->count);
        if ((peer = peerFind(connectionId)) != NULL)
        {
            task->mCount = evtHvn->count;
            task->mConnection = peer;
            task->mType = kHandleC2NotificationDone;
            otLogDebgPlat("task at %p count=%d", task, task->mCount);
            SignalBleTaskPending(task);
        }
        break;
    }
        // Fall through

    case BLE_GATTS_EVT_HVC:
    {
        if ((peer = peerFind(connectionId)) != NULL)
        {
            task->mCount = 1;
            task->mConnection = peer;
            task->mType = kHandleC2NotificationDone;
            SignalBleTaskPending(task);
        }
        break;
    }

    case BLE_GATTS_EVT_TIMEOUT:
        sd_ble_gap_disconnect(connectionId, OT_BLE_HCI_REMOTE_USER_TERMINATED);
        break;

    case BLE_GATTC_EVT_EXCHANGE_MTU_RSP:
    {
        ble_gattc_evt_exchange_mtu_rsp_t *evtMtu = &evt->evt.gattc_evt.params.exchange_mtu_rsp;

        if ((peer = peerFind(connectionId)) != NULL)
        {
            ble_uuid_t uuid;

            otLogDebgPlat("[BLE] BLE_GATTC_EVT_EXCHANGE_MTU_RSP, mAttMtu=%d, server_rx_mtu=%d", peer->mAttMtu,
                          evtMtu->server_rx_mtu);

            peer->mAttMtu = MIN(peer->mAttMtu, evtMtu->server_rx_mtu);

            retval = uuidDecode(&uuid, &sBtpServices[0].mUuid);
            otEXPECT(retval == NRF_SUCCESS);

            if (sd_ble_gattc_primary_services_discover(peer->mConnHandle, BLE_GATT_HANDLE_START, &uuid) != NRF_SUCCESS)
            {
                otLogCritPlat("[BLE] sd_ble_gattc_primary_services_discover error: 0x%x", retval);
            }
        }
        break;
    }

    case BLE_GATTC_EVT_TIMEOUT:
        sd_ble_gap_disconnect(connectionId, OT_BLE_HCI_REMOTE_USER_TERMINATED);
        break;

    case BLE_GATTC_EVT_HVX:
    {
        ble_gattc_evt_hvx_t *evtHvx = &evt->evt.gattc_evt.params.hvx;

        retval = sd_ble_gattc_hv_confirm(connectionId, evtHvx->handle);
        if (retval != NRF_SUCCESS)
        {
            otLogCritPlat("[BLE] sd_ble_gattc_hv_confirm error: 0x%x", retval);
        }

        if ((peer = peerFind(connectionId)) != NULL)
        {
            task->mConnection = peer;
            memcpy(task->mBuffer, evtHvx->data, evtHvx->len);
            task->mBufferLength = evtHvx->len;
            task->mType = kHandleC2Notification;
            SignalBleTaskPending(task);
        }
        break;
    }

    case BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE:
    {
        ble_gattc_evt_write_cmd_tx_complete_t *evtWcmd = &evt->evt.gattc_evt.params.write_cmd_tx_complete;
        otLogDebgPlat("[BLE] BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE: count=%d", evtWcmd->count);

        if (sBle.mState != kStateGattSubscribing)
        {
            if ((peer = peerFind(connectionId)) != NULL)
            {
                task->mCount = evtWcmd->count;
                task->mConnection = peer;
                task->mType = kHandleC1WriteDone;
                SignalBleTaskPending(task);
            }
        }

        sBle.mState = kStateIdle;
        break;
    }
        // Fall through

    case BLE_GATTC_EVT_WRITE_RSP:
    {
        if (sBle.mState != kStateGattSubscribing)
        {
            if ((peer = peerFind(connectionId)) != NULL)
            {
                task->mCount = 1;
                task->mConnection = peer;
                task->mType = kHandleC1WriteDone;
                SignalBleTaskPending(task);
            }
        }

        sBle.mState = kStateIdle;

        break;
    }

    case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
    {
        ble_gattc_evt_prim_srvc_disc_rsp_t *evtDisc = &evt->evt.gattc_evt.params.prim_srvc_disc_rsp;

        if (evtDisc->count == 1)
        {
            sBle.mServiceHandleRange = evtDisc->services[0].handle_range;
            sBle.mState              = kStateGattiDiscoverUuidC1Handle;

            otLogDebgPlat("[BLE] BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP, start_handle=%d end_handle=%d",
                          sBle.mServiceHandleRange.start_handle, sBle.mServiceHandleRange.end_handle);
            bleGattcCharValueByUuidRead(connectionId, &sBle.mServiceHandleRange,
                                        &sBtpServices[0].mCharacteristics[0].mUuid);
        }
        else
        {
            otLogDebgPlat("[BLE] BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP, Not Found");
        }

        break;
    }

    case BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP:
    {
        ble_gattc_evt_char_val_by_uuid_read_rsp_t *evtDisc = &evt->evt.gattc_evt.params.char_val_by_uuid_read_rsp;

        otEXPECT(evtDisc->count > 0);

        if (sBle.mState == kStateGattiDiscoverUuidC1Handle)
        {
            otLogDebgPlat("[BLE] BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP, count=%d value_len=%d UuidC1Handle=%d",
                          evtDisc->count, evtDisc->value_len, evtDisc->handle_value[0]);
            sBle.mState    = kStateGattiDiscoverUuidC2Handle;
            sBle.mC1Handle = evtDisc->handle_value[0];

            bleGattcCharValueByUuidRead(connectionId, &sBle.mServiceHandleRange,
                                        &sBtpServices[0].mCharacteristics[1].mUuid);
        }
        else if (sBle.mState == kStateGattiDiscoverUuidC2Handle)
        {
            otPlatBleUuid uuid = {.mType = OT_BLE_UUID_TYPE_16, .mValue.mUuid16 = OT_BLE_UUID_CCCD};

            otLogDebgPlat("[BLE] BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP, count=%d value_len=%d UuidC2Handle=%d",
                          evtDisc->count, evtDisc->value_len, evtDisc->handle_value[0]);
            sBle.mState    = kStateGattiDiscoverUuidC2CccdHandle;
            sBle.mC2Handle = evtDisc->handle_value[0];

            bleGattcCharValueByUuidRead(connectionId, &sBle.mServiceHandleRange, &uuid);
        }
        else if (sBle.mState == kStateGattiDiscoverUuidC2CccdHandle)
        {
            otLogDebgPlat("[BLE] BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP, count=%d value_len=%d UuidC2CccdHandle=%d",
                          evtDisc->count, evtDisc->value_len, evtDisc->handle_value[0]);
            sBle.mC2CccdHandle = evtDisc->handle_value[0];

            if ((peer = peerFind(connectionId)) != NULL)
            {
                sPeripheralPeer = NULL;
                task->mConnection = peer;
                task->mLinkType = peer->mLinkType;
                task->mType = kHandleConnectionIsReady;
                SignalBleTaskPending(task);
            }
        }

        break;
    }

    case BLE_L2CAP_EVT_CH_SETUP_REQUEST:
    {
        ble_l2cap_evt_t *                 l2capEvt       = &evt->evt.l2cap_evt;
        ble_l2cap_evt_ch_setup_request_t *chSetupRequest = &evt->evt.l2cap_evt.params.ch_setup_request;

        otLogDebgPlat("[BLE] BLE_L2CAP_EVT_CH_SETUP_REQUEST: mtu %d psm 0x%x local_cid=%d connectionId=%d",
                      chSetupRequest->tx_params.tx_mtu, chSetupRequest->le_psm, l2capEvt->local_cid,
                      l2capEvt->conn_handle);

        if (chSetupRequest->le_psm == BLE_L2CAP_PSM)
        {
            if ((peer = peerFind(connectionId)) != NULL)
            {
                peer->mLocalCid = l2capEvt->local_cid;
                peer->mLinkType = kConnectionLinkTypeL2Cap;
                bleL2capSendConnectionResponse(peer, 0, BLE_L2CAP_MTU);
            }
        }

        break;
    }

    case BLE_L2CAP_EVT_CH_SETUP:
    {
        ble_l2cap_evt_t *l2capEvt = &evt->evt.l2cap_evt;
        ble_data_t       sduBuf;

        otLogDebgPlat("[BLE] BLE_L2CAP_EVT_CH_SETUP: local_cid=%d connectionId=%d, L2CAP connected",
                      l2capEvt->local_cid, l2capEvt->conn_handle);

        if (((peer = peerFind(connectionId))) != NULL && (peer->mIsPeripheral))
        {
            sPeripheralPeer = NULL;
            task->mConnection = peer;
            task->mLinkType = peer->mLinkType;
            task->mType = kHandleConnectionIsReady;
            SignalBleTaskPending(task);
        }

        // TODO: Currently only 1 RX buffer is available.
        sduBuf.p_data = sL2capRxBuffer;
        sduBuf.len    = sizeof(sL2capRxBuffer);

        retval = sd_ble_l2cap_ch_rx(connectionId, l2capEvt->local_cid, &sduBuf);

        if (retval != NRF_SUCCESS)
        {
            otLogCritPlat("[BLE] sd_ble_l2cap_ch_rx error: 0x%x", retval);
            assert(retval == NRF_SUCCESS);
        }

        break;
    }

    case BLE_L2CAP_EVT_CH_SETUP_REFUSED:
    {
        sd_ble_gap_disconnect(connectionId, OT_BLE_HCI_REMOTE_USER_TERMINATED);
        break;
    }

    case BLE_L2CAP_EVT_CH_RELEASED:
    {
        break;
    }

    case BLE_L2CAP_EVT_CH_RX:
    {
        ble_l2cap_evt_t *      l2capEvt = &evt->evt.l2cap_evt;
        ble_l2cap_evt_ch_rx_t *chRx     = &l2capEvt->params.rx;
        otBleRadioPacket       packet;
        ble_data_t             sduBuf;
        uint8_t                channel_index;

        retval = sd_ble_gap_rssi_get(connectionId, &packet.mPower, &channel_index);

        packet.mValue  = chRx->sdu_buf.p_data;
        packet.mLength = MIN(l2capEvt->params.rx.sdu_buf.len, l2capEvt->params.rx.sdu_len);

        if ((peer = peerFind(l2capEvt->conn_handle)) != NULL)
        {
            // FIXME: @gjc @zhanglongxia Should post task
            if (sDiagMode)
            {
                otPlatTobleDiagL2CapFrameReceived(sInstance, (otTobleConnection *)peer, chRx->sdu_buf.p_data,
                                                  MIN(l2capEvt->params.rx.sdu_buf.len, l2capEvt->params.rx.sdu_len));
            }
            else
            {
                otPlatTobleL2CapFrameReceived(sInstance, (otTobleConnection *)peer, chRx->sdu_buf.p_data,
                                              MIN(l2capEvt->params.rx.sdu_buf.len, l2capEvt->params.rx.sdu_len));
            }
        }
        else
        {
            otLogDebgPlat("[BLE] BLE_L2CAP_EVT_CH_RX: ConnectionId=%d Peer=NULL", l2capEvt->conn_handle);
        }

        // TODO: Currently only 1 RX buffer is available.
        sduBuf.p_data = sL2capRxBuffer;
        sduBuf.len    = sizeof(sL2capRxBuffer);

        retval = sd_ble_l2cap_ch_rx(connectionId, l2capEvt->local_cid, &sduBuf);

        if (retval != NRF_SUCCESS)
        {
            otLogCritPlat("[BLE] sd_ble_l2cap_ch_rx error: 0x%x", retval);
            assert(retval == NRF_SUCCESS);
        }

        break;
    }

    case BLE_L2CAP_EVT_CH_TX:
    {
        ble_l2cap_evt_t *l2capEvt = &evt->evt.l2cap_evt;

        if ((peer = peerFind(l2capEvt->conn_handle)) != NULL)
        {
            // FIXME: @gjc @zhanglongxia Should post task
            if (sDiagMode)
            {
                otPlatTobleDiagL2capSendDone(sInstance, (otTobleConnection *)peer);
            }
            else
            {
                otPlatTobleL2capSendDone(sInstance, (otTobleConnection *)peer);
            }
        }
        break;
    }

    default:
        otLogInfoPlat("[BLE] Unhandled BLE event 0x%02x", evt->header.evt_id);
        break;
    }

exit:
    return;
}

void nrf5TobleProcess(void)
{
    otLogCritPlat("nrf5TobleProcess %d %d", sTaskQueueHead, sTaskQueueTail);
    for (uint8_t i = sTaskQueueHead; i != sTaskQueueTail; i++, i %= BLE_TASK_COUNT)
    {
        otLogInfoPlat("nrf5TobleProcess: type=%d", sPendingTasks[i].mType);
        switch (sPendingTasks[i].mType)
        {
        case kHandleConnected:
            if (sDiagMode)
            {
                otPlatTobleDiagHandleConnected(sInstance, sPendingTasks[i].mConnection);
            }
            else
            {
                otPlatTobleHandleConnected(sInstance, sPendingTasks[i].mConnection);
            }
            break;
        case kHandleDisconnected:
            if (sDiagMode)
            {
                otPlatTobleDiagHandleDisconnected(sInstance, sPendingTasks[i].mConnection);
            }
            else
            {
                otPlatTobleHandleDisconnected(sInstance, sPendingTasks[i].mConnection);
            }
            break;
        case kHandleScanResponse:
            if (sDiagMode)
            {
                otPlatTobleDiagGapOnScanRespReceived(sInstance, &sPendingTasks[i].mAdvertisement);
            }
            else
            {
                otPlatTobleGapOnScanRespReceived(sInstance, &sPendingTasks[i].mAdvertisement);
            }
            break;
        case kHandleAdvertisement:
            if (sDiagMode)
            {
                otPlatTobleDiagGapOnAdvReceived(sInstance, sPendingTasks[i].mAdvertisementType,
                                                &sPendingTasks[i].mAdvertisement);
            }
            else
            {
                otPlatTobleGapOnAdvReceived(sInstance, sPendingTasks[i].mAdvertisementType,
                                            &sPendingTasks[i].mAdvertisement);
            }
            break;
        case kHandleC1Write:
            if (sDiagMode)
            {
                otPlatTobleDiagHandleC1Write(sInstance, sPendingTasks[i].mConnection, sPendingTasks[i].mBuffer,
                                             sPendingTasks[i].mBufferLength);
            }
            else
            {
                otPlatTobleHandleC1Write(sInstance, sPendingTasks[i].mConnection, sPendingTasks[i].mBuffer,
                                         sPendingTasks[i].mBufferLength);
            }
            break;
        case kHandleC1WriteDone:
            for (uint8_t j = 0; j < sPendingTasks[i].mCount; j++)
            {
                if (sDiagMode)
                {
                    otPlatTobleDiagHandleC1WriteDone(sInstance, sPendingTasks[i].mConnection);
                }
                else
                {
                    otPlatTobleHandleC1WriteDone(sInstance, sPendingTasks[i].mConnection);
                }
            }
            break;
        case kHandleC2Subscribed:
            if (sDiagMode)
            {
                otPlatTobleDiagHandleC2Subscribed(sInstance, sPendingTasks[i].mConnection,
                                                  sPendingTasks[i].mSubscribed);
            }
            else
            {
                otPlatTobleHandleC2Subscribed(sInstance, sPendingTasks[i].mConnection, sPendingTasks[i].mSubscribed);
            }
            break;
        case kHandleC2Notification:
            if (sDiagMode)
            {
                otPlatTobleDiagHandleC2Notification(sInstance, sPendingTasks[i].mConnection, sPendingTasks[i].mBuffer,
                                                    sPendingTasks[i].mBufferLength);
            }
            else
            {
                otPlatTobleHandleC2Notification(sInstance, sPendingTasks[i].mConnection, sPendingTasks[i].mBuffer,
                                                sPendingTasks[i].mBufferLength);
            }
            break;
        case kHandleC2NotificationDone:
            otLogInfoPlat("nrf5TobleProcess: %p count=%d", &sPendingTasks[i], sPendingTasks[i].mCount);
            otLogInfoPlat("WTF!!!\n");
            for (uint8_t j = 0; j < sPendingTasks[i].mCount; j++)
            {
                otLogInfoPlat("Inside loop\n");
                if (sDiagMode)
                {
                    otPlatTobleDiagHandleC2NotificateDone(sInstance, sPendingTasks[i].mConnection);
                }
                else
                {
                    otLogInfoPlat("Call otPlatTobleHandleC2NotificateDone\n");
                    otPlatTobleHandleC2NotificateDone(sInstance, sPendingTasks[i].mConnection);
                }
            }
            otLogInfoPlat("nrf5TobleProcess: %p count=%d", &sPendingTasks[i], sPendingTasks[i].mCount);
            break;
        case kHandleConnectionIsReady:
            if (sDiagMode)
            {
                otPlatTobleDiagHandleConnectionIsReady(sInstance, sPendingTasks[i].mConnection,
                                                       sPendingTasks[i].mLinkType);
            }
            else
            {
                otPlatTobleHandleConnectionIsReady(sInstance, sPendingTasks[i].mConnection, sPendingTasks[i].mLinkType);
            }
            break;
        case kHandleL2CapFrameReceived:
        case kHandleL2CapScanSendDone:
        default:
            break;
        }
        sPendingTasks[i].mPending = false;
    }

    sTaskQueueHead = sTaskQueueTail;
    return;
}

NRF_SDH_BLE_OBSERVER(m_ot_ble_observer, 0, ble_evt_handler, NULL);

/*******************************************************************************
 * @section Definition of weak ToBLE platform functions.
 *******************************************************************************/

OT_TOOL_WEAK void otPlatTobleGapOnAdvReceived(otInstance *      aInstance,
                                              otTobleAdvType    aAdvType,
                                              otTobleAdvPacket *aAdvPacket)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aAdvType);
    OT_UNUSED_VARIABLE(aAdvPacket);
}

OT_TOOL_WEAK void otPlatTobleGapOnScanRespReceived(otInstance *aInstance, otTobleAdvPacket *aAdvPacket)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aAdvPacket);
}

OT_TOOL_WEAK void otPlatTobleHandleConnected(otInstance *aInstance, otTobleConnection *aConn)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConn);
}

OT_TOOL_WEAK void otPlatTobleHandleDisconnected(otInstance *aInstance, otTobleConnection *aConn)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConn);
}

OT_TOOL_WEAK void otPlatTobleHandleAdv(otInstance *          aInstance,
                                       otTobleAdvType        aAdvType,
                                       const otTobleAddress *aSource,
                                       const uint8_t *       aData,
                                       uint16_t              aLength,
                                       int8_t                aRssi)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aAdvType);
    OT_UNUSED_VARIABLE(aSource);
    OT_UNUSED_VARIABLE(aData);
    OT_UNUSED_VARIABLE(aLength);
    OT_UNUSED_VARIABLE(aRssi);
}

OT_TOOL_WEAK void otPlatTobleHandleConnectionIsReady(otInstance *              aInstance,
                                                     otTobleConnection *       aConn,
                                                     otTobleConnectionLinkType aLinkType)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConn);
    OT_UNUSED_VARIABLE(aLinkType);
}

OT_TOOL_WEAK void otPlatTobleHandleC1WriteDone(otInstance *aInstance, otTobleConnection *aConn)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConn);
}

OT_TOOL_WEAK void otPlatTobleHandleC2Notification(otInstance *       aInstance,
                                                  otTobleConnection *aConn,
                                                  const uint8_t *    aBuffer,
                                                  uint16_t           aLength)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConn);
    OT_UNUSED_VARIABLE(aBuffer);
    OT_UNUSED_VARIABLE(aLength);
}

OT_TOOL_WEAK void otPlatTobleHandleC2Subscribed(otInstance *aInstance, otTobleConnection *aConn, bool aIsSubscribed)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConn);
    OT_UNUSED_VARIABLE(aIsSubscribed);
}

OT_TOOL_WEAK void otPlatTobleHandleC2NotificateDone(otInstance *aInstance, otTobleConnection *aConn)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConn);
}

OT_TOOL_WEAK void otPlatTobleHandleC1Write(otInstance *       aInstance,
                                           otTobleConnection *aConn,
                                           const uint8_t *    aBuffer,
                                           uint16_t           aLength)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConn);
    OT_UNUSED_VARIABLE(aBuffer);
    OT_UNUSED_VARIABLE(aLength);
}

OT_TOOL_WEAK void otPlatTobleL2capSendDone(otInstance *aInstance, otTobleConnection *aConn)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConn);
}

OT_TOOL_WEAK void otPlatTobleL2CapFrameReceived(otInstance *       aInstance,
                                                otTobleConnection *aConn,
                                                const uint8_t *    aBuffer,
                                                uint16_t           aLength)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConn);
    OT_UNUSED_VARIABLE(aBuffer);
    OT_UNUSED_VARIABLE(aLength);
}

//---------------------------

OT_TOOL_WEAK void otPlatTobleDiagGapOnAdvReceived(otInstance *      aInstance,
                                                  otTobleAdvType    aAdvType,
                                                  otTobleAdvPacket *aAdvPacket)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aAdvPacket);
}

OT_TOOL_WEAK void otPlatTobleDiagGapOnScanRespReceived(otInstance *aInstance, otTobleAdvPacket *aAdvPacket)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aAdvPacket);
}

OT_TOOL_WEAK void otPlatTobleDiagHandleConnected(otInstance *aInstance, otTobleConnection *aConn)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConn);
}

OT_TOOL_WEAK void otPlatTobleDiagHandleDisconnected(otInstance *aInstance, otTobleConnection *aConn)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConn);
}

OT_TOOL_WEAK void otPlatTobleDiagHandleConnectionIsReady(otInstance *              aInstance,
                                                         otTobleConnection *       aConn,
                                                         otTobleConnectionLinkType aLinkType)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConn);
    OT_UNUSED_VARIABLE(aLinkType);
}

OT_TOOL_WEAK void otPlatTobleDiagHandleC1WriteDone(otInstance *aInstance, otTobleConnection *aConn)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConn);
}

OT_TOOL_WEAK void otPlatTobleDiagHandleC2Notification(otInstance *       aInstance,
                                                      otTobleConnection *aConn,
                                                      const uint8_t *    aBuffer,
                                                      uint16_t           aLength)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConn);
    OT_UNUSED_VARIABLE(aBuffer);
    OT_UNUSED_VARIABLE(aLength);
}

OT_TOOL_WEAK void otPlatTobleDiagHandleC2Subscribed(otInstance *aInstance, otTobleConnection *aConn, bool aIsSubscribed)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConn);
    OT_UNUSED_VARIABLE(aIsSubscribed);
}

OT_TOOL_WEAK void otPlatTobleDiagHandleC2NotificateDone(otInstance *aInstance, otTobleConnection *aConn)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConn);
}

OT_TOOL_WEAK void otPlatTobleDiagHandleC1Write(otInstance *       aInstance,
                                               otTobleConnection *aConn,
                                               const uint8_t *    aBuffer,
                                               uint16_t           aLength)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConn);
    OT_UNUSED_VARIABLE(aBuffer);
    OT_UNUSED_VARIABLE(aLength);
}

OT_TOOL_WEAK void otPlatTobleDiagL2capSendDone(otInstance *aInstance, otTobleConnection *aConn)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConn);
}

OT_TOOL_WEAK void otPlatTobleDiagL2CapFrameReceived(otInstance *       aInstance,
                                                    otTobleConnection *aConn,
                                                    const uint8_t *    aBuffer,
                                                    uint16_t           aLength)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConn);
    OT_UNUSED_VARIABLE(aBuffer);
    OT_UNUSED_VARIABLE(aLength);
}

#endif // #if OPENTHREAD_CONFIG_TOBLE_ENABLE && SOFTDEVICE_PRESENT
