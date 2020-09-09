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
#include "openthread-posix-config.h"
#include "platform-posix.h"

#include "common/code_utils.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <openthread/error.h>
#include <openthread/instance.h>
#include <openthread/platform/ble.h>
#include <openthread/platform/toble.h>
#include <openthread/platform/toolchain.h>

#include "common/debug.hpp"
#include "common/logging.hpp"
#include "lib/platform/exit_code.h"

// Bluez local version of public includes
#include "lib/bluetooth.h"
#include "lib/hci.h"
#include "lib/hci_lib.h"
#include "lib/l2cap.h"
#include "lib/mgmt.h"
#include "lib/uuid.h"

// Bluez internal includes
#include "src/shared/att.h"
#include "src/shared/gap.h"
#include "src/shared/gatt-client.h"
#include "src/shared/gatt-db.h"
#include "src/shared/gatt-helpers.h"
#include "src/shared/gatt-server.h"
#include "src/shared/hci.h"
#include "src/shared/mainloop.h"
#include "src/shared/mgmt.h"
#include "src/shared/queue.h"
#include "src/shared/util.h"

#include "events.hpp"

#if OPENTHREAD_CONFIG_TOBLE_ENABLE && OPENTHREAD_CONFIG_BLE_HOST_BLUEZ_ENABLE

/*
 * This enum defines the standard GAP service UUID and standard GATT service UUID.
 */
enum
{
    kUuidServiceGap  = 0x1800, ///< Generic Access.
    kUuidServiceGatt = 0x1801, ///< Generic Attribute.
};

#define BLE_EIR_FLAGS 0x01           ///< The Flags AD type.
#define BLE_ADV_INSTANCE 2           ///< Advertisement instance.
#define BLE_DEFAULT_ATT_MTU_SIZE 131 ///< Default ATT_MTU.
#define BLE_MAX_ADV_DATA_LEN 31      ///< Maximum advertisement data length.

// BLE_GATT_CCCD_TYPES GATT Handle Value operations
#define BLE_GATT_CCCD_INVALID 0x00      ///< Invalid Operation.
#define BLE_GATT_CCCD_NOTIFICATION 0x01 ///< Handle Value Notification.
#define BLE_GATT_CCCD_INDICATION 0x02   ///< Handle Value Indication.

#define BLE_CONN_MAX_SUPERVISION_TIMEOUT 0x0C80 ///< Maximum supervision timeout of BLE connection.

#define BLE_MS_TO_625US_TICK(aMs) (((aMs)*1000UL) / 625)   ///< Convert milliseconds to 625us ticks.
#define BLE_MS_TO_1_25MS_TICK(aMs) (((aMs)*1000UL) / 1250) ///< Convert milliseconds to 1.25ms ticks.

#define BLE_NUM_CONNCETIONS 10 ///< Number of BLE connections.
#define BLE_DEBUG 0            ///< Enable debug.

/**
 * BLE Gatt states.
 */
typedef enum
{
    kConnStateIdle,                         ///< Idle state.
    kConnStateConnecting,                   ///< Waiting for BLE connection to be created.
    kConnStateGattDiscoverBtpService,       ///< Gatt discover service request sent, and wait for response.
    kConnStateGattDiscoverUuidC1C2Handle,   ///< Gatt discover characteristics request sent, and wait for response.
    kConnStateGattDiscoverUuidC2CccdHandle, ///< Gatt discover descriptor request sent, and wait for response.
    kConnStateGattWaitServiceAdd,           ///< Waiting for the BTP service to be added to the client gatt DB.
    kConnStateConnected,                    ///< Connected.
} ConnState;

typedef enum
{
    kBleStateIdle,        ///< Idle state.
    kBleStateAdvertising, ///< Device is advertising.
    kBleStateScanning,    ///< Device is scanning.
} BleState;

static pthread_t      sMainLoopThreadId;
static otInstance *   sInstance;
static otTobleAddress sBleRandomAddress;
static int            sPipeFd[2]    = {-1, -1};
static struct mgmt *  sMgmt         = NULL;
static int            sHciIndex     = MGMT_INDEX_NONE;
static bool           sDiagMode     = false;
static bool           sBluezVerbose = false;
static BleState       sBleState     = kBleStateIdle;

struct TobleConnection
{
    bool           mIsUsed;
    otTobleAddress mPeerAddress;

    unsigned int            mNotifyId;
    struct bt_att *         mAtt;
    struct gatt_db *        mDb;
    struct bt_gatt_client * mGattClient;
    struct bt_gatt_request *gatt_request;
    struct bt_gatt_server * mGattServer;

    uint16_t mC1Handle;
    uint16_t mC2Handle;
    uint16_t mC2CccdHandle;

    ConnState mState;
};

static int  sServerAttListenFd   = -1;
static bool sServerNotifyEnabled = false;

static uint16_t sScanInterval;
static uint16_t sScanWindow;

static const char sDeviceName[] = "OpenThread";

static otError bzScanStart(uint16_t aInterval, uint16_t aWindow);
static otError bzScanStop(uint16_t aIndex);
static otError bzRemoveAdvertising(uint16_t aIndex);
static void    bzRemoveDevice(const otTobleAddress *aPeerAddress);

#define OT_TOBLE_UUID_C1 0x18, 0xEE, 0x2E, 0xF5, 0x26, 0x3D, 0x45, 0x59, 0x95, 0x9F, 0x4F, 0x9C, 0x42, 0x9F, 0x9D, 0x11
#define OT_TOBLE_UUID_C2 0x18, 0xEE, 0x2E, 0xF5, 0x26, 0x3D, 0x45, 0x59, 0x95, 0x9F, 0x4F, 0x9C, 0x42, 0x9F, 0x9D, 0x12

/**
 * Characteristic C1 UUID: 18EE2EF5-263D-4559-959F-4F9C429F9D11
 */
static uint128_t sBtpUuidC1 = {{OT_TOBLE_UUID_C1}};

/**
 * Characteristic C2 UUID: 18EE2EF5-263D-4559-959F-4F9C429F9D12
 */
static uint128_t sBtpUuidC2 = {{OT_TOBLE_UUID_C2}};

static TobleConnection *sServerConn;
static TobleConnection  sConnTable[BLE_NUM_CONNCETIONS];

static TobleConnection *ConnectionNew(const otTobleAddress *aPeerAddress)
{
    TobleConnection *conn = NULL;

    for (size_t i = 0; i < OT_ARRAY_LENGTH(sConnTable); i++)
    {
        if (!sConnTable[i].mIsUsed)
        {
            conn          = &sConnTable[i];
            conn->mIsUsed = true;

            if (aPeerAddress != NULL)
            {
                memcpy(&conn->mPeerAddress, aPeerAddress, sizeof(otTobleAddress));
            }
            break;
        }
    }

    return conn;
}

static TobleConnection *ConnectionFind(const otTobleAddress *aPeerAddress)
{
    TobleConnection *retConn = NULL;

    for (TobleConnection *conn = &sConnTable[0]; conn < OT_ARRAY_END(sConnTable); conn++)
    {
        if (conn->mIsUsed && (memcmp(&conn->mPeerAddress.mAddress, aPeerAddress->mAddress, OT_TOBLE_ADDRESS_SIZE) == 0))
        {
            retConn = conn;
            break;
        }
    }

    return retConn;
}

static void ConnectionFree(TobleConnection *aConn)
{
    OT_ASSERT(aConn != NULL);

    if (aConn == sServerConn)
    {
        memset(&aConn->mPeerAddress, 0, sizeof(aConn->mPeerAddress));
    }
    else
    {
        aConn->mIsUsed = false;
        aConn->mState  = kConnStateIdle;
    }
}

#if (OPENTHREAD_CONFIG_LOG_LEVEL >= OT_LOG_LEVEL_INFO) && (OPENTHREAD_CONFIG_LOG_PLATFORM == 1)
static const char *AddressToString(const otTobleAddress *aAddr)
{
    static char string[25];

    snprintf(string, sizeof(string) - 1, "%u %02x%02x%02x%02x%02x%02x", aAddr->mType, aAddr->mAddress[0],
             aAddr->mAddress[1], aAddr->mAddress[2], aAddr->mAddress[3], aAddr->mAddress[4], aAddr->mAddress[5]);

    return string;
}
#endif

static void bzAttGattDebugCb(const char *aString, void *aUserData)
{
    const char *prefix = reinterpret_cast<const char *>(aUserData);
    OT_UNUSED_VARIABLE(aString);
    OT_UNUSED_VARIABLE(prefix);

    otLogInfoPlat("%s %s", prefix, aString);
}

static inline uint8_t AddressTypeTobleToBluez(otTobleAddressType aType)
{
    return (aType == OT_TOBLE_ADDRESS_TYPE_PUBLIC) ? BDADDR_LE_PUBLIC : BDADDR_LE_RANDOM;
}

static inline otTobleAddressType AddressTypeBluezToToble(uint8_t aType)
{
    return (aType == BDADDR_LE_PUBLIC) ? OT_TOBLE_ADDRESS_TYPE_PUBLIC : OT_TOBLE_ADDRESS_TYPE_RANDOM_STATIC;
}

/*******************************************************************************
 * @section Bluez ATT and GATT.
 *******************************************************************************/

static void bzGattDbAttributeWriteCb(struct gatt_db_attribute *aAttribute, int aError, void *aUserData)
{
    OT_UNUSED_VARIABLE(aAttribute);
    OT_UNUSED_VARIABLE(aUserData);

    if (aError)
    {
        otLogCritPlat("[Bluez] Error setting attribute: %d\n", aError);
        DieNow(OT_EXIT_FAILURE);
    }
}

static void bzGattDbDeviceNameReadCb(struct gatt_db_attribute *aAttribute,
                                     unsigned int              aId,
                                     uint16_t                  aOffset,
                                     uint8_t                   aOpcode,
                                     struct bt_att *           aAtt,
                                     void *                    aUserData)
{
    OT_UNUSED_VARIABLE(aOpcode);
    OT_UNUSED_VARIABLE(aAtt);
    OT_UNUSED_VARIABLE(aUserData);

    uint8_t        error;
    const uint8_t *value;
    size_t         len;

    if (aOffset > strlen(sDeviceName))
    {
        error = BT_ATT_ERROR_INVALID_OFFSET;
        value = NULL;
        len   = strlen(sDeviceName);
    }
    else
    {
        error = 0;
        len   = strlen(sDeviceName) - aOffset;
        value = len ? reinterpret_cast<const uint8_t *>(&sDeviceName[aOffset]) : NULL;
    }

    gatt_db_attribute_read_result(aAttribute, aId, error, value, len);
}

static void bzGattDbC2CccdReadCb(struct gatt_db_attribute *aAttribute,
                                 unsigned int              aId,
                                 uint16_t                  aOffset,
                                 uint8_t                   aOpcode,
                                 struct bt_att *           aAtt,
                                 void *                    aUserData)
{
    OT_UNUSED_VARIABLE(aOffset);
    OT_UNUSED_VARIABLE(aOpcode);
    OT_UNUSED_VARIABLE(aAtt);
    OT_UNUSED_VARIABLE(aUserData);

    uint8_t value[2];

    value[0] = sServerNotifyEnabled ? BLE_GATT_CCCD_NOTIFICATION : 0x00;
    value[1] = 0x00;

    gatt_db_attribute_read_result(aAttribute, aId, 0, value, sizeof(value));
}

static void bzGattDbC2CccdWriteCb(struct gatt_db_attribute *aAttribute,
                                  unsigned int              aId,
                                  uint16_t                  aOffset,
                                  const uint8_t *           aValue,
                                  size_t                    aLength,
                                  uint8_t                   aOpcode,
                                  struct bt_att *           aAtt,
                                  void *                    aUserData)
{
    OT_UNUSED_VARIABLE(aOpcode);
    OT_UNUSED_VARIABLE(aAtt);

    int                         error = 0;
    TobleConnection *           conn  = reinterpret_cast<TobleConnection *>(aUserData);
    struct BluezGattC2CccdEvent event;

    VerifyOrExit(aOffset == 0, error = BT_ATT_ERROR_INVALID_OFFSET);
    VerifyOrExit((aValue != NULL) && (aLength == 2), error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN);

    if (aValue[0] == 0x00)
    {
        sServerNotifyEnabled = false;
    }
    else if (aValue[0] == BLE_GATT_CCCD_NOTIFICATION)
    {
        sServerNotifyEnabled = true;
    }
    else
    {
        error = BT_ATT_ERROR_VALUE_NOT_ALLOWED;
    }

    VerifyOrExit(error == 0, OT_NOOP);
    event.Init(reinterpret_cast<otTobleConnection *>(conn), sServerNotifyEnabled);
    VerifyOrDie(write(sPipeFd[1], &event, event.GetSize()) > 0, OT_EXIT_FAILURE);

exit:
    gatt_db_attribute_write_result(aAttribute, aId, error);
}

static void bzGattDbC1WriteCb(struct gatt_db_attribute *aAttribute,
                              unsigned int              aId,
                              uint16_t                  aOffset,
                              const uint8_t *           aValue,
                              size_t                    aLength,
                              uint8_t                   aOpcode,
                              struct bt_att *           aAtt,
                              void *                    aUserData)
{
    OT_UNUSED_VARIABLE(aAttribute);
    OT_UNUSED_VARIABLE(aOpcode);
    OT_UNUSED_VARIABLE(aAtt);
    OT_UNUSED_VARIABLE(aUserData);

    int                          error = 0;
    struct BluezGattC1WriteEvent event;

    VerifyOrExit((aOffset == 0) && (aLength > 0) && (aLength <= GATT_DATA_MAX_LEN), OT_NOOP);
    SuccessOrExit(event.Init(reinterpret_cast<otTobleConnection *>(aUserData), aValue, aLength));

    VerifyOrDie(write(sPipeFd[1], &event, event.GetSize()) > 0, OT_EXIT_ERROR_ERRNO);

exit:
    gatt_db_attribute_write_result(aAttribute, aId, error);
}

static void bzPopulateGapService(struct gatt_db *aDb)
{
    bt_uuid_t                 uuid;
    struct gatt_db_attribute *service;
    struct gatt_db_attribute *attr;
    uint16_t                  appearance;

    // Standard GAP Services
    bt_uuid16_create(&uuid, kUuidServiceGap);
    service = gatt_db_add_service(aDb, &uuid, true, 6);

    bt_uuid16_create(&uuid, GATT_CHARAC_DEVICE_NAME);
    attr = gatt_db_service_add_characteristic(service, &uuid, BT_ATT_PERM_READ, BT_GATT_CHRC_PROP_READ,
                                              bzGattDbDeviceNameReadCb, NULL, NULL);

    gatt_db_attribute_write(attr, 0, reinterpret_cast<const uint8_t *>(sDeviceName), sizeof(sDeviceName),
                            BT_ATT_OP_WRITE_REQ, NULL, bzGattDbAttributeWriteCb, NULL);

    bt_uuid16_create(&uuid, GATT_CHARAC_APPEARANCE);
    attr =
        gatt_db_service_add_characteristic(service, &uuid, BT_ATT_PERM_READ, BT_GATT_CHRC_PROP_READ, NULL, NULL, NULL);

    put_le16(128, &appearance);
    gatt_db_attribute_write(attr, 0, (const uint8_t *)&appearance, sizeof(appearance), BT_ATT_OP_WRITE_REQ, NULL,
                            bzGattDbAttributeWriteCb, NULL);

    gatt_db_service_set_active(service, true);

    // Standard GATT Services
    bt_uuid16_create(&uuid, kUuidServiceGap);
    service = gatt_db_add_service(aDb, &uuid, true, 4);
    gatt_db_service_set_active(service, true);
}

static void bzPopulateTobleBtpService(struct gatt_db *aDb)
{
    bt_uuid_t                 uuid;
    struct gatt_db_attribute *service;
    struct gatt_db_attribute *attr;

    bt_uuid16_create(&uuid, OT_PLAT_TOBLE_UUID_THREAD_GROUP);
    service = gatt_db_add_service(aDb, &uuid, true, 6);

    bt_uuid128_create(&uuid, sBtpUuidC1);
    attr = gatt_db_service_add_characteristic(service, &uuid, BT_ATT_PERM_WRITE | BT_ATT_PERM_READ,
                                              BT_GATT_CHRC_PROP_WRITE | BT_GATT_CHRC_PROP_WRITE_WITHOUT_RESP, NULL,
                                              bzGattDbC1WriteCb, sServerConn);

    sServerConn->mC1Handle = gatt_db_attribute_get_handle(attr);

    bt_uuid128_create(&uuid, sBtpUuidC2);
    attr = gatt_db_service_add_characteristic(
        service, &uuid, BT_ATT_PERM_WRITE | BT_ATT_PERM_READ,
        BT_GATT_CHRC_PROP_READ | BT_GATT_CHRC_PROP_NOTIFY | BT_GATT_CHRC_PROP_INDICATE, NULL, NULL, NULL);

    sServerConn->mC2Handle = gatt_db_attribute_get_handle(attr);

    bt_uuid16_create(&uuid, GATT_CLIENT_CHARAC_CFG_UUID);
    attr = gatt_db_service_add_descriptor(attr, &uuid, BT_ATT_PERM_READ | BT_ATT_PERM_WRITE, bzGattDbC2CccdReadCb,
                                          bzGattDbC2CccdWriteCb, sServerConn);
    sServerConn->mC2CccdHandle = gatt_db_attribute_get_handle(attr);

    gatt_db_service_set_active(service, true);
}

static void bzGattConnDisconnectCb(int aError, void *aUserData)
{
    OT_UNUSED_VARIABLE(aError);

    TobleConnection *conn = reinterpret_cast<TobleConnection *>(aUserData);

    otLogInfoPlat("[Bluez] GATT Device disconnected: %s", strerror(aError));

    bt_gatt_server_unref(conn->mGattServer);
    bt_att_unref(conn->mAtt);
    conn->mAtt = NULL;
}

static void bzListenSocketConnectedCb(int aFd, unsigned aEvents, void *aUserData)
{
    int                clientFd;
    struct sockaddr_l2 clientAddr;
    socklen_t          clientAddrLen = sizeof(struct sockaddr_l2);
    TobleConnection *  conn          = reinterpret_cast<TobleConnection *>(aUserData);

    otLogInfoPlat("[Bluez] L2cap socket connected");

    if (aEvents & (EPOLLERR | EPOLLHUP))
    {
        otLogNotePlat("[Bluez] Remove L2cap socket from mainloop");
        mainloop_remove_fd(aFd);
        ExitNow();
    }

    VerifyOrExit(aFd == sServerAttListenFd, OT_NOOP);
    VerifyOrExit(aEvents & EPOLLIN, OT_NOOP);

    if ((clientFd = accept(aFd, (struct sockaddr *)&clientAddr, &clientAddrLen)) < 0)
    {
        otLogNotePlat("[Bluez] Failed to accept socket, errno: %s", strerror(errno));
        ExitNow();
    }

    if (conn->mAtt != NULL)
    {
        otLogNotePlat("[Bluez] Client connection already exits, reject the new connection");
        close(clientFd);
        ExitNow();
    }

    memcpy(conn->mPeerAddress.mAddress, &clientAddr.l2_bdaddr.b, OT_TOBLE_ADDRESS_SIZE);
    conn->mPeerAddress.mType = AddressTypeBluezToToble(clientAddr.l2_bdaddr_type);

    conn->mAtt = bt_att_new(clientFd, false);
    if (!conn->mAtt)
    {
        otLogNotePlat("[Bluez] Failed to initialize ATT transport layer");
        close(clientFd);
        ExitNow();
    }

    bt_att_set_close_on_unref(conn->mAtt, true);
    bt_att_register_disconnect(conn->mAtt, bzGattConnDisconnectCb, conn, NULL);

    bt_att_set_security(conn->mAtt, BT_SECURITY_LOW);

    conn->mGattServer = bt_gatt_server_new(conn->mDb, conn->mAtt, BLE_DEFAULT_ATT_MTU_SIZE, 0);
    if (!conn->mGattServer)
    {
        otLogNotePlat("[Bluez] Failed to create GATT server");
        close(clientFd);
        bt_att_unref(conn->mAtt);
        ExitNow();
    }

    otLogInfoPlat("[Bluez] New BLE connection from address %s", AddressToString(&conn->mPeerAddress));

exit:
    return;
}

static otError bzGattServerStart(void)
{
    struct sockaddr_l2 addr;
    int                att_fd = sServerAttListenFd;
    otError            error  = OT_ERROR_NONE;

    VerifyOrExit(att_fd < 0, error = OT_ERROR_FAILED);

    att_fd = socket(PF_BLUETOOTH, SOCK_SEQPACKET | SOCK_CLOEXEC, BTPROTO_L2CAP);
    if (att_fd < 0)
    {
        DieNowWithMessage("socket", OT_EXIT_ERROR_ERRNO);
    }

    memset(&addr, 0, sizeof(addr));
    addr.l2_family      = AF_BLUETOOTH;
    addr.l2_cid         = htobs(BT_ATT_CID);
    addr.l2_bdaddr_type = AddressTypeTobleToBluez(sBleRandomAddress.mType);
    memcpy(&addr.l2_bdaddr, sBleRandomAddress.mAddress, OT_TOBLE_ADDRESS_SIZE);

    if (bind(att_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        close(att_fd);
        DieNowWithMessage("bind", OT_EXIT_ERROR_ERRNO);
    }

    if (listen(att_fd, 1) < 0)
    {
        close(att_fd);
        DieNowWithMessage("listen", OT_EXIT_ERROR_ERRNO);
    }

    sServerConn->mDb = gatt_db_new();
    if (sServerConn->mDb == NULL)
    {
        otLogCritPlat("[Bluez] Failed to create GATT data base\n");
        close(att_fd);
        DieNowWithMessage("gatt_db_new", OT_EXIT_FAILURE);
    }

    bzPopulateGapService(sServerConn->mDb);
    bzPopulateTobleBtpService(sServerConn->mDb);

    mainloop_add_fd(att_fd, EPOLLIN | EPOLLRDHUP | EPOLLHUP | EPOLLERR, bzListenSocketConnectedCb, sServerConn, NULL);

    sServerAttListenFd = att_fd;

exit:
    return error;
}

static void bzGattServerStop(void)
{
    if (sServerAttListenFd < 0)
        return;

    mainloop_remove_fd(sServerAttListenFd);

    gatt_db_unref(sServerConn->mDb);
    sServerConn->mDb = NULL;

    close(sServerAttListenFd);
    sServerAttListenFd = -1;
}

static int bzL2capLeAttConnect(const bdaddr_t *aDst, uint8_t aDstType, int aSecurityLevel)
{
    int                sock;
    struct sockaddr_l2 srcaddr, dstaddr;
    struct bt_security btsec;
    char               dstaddr_str[18];

    ba2str(aDst, dstaddr_str);

    otLogInfoPlat("[Bluez] Opening L2CAP LE connection on ATT channel: dest: %s dst_type:%d\n", dstaddr_str, aDstType);

    sock = socket(PF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);
    if (sock < 0)
    {
        otLogNotePlat("[Bluez] Failed to create L2CAP socket, errno:%s", strerror(errno));
        ExitNow();
    }

    // Set up source address
    memset(&srcaddr, 0, sizeof(srcaddr));
    srcaddr.l2_family = AF_BLUETOOTH;
    srcaddr.l2_cid    = htobs(BT_ATT_CID);

    memcpy(&srcaddr.l2_bdaddr, sBleRandomAddress.mAddress, OT_TOBLE_ADDRESS_SIZE);
    srcaddr.l2_bdaddr_type = AddressTypeTobleToBluez(sBleRandomAddress.mType);

    if (bind(sock, (struct sockaddr *)&srcaddr, sizeof(srcaddr)) < 0)
    {
        otLogNotePlat("[Bluez] Failed to bind L2CAP socket, errno:%s", strerror(errno));
        close(sock);
        ExitNow(sock = -1);
    }

    // Set the security level
    memset(&btsec, 0, sizeof(btsec));
    btsec.level = aSecurityLevel;
    if (setsockopt(sock, SOL_BLUETOOTH, BT_SECURITY, &btsec, sizeof(btsec)) != 0)
    {
        otLogNotePlat("[Bluez] Failed to set L2CAP security level, errno:%s", strerror(errno));
        close(sock);
        ExitNow(sock = -1);
    }

    // Set up destination address
    memset(&dstaddr, 0, sizeof(dstaddr));
    dstaddr.l2_family      = AF_BLUETOOTH;
    dstaddr.l2_cid         = htobs(BT_ATT_CID);
    dstaddr.l2_bdaddr_type = aDstType;
    bacpy(&dstaddr.l2_bdaddr, aDst);

    otLogInfoPlat("[Bluez] Connecting to device...\n");

    if (connect(sock, (struct sockaddr *)&dstaddr, sizeof(dstaddr)) < 0)
    {
        otLogNotePlat("[Bluez] Failed to connect, error:%s", strerror(errno));
        close(sock);
        ExitNow(sock = -1);
    }

    otLogNotePlat("[Bluez] L2CAP socket connected\n");

exit:
    return sock;
}

static void bzAttDisconnectCb(int aError, void *aUserData)
{
    OT_UNUSED_VARIABLE(aUserData);
    OT_UNUSED_VARIABLE(aError);

    TobleConnection *conn = reinterpret_cast<TobleConnection *>(aUserData);

    otLogInfoPlat("[Bluez] ATT Device disconnected: %s", strerror(aError));

    bt_gatt_client_unref(conn->mGattClient);
    conn->mGattClient = NULL;

    bt_att_unref(conn->mAtt);
    conn->mAtt = NULL;

    ConnectionFree(conn);
}

static void bzGattDbServiceAddedCb(struct gatt_db_attribute *aAttribute, void *aUserData)
{
    OT_UNUSED_VARIABLE(aAttribute);
    OT_UNUSED_VARIABLE(aUserData);

    TobleConnection *conn = reinterpret_cast<TobleConnection *>(aUserData);
    uint16_t         start, end;

    if (gatt_db_attribute_get_service_handles(aAttribute, &start, &end))
    {
        if ((conn->mState == kConnStateGattWaitServiceAdd) && (conn->mC2Handle >= start) && (conn->mC2Handle <= end))
        {
            struct BluezGattDiscoverEvent event;

            otLogInfoPlat("[Bluez] C2Handle has been added to the client gatt db");

            conn->mState = kConnStateConnected;
            event.Init(conn);
            VerifyOrDie(write(sPipeFd[1], &event, event.GetSize()) > 0, OT_EXIT_ERROR_ERRNO);
        }
    }

    otLogInfoPlat("[Bluez] Service Added handle=%d", gatt_db_attribute_get_handle(aAttribute));
}

static void bzGattDbServiceRemovedCb(struct gatt_db_attribute *aAttribute, void *aUserData)
{
    OT_UNUSED_VARIABLE(aAttribute);
    OT_UNUSED_VARIABLE(aUserData);

    otLogInfoPlat("[Bluez] Service Removed");
}

static void bzGattClientReadyCb(bool aSuccess, uint8_t aAttError, void *aUserData)
{
    OT_UNUSED_VARIABLE(aUserData);
    OT_UNUSED_VARIABLE(aAttError);

    if (!aSuccess)
    {
        otLogDebgPlat("[Bluez] GATT discovery procedures complete");
    }
    else
    {
        otLogDebgPlat("[Bluez] GATT discovery procedures failed - att code: 0x%02x", aAttError);
    }
}

static otError bzGattClientCreate(TobleConnection *aConn, int aFd, uint16_t aMtu)
{
    otError error = OT_ERROR_FAILED;

    aConn->mAtt = bt_att_new(aFd, false);
    if (!aConn->mAtt)
    {
        otLogNotePlat("[Bluez] Failed to initialze ATT transport layer");
        bt_att_unref(aConn->mAtt);
        ExitNow();
    }

    if (!bt_att_set_close_on_unref(aConn->mAtt, true))
    {
        otLogNotePlat("[Bluez] Failed to set up ATT transport layer");
        bt_att_unref(aConn->mAtt);
        ExitNow();
    }

    if (!bt_att_register_disconnect(aConn->mAtt, bzAttDisconnectCb, aConn, NULL))
    {
        otLogNotePlat("[Bluez] Failed to set ATT disconnect handler");
        bt_att_unref(aConn->mAtt);
        ExitNow();
    }

    aConn->mDb = gatt_db_new();
    if (!aConn->mDb)
    {
        otLogNotePlat("[Bluez] Failed to create GATT database");
        bt_att_unref(aConn->mAtt);
        ExitNow();
    }

    aConn->mGattClient = bt_gatt_client_new(aConn->mDb, aConn->mAtt, aMtu, 0);
    if (!aConn->mGattClient)
    {
        otLogNotePlat("[Bluez] Failed to create GATT aConnent");
        bt_att_unref(aConn->mAtt);
        gatt_db_unref(aConn->mDb);
        ExitNow();
    }

    gatt_db_register(aConn->mDb, bzGattDbServiceAddedCb, bzGattDbServiceRemovedCb, aConn, NULL);
    bt_gatt_client_ready_register(aConn->mGattClient, bzGattClientReadyCb, aConn, NULL);

    gatt_db_register(aConn->mDb, bzGattDbServiceAddedCb, bzGattDbServiceRemovedCb, aConn, NULL);
    if (sBluezVerbose)
    {
        static char att[]  = "[Bluez] att: ";
        static char gatt[] = "[Bluez] gatt: ";

        bt_att_set_debug(aConn->mAtt, bzAttGattDebugCb, att, NULL);
        bt_gatt_client_set_debug(aConn->mGattClient, bzAttGattDebugCb, gatt, NULL);
    }

    gatt_db_register(aConn->mDb, bzGattDbServiceAddedCb, bzGattDbServiceRemovedCb, aConn, NULL);
    // bt_gatt_client already holds a reference.
    gatt_db_unref(aConn->mDb);
    error = OT_ERROR_NONE;

exit:
    return error;
}

static void bzGattConnDisconnect(TobleConnection *aConn)
{
    if (sServerConn == aConn)
    {
        if (aConn->mGattServer)
        {
            bt_gatt_server_unref(aConn->mGattServer);
            aConn->mGattServer = NULL;
        }
    }
    else
    {
        if (aConn->mGattClient)
        {
            bt_gatt_client_unref(aConn->mGattClient);
            aConn->mGattClient = NULL;
        }
    }

    if (aConn->mAtt)
    {
        bt_att_unref(aConn->mAtt);
        aConn->mAtt = NULL;
    }
}

/*******************************************************************************
 * @section Bluez Mgmt.
 *******************************************************************************/
static otError bzResetHciControlChannel(uint16_t aIndex)
{
    otError error = OT_ERROR_NONE;
    int     fd;
    int     ret;

    fd = hci_open_dev(aIndex);
    if (fd < 0)
    {
        otLogWarnPlat("[Bluez] Open HCI device failed");
        ExitNow(error = OT_ERROR_FAILED);
    }

    ret = ioctl(fd, HCIDEVDOWN, aIndex);
    if (ret < 0)
    {
        otLogWarnPlat("[Bluez] Power off HCI device failed");
        ExitNow(error = OT_ERROR_FAILED);
    }

    ret = ioctl(fd, HCIDEVUP, aIndex);
    if (ret < 0)
    {
        otLogWarnPlat("[Bluez] Power on HCI device failed");
        ExitNow(error = OT_ERROR_FAILED);
    }

    hci_close_dev(fd);

exit:
    return error;
}

static void *bzMainLoopThread(void *aArg)
{
    OT_UNUSED_VARIABLE(aArg);
    mainloop_run();
    return NULL;
}

static void bzMgmtDefaultSentCb(uint8_t aStatus, uint16_t aLength, const void *aParam, void *aUserData)
{
    struct BluezMgmtSendEvent event;

    SuccessOrExit(event.Init(aStatus, PTR_TO_UINT(aUserData), aParam, aLength));

    otLogDebgPlat("[Bluez] Mgmt sent callback: opcode=0x%04x len=%d status=0x%02x (%s)", PTR_TO_UINT(aUserData),
                  aLength, aStatus, mgmt_errstr(aStatus));

    VerifyOrDie(write(sPipeFd[1], &event, event.GetSize()) > 0, OT_EXIT_ERROR_ERRNO);

exit:
    return;
}

static void bzMgmtDefaultNotifyCb(uint16_t aIndex, uint16_t aLength, const void *aParam, void *aUserData)
{
    struct BluezMgmtNotifyEvent event;

    SuccessOrExit(event.Init(aIndex, PTR_TO_UINT(aUserData), aParam, aLength));

    otLogDebgPlat("[Bluez] Mgmt notify callback: index=%d event=0x%04x length=%d\r\n", aIndex, event.mEvent, aLength);
    VerifyOrDie(write(sPipeFd[1], &event, event.GetSize()) > 0, OT_EXIT_ERROR_ERRNO);

exit:
    return;
}

static int bzMgmtSend(struct mgmt *aMgmt, uint16_t aOpcode, uint16_t aIndex, uint16_t aLength, const void *aParam)
{
    otLogDebgPlat("[Bluez] Mgmt send opcode %04x (%s)", aOpcode, mgmt_opstr(aOpcode));
    return mgmt_send(aMgmt, aOpcode, aIndex, aLength, aParam, bzMgmtDefaultSentCb, UINT_TO_PTR(aOpcode), NULL);
}

static int bzMgmtRegister(struct mgmt *aMgmt, uint16_t aEvent, uint16_t aIndex)
{
    return mgmt_register(aMgmt, aEvent, aIndex, bzMgmtDefaultNotifyCb, UINT_TO_PTR(aEvent), NULL);
}

static void bzGattClientRegisterNotifyCb(uint16_t aAttError, void *aUserData)
{
    OT_UNUSED_VARIABLE(aAttError);
    OT_UNUSED_VARIABLE(aUserData);

    otLogDebgPlat("Gatt client register notify: aAttError=%04x\n", aAttError);
}

static void bzGattClientNotifyCb(uint16_t aValueHandle, const uint8_t *aValue, uint16_t aLength, void *aUserData)
{
    struct BluezGattC2NotifyEvent event;

    TobleConnection *conn = reinterpret_cast<TobleConnection *>(aUserData);

    VerifyOrExit(conn->mC2Handle == aValueHandle, OT_NOOP);

    SuccessOrExit(event.Init(reinterpret_cast<otTobleConnection *>(conn), aValue, aLength));
    VerifyOrDie(write(sPipeFd[1], &event, event.GetSize()) > 0, OT_EXIT_ERROR_ERRNO);

exit:
    return;
}

static void bzGattDiscoverCb(bool aSuccess, uint8_t aAttError, struct bt_gatt_result *aResult, void *aUserData)
{
    OT_UNUSED_VARIABLE(aAttError);

    TobleConnection *   conn = reinterpret_cast<TobleConnection *>(aUserData);
    struct bt_gatt_iter iter;
    uint16_t            start_handle;
    uint16_t            end_handle;
    uint128_t           uuid128;
    bt_uuid_t           uuid;
    char                uuidString[MAX_LEN_UUID_STR];

    conn->gatt_request = NULL;

    otLogInfoPlat("[Bluez] Gatt Discocer: success=%d aAttError=%d", aSuccess, aAttError);
    VerifyOrExit(aSuccess, OT_NOOP);

    bt_gatt_iter_init(&iter, aResult);

    switch (conn->mState)
    {
    case kConnStateGattDiscoverBtpService:
    {
        while (bt_gatt_iter_next_service(&iter, &start_handle, &end_handle, uuid128.data))
        {
            bt_uuid128_create(&uuid, uuid128);
            bt_uuid_to_string(&uuid, uuidString, sizeof(uuidString));
            otLogInfoPlat("[Bluez] Gatt Service: uuid=%s start_handle=%d end_handle=%d", uuidString, start_handle,
                          end_handle);

            conn->mState = kConnStateGattDiscoverUuidC1C2Handle;
            conn->gatt_request =
                bt_gatt_discover_characteristics(conn->mAtt, start_handle, end_handle, bzGattDiscoverCb, conn, NULL);
        }

        break;
    }

    case kConnStateGattDiscoverUuidC1C2Handle:
    {
        bt_uuid_t c1uuid;
        bt_uuid_t c2uuid;
        uint16_t  value_handle;
        uint8_t   properties;

        bt_uuid128_create(&c1uuid, sBtpUuidC1);
        bt_uuid128_create(&c2uuid, sBtpUuidC2);

        while (bt_gatt_iter_next_characteristic(&iter, &start_handle, &end_handle, &value_handle, &properties,
                                                uuid128.data))
        {
            bt_uuid128_create(&uuid, uuid128);
            bt_uuid_to_string(&uuid, uuidString, sizeof(uuidString));

            otLogInfoPlat("[Bluez] Gatt Characteristic: uuid=%s start_handle=%d end_handle=%d value_handle=%d "
                          "properties=0x%02x",
                          uuidString, start_handle, end_handle, value_handle, properties);

            if (bt_uuid_cmp(&c1uuid, &uuid) == 0)
            {
                otLogInfoPlat("[Bluez] C1Handle=%d", value_handle);
                conn->mC1Handle = value_handle;
            }
            else if (bt_uuid_cmp(&c2uuid, &uuid) == 0)
            {
                otLogInfoPlat("[Bluez] C2Handle=%d", value_handle);

                conn->mC2Handle = value_handle;
                conn->mState    = kConnStateGattDiscoverUuidC2CccdHandle;
                conn->gatt_request =
                    bt_gatt_discover_descriptors(conn->mAtt, start_handle, end_handle, bzGattDiscoverCb, conn, NULL);
            }
        }

        break;
    }

    case kConnStateGattDiscoverUuidC2CccdHandle:
    {
        bt_uuid_t cccd_uuid;
        uint16_t  handle;

        bt_uuid16_create(&cccd_uuid, GATT_CLIENT_CHARAC_CFG_UUID);

        while (bt_gatt_iter_next_descriptor(&iter, &handle, uuid128.data))
        {
            bt_uuid128_create(&uuid, uuid128);
            bt_uuid_to_string(&uuid, uuidString, sizeof(uuidString));
            otLogInfoPlat("[Bluez] Gatt Descriptor: uuid=%s handle=%d", uuidString, handle);

            if (bt_uuid_cmp(&cccd_uuid, &uuid) == 0)
            {
                conn->mC2CccdHandle = handle;
                conn->mState        = kConnStateGattWaitServiceAdd;
                break;
            }
        }

        break;
    }

    default:
        break;
    }

exit:
    return;
}

static void bzMgmtDeviceConnectedEvent(uint16_t aIndex, uint16_t aLength, const void *aParam, void *aUserData)
{
    OT_UNUSED_VARIABLE(aIndex);
    OT_UNUSED_VARIABLE(aUserData);

    uint16_t                               eir_len;
    const struct mgmt_ev_device_connected *ev   = reinterpret_cast<const struct mgmt_ev_device_connected *>(aParam);
    TobleConnection *                      conn = sServerConn;
    otTobleAddress                         peerAddress;

    VerifyOrExit(aLength >= sizeof(*ev), OT_NOOP);

    eir_len = get_le16(&ev->eir_len);
    VerifyOrExit(aLength == (sizeof(*ev) + eir_len), OT_NOOP);

    memcpy(peerAddress.mAddress, &ev->addr.bdaddr, OT_TOBLE_ADDRESS_SIZE);
    peerAddress.mType = AddressTypeBluezToToble(ev->addr.type);

    otLogInfoPlat("[Bluez] Hci%d address %s connected eir_len %u", aIndex, AddressToString(&peerAddress), eir_len);

    conn = ConnectionFind(&peerAddress);
    if (conn == NULL)
    {
        conn = sServerConn;
        memcpy(conn->mPeerAddress.mAddress, peerAddress.mAddress, OT_TOBLE_ADDRESS_SIZE);
        conn->mPeerAddress.mType = peerAddress.mType;
    }
    else if (conn->mState == kConnStateConnecting)
    {
        int       fd;
        bt_uuid_t uuid;

        if ((fd = bzL2capLeAttConnect(&ev->addr.bdaddr, ev->addr.type, BT_SECURITY_LOW)) < 0)
        {
            otLogNotePlat("[Bluez] Create a L2CAP LE att connection failed");
            bzRemoveDevice(&conn->mPeerAddress);
            ConnectionFree(conn);
            ExitNow();
        }

        if (bzGattClientCreate(conn, fd, BLE_DEFAULT_ATT_MTU_SIZE) != OT_ERROR_NONE)
        {
            otLogNotePlat("[Bluez] Create a Gatt client failed");
            bzRemoveDevice(&conn->mPeerAddress);
            ConnectionFree(conn);
            ExitNow();
        }

        bt_uuid16_create(&uuid, OT_PLAT_TOBLE_UUID_THREAD_GROUP);

        conn->mState = kConnStateGattDiscoverBtpService;
        conn->gatt_request =
            bt_gatt_discover_primary_services(conn->mAtt, &uuid, 0x0001, 0xffff, bzGattDiscoverCb, conn, NULL);
    }
    else
    {
        otLogNotePlat("[Bluez] Unknown connection");
        bzRemoveDevice(&peerAddress);
        ConnectionFree(conn);

        ExitNow();
    }

    if (sDiagMode)
    {
        otPlatTobleDiagHandleConnected(sInstance, reinterpret_cast<otTobleConnection *>(conn));
    }
    else
    {
        otPlatTobleHandleConnected(sInstance, reinterpret_cast<otTobleConnection *>(conn));
    }

exit:
    return;
}

static void bzMgmtDeviceDisconnectedEvent(uint16_t aIndex, uint16_t aLength, const void *aParam, void *aUserData)
{
    OT_UNUSED_VARIABLE(aIndex);
    OT_UNUSED_VARIABLE(aUserData);

    const struct mgmt_ev_device_disconnected *ev = reinterpret_cast<const struct mgmt_ev_device_disconnected *>(aParam);
    otTobleAddress                            peerAddress;
    TobleConnection *                         conn;

    VerifyOrExit(aLength >= sizeof(struct mgmt_addr_info), OT_NOOP);

    memcpy(peerAddress.mAddress, ev->addr.bdaddr.b, OT_TOBLE_ADDRESS_SIZE);
    peerAddress.mType = AddressTypeBluezToToble(ev->addr.type);

    otLogInfoPlat("[Bluez] Hci%u address %s disconnected with reason %u", aIndex, AddressToString(&peerAddress),
                  ev->reason);

    if ((conn = ConnectionFind(&peerAddress)) == NULL)
    {
        otLogInfoPlat("[Bluez] Failed to find the BLE connection");
        ExitNow();
    }

    if (conn == sServerConn)
    {
        memset(&conn->mPeerAddress, 0, sizeof(otTobleAddress));
    }
    else
    {
        bzRemoveDevice(&conn->mPeerAddress);
    }

    if (sDiagMode)
    {
        otPlatTobleDiagHandleDisconnected(sInstance, reinterpret_cast<otTobleConnection *>(conn));
    }
    else
    {
        otPlatTobleHandleDisconnected(sInstance, reinterpret_cast<otTobleConnection *>(conn));
    }

    ConnectionFree(conn);

exit:
    return;
}

static void bzMgmtDeviceConnectFailedEvent(uint16_t aIndex, uint16_t aLength, const void *aParam, void *aUserData)
{
    OT_UNUSED_VARIABLE(aIndex);
    OT_UNUSED_VARIABLE(aUserData);

    const struct mgmt_ev_connect_failed *ev = reinterpret_cast<const struct mgmt_ev_connect_failed *>(aParam);
    otTobleAddress                       address;

    VerifyOrExit(aLength == sizeof(*ev), OT_NOOP);

    memcpy(address.mAddress, ev->addr.bdaddr.b, OT_TOBLE_ADDRESS_SIZE);
    address.mType = AddressTypeBluezToToble(ev->addr.type);

    otLogInfoPlat("[Bluez] Hci%u address %s connect failed (status 0x%02x, %s)", aIndex, AddressToString(&address),
                  ev->status, mgmt_errstr(ev->status));

    OT_UNUSED_VARIABLE(address);

exit:
    return;
}

#if BLE_DEBUG
static bool FindBtpAdvData(const uint8_t *aAdvData, uint8_t aAdvLength)
{
    uint8_t i;
    uint8_t length;
    uint8_t type;
    uint8_t uuid1;
    uint8_t uuid2;
    bool    found = false;

    for (i = 0; i < aAdvLength; i += (length + 1))
    {
        length = aAdvData[i];
        type   = aAdvData[i + 1];
        uuid1  = aAdvData[i + 2];
        uuid2  = aAdvData[i + 3];

        if ((type == 0x16) && (uuid1 == 0xFF) && ((uuid2 == 0xFB) || (uuid2 == 0xFC) || (uuid2 == 0xFD)))
        {
            found = true;
            break;
        }
    }

    return found;
}
#endif

static void bzMgmtDeviceFoundEvent(uint16_t aIndex, uint16_t aLength, const void *aParam, void *aUserData)
{
    OT_UNUSED_VARIABLE(aIndex);
    OT_UNUSED_VARIABLE(aLength);
    OT_UNUSED_VARIABLE(aUserData);

    otTobleAdvPacket                   advPacket;
    const struct mgmt_ev_device_found *ev = reinterpret_cast<const struct mgmt_ev_device_found *>(aParam);

    advPacket.mRssi   = ev->rssi;
    advPacket.mData   = (uint8_t *)ev->eir;
    advPacket.mLength = ev->eir_len;
    memcpy(advPacket.mSrcAddress.mAddress, ev->addr.bdaddr.b, OT_TOBLE_ADDRESS_SIZE);
    advPacket.mSrcAddress.mType = AddressTypeBluezToToble(ev->addr.type);

    otLogDebgPlat("[Bluez] Hci%u, Advertising Received: address=%s rssi=%d len=%d", aIndex,
                  AddressToString(&advPacket.mSrcAddress), ev->rssi, aLength);

#if BLE_DEBUG
    if (FindBtpAdvData(advPacket.mData, advPacket.mLength))
#endif
    {
        if (sDiagMode)
        {
            otPlatTobleDiagGapOnAdvReceived(sInstance, OT_TOBLE_ADV_IND, &advPacket);
        }
        else
        {
            otPlatTobleGapOnAdvReceived(sInstance, OT_TOBLE_ADV_IND, &advPacket);
        }
    }
}

static void bzMgmtDiscoveringEvent(uint16_t aIndex, uint16_t aLength, const void *aParam, void *aUserData)
{
    OT_UNUSED_VARIABLE(aIndex);
    OT_UNUSED_VARIABLE(aParam);
    OT_UNUSED_VARIABLE(aUserData);

    const struct mgmt_ev_discovering *ev = reinterpret_cast<const struct mgmt_ev_discovering *>(aParam);

    VerifyOrExit(aLength >= sizeof(*ev), OT_NOOP);

    otLogDebgPlat("[Bluez] Hci%u type %u discovering %s", aIndex, ev->type, ev->discovering ? "on" : "off");

    if (!ev->discovering && (sBleState == kBleStateScanning))
    {
        bzScanStart(sScanInterval, sScanWindow);
    }

exit:
    return;
}

static void bzMgmtCommandCompleteEvent(uint16_t aIndex, uint16_t aLength, const void *aParam, void *aUserData)
{
    OT_UNUSED_VARIABLE(aIndex);
    OT_UNUSED_VARIABLE(aLength);
    OT_UNUSED_VARIABLE(aUserData);
    const struct mgmt_ev_cmd_complete *evt = reinterpret_cast<const struct mgmt_ev_cmd_complete *>(aParam);

    OT_UNUSED_VARIABLE(evt);
    otLogDebgPlat("[Bluez] Hci%u Len=%u OpCode=%04x Status: %s", aIndex, aLength, evt->opcode,
                  mgmt_errstr(evt->status));
}

static void bzRemoveAllDevices(uint16_t aIndex)
{
    struct mgmt_cp_remove_device cp;

    memset(&cp, 0, sizeof(cp));
    cp.addr.type = BDADDR_BREDR;

    if (!bzMgmtSend(sMgmt, MGMT_OP_REMOVE_DEVICE, aIndex, sizeof(cp), &cp))
    {
        otLogWarnPlat("[Bluez] Failed to send remove device command");
    }
}

static void bzMgmtReadInfoCompleteCb(uint8_t aStatus, uint16_t aLength, const void *aParam, void *aUserData)
{
    OT_UNUSED_VARIABLE(aLength);

    const struct mgmt_rp_read_info *rp    = reinterpret_cast<const struct mgmt_rp_read_info *>(aParam);
    uint16_t                        index = PTR_TO_UINT(aUserData);
    uint32_t                        supportedSettings;
    uint8_t                         val = 0x00;

    if (aStatus)
    {
        otLogCritPlat("[Bluez] Reading info for index %u failed: %s\n", index, mgmt_errstr(aStatus));
        DieNow(OT_EXIT_FAILURE);
    }

    supportedSettings = le32_to_cpu(rp->supported_settings);

    if (!(supportedSettings & (MGMT_SETTING_LE | MGMT_SETTING_STATIC_ADDRESS)))
    {
        DieNowWithMessage("HCI doesn't support LE", OT_EXIT_FAILURE);
    }

    if (supportedSettings & MGMT_SETTING_POWERED)
    {
        bzMgmtSend(sMgmt, MGMT_OP_SET_POWERED, index, 1, &val);
    }

    if (supportedSettings & MGMT_SETTING_BREDR)
    {
        bzMgmtSend(sMgmt, MGMT_OP_SET_BREDR, index, 1, &val);
    }

    // In case the previous operation forgot to remove all devices.
    bzRemoveAllDevices(index);

    val = 0x01;
    bzMgmtSend(sMgmt, MGMT_OP_SET_LE, index, 1, &val);
    bzMgmtSend(sMgmt, MGMT_OP_SET_STATIC_ADDRESS, index, 6, sBleRandomAddress.mAddress);

    if (supportedSettings & MGMT_SETTING_CONNECTABLE)
    {
        bzMgmtSend(sMgmt, MGMT_OP_SET_CONNECTABLE, index, 1, &val);
    }

    if (supportedSettings & MGMT_SETTING_POWERED)
    {
        bzMgmtSend(sMgmt, MGMT_OP_SET_POWERED, index, 1, &val);
    }

    bzMgmtRegister(sMgmt, MGMT_EV_DEVICE_CONNECTED, index);
    bzMgmtRegister(sMgmt, MGMT_EV_DEVICE_DISCONNECTED, index);
    bzMgmtRegister(sMgmt, MGMT_EV_CONNECT_FAILED, index);
    bzMgmtRegister(sMgmt, MGMT_EV_CMD_COMPLETE, index);
    bzMgmtRegister(sMgmt, MGMT_EV_CMD_STATUS, index);
    bzMgmtRegister(sMgmt, MGMT_EV_DEVICE_FOUND, index);
    bzMgmtRegister(sMgmt, MGMT_EV_DISCOVERING, index);

    bzGattServerStart();
}

static void bzMgmtReadExtIndexListCompleteCb(uint8_t aStatus, uint16_t aLength, const void *aParam, void *aUserData)
{
    OT_UNUSED_VARIABLE(aLength);
    OT_UNUSED_VARIABLE(aUserData);

    const struct mgmt_rp_read_ext_index_list *rp = reinterpret_cast<const struct mgmt_rp_read_ext_index_list *>(aParam);
    uint16_t                                  count;
    uint16_t                                  index;
    int                                       i;

    if (aStatus)
    {
        otLogCritPlat("[Bluez] Reading extended index list failed: %s", mgmt_errstr(aStatus));
        DieNow(OT_EXIT_FAILURE);
    }

    count = le16_to_cpu(rp->num_controllers);

    for (i = 0; i < count; i++)
    {
        index = cpu_to_le16(rp->entry[i].index);

        if (rp->entry[i].type != 0x00)
            continue;

        if (index == sHciIndex)
        {
            break;
        }
    }

    if (i < count)
    {
        if (!bzMgmtSend(sMgmt, MGMT_OP_READ_INFO, index, 0, NULL))
        {
            otLogCritPlat("[Bluez] Failed to read BLE controller info");
            DieNow(OT_EXIT_FAILURE);
        }
    }
    else
    {
        otLogCritPlat("[Bluez] Hci%u is not found", sHciIndex);
        DieNow(OT_EXIT_FAILURE);
    }
}

static void bzMgmtReadIndexListCompleteCb(uint8_t aStatus, uint16_t aLength, const void *aParam, void *aUserData)
{
    OT_UNUSED_VARIABLE(aLength);
    OT_UNUSED_VARIABLE(aUserData);

    const struct mgmt_rp_read_index_list *rp = reinterpret_cast<const struct mgmt_rp_read_index_list *>(aParam);
    uint16_t                              count;
    uint16_t                              index;
    int                                   i;

    if (aStatus)
    {
        otLogCritPlat("[Bluez] Reading index list failed: %s", mgmt_errstr(aStatus));
        DieNow(OT_EXIT_FAILURE);
    }

    count = le16_to_cpu(rp->num_controllers);

    for (i = 0; i < count; i++)
    {
        index = cpu_to_le16(rp->index[i]);

        if (index == sHciIndex)
        {
            break;
        }
    }

    if (i < count)
    {
        if (!bzMgmtSend(sMgmt, MGMT_OP_READ_INFO, index, 0, NULL))
        {
            otLogCritPlat("[Bluez] Failed to read BLE controller info");
            DieNow(OT_EXIT_FAILURE);
        }
    }
    else
    {
        otLogCritPlat("[Bluez] Hci%u is not found", sHciIndex);
        DieNow(OT_EXIT_FAILURE);
    }
}

static void bzMgmtExtMgmtIndexRemovedEvent(uint16_t aIndex, uint16_t aLength, const void *aParam, void *aUserData)
{
    OT_UNUSED_VARIABLE(aLength);
    OT_UNUSED_VARIABLE(aUserData);

    const struct mgmt_ev_ext_index_added *ev = reinterpret_cast<const struct mgmt_ev_ext_index_added *>(aParam);

    VerifyOrExit((sHciIndex == aIndex) && (ev->type == 0x00), OT_NOOP);
    otLogCritPlat("[Bluez] Extended index %u is removed\n", aIndex);
    DieNow(OT_EXIT_FAILURE);

exit:
    return;
}

static void bzMgmtIndexRemovedEvent(uint16_t aIndex, uint16_t aLength, const void *aParam, void *aUserData)
{
    OT_UNUSED_VARIABLE(aLength);
    OT_UNUSED_VARIABLE(aParam);
    OT_UNUSED_VARIABLE(aUserData);

    VerifyOrExit(sHciIndex == aIndex, OT_NOOP);
    otLogCritPlat("[Bluez] Index %u is removed\n", aIndex);
    DieNow(OT_EXIT_FAILURE);

exit:
    return;
}

static void bzMgmtReadCommandsCompleteCb(uint8_t aStatus, uint16_t aLength, const void *aParam, void *aUserData)
{
    OT_UNUSED_VARIABLE(aLength);
    OT_UNUSED_VARIABLE(aUserData);

    const struct mgmt_rp_read_commands *rp = reinterpret_cast<const struct mgmt_rp_read_commands *>(aParam);
    uint16_t                            numCommands;
    bool                                extIndexList = false;
    int                                 i;

    if (aStatus)
    {
        otLogCritPlat("[Bluez] Reading mgmt supported commands failed: %s", mgmt_errstr(aStatus));
        DieNow(OT_EXIT_FAILURE);
    }

    numCommands = le16_to_cpu(rp->num_commands);

    for (i = 0; i < numCommands; i++)
    {
        uint16_t op = get_le16(rp->opcodes + 1);
        if (op == MGMT_OP_READ_EXT_INDEX_LIST)
        {
            extIndexList = true;
            break;
        }
    }

    if (extIndexList)
    {
        bzMgmtRegister(sMgmt, MGMT_EV_EXT_INDEX_REMOVED, MGMT_INDEX_NONE);

        if (!bzMgmtSend(sMgmt, MGMT_OP_READ_EXT_INDEX_LIST, MGMT_INDEX_NONE, 0, NULL))
        {
            otLogCritPlat("[Bluez] Failed to read extended index list");
            DieNow(OT_EXIT_FAILURE);
        }
    }
    else
    {
        bzMgmtRegister(sMgmt, MGMT_EV_INDEX_REMOVED, MGMT_INDEX_NONE);

        if (!bzMgmtSend(sMgmt, MGMT_OP_READ_INDEX_LIST, MGMT_INDEX_NONE, 0, NULL))
        {
            otLogCritPlat("[Bluez] Failed to read index list");
            DieNow(OT_EXIT_FAILURE);
        }
    }
}

void bzGapStart(void)
{
    sMgmt = mgmt_new_default();

    if (!sMgmt)
    {
        otLogCritPlat("[Bluez] Failed to open management socket");
        DieNow(OT_EXIT_FAILURE);
    }

    if (!bzMgmtSend(sMgmt, MGMT_OP_READ_COMMANDS, MGMT_INDEX_NONE, 0, NULL))
    {
        otLogCritPlat("[Bluez] Failed to management supported commands");
        DieNow(OT_EXIT_FAILURE);
    }
}

static void bzGapStop(void)
{
    VerifyOrExit(sMgmt != NULL, OT_NOOP);

    bzGattServerStop();
    mgmt_unref(sMgmt);
    sMgmt = NULL;

exit:
    return;
}

static void GenarateRandomBleAddress(void)
{
    int     fd;
    ssize_t len;

    VerifyOrDie((fd = open("/dev/urandom", O_RDONLY)) >= 0, OT_EXIT_ERROR_ERRNO);
    len = read(fd, sBleRandomAddress.mAddress, sizeof(sBleRandomAddress.mAddress));
    if (len < 0 || len != sizeof(sBleRandomAddress.mAddress))
    {
        DieNow(OT_EXIT_FAILURE);
    }

    // Set top most significant bits
    sBleRandomAddress.mAddress[5] |= 0xc0;
    sBleRandomAddress.mType = OT_TOBLE_ADDRESS_TYPE_RANDOM_STATIC;

    close(fd);

#if BLE_DEBUG
    memset(sBleRandomAddress.mAddress, 0, OT_TOBLE_ADDRESS_SIZE);
    sBleRandomAddress.mAddress[5] = 0xc0;
    sBleRandomAddress.mAddress[0] = 0x01;
#endif
}

void platformTobleInit(const otPlatformConfig *aPlatformConfig)
{
    VerifyOrDie(sscanf(aPlatformConfig->mRadioFile, "hci%u", &sHciIndex) > 0, OT_EXIT_ERROR_ERRNO);

    memset(&sConnTable[0], 0, sizeof(sConnTable));
    GenarateRandomBleAddress();

    VerifyOrDie((sServerConn = ConnectionNew(NULL)) != NULL, OT_EXIT_FAILURE);
    VerifyOrDie(pipe(sPipeFd) == 0, OT_EXIT_ERROR_ERRNO);

    bzResetHciControlChannel(sHciIndex);

    mainloop_init();

    VerifyOrDie((pthread_create(&sMainLoopThreadId, NULL, bzMainLoopThread, NULL)) == 0, OT_EXIT_ERROR_ERRNO);

    bzGapStart();
}

void platformTobleDeinit(void)
{
    if (sBleState == kBleStateAdvertising)
    {
        bzRemoveAdvertising(sHciIndex);
    }
    else if (sBleState == kBleStateScanning)
    {
        bzScanStop(sHciIndex);
    }

    for (TobleConnection *conn = &sConnTable[0]; conn < OT_ARRAY_END(sConnTable); conn++)
    {
        if (conn->mIsUsed)
        {
            bzGattConnDisconnect(conn);
            ConnectionFree(conn);
        }
    }

    bzRemoveAllDevices(sHciIndex);
    bzGapStop();
    mainloop_quit();

    if (sPipeFd[0] >= 0)
    {
        close(sPipeFd[0]);
        sPipeFd[0] = -1;
    }

    if (sPipeFd[1] >= 0)
    {
        close(sPipeFd[1]);
        sPipeFd[1] = -1;
    }
}

void platformTobleUpdateFdSet(fd_set *aReadFdSet, fd_set *aErrorFdSet, int *aMaxFd)
{
    int fd = sPipeFd[0];

    VerifyOrExit((fd >= 0) && (aReadFdSet != NULL), OT_NOOP);

    FD_SET(fd, aReadFdSet);

    if (aErrorFdSet != NULL)
    {
        FD_SET(fd, aErrorFdSet);
    }

    if (aMaxFd != NULL && *aMaxFd < fd)
    {
        *aMaxFd = fd;
    }

exit:
    return;
}

static void processEvent(struct BluezMgmtEventHeader *aHeader)
{
    switch (aHeader->mType)
    {
    case kBluezEventTypeMgmtSend:
    {
        BluezMgmtSendEvent *event = reinterpret_cast<BluezMgmtSendEvent *>(aHeader);

        otLogDebgPlat("[Bluez] Mgmt sent event: opcode=%04x (%s), status=%02x (%s), length=%u", event->mOpcode,
                      mgmt_opstr(event->mOpcode), event->mStatus, mgmt_errstr(event->mStatus), event->mLength);

        switch (event->mOpcode)
        {
        case MGMT_OP_READ_INFO:
            bzMgmtReadInfoCompleteCb(event->mStatus, event->GetParamLength(), event->mParam, NULL);
            break;
        case MGMT_OP_READ_EXT_INDEX_LIST:
            bzMgmtReadExtIndexListCompleteCb(event->mStatus, event->GetParamLength(), event->mParam, NULL);
            break;
        case MGMT_OP_READ_INDEX_LIST:
            bzMgmtReadIndexListCompleteCb(event->mStatus, event->GetParamLength(), event->mParam, NULL);
            break;
        case MGMT_OP_READ_COMMANDS:
            bzMgmtReadCommandsCompleteCb(event->mStatus, event->GetParamLength(), event->mParam, NULL);
            break;
        default:
            break;
        }
    }
    break;

    case kBluezEventTypeMgmtNotify:
    {
        struct BluezMgmtNotifyEvent *event = reinterpret_cast<struct BluezMgmtNotifyEvent *>(aHeader);

        otLogDebgPlat("[Bluez] Mgmt notify event: index=%d, event=%04x (%s), length=%d", event->mIndex, event->mEvent,
                      mgmt_evstr(event->mEvent), event->GetParamLength());

        switch (event->mEvent)
        {
        case MGMT_EV_DEVICE_CONNECTED:
            bzMgmtDeviceConnectedEvent(event->mIndex, event->GetParamLength(), event->mParam, NULL);
            break;
        case MGMT_EV_DEVICE_DISCONNECTED:
            bzMgmtDeviceDisconnectedEvent(event->mIndex, event->GetParamLength(), event->mParam, NULL);
            break;
        case MGMT_EV_CONNECT_FAILED:
            bzMgmtDeviceConnectFailedEvent(event->mIndex, event->GetParamLength(), event->mParam, NULL);
            break;
        case MGMT_EV_CMD_COMPLETE:
        case MGMT_EV_CMD_STATUS:
            bzMgmtCommandCompleteEvent(event->mIndex, event->GetParamLength(), event->mParam, NULL);
            break;
        case MGMT_EV_DEVICE_FOUND:
            bzMgmtDeviceFoundEvent(event->mIndex, event->GetParamLength(), event->mParam, NULL);
            break;
        case MGMT_EV_DISCOVERING:
            bzMgmtDiscoveringEvent(event->mIndex, event->GetParamLength(), event->mParam, NULL);
            break;
        case MGMT_EV_EXT_INDEX_REMOVED:
            bzMgmtExtMgmtIndexRemovedEvent(event->mIndex, event->GetParamLength(), event->mParam, NULL);
            break;
        case MGMT_EV_INDEX_REMOVED:
            bzMgmtIndexRemovedEvent(event->mIndex, event->GetParamLength(), event->mParam, NULL);
            break;
        default:
            break;
        }
    }
    break;

    case kBluezEventTypeGattDiscover:
    {
        struct BluezGattDiscoverEvent *event = reinterpret_cast<struct BluezGattDiscoverEvent *>(aHeader);

        if (sDiagMode)
        {
            otPlatTobleDiagHandleConnectionIsReady(sInstance, event->mConn, kConnectionLinkTypeGatt);
        }
        else
        {
            otPlatTobleHandleConnectionIsReady(sInstance, event->mConn, kConnectionLinkTypeGatt);
        }
    }
    break;

    case kBluezEventTypeGattC1Write:
    {
        struct BluezGattC1WriteEvent *event = reinterpret_cast<struct BluezGattC1WriteEvent *>(aHeader);

        if (sDiagMode)
        {
            otPlatTobleDiagHandleC1Write(sInstance, event->mConn, event->mData, event->GetDataLength());
        }
        else
        {
            otPlatTobleHandleC1Write(sInstance, event->mConn, event->mData, event->GetDataLength());
        }
    }
    break;

    case kBluezEventTypeGattC2Notify:
    {
        struct BluezGattC2NotifyEvent *event = reinterpret_cast<struct BluezGattC2NotifyEvent *>(aHeader);

        if (sDiagMode)
        {
            otPlatTobleDiagHandleC2Notification(sInstance, event->mConn, event->mData, event->GetDataLength());
        }
        else
        {
            otPlatTobleHandleC2Notification(sInstance, event->mConn, event->mData, event->GetDataLength());
        }
    }
    break;

    case kBluezEventTypeGattC2CccdWrite:
    {
        struct BluezGattC2CccdEvent *event = reinterpret_cast<struct BluezGattC2CccdEvent *>(aHeader);

        if (sDiagMode)
        {
            otPlatTobleDiagHandleC2Subscribed(sInstance, event->mConn, event->mSubscribed);
        }
        else
        {
            otPlatTobleHandleC2Subscribed(sInstance, event->mConn, event->mSubscribed);
        }
    }
    break;

    case kBluezEventTypeC1WriteDone:
    {
        struct BluezGattC1WriteDoneEvent *event = reinterpret_cast<struct BluezGattC1WriteDoneEvent *>(aHeader);

        if (sDiagMode)
        {
            otPlatTobleDiagHandleC1WriteDone(sInstance, event->mConn);
        }
        else
        {
            otPlatTobleHandleC1WriteDone(sInstance, event->mConn);
        }
    }
    break;

    case kBluezEventTypeC2NotificateDone:
    {
        struct BluezGattC2NotifyDoneEvent *event = reinterpret_cast<struct BluezGattC2NotifyDoneEvent *>(aHeader);

        if (sDiagMode)
        {
            otPlatTobleDiagHandleC2NotificateDone(sInstance, event->mConn);
        }
        else
        {
            otPlatTobleHandleC2NotificateDone(sInstance, event->mConn);
        }
    }
    break;

    default:
        otLogNotePlat("[Bluez] Unknown Event Type: 0x%02x", aHeader->mType);
        break;
    }
}

void platformTobleProcess(const fd_set *aReadFdSet, const fd_set *aErrorFdSet)
{
    int fd = sPipeFd[0];

    if (FD_ISSET(fd, aErrorFdSet))
    {
        DieNowWithMessage("sPipeFd[0]", OT_EXIT_FAILURE);
    }

    if (FD_ISSET(fd, aReadFdSet))
    {
        uint8_t buffer[1024];
        int     rval;

        rval = read(fd, buffer, sizeof(buffer));

        if (rval > 0)
        {
            BluezMgmtEventHeader *header = reinterpret_cast<BluezMgmtEventHeader *>(&buffer[0]);
            uint8_t *             end    = buffer + rval;

            while (static_cast<uint32_t>(end - reinterpret_cast<uint8_t *>(header)) >= sizeof(BluezMgmtEventHeader))
            {
                processEvent(header);
                header = header->Next();
            }
        }
        else if ((rval < 0) && (errno != EINTR) && (errno != EINTR))
        {
            DieNowWithMessage("sPipe[0] read", OT_EXIT_ERROR_ERRNO);
        }
    }
}

/*******************************************************************************
 * @section Bluetooth Low Energy GAP.
 *******************************************************************************/

void otPlatTobleInit(otInstance *aInstance)
{
    sInstance = aInstance;
}

otTobleConnectionLinkType otPlatTobleGetConnectionLinkType(const otTobleConnection *aConn)
{
    OT_UNUSED_VARIABLE(aConn);
    return kConnectionLinkTypeGatt;
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
static uint8_t CopyAdvData(uint8_t *aData, const uint8_t *aAdvData, uint8_t aAdvLength)
{
    uint8_t i;
    uint8_t length;
    uint8_t type;
    uint8_t offset = 0;

    VerifyOrExit((aAdvData != NULL) && (aAdvLength != 0), OT_NOOP);

    for (i = 0; i < aAdvLength; i += (length + 1))
    {
        length = aAdvData[i];
        type   = aAdvData[i + 1];

        // Bluez automatic prepend the FLAGS to the advertisement. If the advertisement data contains
        // the FLAGS, the mgmt command MGMT_OP_ADD_ADVERTISING returns an invalid parameters error.
        if (type == BLE_EIR_FLAGS)
        {
            continue;
        }

        memcpy(aData + offset, aAdvData + i, length + 1);
        offset += length + 1;
    }

exit:
    return offset;
}

static otError bzAddAdvertising(uint16_t aIndex, const otTobleAdvConfig *aConfig)
{
    otError                         error                         = OT_ERROR_NONE;
    uint8_t                         buf[BLE_MAX_ADV_DATA_LEN * 2] = {0};
    struct mgmt_cp_add_advertising *cp;

    VerifyOrExit(((aConfig != NULL) && (aConfig->mData != NULL)) && (sMgmt != NULL), error = OT_ERROR_FAILED);
    VerifyOrExit((aConfig->mLength + aConfig->mScanRspDataLength) <= sizeof(buf), error = OT_ERROR_INVALID_ARGS);

    cp               = reinterpret_cast<struct mgmt_cp_add_advertising *>(buf);
    cp->instance     = BLE_ADV_INSTANCE;
    cp->flags        = MGMT_ADV_FLAG_CONNECTABLE | MGMT_ADV_FLAG_DISCOV | MGMT_ADV_FLAG_MANAGED_FLAGS;
    cp->duration     = cpu_to_le16(1);
    cp->timeout      = cpu_to_le16(0);
    cp->adv_data_len = CopyAdvData(cp->data, aConfig->mData, aConfig->mLength);
    cp->scan_rsp_len = CopyAdvData(cp->data + cp->adv_data_len, aConfig->mScanRspData, aConfig->mScanRspDataLength);

    VerifyOrExit(
        bzMgmtSend(sMgmt, MGMT_OP_ADD_ADVERTISING, aIndex, sizeof(*cp) + cp->adv_data_len + cp->scan_rsp_len, buf) > 0,
        error = OT_ERROR_FAILED);

exit:
    return error;
}

static otError bzRemoveAdvertising(uint16_t aIndex)
{
    otError                           error = OT_ERROR_NONE;
    struct mgmt_cp_remove_advertising cp    = {.instance = BLE_ADV_INSTANCE};

    VerifyOrExit(sMgmt != NULL, error = OT_ERROR_FAILED);
    VerifyOrExit(bzMgmtSend(sMgmt, MGMT_OP_REMOVE_ADVERTISING, aIndex, sizeof(cp), &cp) > 0, error = OT_ERROR_FAILED);
exit:
    return error;
}

otError otPlatTobleAdvStart(otInstance *aInstance, const otTobleAdvConfig *aConfig)
{
    OT_UNUSED_VARIABLE(aInstance);
    otError error = OT_ERROR_NONE;

    if (sBleState == kBleStateScanning)
    {
        bzRemoveAdvertising(sHciIndex);
        sBleState = kBleStateIdle;
    }

    SuccessOrExit(error = bzAddAdvertising(sHciIndex, aConfig));
    sBleState = kBleStateAdvertising;

exit:
    return error;
}

otError otPlatTobleAdvStop(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    otError error = OT_ERROR_NONE;

    SuccessOrExit(error = bzRemoveAdvertising(sHciIndex));
    sBleState = kBleStateIdle;

exit:
    return error;
}

static otError bzScanStart(uint16_t aInterval, uint16_t aWindow)
{
    otError                        error = OT_ERROR_NONE;
    struct mgmt_cp_set_scan_params param = {.interval = static_cast<uint16_t>(BLE_MS_TO_625US_TICK(aInterval)),
                                            .window   = static_cast<uint16_t>(BLE_MS_TO_625US_TICK(aWindow))};
    struct mgmt_cp_start_discovery disc  = {.type = ((1 << BDADDR_LE_PUBLIC) | (1 << BDADDR_LE_RANDOM))};

    VerifyOrExit(sMgmt != NULL, error = OT_ERROR_FAILED);
    VerifyOrExit(bzMgmtSend(sMgmt, MGMT_OP_SET_SCAN_PARAMS, sHciIndex, sizeof(param), &param) > 0,
                 error = OT_ERROR_FAILED);
    VerifyOrExit(bzMgmtSend(sMgmt, MGMT_OP_START_DISCOVERY, sHciIndex, sizeof(disc), &disc) > 0,
                 error = OT_ERROR_FAILED);

exit:
    return error;
}

static otError bzScanStop(uint16_t aIndex)
{
    otError                       error = OT_ERROR_NONE;
    struct mgmt_cp_stop_discovery cp    = {.type = ((1 << BDADDR_LE_PUBLIC) | (1 << BDADDR_LE_RANDOM))};

    VerifyOrExit(sMgmt != NULL, error = OT_ERROR_FAILED);

    VerifyOrExit(bzMgmtSend(sMgmt, MGMT_OP_STOP_DISCOVERY, aIndex, sizeof(cp), &cp) > 0, error = OT_ERROR_FAILED);

exit:
    return error;
}

otError otPlatTobleScanStart(otInstance *aInstance, uint16_t aInterval, uint16_t aWindow, bool aActive)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aActive);

    otError error = OT_ERROR_NONE;
    if (sBleState == kBleStateAdvertising)
    {
        bzRemoveAdvertising(sHciIndex);
        sBleState = kBleStateIdle;
    }

    SuccessOrExit(error = bzScanStart(aInterval, aWindow));

    sScanInterval = aInterval;
    sScanWindow   = aWindow;
    sBleState     = kBleStateScanning;

exit:
    return error;
}

otError otPlatTobleScanStop(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    otError error = OT_ERROR_NONE;
    SuccessOrExit(error = bzScanStop(sHciIndex));
    sBleState = kBleStateIdle;

exit:
    return error;
}

static otError bzAddDevice(const otTobleAddress *aPeerAddress, otTobleConnectionConfig *aConfig)
{
    otError                         error = OT_ERROR_NONE;
    struct mgmt_cp_add_device       cp;
    uint8_t                         buffer[sizeof(struct mgmt_cp_load_conn_param) + sizeof(struct mgmt_conn_param)];
    struct mgmt_cp_load_conn_param *cp_load = reinterpret_cast<struct mgmt_cp_load_conn_param *>(buffer);
    struct mgmt_conn_param *        param   = &cp_load->params[0];

    cp_load->param_count = 1;

    memcpy(&param->addr.bdaddr, &sBleRandomAddress.mAddress, OT_TOBLE_ADDRESS_SIZE);
    param->addr.type = AddressTypeTobleToBluez(sBleRandomAddress.mType);

    param->min_interval = htobs(BLE_MS_TO_1_25MS_TICK(aConfig->mInterval));
    param->max_interval = htobs(BLE_MS_TO_1_25MS_TICK(aConfig->mInterval));
    param->latency      = htobs(0);
    param->timeout      = htobs(BLE_CONN_MAX_SUPERVISION_TIMEOUT);

    if (!bzMgmtSend(sMgmt, MGMT_OP_LOAD_CONN_PARAM, sHciIndex, sizeof(buffer), buffer))
    {
        otLogNotePlat("[Bluez] Unable to send load connection parameters command");
        ExitNow(error = OT_ERROR_FAILED);
    }

    memcpy(&cp.addr.bdaddr, aPeerAddress->mAddress, OT_TOBLE_ADDRESS_SIZE);
    cp.addr.type = AddressTypeTobleToBluez(aPeerAddress->mType);
    cp.action    = 2; // 0 Background scan for device, 1 Allow incoming connection, 2 Auto-connect remote device

    if (bzMgmtSend(sMgmt, MGMT_OP_ADD_DEVICE, sHciIndex, sizeof(cp), &cp) == 0)
    {
        otLogNotePlat("[Bluez] Unable to send add device command");
        ExitNow(error = OT_ERROR_FAILED);
    }

exit:
    return error;
}

static void bzRemoveDevice(const otTobleAddress *aPeerAddress)
{
    struct mgmt_cp_remove_device cp;

    memset(&cp, 0, sizeof(cp));
    memcpy(&cp.addr.bdaddr, aPeerAddress->mAddress, OT_TOBLE_ADDRESS_SIZE);
    cp.addr.type = AddressTypeTobleToBluez(aPeerAddress->mType);

    if (!bzMgmtSend(sMgmt, MGMT_OP_REMOVE_DEVICE, sHciIndex, sizeof(cp), &cp))
    {
        otLogInfoPlat("[Bluez] Unable to send remove device command");
    }
}

otTobleConnection *otPlatTobleCreateConnection(otInstance *             aInstance,
                                               const otTobleAddress *   aPeerAddress,
                                               otTobleConnectionConfig *aConfig)
{
    OT_UNUSED_VARIABLE(aInstance);

    TobleConnection *conn = NULL;

    VerifyOrExit((conn = ConnectionNew(aPeerAddress)) != NULL, OT_NOOP);

    if (bzAddDevice(aPeerAddress, aConfig) != OT_ERROR_NONE)
    {
        ConnectionFree(conn);
        conn = NULL;
    }
    else
    {
        conn->mState = kConnStateConnecting;
    }

exit:
    return conn;
}

void otPlatTobleDisconnect(otInstance *aInstance, otTobleConnection *aConn)
{
    OT_UNUSED_VARIABLE(aInstance);
    bzGattConnDisconnect(reinterpret_cast<TobleConnection *>(aConn));
}

uint16_t otPlatTobleGetMtu(otInstance *aInstance, otTobleConnection *aConn)
{
    OT_UNUSED_VARIABLE(aInstance);

    uint16_t         mtu  = 0;
    TobleConnection *conn = reinterpret_cast<TobleConnection *>(aConn);

    VerifyOrExit((conn != NULL) && (conn->mAtt != NULL), OT_NOOP);

    mtu = bt_att_get_mtu(conn->mAtt);

exit:
    return mtu;
}

void otPlatTobleC2Subscribe(otInstance *aInstance, otTobleConnection *aConn, bool aSubscribe)
{
    OT_UNUSED_VARIABLE(aInstance);

    TobleConnection *conn = reinterpret_cast<TobleConnection *>(aConn);

    VerifyOrExit((conn != NULL) && (conn->mGattClient != NULL), OT_NOOP);

    if (aSubscribe)
    {
        conn->mNotifyId = bt_gatt_client_register_notify(
            conn->mGattClient, conn->mC2Handle, bzGattClientRegisterNotifyCb, bzGattClientNotifyCb, conn, NULL);
        if (conn->mNotifyId == 0)
        {
            otLogCritPlat("[Bluez] Subscribe C2 failed");
        }
    }
    else
    {
        if (conn->mNotifyId != 0)
        {
            bt_gatt_client_unregister_notify(conn->mGattClient, conn->mNotifyId);
            conn->mNotifyId = 0;
        }
    }

exit:
    return;
}

otError otPlatTobleC1Write(otInstance *aInstance, otTobleConnection *aConn, const void *aBuffer, uint16_t aLength)
{
    OT_UNUSED_VARIABLE(aInstance);

    otError                          error = OT_ERROR_NONE;
    TobleConnection *                conn  = reinterpret_cast<TobleConnection *>(aConn);
    struct BluezGattC1WriteDoneEvent event;

    VerifyOrExit((conn != NULL) && (conn->mGattClient != NULL), error = OT_ERROR_FAILED);

    VerifyOrExit(bt_gatt_client_write_without_response(conn->mGattClient, conn->mC1Handle, false,
                                                       reinterpret_cast<const uint8_t *>(aBuffer), aLength) > 0,
                 error = OT_ERROR_FAILED);

    event.Init(aConn);
    VerifyOrDie(write(sPipeFd[1], &event, event.GetSize()) > 0, OT_EXIT_FAILURE);

exit:
    return error;
}

/*******************************************************************************
 * @section Bluetooth Low Energy GATT Server.
 *******************************************************************************/

otError otPlatTobleC2Notificate(otInstance *aInstance, otTobleConnection *aConn, const void *aBuffer, uint16_t aLength)
{
    OT_UNUSED_VARIABLE(aInstance);

    otError                           error = OT_ERROR_NONE;
    TobleConnection *                 conn  = reinterpret_cast<TobleConnection *>(aConn);
    struct BluezGattC2NotifyDoneEvent event;

    VerifyOrExit((conn != NULL) && (conn->mGattServer != NULL), error = OT_ERROR_FAILED);
    VerifyOrExit(sServerNotifyEnabled, error = OT_ERROR_INVALID_STATE);

    VerifyOrExit(bt_gatt_server_send_notification(conn->mGattServer, conn->mC2Handle,
                                                   reinterpret_cast<const uint8_t *>(aBuffer), aLength, false),
                 error = OT_ERROR_FAILED);

    event.Init(aConn);
    VerifyOrDie(write(sPipeFd[1], &event, event.GetSize()) > 0, OT_EXIT_FAILURE);

exit:
    return error;
}

/****************************************************************************
 * @section Bluetooth Low Energy L2CAP Connection Oriented Channels.
 ***************************************************************************/

uint8_t otPlatTobleGetL2capPsm(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return 0;
}

otError otPlatTobleL2capSend(otInstance *aInstance, otTobleConnection *aConn, const uint8_t *aBuffer, uint16_t aLength)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConn);
    OT_UNUSED_VARIABLE(aBuffer);
    OT_UNUSED_VARIABLE(aLength);
    return OT_ERROR_NONE;
}

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
    OT_UNUSED_VARIABLE(aAdvType);
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

#endif // #if OPENTHREAD_CONFIG_TOBLE_ENABLE && OPENTHREAD_CONFIG_BLE_HOST_BLUEZ_ENABLE

#ifdef __cplusplus
} // extern "C"
#endif
