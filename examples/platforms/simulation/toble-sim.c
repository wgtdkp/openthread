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

#include "platform-simulation.h"

#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/toble.h>

#include "utils/code_utils.h"

#if OPENTHREAD_CONFIG_TOBLE_ENABLE

// Change DEBUG_LOG to all extra logging
#define DEBUG_LOG 0

// The IPv4 group for receiving
#define TOBLE_SIM_GROUP "224.0.0.116"

#define MS_PER_S 1000
#define NS_PER_US 1000
#define US_PER_MS 1000
#define US_PER_S 1000000

#define TOBLE_SIM_PORT 9200

enum
{
    TOBLE_SIM_MAX_CONNECTIONS = 10,

    TOBLE_CMD_NONE         = 0, // Used for pending command only
    TOBLE_CMD_ADV          = 1,
    TOBLE_CMD_CONNECT      = 2,
    TOBLE_CMD_DISCONNECT   = 3,
    TOBLE_CMD_WRITE_C1     = 4,
    TOBLE_CMD_SUBSCRIBE_C2 = 5,
    TOBLE_CMD_INDICATE_C2  = 6,

    TOBLE_SIM_RSSI = -70,
    TOBLE_SIM_MTU  = 40,

    TOBLE_MAX_CMD_SIZE = 4096,
};

typedef struct TobleConnection
{
    uint8_t mCentral;
    uint8_t mPeripheral;
    bool    mIsInUse;
} TobleConnection;

// For adv timings (during simulation)
static uint32_t sSpeedUpFactor = 1;

static uint8_t         sTobleNodeId   = 0;
static bool            sTobleScanning = false; // only on central
static TobleConnection sTobleConnList[TOBLE_SIM_MAX_CONNECTIONS];

// Pending command to be sent
static uint8_t          sToblePendingCmd;
static TobleConnection *sTobleConn;
static uint8_t          sTobleCmdFrame[TOBLE_MAX_CMD_SIZE];
static uint16_t         sTobleCmdLength;

static bool     sTobleAdvertising = false; // only on peripheral.
static uint8_t  sTobleAdvCmdFrame[TOBLE_MAX_CMD_SIZE];
static uint16_t sTobleAdvCmdLength;
static uint16_t sTobleAdvInterval;
static uint32_t sTobleAdvLastTxTime;

static int      sTxFd       = -1;
static int      sRxFd       = -1;
static uint16_t sPortOffset = 0;

#if DEBUG_LOG
static void dumpBuffer(const uint8_t *aBuffer, uint16_t aLength)
{
    fprintf(stderr, "[ (len:%d) ", aLength);

    while (aLength--)
    {
        fprintf(stderr, "%02x ", *aBuffer++);
    }

    fprintf(stderr, "]");
}
#endif

static inline otTobleConnectionId GetConnId(TobleConnection *aConn)
{
    return (otTobleConnectionId)(aConn - sTobleConnList);
}

static void initFds(void)
{
    int                fd;
    int                one = 1;
    struct sockaddr_in sockaddr;
    struct ip_mreqn    mreq;

    memset(&sockaddr, 0, sizeof(sockaddr));

    otEXPECT_ACTION((fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) != -1, perror("socket(sTxFd)"));

    sockaddr.sin_family      = AF_INET;
    sockaddr.sin_port        = htons((uint16_t)(TOBLE_SIM_PORT + sPortOffset + gNodeId));
    sockaddr.sin_addr.s_addr = inet_addr("127.0.0.1");

    otEXPECT_ACTION(setsockopt(fd, IPPROTO_IP, IP_MULTICAST_IF, &sockaddr.sin_addr, sizeof(sockaddr.sin_addr)) != -1,
                    perror("setsockopt(sTxFd, IP_MULTICAST_IF)"));

    otEXPECT_ACTION(setsockopt(fd, IPPROTO_IP, IP_MULTICAST_LOOP, &one, sizeof(one)) != -1,
                    perror("setsockopt(sTxFd, IP_MULTICAST_LOOP)"));

    otEXPECT_ACTION(bind(fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != -1, perror("bind(sTxFd)"));

    // Tx fd is successfully initialized.
    sTxFd = fd;

    otEXPECT_ACTION((fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) != -1, perror("socket(sRxFd)"));

    otEXPECT_ACTION(setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) != -1,
                    perror("setsockopt(sRxFd, SO_REUSEADDR)"));
    otEXPECT_ACTION(setsockopt(fd, SOL_SOCKET, SO_REUSEPORT, &one, sizeof(one)) != -1,
                    perror("setsockopt(sRxFd, SO_REUSEPORT)"));

    memset(&mreq, 0, sizeof(mreq));
    inet_pton(AF_INET, TOBLE_SIM_GROUP, &mreq.imr_multiaddr);

    // Always use loopback device to send simulation packets.
    mreq.imr_address.s_addr = inet_addr("127.0.0.1");

    otEXPECT_ACTION(setsockopt(fd, IPPROTO_IP, IP_MULTICAST_IF, &mreq.imr_address, sizeof(mreq.imr_address)) != -1,
                    perror("setsockopt(sRxFd, IP_MULTICAST_IF)"));
    otEXPECT_ACTION(setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) != -1,
                    perror("setsockopt(sRxFd, IP_ADD_MEMBERSHIP)"));

    sockaddr.sin_family      = AF_INET;
    sockaddr.sin_port        = htons((uint16_t)(TOBLE_SIM_PORT + sPortOffset + WELLKNOWN_NODE_ID));
    sockaddr.sin_addr.s_addr = inet_addr(TOBLE_SIM_GROUP);

    otEXPECT_ACTION(bind(fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != -1, perror("bind(sRxFd)"));

    // Rx fd is successfully initialized.
    sRxFd = fd;

exit:
    if (sRxFd == -1 || sTxFd == -1)
    {
        exit(EXIT_FAILURE);
    }
}

static void deinitFds(void)
{
    if ((sRxFd != -1) && (sTxFd != -1))
    {
        // Send a disconnect to all existing connection
        for (TobleConnection *conn = &sTobleConnList[0]; conn < &sTobleConnList[TOBLE_SIM_MAX_CONNECTIONS]; conn++)
        {
            // `otPlatTobleDisconnect()` would check if connection is in-use.
            otPlatTobleDisconnect(NULL, GetConnId(conn));
        }
    }

    if (sRxFd != -1)
    {
        close(sRxFd);
    }

    if (sTxFd != -1)
    {
        close(sTxFd);
    }
}

static TobleConnection *findConnection(uint8_t aCentral, uint8_t aPeripheral)
{
    TobleConnection *conn;

    for (conn = &sTobleConnList[0]; conn < &sTobleConnList[TOBLE_SIM_MAX_CONNECTIONS]; conn++)
    {
        otEXPECT(!conn->mIsInUse || (conn->mCentral != aCentral) || (conn->mPeripheral != aPeripheral));
    }

    conn = NULL;

exit:
    return conn;
}

static TobleConnection *getNewConnection(void)
{
    TobleConnection *conn;

    for (conn = &sTobleConnList[0]; conn < &sTobleConnList[TOBLE_SIM_MAX_CONNECTIONS]; conn++)
    {
        otEXPECT(conn->mIsInUse);
    }

    conn = NULL;

exit:
    return conn;
}

#if DEBUG_LOG
static const char *cmdToString(uint8_t aCommand)
{
    const char *str = "Unknown";

    switch (aCommand)
    {
    case TOBLE_CMD_NONE:
        str = "None";
        break;
    case TOBLE_CMD_ADV:
        str = "Adv";
        break;
    case TOBLE_CMD_CONNECT:
        str = "Connect";
        break;
    case TOBLE_CMD_DISCONNECT:
        str = "Disconnect";
        break;
    case TOBLE_CMD_WRITE_C1:
        str = "Write_c1";
        break;
    case TOBLE_CMD_SUBSCRIBE_C2:
        str = "Subscribe_c2";
        break;
    case TOBLE_CMD_INDICATE_C2:
        str = "Indicate_c2";
        break;
    default:
        str = "Unknown";
        break;
    }

    return str;
}
#endif

void sendCommand(const uint8_t *aCommandFrame, uint16_t aLength)
{
    ssize_t            rval;
    struct sockaddr_in sockaddr;

    memset(&sockaddr, 0, sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    inet_pton(AF_INET, TOBLE_SIM_GROUP, &sockaddr.sin_addr);

    sockaddr.sin_port = htons((uint16_t)(TOBLE_SIM_PORT + sPortOffset + WELLKNOWN_NODE_ID));

#if DEBUG_LOG
    fprintf(stderr, "\n\r--->sendCommand(%s)", cmdToString(*aCommandFrame));
    dumpBuffer(aCommandFrame, aLength);
#endif

    rval = sendto(sTxFd, aCommandFrame, aLength, 0, (struct sockaddr *)&sockaddr, sizeof(sockaddr));

    if (rval < 0)
    {
        perror("sendto(sTxFd)");
        exit(EXIT_FAILURE);
    }
}

static void populateAddress(otTobleAddress *aAddress, uint8_t aNodeId)
{
    aAddress->mType = OT_TOBLE_ADDRESS_TYPE_RANDOM_STATIC;
    memset(aAddress->mAddress, 0, sizeof(aAddress->mAddress));
    aAddress->mAddress[0] = aNodeId;
}

static void handleCmdAdv(otInstance *aInstance, const uint8_t *aCommandFrame, uint16_t aLength)
{
    otTobleAddress srcAddr;

    // Only handle adv when scanning
    otEXPECT(sTobleScanning);

    otEXPECT(aLength >= 3); // srcNodeId, advType, data

    populateAddress(&srcAddr, aCommandFrame[0]);
    otPlatTobleHandleAdv(aInstance, (otTobleAdvType)aCommandFrame[1], &srcAddr, &aCommandFrame[2], aLength - 2,
                         TOBLE_SIM_RSSI);

exit:
    return;
}

static void handleCmdConnect(otInstance *aInstance, const uint8_t *aCommandFrame, uint16_t aLength)
{
    TobleConnection *conn;
    uint8_t          centralId;
    uint8_t          peripheralId;

    // TOBLE_CMD_CONNECT comes from a central to a peripheral

    otEXPECT(aLength >= 2);

    centralId    = aCommandFrame[0];
    peripheralId = aCommandFrame[1];

    otEXPECT(peripheralId == sTobleNodeId);

    conn = findConnection(centralId, peripheralId);
    otEXPECT(conn == NULL);

    conn = getNewConnection();

    if (conn != NULL)
    {
        conn->mIsInUse    = true;
        conn->mCentral    = centralId;
        conn->mPeripheral = peripheralId;

        otPlatTobleHandleConnected(aInstance, GetConnId(conn));
    }
    else
    {
        // TODO:(optional) Inform the central that could not connect
        // send disconnect command
    }

exit:
    return;
}

static void handleCmdDisconnect(otInstance *aInstance, const uint8_t *aCommandFrame, uint16_t aLength)
{
    TobleConnection *conn;
    uint8_t          centralId;
    uint8_t          peripheralId;

    // TOBLE_CMD_DISCONNECT can come from either a central or a peripheral.

    otEXPECT(aLength >= 2);

    centralId    = aCommandFrame[0];
    peripheralId = aCommandFrame[1];

    conn = findConnection(centralId, peripheralId);
    otEXPECT(conn != NULL);

    otPlatTobleHandleDisconnected(aInstance, GetConnId(conn));
    conn->mIsInUse = false;

exit:
    return;
}

static void handleCmdWriteC1(otInstance *aInstance, const uint8_t *aCommandFrame, uint16_t aLength)
{
    TobleConnection *conn;
    uint8_t          centralId;
    uint8_t          peripheralId;

    // TOBLE_CMD_WRITE_C1 comes from a central to a peripheral.

    otEXPECT(aLength >= 2);

    centralId    = aCommandFrame[0];
    peripheralId = aCommandFrame[1];

    otEXPECT(peripheralId == sTobleNodeId);

    conn = findConnection(centralId, peripheralId);
    otEXPECT(conn != NULL);

    aLength -= 2;

    otPlatTobleHandleC1Write(aInstance, GetConnId(conn), &aCommandFrame[2], aLength);

exit:
    return;
}

static void handleCmdSubscribeC2(otInstance *aInstance, const uint8_t *aCommandFrame, uint16_t aLength)
{
    TobleConnection *conn;
    uint8_t          centralId;
    uint8_t          peripheralId;
    bool             subscribed;

    // TOBLE_CMD_SUBSCRIBE_C2 comes from a central to a peripheral.

    otEXPECT(aLength >= 3);

    centralId    = aCommandFrame[0];
    peripheralId = aCommandFrame[1];
    subscribed   = (aCommandFrame[2] != 0);

    otEXPECT(peripheralId == sTobleNodeId);

    conn = findConnection(centralId, peripheralId);
    otEXPECT(conn != NULL);

    otPlatTobleHandleC2Subscribed(aInstance, GetConnId(conn), subscribed);

exit:
    return;
}

static void handleCmdIndicateC2(otInstance *aInstance, const uint8_t *aCommandFrame, uint16_t aLength)
{
    TobleConnection *conn;
    uint8_t          centralId;
    uint8_t          peripheralId;

    // TOBLE_CMD_INDICATE_C2 comes from a peripheral to a central

    otEXPECT(aLength >= 2);

    centralId    = aCommandFrame[0];
    peripheralId = aCommandFrame[1];

    otEXPECT(centralId == sTobleNodeId);

    conn = findConnection(centralId, peripheralId);
    otEXPECT(conn != NULL);

    aLength -= 2;

    otPlatTobleHandleC2Indication(aInstance, GetConnId(conn), &aCommandFrame[2], aLength);

exit:
    return;
}

static void handleCommand(otInstance *aInstance, const uint8_t *aCommandFrame, uint16_t aLength)
{
    uint8_t command;

    otEXPECT(aLength > 0);

    command = *aCommandFrame++;
    aLength--;

    switch (command)
    {
    case TOBLE_CMD_ADV:
        handleCmdAdv(aInstance, aCommandFrame, aLength);
        break;

    case TOBLE_CMD_CONNECT:
        handleCmdConnect(aInstance, aCommandFrame, aLength);
        break;

    case TOBLE_CMD_DISCONNECT:
        handleCmdDisconnect(aInstance, aCommandFrame, aLength);
        break;

    case TOBLE_CMD_WRITE_C1:
        handleCmdWriteC1(aInstance, aCommandFrame, aLength);
        break;

    case TOBLE_CMD_SUBSCRIBE_C2:
        handleCmdSubscribeC2(aInstance, aCommandFrame, aLength);
        break;

    case TOBLE_CMD_INDICATE_C2:
        handleCmdIndicateC2(aInstance, aCommandFrame, aLength);
        break;

    default:
        break;
    }

exit:
    return;
}

//---------------------------------------------------------------------------------------------------------------------
// otPlatToble

void otPlatTobleInit(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    memset(sTobleConnList, 0, sizeof(sTobleConnList));
    sTobleScanning    = false;
    sTobleAdvertising = false;
    sToblePendingCmd  = TOBLE_CMD_NONE;
}

void otPlatTobleDisconnect(otInstance *aInstance, otTobleConnectionId aConnId)
{
    uint8_t cmdBuffer[3];

    assert(aConnId < TOBLE_SIM_MAX_CONNECTIONS);
    otEXPECT(sTobleConnList[aConnId].mIsInUse);

    cmdBuffer[0] = TOBLE_CMD_DISCONNECT;
    cmdBuffer[1] = sTobleConnList[aConnId].mCentral;
    cmdBuffer[2] = sTobleConnList[aConnId].mPeripheral;

    sendCommand(cmdBuffer, 3);

exit:
    OT_UNUSED_VARIABLE(aInstance);
}

bool otPlatTobleIsConnected(otInstance *aInstance, otTobleConnectionId aConnId)
{
    OT_UNUSED_VARIABLE(aInstance);
    assert(aConnId < TOBLE_SIM_MAX_CONNECTIONS);

    return sTobleConnList[aConnId].mIsInUse;
}

void otPlatTobleGetPeerAddress(otInstance *aInstance, otTobleConnectionId aConnId, otTobleAddress *aPeerAddress)
{
    otEXPECT(sTobleConnList[aConnId].mIsInUse);

    if (sTobleConnList[aConnId].mCentral == sTobleNodeId)
    {
        populateAddress(aPeerAddress, sTobleConnList[aConnId].mPeripheral);
    }
    else
    {
        populateAddress(aPeerAddress, sTobleConnList[aConnId].mCentral);
    }

exit:
    OT_UNUSED_VARIABLE(aInstance);
}

uint16_t otPlatTobleGetMtu(otInstance *aInstance, otTobleConnectionId aConnId)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConnId);

    return TOBLE_SIM_MTU;
}

// - - - - - - - - - - -
// Central only

otError otPlatTobleScanStart(otInstance *aInstance, uint16_t aInterval, uint16_t aWindow)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aInterval);
    OT_UNUSED_VARIABLE(aWindow);

    sTobleScanning = true;

    return OT_ERROR_NONE;
}

otError otPlatTobleScanStop(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    sTobleScanning = false;

    return OT_ERROR_NONE;
}

otTobleConnectionId otPlatTobleCreateConnection(otInstance *             aInstance,
                                                const otTobleAddress *   aPeerAddress,
                                                otTobleConnectionConfig *aConfig)
{
    otEXPECT(sToblePendingCmd == TOBLE_CMD_NONE);

    sTobleConn = getNewConnection();
    otEXPECT(sTobleConn != NULL);

    sTobleConn->mIsInUse    = true;
    sTobleConn->mCentral    = sTobleNodeId;
    sTobleConn->mPeripheral = aPeerAddress->mAddress[0];

    // Prepare the command
    sTobleCmdFrame[0] = TOBLE_CMD_CONNECT;
    sTobleCmdFrame[1] = sTobleConn->mCentral;
    sTobleCmdFrame[2] = sTobleConn->mPeripheral;
    sTobleCmdLength   = 3;

    sToblePendingCmd = TOBLE_CMD_CONNECT;

    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConfig);

exit:
    return GetConnId(sTobleConn);
}

void otPlatTobleC1Write(otInstance *aInstance, otTobleConnectionId aConnId, const void *aBuf, uint16_t aLength)
{
    otEXPECT(sToblePendingCmd == TOBLE_CMD_NONE);
    otEXPECT(sTobleConnList[aConnId].mIsInUse);

    sTobleConn = &sTobleConnList[aConnId];

    sTobleCmdFrame[0] = TOBLE_CMD_WRITE_C1;
    sTobleCmdFrame[1] = sTobleConn->mCentral;
    sTobleCmdFrame[2] = sTobleConn->mPeripheral;
    assert(aLength < TOBLE_MAX_CMD_SIZE - 3);
    memcpy(&sTobleCmdFrame[3], aBuf, aLength);

    sTobleCmdLength = 3 + aLength;

    sToblePendingCmd = TOBLE_CMD_WRITE_C1;

exit:
    OT_UNUSED_VARIABLE(aInstance);
}

void otPlatTobleC2Subscribe(otInstance *aInstance, otTobleConnectionId aConnId, bool aSubscribe)
{
    uint8_t cmdBuffer[4];

    otEXPECT(sTobleConnList[aConnId].mIsInUse);

    cmdBuffer[0] = TOBLE_CMD_SUBSCRIBE_C2;
    cmdBuffer[1] = sTobleConnList[aConnId].mCentral;
    cmdBuffer[2] = sTobleConnList[aConnId].mPeripheral;
    cmdBuffer[3] = (aSubscribe ? 1 : 0);

    sendCommand(cmdBuffer, 4);

exit:
    OT_UNUSED_VARIABLE(aInstance);
}

// - - - - - - - - - - -
// Peripheral only

otError otPlatTobleAdvStart(otInstance *aInstance, const otTobleAdvConfig *aAdvConfig)
{
    otError error = OT_ERROR_NONE;

    otEXPECT_ACTION(!sTobleAdvertising, error = OT_ERROR_ALREADY);

    // Prepare the adv command frame

    sTobleAdvCmdFrame[0] = TOBLE_CMD_ADV;
    sTobleAdvCmdFrame[1] = sTobleNodeId;
    sTobleAdvCmdFrame[2] = (uint8_t)aAdvConfig->mType;
    memcpy(&sTobleAdvCmdFrame[3], aAdvConfig->mData, aAdvConfig->mLength);

    sTobleAdvCmdLength = 3 + aAdvConfig->mLength;

    sTobleAdvertising   = true;
    sTobleAdvInterval   = aAdvConfig->mInterval;
    sTobleAdvLastTxTime = otPlatAlarmMilliGetNow();

    sendCommand(sTobleAdvCmdFrame, sTobleAdvCmdLength);

    OT_UNUSED_VARIABLE(aInstance);

exit:
    return error;
}

otError otPlatTobleAdvStop(otInstance *aInstance)
{
    otError error = OT_ERROR_NONE;

    otEXPECT_ACTION(sTobleAdvertising, error = OT_ERROR_ALREADY);
    sTobleAdvertising = false;

    OT_UNUSED_VARIABLE(aInstance);

exit:
    return error;
}

void otPlatTobleC2Indicate(otInstance *aInstance, otTobleConnectionId aConnId, const void *aFrame, uint16_t aLength)
{
    otEXPECT(sToblePendingCmd == TOBLE_CMD_NONE);
    otEXPECT(sTobleConnList[aConnId].mIsInUse);

    sTobleConn = &sTobleConnList[aConnId];

    sTobleCmdFrame[0] = TOBLE_CMD_INDICATE_C2;
    sTobleCmdFrame[1] = sTobleConn->mCentral;
    sTobleCmdFrame[2] = sTobleConn->mPeripheral;
    assert(aLength < TOBLE_MAX_CMD_SIZE - 3);
    memcpy(&sTobleCmdFrame[3], aFrame, aLength);

    sTobleCmdLength = 3 + aLength;

    sToblePendingCmd = TOBLE_CMD_INDICATE_C2;

exit:
    OT_UNUSED_VARIABLE(aInstance);
}

#if OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
uint8_t otPlatTobleGetL2capPsm(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    // TODO: This needs to change when support for L2CAP simulation
    // is added. For now, return a fixed value.

    return 0x77;
}
#endif // OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE

//---------------------------------------------------------------------------------------------------------------------
// platformToble system

void platformTobleInit(uint32_t aSpeedUpFactor)
{
    char *offset;

    offset = getenv("PORT_OFFSET");

    if (offset)
    {
        char *endptr;

        sPortOffset = (uint16_t)strtol(offset, &endptr, 0);

        if (*endptr != '\0')
        {
            fprintf(stderr, "Invalid PORT_OFFSET: %s\n", offset);
            exit(EXIT_FAILURE);
        }

        sPortOffset *= WELLKNOWN_NODE_ID;
    }

    initFds();

    sTobleNodeId = (uint8_t)(gNodeId & 0xff);

    sSpeedUpFactor = aSpeedUpFactor;
}

void platformTobleDeinit(void)
{
    deinitFds();
}

void platformTobleUpdateFdSet(fd_set *aReadFdSet, fd_set *aWriteFdSet, struct timeval *aTimeout, int *aMaxFd)
{
    // Always ready to receive
    if (aReadFdSet != NULL)
    {
        FD_SET(sRxFd, aReadFdSet);

        if (aMaxFd != NULL && *aMaxFd < sRxFd)
        {
            *aMaxFd = sRxFd;
        }
    }

    if ((aWriteFdSet != NULL) && (sToblePendingCmd != TOBLE_CMD_NONE))
    {
        FD_SET(sTxFd, aWriteFdSet);

        if (aMaxFd != NULL && *aMaxFd < sTxFd)
        {
            *aMaxFd = sTxFd;
        }
    }

    if (sTobleAdvertising && (aTimeout != NULL))
    {
        // Calculate remaining time to next adv tx time (any wrapping during +/- is ok)
        uint32_t diff = (sTobleAdvLastTxTime + sTobleAdvInterval - otPlatAlarmMilliGetNow());

        if (diff >= (1UL << 31))
        {
            aTimeout->tv_sec  = 0;
            aTimeout->tv_usec = 0;
        }
        else
        {
            int64_t curTimeoutRemaining = ((int64_t)aTimeout->tv_sec) * US_PER_S + aTimeout->tv_usec;
            int64_t remaining           = ((int64_t)diff) * US_PER_MS;

            remaining = remaining / sSpeedUpFactor;

            // Update if the adv tx remaining time is smaller than current one in `aTimeout`
            if (remaining < curTimeoutRemaining)
            {
                aTimeout->tv_sec  = (time_t)remaining / US_PER_S;
                aTimeout->tv_usec = remaining % US_PER_S;
            }
        }
    }
}

void platformTobleProcess(otInstance *aInstance, const fd_set *aReadFdSet, const fd_set *aWriteFdSet)
{
    if (FD_ISSET(sTxFd, aWriteFdSet) && (sToblePendingCmd != TOBLE_CMD_NONE))
    {
        uint8_t pendingCmd = sToblePendingCmd;

        sendCommand(sTobleCmdFrame, sTobleCmdLength);

        // Ensure sToblePendingCmd is cleared before possibly invoking any callback.
        // since the callback may itself call `otPlatToble<fn>()`.

        sToblePendingCmd = TOBLE_CMD_NONE;

        switch (pendingCmd)
        {
        case TOBLE_CMD_NONE:
            break;

        case TOBLE_CMD_CONNECT:
            otPlatTobleHandleConnected(aInstance, GetConnId(sTobleConn));

            if (sTobleConn->mIsInUse) // TODO: check if we are peripheral?
            {
                otPlatTobleHandleConnectionIsReady(aInstance, GetConnId(sTobleConn), kConnectionLinkTypeGatt);
            }
            break;

        case TOBLE_CMD_WRITE_C1:
            otPlatTobleHandleC1WriteDone(aInstance, GetConnId(sTobleConn));
            break;

        case TOBLE_CMD_INDICATE_C2:
            otPlatTobleHandleC2IndicateDone(aInstance, GetConnId(sTobleConn));
            break;

        default:
            assert(false);
            break;
        }
    }

    if (FD_ISSET(sRxFd, aReadFdSet))
    {
        uint8_t  rxCommandFrame[TOBLE_MAX_CMD_SIZE];
        uint16_t rxCommandLength;
        ssize_t  rval;

        rval = recvfrom(sRxFd, (char *)rxCommandFrame, sizeof(rxCommandFrame), 0, NULL, NULL);

        if (rval < 0)
        {
            perror("recvfrom(sRxFd)");
            exit(EXIT_FAILURE);
        }

        rxCommandLength = (uint16_t)(rval);

#if DEBUG_LOG
        fprintf(stderr, "\n\r--->recvdCommand(%s)", cmdToString(*rxCommandFrame));
        dumpBuffer(rxCommandFrame, rxCommandLength);
#endif

        handleCommand(aInstance, rxCommandFrame, rxCommandLength);
    }

    if (sTobleAdvertising)
    {
        uint32_t now = otPlatAlarmMilliGetNow();

        if (now - sTobleAdvLastTxTime >= sTobleAdvInterval)
        {
            sendCommand(sTobleAdvCmdFrame, sTobleAdvCmdLength);
            sTobleAdvLastTxTime = now;
        }
    }
}

OT_TOOL_WEAK void otPlatTobleHandleConnected(otInstance *aInstance, otTobleConnectionId aConnId)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConnId);
}

OT_TOOL_WEAK void otPlatTobleHandleDisconnected(otInstance *aInstance, otTobleConnectionId aConnId)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConnId);
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
                                                     otTobleConnectionId       aConnId,
                                                     otTobleConnectionLinkType aLinkType)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConnId);
    OT_UNUSED_VARIABLE(aLinkType);
}

OT_TOOL_WEAK void otPlatTobleHandleC1WriteDone(otInstance *aInstance, otTobleConnectionId aConnId)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConnId);
}

OT_TOOL_WEAK void otPlatTobleHandleC2Indication(otInstance *        aInstance,
                                                otTobleConnectionId aConnId,
                                                const uint8_t *     aBuffer,
                                                uint16_t            aLength)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConnId);
    OT_UNUSED_VARIABLE(aBuffer);
    OT_UNUSED_VARIABLE(aLength);
}

OT_TOOL_WEAK void otPlatTobleHandleC2Subscribed(otInstance *aInstance, otTobleConnectionId aConnId, bool aIsSubscribed)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConnId);
    OT_UNUSED_VARIABLE(aIsSubscribed);
}

OT_TOOL_WEAK void otPlatTobleHandleC2IndicateDone(otInstance *aInstance, otTobleConnectionId aConnId)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConnId);
}

OT_TOOL_WEAK void otPlatTobleHandleC1Write(otInstance *        aInstance,
                                           otTobleConnectionId aConnId,
                                           const uint8_t *     aBuffer,
                                           uint16_t            aLength)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aConnId);
    OT_UNUSED_VARIABLE(aBuffer);
    OT_UNUSED_VARIABLE(aLength);
}
#endif // OPENTHREAD_CONFIG_TOBLE_ENABLE
