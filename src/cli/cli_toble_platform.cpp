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
 *   This file implements a simple CLI for the ToBLE platform service.
 */

#include "cli_toble_platform.hpp"

#include "cli/cli.hpp"
#include "cli/cli_server.hpp"
#include "common/encoding.hpp"

#if OPENTHREAD_CONFIG_TOBLE_ENABLE

namespace ot {
namespace Cli {

const struct ToblePlatform::Command ToblePlatform::sCommands[] = {
    {"help", &ToblePlatform::ProcessHelp},       {"diag", &ToblePlatform::ProcessDiag},
    {"adv", &ToblePlatform::ProcessAdv},         {"scan", &ToblePlatform::ProcessScan},
    {"connect", &ToblePlatform::ProcessConnect}, {"disconnect", &ToblePlatform::ProcessDisconnect},
    {"mtu", &ToblePlatform::ProcessMtu},         {"subscribe", &ToblePlatform::ProcessSubscribe},
    {"send", &ToblePlatform::ProcessSend},       {"role", &ToblePlatform::ProcessRole},
    {"link", &ToblePlatform::ProcessLink},       {"show", &ToblePlatform::ProcessShow}};

ToblePlatform::ToblePlatform(Interpreter &aInterpreter)
    : mInterpreter(aInterpreter)
    , mLinkType(kConnectionLinkTypeGatt)
    , mRole(OT_TOBLE_ROLE_PERIPHERAL)
{
    memset(mConns, 0, sizeof(mConns));
    otPlatTobleInit(mInterpreter.mInstance);
}

ToblePlatform::Connection *ToblePlatform::FindConnection(otTobleConnection *aConn)
{
    Connection *conn = NULL;

    for (uint8_t i = 0; i < kNumConnections; i++)
    {
        if (mConns[i].mPlatConn == aConn)
        {
            conn = &mConns[i];
            break;
        }
    }

    return conn;
}

uint8_t ToblePlatform::GetConnectionId(otTobleConnection *aConn)
{
    uint8_t index = 0xff;

    Connection *connSession = GetConnection(aConn);

    if (connSession != NULL)
    {
        index = GetConnectionId(connSession);
    }

    return index;
}

uint8_t ToblePlatform::GetNumValidConnections(void)
{
    uint8_t count = 0;

    for (Connection *conn = &mConns[0]; conn < OT_ARRAY_END(mConns); conn++)
    {
        if (conn->mPlatConn != NULL)
        {
            count++;
        }
    }

    return count;
}

const char *ToblePlatform::StateToString(State aState)
{
    const char *string = "Unknown";

    switch (aState)
    {
    case kStateFree:
        string = "Free";
        break;

    case kStateConnecting:
        string = "Connecting";
        break;

    case kStateConnected:
        string = "Connected";
        break;

    case kStateReady:
        string = "Ready";
        break;

    case kStateDisconnecting:
        string = "Disconnesting";
        break;

    default:
        break;
    }

    return string;
}

const char *ToblePlatform::LinkTypeToString(otTobleConnectionLinkType aLinkType)
{
    const char *string = "Unknown";

    if (aLinkType == kConnectionLinkTypeGatt)
    {
        string = "gatt";
    }
    else if (aLinkType == kConnectionLinkTypeL2Cap)
    {
        string = "l2cap";
    }

    return string;
}

otError ToblePlatform::ProcessHelp(uint8_t aArgsLength, char *aArgs[])
{
    OT_UNUSED_VARIABLE(aArgsLength);
    OT_UNUSED_VARIABLE(aArgs);

    for (size_t i = 0; i < OT_ARRAY_LENGTH(sCommands); i++)
    {
        mInterpreter.mServer->OutputFormat("%s\r\n", sCommands[i].mName);
    }

    return OT_ERROR_NONE;
}

otError ToblePlatform::ProcessDiag(uint8_t aArgsLength, char *aArgs[])
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(aArgsLength == 1, error = OT_ERROR_INVALID_ARGS);

    if (strcmp(aArgs[0], "start") == 0)
    {
        otPlatTobleDiagModeSet(mInterpreter.mInstance, true);
    }
    else if (strcmp(aArgs[0], "stop") == 0)
    {
        otPlatTobleDiagModeSet(mInterpreter.mInstance, false);
    }
    else
    {
        error = OT_ERROR_INVALID_ARGS;
    }

exit:
    return error;
}

otError ToblePlatform::ProcessAdv(uint8_t aArgsLength, char *aArgs[])
{
    otError error = OT_ERROR_INVALID_ARGS;
    long    value;

    VerifyOrExit(aArgsLength >= 1, error = OT_ERROR_INVALID_ARGS);

    if ((strcmp(aArgs[0], "start") == 0) && (aArgsLength == 2))
    {
        otTobleAdvConfig config;

#if 1
        const uint8_t advData[] = {0x02, 0x01, 0x06, 0x12, 0x16, 0xFF, 0xFB, 0x30, 0x00, 0x21, 0x00,
                                   0x01, 0xAA, 0xBB, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
        const uint8_t scanRsp[] = {0x16, 0x16, 0xFF, 0xFB, 0x31, 0x08, 0x10, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
#else

#if 0
        const uint8_t advData[] = {0x02, 0x01, 0x06, 0x04, 0x09, 0x41, 0x41, 0x41, 0x12, 0x16, 0xFF, 0xFB, 0x30, 0x00,
                                   0x21, 0x00, 0x01, 0xAA, 0xBB, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
        const uint8_t scanRsp[] = {0x16, 0xFF, 0xFF, 0xFB, 0x31, 0x08, 0x10, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        // Sevice Data: FBFF, 8877665544332211

#else
        const uint8_t advData[] = {0x02, 0x01, 0x06, 0x04, 0x09, 0x41, 0x41, 0x41, 0x12, 0x16, 0xFF, 0xFA, 0x30, 0x00,
                                   0x21, 0x00, 0x01, 0xAA, 0xBB, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
        const uint8_t scanRsp[] = {0x16, 0x16, 0xFF, 0xFB, 0x31, 0x08, 0x10, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        // Sevice Data: FBFF, FFFFFFFFFFFFFFFFF
#endif
#endif

        SuccessOrExit(error = Interpreter::ParseLong(aArgs[1], value));
        config.mType              = OT_TOBLE_ADV_IND;
        config.mInterval          = static_cast<uint16_t>(value);
        config.mData              = advData;
        config.mLength            = sizeof(advData);
        config.mScanRspData       = scanRsp;
        config.mScanRspDataLength = sizeof(scanRsp);

        SuccessOrExit(error = otPlatTobleAdvStart(mInterpreter.mInstance, &config));
    }
    else if (strcmp(aArgs[0], "stop") == 0)
    {
        error = otPlatTobleAdvStop(mInterpreter.mInstance);
    }

exit:
    return error;
}

otError ToblePlatform::ProcessScan(uint8_t aArgsLength, char *aArgs[])
{
    otError  error = OT_ERROR_INVALID_ARGS;
    long     value;
    uint16_t interval;
    uint16_t window;
    bool     active;

    VerifyOrExit(aArgsLength >= 1, error = OT_ERROR_INVALID_ARGS);

    if ((strcmp(aArgs[0], "start") == 0) && (aArgsLength == 4))
    {
        SuccessOrExit(error = Interpreter::ParseLong(aArgs[1], value));
        interval = static_cast<uint16_t>(value);
        SuccessOrExit(error = Interpreter::ParseLong(aArgs[2], value));
        window = static_cast<uint16_t>(value);
        SuccessOrExit(error = Interpreter::ParseLong(aArgs[3], value));
        active = (static_cast<uint16_t>(value) > 0);

        SuccessOrExit(error = otPlatTobleScanStart(mInterpreter.mInstance, interval, window, active));
        mRole = OT_TOBLE_ROLE_CENTRAL;

        mInterpreter.mServer->OutputFormat(
            "\r\n|     advType     | addrType |   address    | rssi | AD or Scan Rsp Data |\r\n");
        mInterpreter.mServer->OutputFormat(
            "+-----------------+----------+--------------+------+---------------------|\r\n");
    }
    else if (strcmp(aArgs[0], "stop") == 0)
    {
        error = otPlatTobleScanStop(mInterpreter.mInstance);
    }

exit:
    return error;
}

otError ToblePlatform::ProcessRole(uint8_t aArgsLength, char *aArgs[])
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(aArgsLength <= 1, error = OT_ERROR_INVALID_ARGS);

    if ((aArgsLength == 1) && (strcmp(aArgs[0], "peripheral") == 0))
    {
        mRole = OT_TOBLE_ROLE_PERIPHERAL;
    }
    else if ((aArgsLength == 1) && (strcmp(aArgs[0], "central") == 0))
    {
        mRole = OT_TOBLE_ROLE_CENTRAL;
    }
    else
    {
        mInterpreter.mServer->OutputFormat("%s\r\n", mRole == OT_TOBLE_ROLE_CENTRAL ? "peripheral" : "central");
    }

exit:
    return error;
}

otError ToblePlatform::ProcessLink(uint8_t aArgsLength, char *aArgs[])
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(aArgsLength <= 1, error = OT_ERROR_INVALID_ARGS);

    if (aArgsLength == 0)
    {
        mInterpreter.mServer->OutputFormat("%s\r\n", LinkTypeToString(mLinkType));
    }
    else
    {
        if (GetNumValidConnections() != 0)
        {
            mInterpreter.mServer->OutputFormat("Please disconnect all connection first.\r\n");
            ExitNow();
        }

        if (strcmp(aArgs[0], "gatt") == 0)
        {
            mLinkType = kConnectionLinkTypeGatt;
        }
#if OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
        else if (strcmp(aArgs[0], "l2cap") == 0)
        {
            mLinkType = kConnectionLinkTypeL2Cap;
        }
#endif
        else
        {
            error = OT_ERROR_INVALID_ARGS;
        }
    }

exit:
    return error;
}

otError ToblePlatform::ProcessMtu(uint8_t aArgsLength, char *aArgs[])
{
    otError     error = OT_ERROR_NONE;
    long        value;
    Connection *connSession;

    VerifyOrExit(aArgsLength == 1, error = OT_ERROR_INVALID_ARGS);

    SuccessOrExit(error = Interpreter::ParseLong(aArgs[0], value));
    VerifyOrExit((connSession = GetConnection(static_cast<uint8_t>(value))) != NULL, error = OT_ERROR_NOT_FOUND);

    mInterpreter.mServer->OutputFormat("%d\r\n", otPlatTobleGetMtu(mInterpreter.mInstance, connSession->mPlatConn));

exit:
    return error;
}

otError ToblePlatform::ProcessConnect(uint8_t aArgsLength, char *aArgs[])
{
    otError        error = OT_ERROR_NONE;
    long           value;
    otTobleAddress address;

    VerifyOrExit(aArgsLength >= 1, error = OT_ERROR_INVALID_ARGS);

    if ((strcmp(aArgs[0], "start") == 0) && (aArgsLength >= 3))
    {
        Connection *            connSession;
        otTobleConnection *     conn;
        otTobleConnectionConfig config;

        SuccessOrExit(error = Interpreter::ParseLong(aArgs[1], value));
        address.mType = static_cast<otTobleAddressType>(value);
        VerifyOrExit(address.mType <= OT_TOBLE_ADDRESS_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE,
                     error = OT_ERROR_INVALID_ARGS);

        VerifyOrExit(Interpreter::Hex2Bin(aArgs[2], address.mAddress, OT_TOBLE_ADDRESS_SIZE) == OT_TOBLE_ADDRESS_SIZE,
                     error = OT_ERROR_INVALID_ARGS);

        config.mInterval     = 200;
        config.mScanInterval = 200;
        config.mScanWindow   = 50;
        config.mPsm          = 0x80;
        config.mL2capMtu     = 500;
        config.mLinkType     = mLinkType;

        VerifyOrExit((conn = otPlatTobleCreateConnection(mInterpreter.mInstance, &address, &config)) != NULL,
                     error = OT_ERROR_FAILED);

        VerifyOrExit((connSession = GetNewConnection()) != NULL, error = OT_ERROR_NO_BUFS);

        connSession->mState    = kStateConnecting;
        connSession->mPlatConn = conn;
        memcpy(&connSession->mAddress, &address, sizeof(address));
    }
    else if ((strcmp(aArgs[0], "stop") == 0) && (aArgsLength >= 2))
    {
        Connection *connSession;

        SuccessOrExit(error = Interpreter::ParseLong(aArgs[1], value));
        VerifyOrExit((connSession = GetConnection(static_cast<uint8_t>(value))) != NULL, error = OT_ERROR_NOT_FOUND);

        otPlatTobleDisconnect(mInterpreter.mInstance, connSession->mPlatConn);
        connSession->mState = kStateDisconnecting;
    }
    else
    {
        error = OT_ERROR_INVALID_ARGS;
    }

exit:
    return error;
}

otError ToblePlatform::ProcessDisconnect(uint8_t aArgsLength, char *aArgs[])
{
    otError     error = OT_ERROR_NONE;
    long        value;
    Connection *connSession;

    VerifyOrExit(aArgsLength == 1, error = OT_ERROR_INVALID_ARGS);
    SuccessOrExit(error = Interpreter::ParseLong(aArgs[0], value));
    VerifyOrExit((connSession = GetConnection(static_cast<uint8_t>(value))) != NULL, error = OT_ERROR_NOT_FOUND);

    otPlatTobleDisconnect(mInterpreter.mInstance, connSession->mPlatConn);

exit:
    return error;
}

otError ToblePlatform::ProcessSubscribe(uint8_t aArgsLength, char *aArgs[])
{
    otError     error = OT_ERROR_NONE;
    long        value;
    uint8_t     index;
    Connection *connSession;

    VerifyOrExit(aArgsLength == 2, error = OT_ERROR_INVALID_ARGS);

    SuccessOrExit(error = Interpreter::ParseLong(aArgs[0], value));

    index       = static_cast<uint8_t>(value);
    connSession = GetConnection(index);
    VerifyOrExit(connSession != NULL, error = OT_ERROR_NOT_FOUND);

    SuccessOrExit(error = Interpreter::ParseLong(aArgs[1], value));
    otPlatTobleC2Subscribe(mInterpreter.mInstance, connSession->mPlatConn, value >= 1);

exit:
    return error;
}

otError ToblePlatform::ProcessSend(uint8_t aArgsLength, char *aArgs[])
{
    otError     error = OT_ERROR_NONE;
    long        value;
    uint8_t     buffer[256];
    uint16_t    length;
    Connection *connSession;

    VerifyOrExit(aArgsLength == 2, error = OT_ERROR_INVALID_ARGS);

    SuccessOrExit(error = Interpreter::ParseLong(aArgs[0], value));
    VerifyOrExit((connSession = GetConnection(static_cast<uint8_t>(value))) != NULL, error = OT_ERROR_NOT_FOUND);

    SuccessOrExit(error = Interpreter::ParseLong(aArgs[1], value));
    length = static_cast<uint16_t>(value);
    VerifyOrExit(length <= sizeof(buffer), error = OT_ERROR_INVALID_ARGS);
    VerifyOrExit(length <= otPlatTobleGetMtu(mInterpreter.mInstance, connSession->mPlatConn),
                 error = OT_ERROR_INVALID_ARGS);

    for (uint16_t i = 0; i < length; i++)
    {
        buffer[i] = i % 256;
    }

    if (mLinkType == kConnectionLinkTypeGatt)
    {
        if (mRole == OT_TOBLE_ROLE_CENTRAL)
        {
            otPlatTobleC1Write(mInterpreter.mInstance, connSession->mPlatConn, buffer, length);
        }
        else if (mRole == OT_TOBLE_ROLE_PERIPHERAL)
        {
            otPlatTobleC2Indicate(mInterpreter.mInstance, connSession->mPlatConn, buffer, length);
        }
        else
        {
            error = OT_ERROR_INVALID_STATE;
        }
    }
#if OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
    else if (mLinkType == kConnectionLinkTypeL2Cap)
    {
        mInterpreter.mServer->OutputFormat("kConnectionLinkTypeL2Cap\r\n");
        otPlatTobleL2capSend(mInterpreter.mInstance, connSession->mPlatConn, buffer, length);
    }
#endif
    else
    {
        error = OT_ERROR_INVALID_STATE;
    }

exit:
    return error;
}

otError ToblePlatform::ProcessShow(uint8_t aArgsLength, char *aArgs[])
{
    OT_UNUSED_VARIABLE(aArgsLength);
    OT_UNUSED_VARIABLE(aArgs);

    mInterpreter.mServer->OutputFormat("\r\n| Index |     State     |   Address    |  PlatConn  |\r\n");
    mInterpreter.mServer->OutputFormat("+-------+---------------+--------------|------------|\r\n");

    for (uint8_t i = 0; i < OT_ARRAY_LENGTH(mConns); i++)
    {
        if (mConns[i].mState != kStateFree)
        {
            uint8_t *address = mConns[i].mAddress.mAddress;

            mInterpreter.mServer->OutputFormat("| %-5d | %-13s | %02x%02x%02x%02x%02x%02x | %p |\r\n", i,
                                               StateToString(mConns[i].mState), address[0], address[1], address[2],
                                               address[3], address[4], address[5], mConns[i].mPlatConn);
        }
    }

    return OT_ERROR_NONE;
}

otError ToblePlatform::Process(uint8_t aArgsLength, char *aArgs[])
{
    otError error = OT_ERROR_INVALID_COMMAND;

    if (aArgsLength < 1)
    {
        IgnoreError(ProcessHelp(0, NULL));
        error = OT_ERROR_INVALID_ARGS;
    }
    else
    {
        for (size_t i = 0; i < OT_ARRAY_LENGTH(sCommands); i++)
        {
            if (strcmp(aArgs[0], sCommands[i].mName) == 0)
            {
                error = (this->*sCommands[i].mCommand)(aArgsLength - 1, aArgs + 1);
                break;
            }
        }
    }
    return error;
}

const char *ToblePlatform::AdvTypeToString(otTobleAdvType aType)
{
    const char *string = "Unknown";

    switch (aType)
    {
    case OT_TOBLE_ADV_IND:
        string = "ADV_IND";
        break;

    case OT_TOBLE_ADV_DIRECT_IND:
        string = "ADV_DIRECT_IND";
        break;

    case OT_TOBLE_ADV_SCAN_IND:
        string = "ADV_SCAN_IND";
        break;

    case OT_TOBLE_ADV_NONCONN_IND:
        string = "ADV_NONCONN_IND";
        break;

    default:
        break;
    }

    return string;
}

void ToblePlatform::PrintBytes(const uint8_t *aBuffer, uint8_t aLength)
{
    for (uint8_t i = 0; i < aLength; i++)
    {
        mInterpreter.mServer->OutputFormat("%02x", aBuffer[i]);
    }
}

//------------------------------------------------------------------------------------

void ToblePlatform::HandleAdvReceived(otTobleAdvType aAdvType, otTobleAdvPacket *aAdvPacket)
{
    otTobleAddress *srcAddress = &aAdvPacket->mSrcAddress;

    mInterpreter.mServer->OutputFormat("| %-15s |    %d     | %02x%02x%02x%02x%02x%02x | %3d  | ",
                                       AdvTypeToString(aAdvType), srcAddress->mType, srcAddress->mAddress[0],
                                       srcAddress->mAddress[1], srcAddress->mAddress[2], srcAddress->mAddress[3],
                                       srcAddress->mAddress[4], srcAddress->mAddress[5], aAdvPacket->mRssi);
    PrintBytes(aAdvPacket->mData, aAdvPacket->mLength);
    mInterpreter.mServer->OutputFormat("\r\n");
}

void ToblePlatform::HandleScanRespReceived(otTobleAdvPacket *aAdvPacket)
{
    otTobleAddress *srcAddress = &aAdvPacket->mSrcAddress;

    mInterpreter.mServer->OutputFormat("| SCAN_RSP        |    %d     | %02x%02x%02x%02x%02x%02x | %3d  | ",
                                       srcAddress->mType, srcAddress->mAddress[0], srcAddress->mAddress[1],
                                       srcAddress->mAddress[2], srcAddress->mAddress[3], srcAddress->mAddress[4],
                                       srcAddress->mAddress[5], aAdvPacket->mRssi);
    PrintBytes(aAdvPacket->mData, aAdvPacket->mLength);
    mInterpreter.mServer->OutputFormat("\r\n");
}

void ToblePlatform::HandleConnected(otTobleConnection *aConn)
{
    Connection *connSession = GetConnection(aConn);

    if (connSession != NULL)
    {
        connSession->mState = kStateConnected;
    }
    else
    {
        VerifyOrExit((connSession = GetNewConnection()) != NULL, OT_NOOP);
        connSession->mState    = kStateConnected;
        connSession->mPlatConn = aConn;
        memset(&connSession->mAddress, 0, sizeof(otTobleAddress));
    }

    mInterpreter.mServer->OutputFormat("HandleConnected: index=%d, platConn=%p\r\n", GetConnectionId(connSession),
                                       aConn);

exit:
    return;
}

void ToblePlatform::HandleDisconnected(otTobleConnection *aConn)
{
    Connection *connSession = GetConnection(aConn);

    mInterpreter.mServer->OutputFormat("HandleDisconnected: platConn=%p\r\n", aConn);

    VerifyOrExit(connSession != NULL, OT_NOOP);

    mInterpreter.mServer->OutputFormat("HandleDisconnected: index=%d, platConn=%p\r\n", GetConnectionId(connSession),
                                       aConn);
    connSession->mState    = kStateFree;
    connSession->mPlatConn = NULL;

exit:
    return;
}

void ToblePlatform::HandleConnectionIsReady(otTobleConnection *aConn, otTobleConnectionLinkType aLinkType)
{
    Connection *connSession = GetConnection(aConn);

    VerifyOrExit(connSession != NULL, OT_NOOP);

    mInterpreter.mServer->OutputFormat("HandleConnectionIsReady: index=%d, linkType=%s \r\n",
                                       GetConnectionId(connSession), LinkTypeToString(aLinkType));
    connSession->mState = kStateReady;

exit:
    return;
}

void ToblePlatform::HandleC1WriteDone(otTobleConnection *aConn)
{
    OT_UNUSED_VARIABLE(aConn);
    mInterpreter.mServer->OutputFormat("HandleC1WriteDone: index=%d\r\n", GetConnectionId(aConn));
}

void ToblePlatform::HandleC2Indication(otTobleConnection *aConn, const uint8_t *aBuffer, uint16_t aLength)
{
    OT_UNUSED_VARIABLE(aConn);
    OT_UNUSED_VARIABLE(aBuffer);
    mInterpreter.mServer->OutputFormat("HandleC2Indication: index=%d, length=%d\r\n", GetConnectionId(aConn), aLength);
}

void ToblePlatform::HandleC2Subscribed(otTobleConnection *aConn, bool aIsSubscribed)
{
    OT_UNUSED_VARIABLE(aConn);
    mInterpreter.mServer->OutputFormat("HandleC2Subscribed: index=%d, isSubscribed=%s\r\n", GetConnectionId(aConn),
                                       aIsSubscribed ? "True" : "False");
}

void ToblePlatform::HandleC2IndicateDone(otTobleConnection *aConn)
{
    OT_UNUSED_VARIABLE(aConn);
    mInterpreter.mServer->OutputFormat("HandleC2IndicateDone: index=%d\r\n", GetConnectionId(aConn));
}

void ToblePlatform::HandleC1Write(otTobleConnection *aConn, const uint8_t *aBuffer, uint16_t aLength)
{
    OT_UNUSED_VARIABLE(aConn);
    OT_UNUSED_VARIABLE(aBuffer);
    mInterpreter.mServer->OutputFormat("HandleC1Write: index=%d, length=%d\r\n", GetConnectionId(aConn), aLength);
}

#if OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
void ToblePlatform::HandleL2capFrameReceived(otTobleConnection *aConn, const uint8_t *aBuffer, uint16_t aLength)
{
    OT_UNUSED_VARIABLE(aConn);
    OT_UNUSED_VARIABLE(aBuffer);
    mInterpreter.mServer->OutputFormat("HandleL2capFrameReceived: index=%d, length=%d\r\n", GetConnectionId(aConn),
                                       aLength);
}

void ToblePlatform::HandleL2capSendDone(otTobleConnection *aConn)
{
    OT_UNUSED_VARIABLE(aConn);
    mInterpreter.mServer->OutputFormat("HandleL2capSendDone: index=%d\r\n", GetConnectionId(aConn));
}
#endif

//------------------------------------------------------------------------------------

extern "C" void otPlatTobleDiagHandleAdv(otInstance *          aInstance,
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

extern "C" void otPlatTobleDiagGapOnAdvReceived(otInstance *      aInstance,
                                                otTobleAdvType    aAdvType,
                                                otTobleAdvPacket *aAdvPacket)
{
    OT_UNUSED_VARIABLE(aInstance);
    Server::sServer->GetInterpreter().GetToblePlatform().HandleAdvReceived(aAdvType, aAdvPacket);
}

extern "C" void otPlatTobleDiagGapOnScanRespReceived(otInstance *aInstance, otTobleAdvPacket *aAdvPacket)
{
    OT_UNUSED_VARIABLE(aInstance);
    Server::sServer->GetInterpreter().GetToblePlatform().HandleScanRespReceived(aAdvPacket);
}

extern "C" void otPlatTobleDiagHandleConnected(otInstance *aInstance, otTobleConnection *aConn)
{
    OT_UNUSED_VARIABLE(aInstance);
    Server::sServer->GetInterpreter().GetToblePlatform().HandleConnected(aConn);
}

extern "C" void otPlatTobleDiagHandleDisconnected(otInstance *aInstance, otTobleConnection *aConn)
{
    OT_UNUSED_VARIABLE(aInstance);
    Server::sServer->GetInterpreter().GetToblePlatform().HandleDisconnected(aConn);
}

extern "C" void otPlatTobleDiagHandleConnectionIsReady(otInstance *              aInstance,
                                                       otTobleConnection *       aConn,
                                                       otTobleConnectionLinkType aLinkType)
{
    OT_UNUSED_VARIABLE(aInstance);
    Server::sServer->GetInterpreter().GetToblePlatform().HandleConnectionIsReady(aConn, aLinkType);
}

extern "C" void otPlatTobleDiagHandleC1WriteDone(otInstance *aInstance, otTobleConnection *aConn)
{
    OT_UNUSED_VARIABLE(aInstance);
    Server::sServer->GetInterpreter().GetToblePlatform().HandleC1WriteDone(aConn);
}

extern "C" void otPlatTobleDiagHandleC2Indication(otInstance *       aInstance,
                                                  otTobleConnection *aConn,
                                                  const uint8_t *    aBuffer,
                                                  uint16_t           aLength)
{
    OT_UNUSED_VARIABLE(aInstance);
    Server::sServer->GetInterpreter().GetToblePlatform().HandleC2Indication(aConn, aBuffer, aLength);
}

extern "C" void otPlatTobleDiagHandleC2Subscribed(otInstance *aInstance, otTobleConnection *aConn, bool aIsSubscribed)
{
    OT_UNUSED_VARIABLE(aInstance);
    Server::sServer->GetInterpreter().GetToblePlatform().HandleC2Subscribed(aConn, aIsSubscribed);
}

extern "C" void otPlatTobleDiagHandleC2IndicateDone(otInstance *aInstance, otTobleConnection *aConn)
{
    OT_UNUSED_VARIABLE(aInstance);
    Server::sServer->GetInterpreter().GetToblePlatform().HandleC2IndicateDone(aConn);
}

extern "C" void otPlatTobleDiagHandleC1Write(otInstance *       aInstance,
                                             otTobleConnection *aConn,
                                             const uint8_t *    aBuffer,
                                             uint16_t           aLength)
{
    OT_UNUSED_VARIABLE(aInstance);
    Server::sServer->GetInterpreter().GetToblePlatform().HandleC1Write(aConn, aBuffer, aLength);
}

extern "C" void otPlatTobleDiagL2capSendDone(otInstance *aInstance, otTobleConnection *aConn)
{
    OT_UNUSED_VARIABLE(aInstance);
#if OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
    Server::sServer->GetInterpreter().GetToblePlatform().HandleL2capSendDone(aConn);
#else
    OT_UNUSED_VARIABLE(aConn);
#endif
}

extern "C" void otPlatTobleDiagL2CapFrameReceived(otInstance *       aInstance,
                                                  otTobleConnection *aConn,
                                                  const uint8_t *    aBuffer,
                                                  uint16_t           aLength)
{
    OT_UNUSED_VARIABLE(aInstance);
#if OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
    Server::sServer->GetInterpreter().GetToblePlatform().HandleL2capFrameReceived(aConn, aBuffer, aLength);
#else
    OT_UNUSED_VARIABLE(aConn);
    OT_UNUSED_VARIABLE(aBuffer);
    OT_UNUSED_VARIABLE(aLength);
#endif
}

} // namespace Cli
} // namespace ot

#endif // OPENTHREAD_CONFIG_TOBLE_ENABLE
