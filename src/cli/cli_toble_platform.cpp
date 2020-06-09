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

using ot::Encoding::BigEndian::HostSwap16;

namespace ot {
namespace Cli {

const struct ToblePlatform::Command ToblePlatform::sCommands[] = {{"help", &ToblePlatform::ProcessHelp},
                                                                  {"adv", &ToblePlatform::ProcessAdv},
                                                                  {"connect", &ToblePlatform::ProcessConnect},
                                                                  {"scan", &ToblePlatform::ProcessScan},
                                                                  {"send", &ToblePlatform::ProcessSend}};

ToblePlatform::ToblePlatform(Interpreter &aInterpreter)
    : mInterpreter(aInterpreter)
    , mLinkType(kConnectionLinkTypeGatt)
{
    otPlatTobleInit(mInterpreter.mInstance);
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

otError ToblePlatform::ProcessAdv(uint8_t aArgsLength, char *aArgs[])
{
    otError          error = OT_ERROR_INVALID_ARGS;
    long             value;
    otTobleAdvConfig config;
    uint8_t          advData[OT_TOBLE_ADV_DATA_MAX_LENGTH];
    int              advDataLength;

    VerifyOrExit(aArgsLength >= 1, error = OT_ERROR_INVALID_ARGS);

    if ((strcmp(aArgs[0], "start") == 0) && (aArgsLength == 3))
    {
        SuccessOrExit(error = Interpreter::ParseLong(aArgs[1], value));
        VerifyOrExit((advDataLength = Interpreter::Hex2Bin(aArgs[2], advData, sizeof(advData))) > 0,
                     error = OT_ERROR_INVALID_ARGS);

        config.mType     = OT_TOBLE_ADV_IND;
        config.mInterval = static_cast<uint16_t>(value);
        config.mData     = advData;
        config.mLength   = static_cast<uint16_t>(advDataLength);

        SuccessOrExit(error = otPlatTobleAdvStart(mInterpreter.mInstance, &config));
        mRole = OT_TOBLE_ROLE_PERIPHERAL;
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

    VerifyOrExit(aArgsLength >= 1, error = OT_ERROR_INVALID_ARGS);

    if ((strcmp(aArgs[0], "start") == 0) && (aArgsLength == 3))
    {
        SuccessOrExit(error = Interpreter::ParseLong(aArgs[1], value));
        interval = static_cast<uint16_t>(value);
        SuccessOrExit(error = Interpreter::ParseLong(aArgs[2], value));
        window = static_cast<uint16_t>(value);

        SuccessOrExit(error = otPlatTobleScanStart(mInterpreter.mInstance, interval, window));
        mRole = OT_TOBLE_ROLE_CENTRAL;

        mInterpreter.mServer->OutputFormat(
            "\r\n| advType | addrType |   address    | rssi | AD or Scan Rsp Data |\r\n");
        mInterpreter.mServer->OutputFormat("+---------+----------+--------------+------+---------------------|\r\n");
    }
    else if (strcmp(aArgs[0], "stop") == 0)
    {
        error = otPlatTobleScanStop(mInterpreter.mInstance);
    }

exit:
    return error;
}

void ToblePlatform::ReverseBuf(uint8_t *aBuffer, uint8_t aLength)
{
    uint8_t temp;

    for (uint8_t i = 0; i < aLength / 2; i++)
    {
        temp                     = aBuffer[i];
        aBuffer[i]               = aBuffer[aLength - 1 - i];
        aBuffer[aLength - 1 - i] = temp;
    }
}

otError ToblePlatform::ProcessConnect(uint8_t aArgsLength, char *aArgs[])
{
    otError        error = OT_ERROR_NONE;
    long           value;
    otTobleAddress address;

    VerifyOrExit(aArgsLength >= 1, error = OT_ERROR_INVALID_ARGS);

    if ((strcmp(aArgs[0], "start") == 0) && (aArgsLength >= 3))
    {
        otTobleConnectionId     id;
        otTobleConnectionConfig config;

        SuccessOrExit(error = Interpreter::ParseLong(aArgs[1], value));
        address.mType = static_cast<otTobleAddressType>(value);
        VerifyOrExit(address.mType <= OT_TOBLE_ADDRESS_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE,
                     error = OT_ERROR_INVALID_ARGS);

        VerifyOrExit(Interpreter::Hex2Bin(aArgs[2], address.mAddress, OT_TOBLE_ADDRESS_SIZE) == OT_TOBLE_ADDRESS_SIZE,
                     error = OT_ERROR_INVALID_ARGS);
        ReverseBuf(address.mAddress, OT_TOBLE_ADDRESS_SIZE);

        config.mInterval     = 200;
        config.mScanInterval = 200;
        config.mScanWindow   = 50;
        config.mLinkType     = kConnectionLinkTypeGatt;

        id = otPlatTobleCreateConnection(mInterpreter.mInstance, &address, &config);
        VerifyOrExit(id != OT_TOBLE_CONNECTION_ID_INVALID, OT_NOOP);
        mConnId = id;
        mInterpreter.mServer->OutputFormat("ConnId=%d\r\n", mConnId);
        mRole = OT_TOBLE_ROLE_CENTRAL;
    }
    else if (strcmp(aArgs[0], "stop") == 0)
    {
        otPlatTobleDisconnect(mInterpreter.mInstance, mConnId);
    }
    else
    {
        error = OT_ERROR_INVALID_ARGS;
    }

exit:
    return error;
}

otError ToblePlatform::ProcessSend(uint8_t aArgsLength, char *aArgs[])
{
    otError  error = OT_ERROR_NONE;
    long     value;
    uint8_t  buffer[200];
    uint16_t length;
    uint8_t  id;

    VerifyOrExit(aArgsLength == 2, error = OT_ERROR_INVALID_ARGS);

    SuccessOrExit(error = Interpreter::ParseLong(aArgs[0], value));
    id = static_cast<otTobleConnectionId>(value);

    SuccessOrExit(error = Interpreter::ParseLong(aArgs[1], value));
    length = static_cast<uint16_t>(value);
    VerifyOrExit(length <= sizeof(buffer), error = OT_ERROR_INVALID_ARGS);
    VerifyOrExit(id == mConnId, error = OT_ERROR_INVALID_ARGS);

    for (uint16_t i = 0; i < length; i++)
    {
        buffer[i] = i % 256;
    }

    if (mLinkType == kConnectionLinkTypeGatt)
    {
        if (mRole == OT_TOBLE_ROLE_CENTRAL)
        {
            otPlatTobleC1Write(mInterpreter.mInstance, mConnId, buffer, length);
        }
        else if (mRole == OT_TOBLE_ROLE_PERIPHERAL)
        {
            otPlatTobleC2Indicate(mInterpreter.mInstance, mConnId, buffer, length);
        }
        else
        {
            error = OT_ERROR_INVALID_STATE;
        }
    }
#if OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
    else if (mLinkType == kConnectionLinkTypeL2Cap)
    {
        otPlatTobleL2capSend(mInterpreter.mInstance, mConnId, buffer, length);
    }
#endif
    else
    {
        error = OT_ERROR_INVALID_STATE;
    }

exit:
    return error;
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

void PrintBytes(const uint8_t *aBuffer, uint8_t aLength)
{
    for (uint8_t i = 0; i < aLength; i++)
    {
        Server::sServer->OutputFormat("%02x", aBuffer[i]);
    }
}

extern "C" void otPlatTobleHandleConnected(otInstance *aInstance, otTobleConnectionId aConnId)
{
    OT_UNUSED_VARIABLE(aInstance);

    Server::sServer->OutputFormat("Connected : ConnId=%d\r\n", aConnId);
}

extern "C" void otPlatTobleHandleDisconnected(otInstance *aInstance, otTobleConnectionId aConnId)
{
    OT_UNUSED_VARIABLE(aInstance);

    Server::sServer->OutputFormat("Disconnected : ConnId=%d\r\n", aConnId);
}

const char *GetTobleAdvTypeString(otTobleAdvType aType)
{
    const char *string;

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
        string = "unknown";
        break;
    }

    return string;
}

extern "C" void otPlatTobleHandleAdv(otInstance *          aInstance,
                                     otTobleAdvType        aAdvType,
                                     const otTobleAddress *aSource,
                                     const uint8_t *       aData,
                                     uint16_t              aLength,
                                     int8_t                aRssi)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aAdvType);

    Server::sServer->OutputFormat("| %-8s|    %d     | %02x%02x%02x%02x%02x%02x | %3d  | ",
                                  GetTobleAdvTypeString(aAdvType), aSource->mType, aSource->mAddress[5],
                                  aSource->mAddress[4], aSource->mAddress[3], aSource->mAddress[2],
                                  aSource->mAddress[1], aSource->mAddress[0], aRssi);
    PrintBytes(aData, aLength);
    Server::sServer->OutputFormat("\r\n");
}

extern "C" void otPlatTobleHandleConnectionIsReady(otInstance *              aInstance,
                                                   otTobleConnectionId       aConnId,
                                                   otTobleConnectionLinkType aLinkType)
{
    OT_UNUSED_VARIABLE(aInstance);

    Server::sServer->OutputFormat("TobleConnectionIsReady : ConnId=%d LinkType=%s\r\n", aConnId,
                                  aLinkType == kConnectionLinkTypeGatt ? "Gatt" : "L2Cap");
}

extern "C" void otPlatTobleHandleC1WriteDone(otInstance *aInstance, otTobleConnectionId aConnId)
{
    OT_UNUSED_VARIABLE(aInstance);

    Server::sServer->OutputFormat("C1WriteDone : ConnId=%d\r\n", aConnId);
}

extern "C" void otPlatTobleHandleC2Indication(otInstance *        aInstance,
                                              otTobleConnectionId aConnId,
                                              const uint8_t *     aBuffer,
                                              uint16_t            aLength)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aBuffer);

    Server::sServer->OutputFormat("C2Indication : ConnId=%d Length=%d\r\n", aConnId, aLength);
}

extern "C" void otPlatTobleHandleC2Subscribed(otInstance *aInstance, otTobleConnectionId aConnId, bool aIsSubscribed)
{
    OT_UNUSED_VARIABLE(aInstance);

    Server::sServer->OutputFormat("C2Subscribed : ConnId=%d IsSubscribed=%s\r\n", aConnId,
                                  aIsSubscribed ? "True" : "False");
}

extern "C" void otPlatTobleHandleC2IndicateDone(otInstance *aInstance, otTobleConnectionId aConnId)
{
    OT_UNUSED_VARIABLE(aInstance);

    Server::sServer->OutputFormat("C2indicationDone : ConnId=%d \r\n", aConnId);
}

extern "C" void otPlatTobleHandleC1Write(otInstance *        aInstance,
                                         otTobleConnectionId aConnId,
                                         const uint8_t *     aBuffer,
                                         uint16_t            aLength)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aBuffer);

    Server::sServer->OutputFormat("C1Write : ConnId=%d Length=%d\r\n", aConnId, aLength);
}
} // namespace Cli
} // namespace ot

#endif // OPENTHREAD_CONFIG_TOBLE_ENABLE
