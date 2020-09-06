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
 *   This file contains the definition for Bluez events.
 */

#ifndef BLUEZ_EVENTS_HPP_
#define BLUEZ_EVENTS_HPP_

#include "openthread-core-config.h"

#include <stdint.h>

#define MGMT_PARAM_MAX_LEN 512
#define GATT_DATA_MAX_LEN 512

typedef enum
{
    kBluezEventTypeMgmtSend         = 1,
    kBluezEventTypeMgmtNotify       = 2,
    kBluezEventTypeGattDiscover     = 3,
    kBluezEventTypeGattC1Write      = 4,
    kBluezEventTypeGattC2Notify     = 5,
    kBluezEventTypeGattC2CccdWrite  = 6,
    kBluezEventTypeC1WriteDone      = 7,
    kBluezEventTypeC2NotificateDone = 8,
} BluezEventType;

OT_TOOL_PACKED_BEGIN
struct BluezMgmtEventHeader
{
    BluezEventType mType;
    uint16_t       mLength;

    struct BluezMgmtEventHeader *Next(void)
    {
        return reinterpret_cast<BluezMgmtEventHeader *>(reinterpret_cast<uint8_t *>(this) + GetSize());
    }

    uint16_t GetSize(void) { return sizeof(*this) + mLength; }
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
struct BluezMgmtSendEvent : public BluezMgmtEventHeader
{
    uint8_t  mStatus;
    uint16_t mOpcode;
    uint8_t  mParam[MGMT_PARAM_MAX_LEN];

    otError Init(uint8_t aStatus, uint16_t aOpcode, const void *aParam, uint16_t aParamLength)
    {
        otError error = OT_ERROR_NONE;

        VerifyOrExit(aParamLength <= sizeof(mParam), error = OT_ERROR_NO_BUFS);

        mType   = kBluezEventTypeMgmtSend;
        mStatus = aStatus;
        mOpcode = aOpcode;
        memcpy(mParam, aParam, aParamLength);
        SetParamLength(aParamLength);

    exit:
        return error;
    }

    void SetParamLength(uint16_t aLength)
    {
        mLength = sizeof(BluezMgmtSendEvent) - sizeof(BluezMgmtEventHeader) - sizeof(mParam) + aLength;
    }

    uint16_t GetParamLength(void)
    {
        return mLength - (sizeof(BluezMgmtSendEvent) - sizeof(BluezMgmtEventHeader) - sizeof(mParam));
    }

} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
struct BluezMgmtNotifyEvent : public BluezMgmtEventHeader
{
    uint16_t mIndex;
    uint16_t mEvent;
    uint8_t  mParam[MGMT_PARAM_MAX_LEN];

    otError Init(uint16_t aIndex, uint16_t aEvent, const void *aParam, uint16_t aParamLength)
    {
        otError error = OT_ERROR_NONE;

        VerifyOrExit(aParamLength <= sizeof(mParam), error = OT_ERROR_NO_BUFS);

        mType  = kBluezEventTypeMgmtNotify;
        mIndex = aIndex;
        mEvent = aEvent;
        memcpy(mParam, aParam, aParamLength);
        SetParamLength(aParamLength);

    exit:
        return error;
    }

    void SetParamLength(uint16_t aLength)
    {
        mLength = sizeof(BluezMgmtNotifyEvent) - sizeof(BluezMgmtEventHeader) - sizeof(mParam) + aLength;
    }

    uint16_t GetParamLength(void)
    {
        return mLength - (sizeof(BluezMgmtNotifyEvent) - sizeof(BluezMgmtEventHeader) - sizeof(mParam));
    }
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
struct BluezGattDiscoverEvent : public BluezMgmtEventHeader
{
    otTobleConnection *mConn;

    void Init(otTobleConnection *aConn)
    {
        mType   = kBluezEventTypeGattDiscover;
        mLength = sizeof(otTobleConnection *);
        mConn   = aConn;
    }
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
struct BluezGattC1WriteDoneEvent : public BluezMgmtEventHeader
{
    otTobleConnection *mConn;

    void Init(otTobleConnection *aConn)
    {
        mType   = kBluezEventTypeC1WriteDone;
        mLength = sizeof(otTobleConnection *);
        mConn   = aConn;
    }
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
struct BluezGattC2NotifyDoneEvent : public BluezMgmtEventHeader
{
    otTobleConnection *mConn;

    void Init(otTobleConnection *aConn)
    {
        mType   = kBluezEventTypeC2NotificateDone;
        mLength = sizeof(otTobleConnection *);
        mConn   = aConn;
    }
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
struct BluezGattC1WriteEvent : public BluezMgmtEventHeader
{
    otTobleConnection *mConn;
    uint8_t            mData[GATT_DATA_MAX_LEN];

    otError Init(otTobleConnection *aConn, const uint8_t *aData, uint16_t aDataLength)
    {
        otError error = OT_ERROR_NONE;

        VerifyOrExit(aDataLength <= sizeof(mData), error = OT_ERROR_NO_BUFS);

        mType = kBluezEventTypeGattC1Write;
        mConn = aConn;
        SetDataLength(aDataLength);
        memcpy(mData, aData, aDataLength);

    exit:
        return error;
    }

    void SetDataLength(uint16_t aLength)
    {
        mLength = sizeof(BluezGattC1WriteEvent) - sizeof(BluezMgmtEventHeader) - sizeof(mData) + aLength;
    }

    uint16_t GetDataLength(void)
    {
        return mLength - (sizeof(BluezGattC1WriteEvent) - sizeof(BluezMgmtEventHeader) - sizeof(mData));
    }
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
struct BluezGattC2NotifyEvent : public BluezMgmtEventHeader
{
    otTobleConnection *mConn;
    uint8_t            mData[GATT_DATA_MAX_LEN];

    otError Init(otTobleConnection *aConn, const uint8_t *aData, uint16_t aDataLength)
    {
        otError error = OT_ERROR_NONE;

        VerifyOrExit(aDataLength <= sizeof(mData), error = OT_ERROR_NO_BUFS);

        mType = kBluezEventTypeGattC2Notify;
        mConn = aConn;
        SetDataLength(aDataLength);
        memcpy(mData, aData, aDataLength);

    exit:
        return error;
    }

    void SetDataLength(uint16_t aLength)
    {
        mLength = sizeof(BluezGattC2NotifyEvent) - sizeof(BluezMgmtEventHeader) - sizeof(mData) + aLength;
    }

    uint16_t GetDataLength(void)
    {
        return mLength - (sizeof(BluezGattC2NotifyEvent) - sizeof(BluezMgmtEventHeader) - sizeof(mData));
    }
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
struct BluezGattC2CccdEvent : public BluezMgmtEventHeader
{
    otTobleConnection *mConn;
    bool               mSubscribed;

    void Init(otTobleConnection *aConn, bool aSubscribed)
    {
        mType       = kBluezEventTypeGattC2CccdWrite;
        mConn       = aConn;
        mSubscribed = aSubscribed;
    }
} OT_TOOL_PACKED_END;

#endif // BLUEZ_EVENTS_HPP_
