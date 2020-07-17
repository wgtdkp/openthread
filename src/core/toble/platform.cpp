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
 *   This file implements ToBLE platform.
 */

#include "toble/platform.hpp"

#include "common/code_utils.hpp"
#include "common/instance.hpp"
#include "common/locator-getters.hpp"
#include "common/logging.hpp"

#if OPENTHREAD_CONFIG_TOBLE_ENABLE

namespace ot {
namespace Toble {

bool Address::operator==(const Address &aOther) const
{
    return (mType == aOther.mType) && (memcmp(mAddress, aOther.mAddress, OT_TOBLE_ADDRESS_SIZE) == 0);
}

bool Address::operator!=(const Address &aOther) const
{
    return (mType != aOther.mType) || (memcmp(mAddress, aOther.mAddress, OT_TOBLE_ADDRESS_SIZE) != 0);
}

Address::InfoString Address::ToString(void) const
{
    InfoString str;

    str.Append("%02x%02x%02x%02x%02x%02x", mAddress[0], mAddress[1], mAddress[2], mAddress[3], mAddress[4],
               mAddress[5]);

    switch (mType)
    {
    case OT_TOBLE_ADDRESS_TYPE_PUBLIC:
        str.Append("(public)");
        break;
    case OT_TOBLE_ADDRESS_TYPE_RANDOM_STATIC:
        str.Append("(rand)");
        break;
    case OT_TOBLE_ADDRESS_TYPE_RANDOM_PRIVATE_RESOLVABLE:
        str.Append("(prv rslv)");
        break;
    case OT_TOBLE_ADDRESS_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE:
        str.Append("(prv non-rslv)");
        break;
    }

    return str;
}

void Platform::Callbacks::HandleConnected(Connection *aConn)
{
    Get<Toble>().HandleConnected(aConn);
}

void Platform::Callbacks::HandleDisconnected(Connection *aConn)
{
    Get<Toble>().HandleDisconnected(aConn);
}

#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
otError Platform::StartScan(uint16_t aInterval, uint16_t aWindow)
{
    return otPlatTobleScanStart(GetInstance(), aInterval, aWindow, false);
}

otError Platform::StopScan(void)
{
    return otPlatTobleScanStop(GetInstance());
}

Platform::Connection *Platform::CreateConnection(const Address &aPeerAddress, ConnectionConfig &aConfig)
{
    return otPlatTobleCreateConnection(GetInstance(), &aPeerAddress, &aConfig);
}

void Platform::WriteC1(Connection *aConn, const void *aBuf, uint16_t aLength)
{
    otPlatTobleC1Write(GetInstance(), aConn, aBuf, aLength);
}

void Platform::SubscribeC2(Connection *aConn, bool aSubscribe)
{
    return otPlatTobleC2Subscribe(GetInstance(), aConn, aSubscribe);
}

void Platform::Callbacks::HandleAdv(AdvType aAdvType, AdvPacket &aAdvPacket)
{
    Get<Central::Controller>().HandleAdv(aAdvType, aAdvPacket);
}

void Platform::Callbacks::HandleScanResponse(AdvPacket &aAdvPacket)
{
    Get<Central::Controller>().HandleScanResponse(aAdvPacket);
}

void Platform::Callbacks::HandleConnectionReady(Connection *aConn, otTobleConnectionLinkType aLinkType)
{
    switch (aLinkType)
    {
    case kConnectionLinkTypeGatt:
        Get<Btp>().HandleConnectionReady(aConn);
        break;
    default:
        otLogCritBle("Unsupported link type");
        break;
    }
}

void Platform::Callbacks::HandleC1WriteDone(Connection *aConn)
{
    Get<Btp>().HandleC1WriteDone(aConn);
}

void Platform::Callbacks::HandleC2Indication(Connection *aConn, const uint8_t *aBuf, uint16_t aLength)
{
    Get<Btp>().HandleC2Indication(aConn, aBuf, aLength);
}

#endif // OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE

#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
otError Platform::StartAdv(const AdvConfig &aConfig)
{
    return otPlatTobleAdvStart(GetInstance(), &aConfig);
}

otError Platform::StopAdv(void)
{
    return otPlatTobleAdvStop(GetInstance());
}

void Platform::IndicateC2(Connection *aConn, const void *aFrame, uint16_t aLength)
{
    otPlatTobleC2Indicate(GetInstance(), aConn, aFrame, aLength);
}

void Platform::Callbacks::HandleC2Subscribed(Connection *aConn, bool aIsSubscribed)
{
    Get<Btp>().HandleC2Subscribed(aConn, aIsSubscribed);
}

void Platform::Callbacks::HandleC2IndicateDone(Connection *aConn)
{
    Get<Btp>().HandleC2IndicateDone(aConn);
}

void Platform::Callbacks::HandleC1Write(Connection *aConn, const uint8_t *aFrame, uint16_t aLength)
{
    Get<Btp>().HandleC1Write(aConn, aFrame, aLength);
}

#endif // OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
} // namespace Toble
} // namespace ot

extern "C" void otPlatTobleHandleConnected(otInstance *aInstance, otTobleConnection *aConn)
{
    ot::Instance &instance = *static_cast<ot::Instance *>(aInstance);

    instance.Get<ot::Toble::Platform::Callbacks>().HandleConnected(aConn);
}

extern "C" void otPlatTobleHandleDisconnected(otInstance *aInstance, otTobleConnection *aConn)
{
    ot::Instance &instance = *static_cast<ot::Instance *>(aInstance);

    instance.Get<ot::Toble::Platform::Callbacks>().HandleDisconnected(aConn);
}

#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE

extern "C" void otPlatTobleGapOnAdvReceived(otInstance *      aInstance,
                                            otTobleAdvType    aAdvType,
                                            otTobleAdvPacket *aAdvPacket)
{
    ot::Instance &instance = *static_cast<ot::Instance *>(aInstance);

    instance.Get<ot::Toble::Platform::Callbacks>().HandleAdv(aAdvType, *aAdvPacket);
}

extern "C" void otPlatTobleGapOnScanRespReceived(otInstance *aInstance, otTobleAdvPacket *aAdvPacket)
{
    ot::Instance &instance = *static_cast<ot::Instance *>(aInstance);

    instance.Get<ot::Toble::Platform::Callbacks>().HandleScanResponse(*aAdvPacket);
}

extern "C" void otPlatTobleHandleConnectionIsReady(otInstance *              aInstance,
                                                   otTobleConnection *       aConn,
                                                   otTobleConnectionLinkType aLinkType)
{
    ot::Instance &instance = *static_cast<ot::Instance *>(aInstance);
    instance.Get<ot::Toble::Platform::Callbacks>().HandleConnectionReady(aConn, aLinkType);
}

extern "C" void otPlatTobleHandleC1WriteDone(otInstance *aInstance, otTobleConnection *aConn)
{
    ot::Instance &instance = *static_cast<ot::Instance *>(aInstance);

    instance.Get<ot::Toble::Platform::Callbacks>().HandleC1WriteDone(aConn);
}

extern "C" void otPlatTobleHandleC2Indication(otInstance *       aInstance,
                                              otTobleConnection *aConn,
                                              const uint8_t *    aFrame,
                                              uint16_t           aLength)
{
    ot::Instance &instance = *static_cast<ot::Instance *>(aInstance);

    instance.Get<ot::Toble::Platform::Callbacks>().HandleC2Indication(aConn, aFrame, aLength);
}

#endif // OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE

#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE

extern "C" void otPlatTobleHandleC1Write(otInstance *       aInstance,
                                         otTobleConnection *aConn,
                                         const uint8_t *    aFrame,
                                         uint16_t           aLength)
{
    ot::Instance &instance = *static_cast<ot::Instance *>(aInstance);

    instance.Get<ot::Toble::Platform::Callbacks>().HandleC1Write(aConn, aFrame, aLength);
}

extern "C" void otPlatTobleHandleC2Subscribed(otInstance *aInstance, otTobleConnection *aConn, bool aIsSubscribed)
{
    ot::Instance &instance = *static_cast<ot::Instance *>(aInstance);

    instance.Get<ot::Toble::Platform::Callbacks>().HandleC2Subscribed(aConn, aIsSubscribed);
}

extern "C" void otPlatTobleHandleC2IndicateDone(otInstance *aInstance, otTobleConnection *aConn)
{
    ot::Instance &instance = *static_cast<ot::Instance *>(aInstance);

    instance.Get<ot::Toble::Platform::Callbacks>().HandleC2IndicateDone(aConn);
}

#endif // #if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
#endif // OPENTHREAD_CONFIG_TOBLE_ENABLE
