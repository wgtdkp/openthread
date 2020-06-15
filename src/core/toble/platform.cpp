/*
 *  Copyright (c) 2019, The OpenThread Authors.
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

//---------------------------------------------------------------------------------------------------------------------
// Address

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

void Platform::Callbacks::HandleConnected(ConnectionId aConnId)
{
    Get<Toble>().HandleConnected(aConnId);
}

void Platform::Callbacks::HandleDisconnected(ConnectionId aConnId)
{
    Get<Toble>().HandleDisconnected(aConnId);
}

#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE

void Platform::Callbacks::HandleAdv(AdvType        aAdvType,
                                    const Address &aSource,
                                    const uint8_t *aData,
                                    uint16_t       aLength,
                                    int8_t         aRssi)
{
    Get<Central::Controller>().HandleAdv(aAdvType, aSource, aData, aLength, aRssi);
}

void Platform::Callbacks::HandleConnectionReady(ConnectionId aConnId, otTobleConnectionLinkType aLinkType)
{
    switch (aLinkType)
    {
    case kConnectionLinkTypeGatt:
        Get<Btp>().HandleConnectionReady(aConnId);
        break;
    default:
        otLogCritBle("Unsupported link type");
        break;
    }
}

void Platform::Callbacks::HandleC1WriteDone(ConnectionId aConnId)
{
    Get<Btp>().HandleC1WriteDone(aConnId);
}

void Platform::Callbacks::HandleC2Indication(ConnectionId aConnId, const uint8_t *aBuf, uint16_t aLength)
{
    Get<Btp>().HandleC2Indication(aConnId, aBuf, aLength);
}

#endif // OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE

#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE

void Platform::Callbacks::HandleC2Subscribed(ConnectionId aConnId, bool aIsSubscribed)
{
    Get<Btp>().HandleC2Subscribed(aConnId, aIsSubscribed);
}

void Platform::Callbacks::HandleC2IndicateDone(ConnectionId aConnId)
{
    Get<Btp>().HandleC2IndicateDone(aConnId);
}

void Platform::Callbacks::HandleC1Write(ConnectionId aConnId, const uint8_t *aFrame, uint16_t aLength)
{
    Get<Btp>().HandleC1Write(aConnId, aFrame, aLength);
}

#endif // OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE

} // namespace Toble
} // namespace ot

extern "C" void otPlatTobleHandleConnected(otInstance *aInstance, otTobleConnectionId aConnId)
{
    ot::Instance &instance = *static_cast<ot::Instance *>(aInstance);

    instance.Get<ot::Toble::Platform::Callbacks>().HandleConnected(aConnId);
}

extern "C" void otPlatTobleHandleDisconnected(otInstance *aInstance, otTobleConnectionId aConnId)
{
    ot::Instance &instance = *static_cast<ot::Instance *>(aInstance);

    instance.Get<ot::Toble::Platform::Callbacks>().HandleDisconnected(aConnId);
}

#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE

extern "C" void otPlatTobleHandleAdv(otInstance *          aInstance,
                                     otTobleAdvType        aAdvType,
                                     const otTobleAddress *aSource,
                                     const uint8_t *       aData,
                                     uint16_t              aLength,
                                     int8_t                aRssi)
{
    ot::Instance &instance = *static_cast<ot::Instance *>(aInstance);

    instance.Get<ot::Toble::Platform::Callbacks>().HandleAdv(
        aAdvType, *static_cast<const ot::Toble::Address *>(aSource), aData, aLength, aRssi);
}

extern "C" void otPlatTobleHandleConnectionIsReady(otInstance *              aInstance,
                                                   otTobleConnectionId       aConnId,
                                                   otTobleConnectionLinkType aLinkType)
{
    ot::Instance &instance = *static_cast<ot::Instance *>(aInstance);
    instance.Get<ot::Toble::Platform::Callbacks>().HandleConnectionReady(aConnId, aLinkType);
}

extern "C" void otPlatTobleHandleC1WriteDone(otInstance *aInstance, otTobleConnectionId aConnId)
{
    ot::Instance &instance = *static_cast<ot::Instance *>(aInstance);

    instance.Get<ot::Toble::Platform::Callbacks>().HandleC1WriteDone(aConnId);
}

extern "C" void otPlatTobleHandleC2Indication(otInstance *        aInstance,
                                              otTobleConnectionId aConnId,
                                              const uint8_t *     aFrame,
                                              uint16_t            aLength)
{
    ot::Instance &instance = *static_cast<ot::Instance *>(aInstance);

    instance.Get<ot::Toble::Platform::Callbacks>().HandleC2Indication(aConnId, aFrame, aLength);
}

#endif // OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE

#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE

extern "C" void otPlatTobleHandleC1Write(otInstance *        aInstance,
                                         otTobleConnectionId aConnId,
                                         const uint8_t *     aFrame,
                                         uint16_t            aLength)
{
    ot::Instance &instance = *static_cast<ot::Instance *>(aInstance);

    instance.Get<ot::Toble::Platform::Callbacks>().HandleC1Write(aConnId, aFrame, aLength);
}

extern "C" void otPlatTobleHandleC2Subscribed(otInstance *aInstance, otTobleConnectionId aConnId, bool aIsSubscribed)
{
    ot::Instance &instance = *static_cast<ot::Instance *>(aInstance);

    instance.Get<ot::Toble::Platform::Callbacks>().HandleC2Subscribed(aConnId, aIsSubscribed);
}

extern "C" void otPlatTobleHandleC2IndicateDone(otInstance *aInstance, otTobleConnectionId aConnId)
{
    ot::Instance &instance = *static_cast<ot::Instance *>(aInstance);

    instance.Get<ot::Toble::Platform::Callbacks>().HandleC2IndicateDone(aConnId);
}

#endif // OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE

#endif // OPENTHREAD_CONFIG_TOBLE_ENABLE
