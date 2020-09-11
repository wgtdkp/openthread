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

#include "openthread-android-config.h"
#include "platform-android.h"

#include <openthread/platform/toble.h>

#include "common/code_utils.hpp"

#include "jni/toble.hpp"
#include "jni/toble_driver.hpp"

static ot::Toble::TobleDriver *GetTobleDriver() {
    return ot::Toble::Toble::GetInstance()->GetTobleDriver();
}

void otPlatTobleInit(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    GetTobleDriver()->Init();
}

void otPlatTobleDisconnect(otInstance *aInstance, otTobleConnection *aConn)
{
    OT_UNUSED_VARIABLE(aInstance);
    GetTobleDriver()->Disconnect(reinterpret_cast<ot::Toble::TobleConnection*>(aConn));
}

uint16_t otPlatTobleGetMtu(otInstance *aInstance, otTobleConnection *aConn)
{
    OT_UNUSED_VARIABLE(aInstance);
    return GetTobleDriver()->GetMtu(reinterpret_cast<ot::Toble::TobleConnection*>(aConn));
}

otError otPlatTobleScanStart(otInstance *aInstance, uint16_t aInterval, uint16_t aWindow, bool aActive)
{
    OT_UNUSED_VARIABLE(aInstance);
    return GetTobleDriver()->ScanStart(aInterval, aWindow, aActive);
}

otError otPlatTobleScanStop(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return GetTobleDriver()->ScanStop();
}

otTobleConnection *otPlatTobleCreateConnection(otInstance *             aInstance,
                                               const otTobleAddress *   aPeerAddress,
                                               otTobleConnectionConfig *aConfig)
{
    OT_UNUSED_VARIABLE(aInstance);
    return GetTobleDriver()->CreateConnection(aPeerAddress, aConfig);
}

otError otPlatTobleC1Write(otInstance *aInstance, otTobleConnection *aConn, const void *aBuffer, uint16_t aLength)
{
    OT_UNUSED_VARIABLE(aInstance);
    return GetTobleDriver()->C1Write(
        reinterpret_cast<ot::Toble::TobleConnection*>(aConn),
        reinterpret_cast<const uint8_t *>(aBuffer),
        aLength);
}

void otPlatTobleC2Subscribe(otInstance *aInstance, otTobleConnection *aConn, bool aSubscribe)
{
    OT_UNUSED_VARIABLE(aInstance);
    GetTobleDriver()->C2Subscribe(reinterpret_cast<ot::Toble::TobleConnection*>(aConn), aSubscribe);
}

otError otPlatTobleAdvStart(otInstance *aInstance, const otTobleAdvConfig *aConfig)
{
    OT_UNUSED_VARIABLE(aInstance);
    return GetTobleDriver()->AdvStart(aConfig);
}

otError otPlatTobleAdvStop(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return GetTobleDriver()->AdvStop();
}

otError otPlatTobleC2Notificate(otInstance *aInstance, otTobleConnection *aConn, const void *aBuffer, uint16_t aLength)
{
    OT_UNUSED_VARIABLE(aInstance);
    return GetTobleDriver()->C2Notificate(
        reinterpret_cast<ot::Toble::TobleConnection*>(aConn),
        reinterpret_cast<const uint8_t *>(aBuffer),
        aLength);
}

void platformTobleProcess()
{
    GetTobleDriver()->Process();
}

namespace ot
{

namespace Toble
{

void TobleDriver::OnAdvReceived(otTobleAdvType aAdvType, otTobleAdvPacket *aAdvPacket)
{
    //otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_TOBLE, "::OnAdvReceived");
    otPlatTobleGapOnAdvReceived(Toble::GetInstance()->GetOtInstance(), aAdvType, aAdvPacket);
}

void TobleDriver::OnConnected(TobleConnection *aConn)
{
    otPlatTobleHandleConnected(Toble::GetInstance()->GetOtInstance(), aConn);
}

void TobleDriver::OnDisconnected(TobleConnection *aConn)
{
    otPlatTobleHandleDisconnected(Toble::GetInstance()->GetOtInstance(), aConn);
}

void TobleDriver::OnConnectionIsReady(TobleConnection *aConn)
{
    otPlatTobleHandleConnectionIsReady(Toble::GetInstance()->GetOtInstance(), aConn, kConnectionLinkTypeGatt);
}

void TobleDriver::OnC1WriteDone(TobleConnection *aConn)
{
    otPlatTobleHandleC1WriteDone(Toble::GetInstance()->GetOtInstance(), aConn);
}

void TobleDriver::OnC2Subscribed(TobleConnection *aConn, bool aIsSubscribed)
{
    otPlatTobleHandleC2Subscribed(Toble::GetInstance()->GetOtInstance(), aConn, aIsSubscribed);
}

void TobleDriver::OnC2Notification(TobleConnection *aConn,
                                   const uint8_t *  aBuffer,
                                   uint16_t         aLength)
{
    otPlatTobleHandleC2Notification(Toble::GetInstance()->GetOtInstance(), aConn, aBuffer, aLength);
}

} // namespace Toble

} // namespace ot
