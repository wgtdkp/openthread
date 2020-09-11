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

#include <vector>

#include <assert.h>

#include <openthread/ip6.h>
#include <openthread/link.h>
#include <openthread/message.h>
#include <openthread/tasklet.h>
#include <openthread/toble.h>

#include "common/code_utils.hpp"

#include "jni/toble.hpp"

namespace ot
{

namespace Toble
{

static const uint16_t kChipUdpSrcPort = 11095;
static const uint16_t kChipUdpDstPort = 11095;

void Toble::Ip6ReceiveCallback(otMessage *aMessage, void *aContext) {
    Toble *toble = reinterpret_cast<Toble*>(aContext);
    uint16_t length = otMessageGetLength(aMessage);

    std::vector<uint8_t> packet(length, 0);

    length = otMessageRead(aMessage, 0, packet.data(), length);
    packet.resize(length);

    if (toble->mTobleCallbacks != nullptr) {
        toble->mTobleCallbacks->OnIp6Receive(packet.data(), length);
    }
}

Toble* Toble::GetInstance()
{
    static Toble *sTobleInstance = new Toble();
    return sTobleInstance;
}

std::string Toble::Init(TobleCallbacks *aTobleCallbacks, TobleDriver *aTobleDriver)
{
    otError error = OT_ERROR_NONE;
    std::string result;

    assert(mInstance == nullptr);

    mTobleCallbacks = aTobleCallbacks;
    mTobleDriver = aTobleDriver;

    platformAlarmInit(1);
    platformRandomInit();

    mInstance = otInstanceInitSingle();
    assert(mInstance != nullptr);

    otIp6SetReceiveCallback(mInstance, Ip6ReceiveCallback, this);

    // Set in BLE central mode.
    SuccessOrExit(error = otTobleSetLinkMode(mInstance, OT_TOBLE_LINK_MODE_CENTRAL));

    SuccessOrExit(error = otLinkSetPanId(mInstance, 1));

    SuccessOrExit(error = otIp6AddUnsecurePort(mInstance, kChipUdpSrcPort));

    // Ifconfig up
    SuccessOrExit(error = otIp6SetEnabled(mInstance, true));

    for (auto addr = otIp6GetUnicastAddresses(mInstance); addr != nullptr; addr = addr->mNext)
    {
        const uint8_t *fields = addr->mAddress.mFields.m8;
        if (fields[0] == 0xfe && fields[1] == 0x80)
        {
            char buf[41];
            sprintf(buf,
                    "%02x%02x:%02x%02x:%02x%02x:%02x%02x:"
                    "%02x%02x:%02x%02x:%02x%02x:%02x%02x",
                    fields[0], fields[1], fields[2], fields[3],
                    fields[4], fields[5], fields[6], fields[7],
                    fields[8], fields[9], fields[10], fields[11],
                    fields[12], fields[13], fields[14], fields[15]);
            result = buf;
        }
    }

exit:
    if (error != OT_ERROR_NONE)
    {
        otPlatLog(OT_LOG_LEVEL_CRIT, OT_LOG_REGION_TOBLE, "failed to initialize ToBLE: %s", otThreadErrorToString(error));
    }
    return result;
}

void Toble::Deinit()
{
    // TODO(wgtdkp):

    if (mInstance != nullptr)
    {
        otInstanceFinalize(mInstance);
        mInstance = nullptr;
    }

    mTobleDriver = nullptr;
    mTobleCallbacks = nullptr;
}

uint32_t Toble::Process()
{
    assert(mInstance != nullptr);
    otTaskletsProcess(mInstance);

    // TODO(wgtdkp): process drivers
    platformAlarmProcess(mInstance);
    platformTobleProcess();
    return 0;
}

otError Toble::Ip6Send(const uint8_t *aPacket, size_t aPacketLength)
{
    otError error = OT_ERROR_NONE;
    otMessage *message = nullptr;

    otMessageSettings messageSettings = {false, OT_MESSAGE_PRIORITY_NORMAL};

    message =otIp6NewMessageFromBuffer(mInstance, aPacket, aPacketLength, &messageSettings);
    VerifyOrExit(message != NULL, error = OT_ERROR_NO_BUFS);

    SuccessOrExit(error = otIp6Send(mInstance, message));

exit:
    return error;
}

TobleDriver* Toble::GetTobleDriver()
{
    return mTobleDriver;
}

otInstance *Toble::GetOtInstance()
{
    return mInstance;
}

} // namespace Toble

} // namespace ot
