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

#include "border_router.hpp"

#include <openthread/platform/toolchain.h>

#include "common/code_utils.hpp"
#include "common/logging.hpp"

namespace ot
{

namespace Posix
{

#define SuccessOrLog(aError, aMessage)          \
    do                                          \
    {                                           \
        LogIfError(aError, __func__, aMessage); \
    }                                           \
    while (false)

static void LogIfError(otError aError, const char *aFuncName, const char *aMessage)
{
    OT_UNUSED_VARIABLE(aFuncName);
    OT_UNUSED_VARIABLE(aMessage);

    if (aError != OT_ERROR_NONE)
    {
        otLogWarnPlat("%s: %s: %s", aFuncName, aMessage, otThreadErrorToString(aError));
    }
}

BorderRouter::BorderRouter(otInstance *aInstance)
    : InstanceLocator(*static_cast<Instance *>(aInstance))
    , mHasBorderRouterConfig(false)
{
    memset(&mBorderRouterConfig, 0, sizeof(mBorderRouterConfig));
}

otError BorderRouter::Init()
{
    otError error = OT_ERROR_NONE;

    SuccessOrExit(error = otSetStateChangedCallback(&GetInstance(), HandleStateChanged, this));

exit:
    return error;
}

void BorderRouter::Deinit()
{
    // TODO(wgtdkp):
}

void BorderRouter::Update(otSysMainloopContext *aMainLoop)
{
    OT_UNUSED_VARIABLE(aMainLoop);
    // TODO(wgtdkp):
}

void BorderRouter::Process(const otSysMainloopContext *aMainLoop)
{
    OT_UNUSED_VARIABLE(aMainLoop);
    // TODO(wgtdkp):
}

otError BorderRouter::Start()
{
    otError error = OT_ERROR_NONE;

    PublishOnMeshPrefixIfNoneExisting();

    // TODO(wgtdkp): Send Route Solicit messages to discovery on-link prefix.
    SuccessOrExit(error = SendRouterSolicit());

exit:
    return error;
}

void BorderRouter::Stop()
{
    // TODO(wgtdkp):
}

void BorderRouter::HandleStateChanged(otChangedFlags aFlags, void *aBorderRouter)
{
    static_cast<BorderRouter *>(aBorderRouter)->HandleStateChanged(aFlags);
}

void BorderRouter::HandleStateChanged(otChangedFlags aFlags)
{
    if (aFlags & OT_CHANGED_THREAD_ROLE)
    {
        otDeviceRole role = otThreadGetDeviceRole(&GetInstance());
        if (role == OT_DEVICE_ROLE_ROUTER || role == OT_DEVICE_ROLE_LEADER)
        {
            SuccessOrLog(Start(), "failed to start the Border Router");
        }
        else
        {
            Stop();
        }
    }

    if (aFlags & OT_CHANGED_THREAD_NETDATA)
    {
        // TODO(wgtdkp): Check if there is already an on-mesh prefix.
    }
}

void BorderRouter::PublishOnMeshPrefixIfNoneExisting()
{
    // TODO(wgtdkp):
}

void BorderRouter::PublishOnMeshPrefix()
{
    // TODO(wgtdkp):
}

otError BorderRouter::SendRouterSolicit()
{
    otError error = OT_ERROR_NONE;
    // TODO(wgtdkp): Send.

    // TODO(wgtdkp): Start a Router Solicit timer. Send
    // Router Advertisement messages when timeouted.

    return error;
}

otError BorderRouter::SendRouterAdvertisement()
{
    otError error = OT_ERROR_NONE;

    // TODO(wgtdkp):

    return error;
}

} // namespace Posix

} // namespace ot
