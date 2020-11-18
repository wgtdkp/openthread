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

#include "router_manager.hpp"

#include <time.h>
#include <stdlib.h>

#include <openthread/border_router.h>
#include <openthread/platform/settings.h>
#include <openthread/platform/toolchain.h>

#include "common/code_utils.hpp"
#include "common/logging.hpp"

namespace ot
{

namespace Posix
{

static struct in6_addr kLinkLocalAllNodes = {{{ 0xff, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 }}};

//static struct in6_addr kKinkLocalAllRouters = {{{ 0xff, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//                                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02 }}};

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

RouterManager::RouterManager(Instance &aInstance)
    : InstanceLocator(aInstance)
{
    memset(&mLocalOmrPrefix, 0, sizeof(mLocalOmrPrefix));
    memset(&mAdvertisedOmrPrefix, 0, sizeof(mAdvertisedOmrPrefix));
}

void RouterManager::Init(const char *aInfraNetifName)
{
    otError error = OT_ERROR_NONE;
    uint16_t omrPrefixLength;

    SuccessOrExit(error = mIcmp6.Init());
    SuccessOrExit(error = mInfraNetif.Init(aInfraNetifName));
    SuccessOrExit(error = otSetStateChangedCallback(&GetInstance(), HandleStateChanged, this));

    if (otPlatSettingsGet(&GetInstance(), kKeyOmrPrefix, 0,
                          reinterpret_cast<uint8_t *>(&mLocalOmrPrefix),
                          &omrPrefixLength) != OT_ERROR_NONE ||
        omrPrefixLength != sizeof(mLocalOmrPrefix) ||
        !IsValidOmrPrefix(mLocalOmrPrefix))
    {
        // TOOD(wgtdkp): dump the OMR prefix.
        otLogInfoPlat("no valid OMR prefix in sotrage, generate new random OMR prefix");

        mLocalOmrPrefix = GenerateRandomOmrPrefix();
        mLocalOnLinkPrefix = mLocalOmrPrefix;
        mLocalOnLinkPrefix.mPrefix.mFields.m8[6] = (mInfraNetif.GetIndex() & 0xff00) >> 8;
        mLocalOnLinkPrefix.mPrefix.mFields.m8[7] = (mInfraNetif.GetIndex() & 0x00ff);

        if (otPlatSettingsSet(&GetInstance(), kKeyOmrPrefix,
                    reinterpret_cast<uint8_t *>(&mLocalOmrPrefix),
                    static_cast<uint16_t>(sizeof(mLocalOmrPrefix))) != OT_ERROR_NONE)
        {
            otLogWarnPlat("failed to save the random OMR prefix");
        }
    }

exit:
    SuccessOrLog(error, "failed to initialize Router Manager");
}

void RouterManager::Deinit()
{
    otRemoveStateChangeCallback(&GetInstance(), HandleStateChanged, this);
    mInfraNetif.Deinit();
    mIcmp6.Deinit();
}

void RouterManager::Update(otSysMainloopContext *aMainloop)
{
    OT_UNUSED_VARIABLE(aMainloop);
    // TODO(wgtdkp):
}

void RouterManager::Process(const otSysMainloopContext *aMainloop)
{
    OT_UNUSED_VARIABLE(aMainloop);
    // TODO(wgtdkp):
}

otError RouterManager::Start()
{
    otError error = OT_ERROR_NONE;

    EvaluateRoutingPolicy();

    // TODO(wgtdkp): Send Route Solicit messages to discovery on-link prefix.
    SuccessOrExit(error = SendRouterSolicit());

exit:
    return error;
}

void RouterManager::Stop()
{
    // TODO(wgtdkp):
}

void RouterManager::HandleStateChanged(otChangedFlags aFlags, void *aRouterManager)
{
    static_cast<RouterManager *>(aRouterManager)->HandleStateChanged(aFlags);
}

void RouterManager::HandleStateChanged(otChangedFlags aFlags)
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
        otDeviceRole role = otThreadGetDeviceRole(&GetInstance());
        if (role == OT_DEVICE_ROLE_ROUTER || role == OT_DEVICE_ROLE_LEADER)
        {
            EvaluateRoutingPolicy();
        }
    }
}

otIp6Prefix RouterManager::GenerateRandomOmrPrefix()
{
    static const uint8_t kRandomPrefixLength = 6; // In bytes.
    otIp6Prefix onMeshPrefix;

    memset(&onMeshPrefix, 0, sizeof(onMeshPrefix));

    onMeshPrefix.mPrefix.mFields.m8[0] = 0xfd;

    srand(time(NULL));
    for (uint8_t i = 1; i < kRandomPrefixLength; ++i)
    {
        onMeshPrefix.mPrefix.mFields.m8[i] = rand() % (UINT8_MAX + 1);
    }
    onMeshPrefix.mLength = 64; // In bits.

    return onMeshPrefix;
}

/**
 * Compares two prefixes byte by byte.
 *
 * The rules:
 *  1. prefix A is lower than B if it has a smaller length.
 *  2. prefix A is lower than B if we have A[i] < B[i] when
 *     iterates from left to right.
 *
 */
static bool operator<(const otIp6Prefix &aLhs, const otIp6Prefix &aRhs)
{
    bool ret = true;

    VerifyOrExit(aLhs.mLength == aRhs.mLength, ret = (aLhs.mLength < aRhs.mLength));
    for (uint8_t i = 0; i * 8 < aLhs.mLength; ++i)
    {
        if (aLhs.mPrefix.mFields.m8[8] < aRhs.mPrefix.mFields.m8[8])
        {
            ExitNow(ret = true);
        }
    }

    ret = false;

exit:
    return ret;
}

static bool operator>(const otIp6Prefix &aLhs, const otIp6Prefix &aRhs)
{
    return (aRhs < aLhs);
}

static bool operator==(const otIp6Prefix &aLhs, const otIp6Prefix &aRhs)
{
    return !(aLhs < aRhs) && !(aLhs > aRhs);
}

static bool operator!=(const otIp6Prefix &aLhs, const otIp6Prefix &aRhs)
{
    return !(aLhs == aRhs);
}

void RouterManager::EvaluateOmrPrefix()
{
    otIp6Prefix lowestOmrPrefix;
    otBorderRouterConfig borderRouterConfig;
    otNetworkDataIterator iterator = OT_NETWORK_DATA_ITERATOR_INIT;
    otDeviceRole role = otThreadGetDeviceRole(&GetInstance());

    memset(&lowestOmrPrefix, 0, sizeof(lowestOmrPrefix));

    VerifyOrExit(role == OT_DEVICE_ROLE_ROUTER || role == OT_DEVICE_ROLE_LEADER);

    while (otNetDataGetNextOnMeshPrefix(&GetInstance(), &iterator, &borderRouterConfig) == OT_ERROR_NONE)
    {
        if (!borderRouterConfig.mDefaultRoute || !borderRouterConfig.mSlaac)
        {
            continue;
        }
        if (!IsValidOmrPrefix(borderRouterConfig.mPrefix))
        {
            continue;
        }

        if (!IsValidOmrPrefix(lowestOmrPrefix) || borderRouterConfig.mPrefix < lowestOmrPrefix)
        {
            lowestOmrPrefix = borderRouterConfig.mPrefix;
        }
    }

    if (IsValidOmrPrefix(lowestOmrPrefix))
    {
        // TODO(wgtdkp): more validation on this OMR prefix
        // to see if it works for us (e.g. check if SLAAC enabled or
        // we have an address with that prefix).
        otLogInfoPlat("there is already an OMR prefix");

        if (mAdvertisedOmrPrefix != lowestOmrPrefix)
        {
            mAdvertisedOmrPrefix = lowestOmrPrefix;
        }
    }
    else
    {
        if (!IsValidOmrPrefix(mLocalOmrPrefix))
        {
           mLocalOmrPrefix = GenerateRandomOmrPrefix();
           // TODO(wgtdkp): save the prefix to settings.
        }

        if (mAdvertisedOmrPrefix != mLocalOmrPrefix)
        {
            mAdvertisedOmrPrefix = mLocalOmrPrefix;
            PublishOmrPrefix();
        }
    }

exit:
    return;
}

void RouterManager::PublishOmrPrefix()
{
    otError error = OT_ERROR_NONE;
    otBorderRouterConfig borderRouterConfig;

    OT_ASSERT(IsValidOmrPrefix(mAdvertisedOmrPrefix));

    // TODO(wgtdkp): dump the prefix.
    otLogInfoPlat("publishing on-mesh prefix");

    memset(&borderRouterConfig, 0, sizeof(borderRouterConfig));
    borderRouterConfig.mPrefix = mLocalOmrPrefix;
    borderRouterConfig.mStable = true;
    borderRouterConfig.mSlaac = true;
    borderRouterConfig.mPreferred = true;
    borderRouterConfig.mOnMesh = true;
    borderRouterConfig.mDefaultRoute = true;

    SuccessOrExit(error = otBorderRouterAddOnMeshPrefix(&GetInstance(), &borderRouterConfig));
    SuccessOrExit(error = otBorderRouterRegister(&GetInstance()));

exit:
    if (error != OT_ERROR_NONE)
    {
        otLogInfoPlat("failed to publish on-mesh prefix: %s", otThreadErrorToString(error));
    }
}

void RouterManager::EvaluateOnLinkPrefix()
{
    if (mInfraNetif.HasUlaOrGuaAddress())
    {
        otLogInfoPlat("there is already IPv6 network on interface %s", mInfraNetif.GetName());
        memset(&mAdvertisedOnLinkPrefix, 0, sizeof(mAdvertisedOnLinkPrefix));
    }
    else
    {
        if (mAdvertisedOnLinkPrefix != mLocalOnLinkPrefix)
        {
            mAdvertisedOnLinkPrefix = mLocalOnLinkPrefix;
        }
    }
}

void RouterManager::EvaluateRoutingPolicy()
{
    otLogInfoPlat("evaluating routing policy");

    EvaluateOmrPrefix();
    EvaluateOnLinkPrefix();


    SendRouterAdvertisement(mAdvertisedOmrPrefix, mAdvertisedOnLinkPrefix);
}

otError RouterManager::SendRouterSolicit()
{
    otError error = OT_ERROR_NONE;
    // TODO(wgtdkp): Send.

    // TODO(wgtdkp): Start a Router Solicit timer. Send
    // Router Advertisement messages when timeouted.

    return error;
}

void RouterManager::SendRouterAdvertisement(const otIp6Prefix &aOmrPrefix, const otIp6Prefix &aOnLinkPrefix)
{
    const otIp6Prefix *omrPrefix = IsValidOmrPrefix(aOmrPrefix) ? &aOmrPrefix : nullptr;
    const otIp6Prefix *onLinkPrefix = IsValidOnLinkPrefix(aOnLinkPrefix) ? &aOnLinkPrefix : nullptr;

    VerifyOrExit(omrPrefix != nullptr || onLinkPrefix != nullptr);

    if (mIcmp6.SendRouterAdvertisement(omrPrefix, onLinkPrefix, mInfraNetif, kLinkLocalAllNodes) != OT_ERROR_NONE)
    {
        otLogWarnPlat("failed to send Router Advertisement message");
    }

exit:
    return;
}

bool RouterManager::IsValidOmrPrefix(const otIp6Prefix &aPrefix)
{
    bool isValid = false;

    VerifyOrExit(aPrefix.mLength == OT_IP6_PREFIX_BITSIZE);
    VerifyOrExit(aPrefix.mPrefix.mFields.m8[0] == 0xfd);

    isValid = true;

exit:
    return isValid;
}

bool RouterManager::IsValidOnLinkPrefix(const otIp6Prefix &aPrefix)
{
    return IsValidOmrPrefix(aPrefix);
}

} // namespace Posix

} // namespace ot
