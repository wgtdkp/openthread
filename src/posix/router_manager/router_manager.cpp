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

#if OPENTHREAD_CONFIG_DUCKHORN_BORDER_ROUTER_ENABLE

#include <stdlib.h>

#include <netinet/in.h>

#include <openthread/border_router.h>
#include <openthread/platform/entropy.h>
#include <openthread/platform/settings.h>
#include <openthread/platform/toolchain.h>

#include "common/code_utils.hpp"
#include "common/logging.hpp"
#include "lib/platform/exit_code.h"
#include "platform/address_utils.hpp"

namespace ot
{

namespace Posix
{

static struct in6_addr kLinkLocalAllNodes = {{{ 0xff, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 }}};

static struct in6_addr kKinkLocalAllRouters = {{{ 0xff, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02 }}};

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
    , mInfraNetif(HandleInfraNetifStateChanged, this)
    , mIcmp6(HandleRouterSolicit, this, HandleRouterAdvertisement, this)
    , mRouterAdvertisementTimer(HandleRouterAdvertisementTimer, this)
    , mRouterSolicitTimer(HandleRouterSolicitTimer, this)
    , mDiscoveredOnLinkPrefixInvalidTimer(HandleDiscoveredOnLinkPrefixInvalidTimer, this)
{
    memset(&mLocalOmrPrefix, 0, sizeof(mLocalOmrPrefix));
    memset(&mAdvertisedOmrPrefix, 0, sizeof(mAdvertisedOmrPrefix));
    memset(&mLocalOnLinkPrefix, 0, sizeof(mLocalOnLinkPrefix));
    memset(&mAdvertisedOnLinkPrefix, 0, sizeof(mAdvertisedOnLinkPrefix));
    memset(&mDiscoveredOnLinkPrefix, 0, sizeof(mDiscoveredOnLinkPrefix));
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
    mInfraNetif.Update(aMainloop);
    mIcmp6.Update(aMainloop);
}

void RouterManager::Process(const otSysMainloopContext *aMainloop)
{
    mInfraNetif.Process(aMainloop);
    mIcmp6.Process(aMainloop);
}

void RouterManager::Start()
{
    SendRouterSolicit();
}

void RouterManager::Stop()
{
    // TODO(wgtdkp): per RFC 4861 6.2.5, we should send Router Advertisements
    // with a Router Lifetime field of zero.

    UnpublishOmrPrefix(mAdvertisedOmrPrefix);
    mRouterAdvertisementTimer.Stop();
    mRouterSolicitTimer.Stop();
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
            Start();
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

    SuccessOrDie(otPlatEntropyGet(&onMeshPrefix.mPrefix.mFields.m8[1], kRandomPrefixLength - 1));

    onMeshPrefix.mLength = OT_IP6_PREFIX_BITSIZE; // In bits.

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

otIp6Prefix RouterManager::EvaluateOmrPrefix()
{
    otIp6Prefix lowestOmrPrefix;
    otIp6Prefix newOmrPrefix;
    otBorderRouterConfig borderRouterConfig;
    otNetworkDataIterator iterator = OT_NETWORK_DATA_ITERATOR_INIT;
    otDeviceRole role = otThreadGetDeviceRole(&GetInstance());

    memset(&lowestOmrPrefix, 0, sizeof(lowestOmrPrefix));
    memset(&newOmrPrefix, 0, sizeof(newOmrPrefix));

    VerifyOrExit(role == OT_DEVICE_ROLE_ROUTER || role == OT_DEVICE_ROLE_LEADER);

    while (otNetDataGetNextOnMeshPrefix(&GetInstance(), &iterator, &borderRouterConfig) == OT_ERROR_NONE)
    {
        // TODO(wgtdkp): more validation on this OMR prefix
        // to see if it works for us.

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
        newOmrPrefix = lowestOmrPrefix;
    }
    else
    {
        newOmrPrefix = mLocalOmrPrefix;
    }

exit:
    return newOmrPrefix;
}

void RouterManager::PublishOmrPrefix(const otIp6Prefix &aPrefix)
{
    otError error = OT_ERROR_NONE;
    otBorderRouterConfig borderRouterConfig;

    OT_ASSERT(IsValidOmrPrefix(aPrefix));

    memset(&borderRouterConfig, 0, sizeof(borderRouterConfig));
    borderRouterConfig.mPrefix = aPrefix;
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
        otLogInfoPlat("failed to publish OMR prefix: %s", otThreadErrorToString(error));
    }
}

void RouterManager::UnpublishOmrPrefix(const otIp6Prefix &aPrefix)
{
    if (IsValidOmrPrefix(aPrefix))
    {
        otBorderRouterRemoveOnMeshPrefix(&GetInstance(), &aPrefix);
        otBorderRouterRegister(&GetInstance());
    }
}

otIp6Prefix RouterManager::EvaluateOnLinkPrefix()
{
    otIp6Prefix newOnLinkPrefix;

    memset(&newOnLinkPrefix, 0, sizeof(newOnLinkPrefix));

    // We don't evalute on-link prefix if we are doing
    // Router Discovery or we has already discovered some
    // on-link prefix.
    VerifyOrExit(!mRouterSolicitTimer.IsRunning());
    if (IsValidOnLinkPrefix(mDiscoveredOnLinkPrefix))
    {
        otLogInfoPlat("there is already on-link prefix (%s) on interface %s",
            Ip6PrefixString(mDiscoveredOnLinkPrefix).AsCString(), mInfraNetif.GetName());
        ExitNow();
    }

    if (IsValidOnLinkPrefix(mAdvertisedOnLinkPrefix))
    {
        newOnLinkPrefix = mAdvertisedOnLinkPrefix;
    }
    else
    {
        newOnLinkPrefix = mLocalOnLinkPrefix;
    }

exit:
    return newOnLinkPrefix;
}

void RouterManager::EvaluateRoutingPolicy()
{
    otIp6Prefix newOnLinkPrefix;
    otIp6Prefix newOmrPrefix;

    otLogInfoPlat("evaluating routing policy");

    newOnLinkPrefix = EvaluateOnLinkPrefix();
    newOmrPrefix =  EvaluateOmrPrefix();

    if (IsValidOnLinkPrefix(newOnLinkPrefix))
    {
        if (!IsValidOnLinkPrefix(mAdvertisedOnLinkPrefix))
        {
            otLogInfoPlat("start advertising prefix %s on interface %s",
                Ip6PrefixString(newOnLinkPrefix).AsCString(), mInfraNetif.GetName());
            mInfraNetif.AddGatewayAddress(newOnLinkPrefix);
        }
    }
    else
    {
        if (IsValidOnLinkPrefix(mAdvertisedOnLinkPrefix))
        {
            otLogInfoPlat("stop advertising prefix %s on interface %s",
                Ip6PrefixString(mAdvertisedOnLinkPrefix).AsCString(), mInfraNetif.GetName());
            mInfraNetif.RemoveGatewayAddress(mAdvertisedOnLinkPrefix);
        }
    }

    if (newOmrPrefix == mLocalOmrPrefix)
    {
        if (!IsValidOmrPrefix(mAdvertisedOmrPrefix))
        {
            otLogInfoPlat("publish new OMR prefix in Thread network");
            PublishOmrPrefix(newOmrPrefix);
        }
    }
    else
    {
        if (IsValidOmrPrefix(mAdvertisedOmrPrefix))
        {
            otLogInfoPlat("there is already OMR prefix in the Thread network, stop publishing");
            UnpublishOmrPrefix(mAdvertisedOmrPrefix);
        }
    }

    if (IsValidOnLinkPrefix(newOnLinkPrefix) || newOmrPrefix != mAdvertisedOmrPrefix)
    {
        SendRouterAdvertisement(newOmrPrefix, newOnLinkPrefix);
    }

    mAdvertisedOnLinkPrefix = newOnLinkPrefix;
    mAdvertisedOmrPrefix = newOmrPrefix;
}

void RouterManager::SendRouterSolicit()
{
    uint32_t timeout;

    mIcmp6.SendRouterSolicit(mInfraNetif, kKinkLocalAllRouters);

    // We wait a bit longer than the solicition interval.
    timeout = kRtrSolicitionInterval * 1000 + GenerateRandomNumber(0, 1000);
    mRouterSolicitTimer.Start(timeout);

    otLogInfoPlat("Router Solicit timer scheduled in %.1f s", timeout / 1000.0f);
}

void RouterManager::SendRouterAdvertisement(const otIp6Prefix &aOmrPrefix, const otIp6Prefix &aOnLinkPrefix)
{
    uint32_t nextSendTime;
    const otIp6Prefix *omrPrefix = IsValidOmrPrefix(aOmrPrefix) ? &aOmrPrefix : nullptr;
    const otIp6Prefix *onLinkPrefix = IsValidOnLinkPrefix(aOnLinkPrefix) ? &aOnLinkPrefix : nullptr;

    VerifyOrExit(omrPrefix != nullptr || onLinkPrefix != nullptr);

    if (mIcmp6.SendRouterAdvertisement(omrPrefix, onLinkPrefix, mInfraNetif, kLinkLocalAllNodes) != OT_ERROR_NONE)
    {
        otLogWarnPlat("failed to send Router Advertisement message");
    }

    ++mRouterAdvertisementCount;

    nextSendTime = GenerateRandomNumber(kMinRtrAdvInterval, kMaxRtrAdvInterval);

    if (mRouterAdvertisementCount <= kMaxInitRtrAdvertisements &&
        nextSendTime > kMaxInitRtrAdvInterval)
    {
        nextSendTime = kMaxInitRtrAdvInterval;
    }

    otLogInfoPlat("Router Advertisement scheduled in %u Seconds", nextSendTime);
    mRouterAdvertisementTimer.Start(nextSendTime * 1000);

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

void RouterManager::HandleInfraNetifStateChanged(void *aRouterManager)
{
    static_cast<RouterManager *>(aRouterManager)->HandleInfraNetifStateChanged();
}

void RouterManager::HandleInfraNetifStateChanged()
{
    if (!mInfraNetif.IsUp())
    {
        Stop();
    }
    else
    {
        EvaluateRoutingPolicy();
    }
}

void RouterManager::HandleRouterAdvertisementTimer(Timer &aTimer, void *aRouterManager)
{
    static_cast<RouterManager *>(aRouterManager)->HandleRouterAdvertisementTimer(aTimer);
}

void RouterManager::HandleRouterAdvertisementTimer(Timer &aTimer)
{
    OT_UNUSED_VARIABLE(aTimer);

    otLogInfoPlat("Router Advertisement timer triggered");

    SendRouterAdvertisement(mAdvertisedOmrPrefix, mAdvertisedOnLinkPrefix);
}

void RouterManager::HandleRouterSolicitTimer(Timer &aTimer, void *aRouterManager)
{
    static_cast<RouterManager *>(aRouterManager)->HandleRouterSolicitTimer(aTimer);
}

void RouterManager::HandleRouterSolicitTimer(Timer &aTimer)
{
    OT_UNUSED_VARIABLE(aTimer);

    otLogInfoPlat("Router Solicit timeouted");

    // We may have received Router Advertisement messages after sending
    // Router Solicit. Thus we need to re-evalute our routing policy.
    EvaluateRoutingPolicy();
}

void RouterManager::HandleDiscoveredOnLinkPrefixInvalidTimer(Timer &aTimer, void *aRouterManager)
{
    static_cast<RouterManager *>(aRouterManager)->HandleDiscoveredOnLinkPrefixInvalidTimer(aTimer);
}

void RouterManager::HandleDiscoveredOnLinkPrefixInvalidTimer(Timer &aTimer)
{
    OT_UNUSED_VARIABLE(aTimer);

    // The discovered on-link prefix becomes invalid, send Router Solicit to
    // discover new one.
    memset(&mDiscoveredOnLinkPrefix, 0, sizeof(mDiscoveredOnLinkPrefix));
    SendRouterSolicit();
}

void RouterManager::HandleRouterSolicit(unsigned int aIfIndex, void *aRouterManager)
{
    static_cast<RouterManager *>(aRouterManager)->HandleRouterSolicit(aIfIndex);
}

void RouterManager::HandleRouterSolicit(unsigned int aIfIndex)
{
    static char kIfName[IFNAMSIZ];
    char *ifName = if_indextoname(aIfIndex, kIfName);
    // We always re-evaluate our routing policy before sending
    // Router Advertisement messages.

    if (aIfIndex != mInfraNetif.GetIndex())
    {
        otLogInfoPlat("ignore Router Solicit message from interface %s", (ifName ? ifName : "UNKNOWN"));
        ExitNow();
    }

    EvaluateRoutingPolicy();

exit:
    return;
}

void RouterManager::HandleRouterAdvertisement(const Icmp6::RouterAdvMessage &aRouterAdv, unsigned int aIfIndex, void *aRouterManager)
{
    static_cast<RouterManager *>(aRouterManager)->HandleRouterAdvertisement(aRouterAdv, aIfIndex);
}

void RouterManager::HandleRouterAdvertisement(const Icmp6::RouterAdvMessage &aRouterAdv, unsigned int aIfIndex)
{
    static char kIfName[IFNAMSIZ];
    char *ifName = if_indextoname(aIfIndex, kIfName);
    const Icmp6::PrefixInfoOption *pio = nullptr;
    bool hasChanges = false;

    // TODO(wgtdkp): check if there is PIO.
    // TODO(wgtdkp): check Router Lifetime.

    if (aIfIndex != mInfraNetif.GetIndex())
    {
        otLogInfoPlat("ignore Router Advertisement message from interface %s", (ifName ? ifName : "UNKNOWN"));
        ExitNow();
    }

    while ((pio = aRouterAdv.GetNextPrefixInfo(pio)) != nullptr)
    {
        if (pio->GetPrefixLength() != OT_IP6_PREFIX_BITSIZE)
        {
            otLogInfoPlat("ignore PIO with prefix length %hu", pio->GetPrefixLength());
            continue;
        }

        if (pio->GetPrefix()[0] != 0xfd)
        {
            otLogInfoPlat("ignore PIO %s, expect prefix fd00::/8", Ip6PrefixString(pio->GetPrefix(), pio->GetPrefixLength()).AsCString());
            continue;
        }

        Ip6PrefixString prefixString {pio->GetPrefix(), pio->GetPrefixLength()};

        otLogInfoPlat("accept PIO %s, valid lifetime: %u seconds", prefixString.AsCString(), pio->GetValidLifetime());

        if (!IsValidOnLinkPrefix(mDiscoveredOnLinkPrefix) ||
            (mDiscoveredOnLinkPrefixInvalidTimer.IsRunning() &&
             (pio->GetValidLifetime() * 1000ULL + otPlatAlarmMilliGetNow()) > mDiscoveredOnLinkPrefixInvalidTimer.GetFireTime()))
        {
            memcpy(&mDiscoveredOnLinkPrefix.mPrefix, pio->GetPrefix(), pio->GetPrefixLength() / 8);
            mDiscoveredOnLinkPrefix.mLength = pio->GetPrefixLength();

            otLogInfoPlat("set discovered on-link prefix to %s, valid lifetime: %u seconds",
                          prefixString.AsCString(), pio->GetValidLifetime());

           if (pio->GetValidLifetime() == Icmp6::kInfiniteLifetime)
           {
               mDiscoveredOnLinkPrefixInvalidTimer.Stop();
           }
           else
           {
               // FIXME(wgtdkp): integer overflow.
               mDiscoveredOnLinkPrefixInvalidTimer.Start(pio->GetValidLifetime() * 1000);
           }

            mRouterSolicitTimer.Stop();
            hasChanges = true;
        }
    }

    if (hasChanges)
    {
        EvaluateRoutingPolicy();
    }

exit:
    return;
}

uint32_t RouterManager::GenerateRandomNumber(uint32_t aBegin, uint32_t aEnd)
{
    uint64_t rand;

    OT_ASSERT(aBegin <= aEnd);

    SuccessOrDie(otPlatEntropyGet(reinterpret_cast<uint8_t *>(&rand), sizeof(rand)));

    return static_cast<uint32_t>(aBegin + rand % (aEnd - aBegin + 1));
}

} // namespace Posix

} // namespace ot

#endif // OPENTHREAD_CONFIG_DUCKHORN_BORDER_ROUTER_ENABLE
