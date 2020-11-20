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

#ifndef POSIX_ROUTER_MANAGER_HPP_
#define POSIX_ROUTER_MANAGER_HPP_

#include "openthread-posix-config.h"

#include <openthread/error.h>
#include <openthread/instance.h>
#include <openthread/ip6.h>
#include <openthread/netdata.h>

#include <openthread/openthread-system.h>

#include "common/instance.hpp"
#include "common/locator.hpp"
#include "common/locator-getters.hpp"

#include "icmp6.hpp"
#include "timer.hpp"

namespace ot
{

namespace Posix
{

class RouterManager : public InstanceLocator
{
public:
    RouterManager(Instance &aInstance);

    /**
     * Initializes the Router Manager with given infrastructure
     * network interface.
     *
     * @param[in]  aInfraNetifName  The infrastructure network interface
     *                              the Border Router will work on.
     *
     */
    void Init(const char *aInfraNetifName);

    /**
     * Deinitializes the Router Manager.
     *
     */
    void Deinit();

    /**
     * Updates the mainloop context.
     *
     */
    void Update(otSysMainloopContext *aMainloop);

    /**
     * Processes mainloop events.
     *
     */
    void Process(const otSysMainloopContext *aMainloop);

private:
    static constexpr uint16_t kKeyOmrPrefix = 0xFF01;

    static constexpr uint32_t kMinRtrAdvInterval = 30;     // In Seconds.
    static constexpr uint32_t kMaxRtrAdvInterval = 1800;   // In Seconds.
    static constexpr uint32_t kMaxInitRtrAdvInterval = 16; // In Seconds.
    static constexpr uint32_t kMaxInitRtrAdvertisements = 3;
    static constexpr uint32_t kRtrSolicitionInterval = 4; // In Seconds.

    /**
     * Start the Border Router functionality.
     *
     * Called when current device becomes a Router or Leader.
     *
     */
    void Start();

    /**
     * Stop acting as a Border Router.
     *
     */
    void Stop();

    /**
     * Thread network events handler.
     *
     */
    static void HandleStateChanged(otChangedFlags aFlags, void *aRouterManager);
    void HandleStateChanged(otChangedFlags aFlags);

    /**
     * Generates random OMR prefix.
     *
     */
    static otIp6Prefix GenerateRandomOmrPrefix();

    /**
     * Evaluate the routing policy depends on prefix and route informations
     * on Thread Network and infra link. As a result, this method May send
     * RA messages on infra link and publish OMR prefix in the Thread Network.
     *
     * @see PublishOnMeshPrefix
     *
     */
    void EvaluateRoutingPolicy();

    /**
     * Evaluates the OMR prefix for the Thread Network.
     *
     */
    void EvaluateOmrPrefix();

    /**
     * Evaluates the on-link prefix for the infra link.
     *
     */
    void EvaluateOnLinkPrefix();


    /**
     * Publish OMR prefix in Thread Network.
     *
     * Called by `EvaluateOmrPrefix`.
     *
     */
    void PublishOmrPrefix();

    /**
     * Unpublish OMR prefix if we have done that.
     *
     */
    void UnpublishOmrPrefix();

    /**
     * Send Router Solicit messages to discovery on-link prefix
     * on infra links.
     *
     * @see HandleRouterAdvertisement
     *
     */
    void SendRouterSolicit();

    /**
     * Send Router Advertisement messages to advertise
     * on-link prefix and route for OMR prefix.
     *
     */
    void SendRouterAdvertisement(const otIp6Prefix &aOmrPrefix, const otIp6Prefix &aOnLinkPrefix);

    /**
     * Decides if given prefix is a valid OMR prefix.
     *
     */
    static bool IsValidOmrPrefix(const otIp6Prefix &aPrefix);

    /**
     * Decides if given prefix is a valid on-link prefix.
     *
     */
    static bool IsValidOnLinkPrefix(const otIp6Prefix &aPrefix);

    static void HandleInfraNetifStateChanged(void *aRouterManager);
    void HandleInfraNetifStateChanged();

    static void HandleRouterAdvertisementTimer(Timer &aTimer, void *aRouterManager);
    void HandleRouterAdvertisementTimer(Timer &aTimer);

    static void HandleRouterSolicitTimer(Timer &aTimer, void *aRouterManager);
    void HandleRouterSolicitTimer(Timer &aTimer);

    static void HandleRouterSolicit(void *aRouterManager);
    void HandleRouterSolicit();

    static uint32_t GenerateRandomNumber(uint32_t aBegin, uint32_t aEnd);


    /**
     * The OMR prefix loaded from local persistent storage.
     *
     */
    otIp6Prefix mLocalOmrPrefix;

    /**
     * The OMR prefix selected to be advertised.
     *
     */
    otIp6Prefix mAdvertisedOmrPrefix;

    /**
     * The on-link prefix created based on the local OMR prefix.
     *
     */
    otIp6Prefix mLocalOnLinkPrefix;

    /**
     * The on-link prefix selected to be advertised.
     *
     */
    otIp6Prefix mAdvertisedOnLinkPrefix;

    InfraNetif  mInfraNetif;
    Icmp6       mIcmp6;

    Timer mRouterAdvertisementTimer;
    uint32_t mRouterAdvertisementCount;

    Timer mRouterSolicitTimer;
};

} // namespace Posix

} // namespace ot

#endif // POSIX_ROUTER_MANAGER_HPP_
