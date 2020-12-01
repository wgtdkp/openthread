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
 *   This file includes definitions for the RA-based routing management.
 *
 */

#ifndef ROUTING_MANAGER_HPP_
#define ROUTING_MANAGER_HPP_

#include "openthread-core-config.h"

#if OPENTHREAD_CONFIG_BORDER_ROUTING_ENABLE

#include <openthread/error.h>
#include <openthread/platform/infra_if.h>

#include "common/locator.hpp"

#include "border_router/router_advertisement.hpp"
#include "common/notifier.hpp"
#include "common/timer.hpp"
#include "net/ip6.hpp"

namespace ot {

/**
 * This class implements bi-directional routing between Thread and
 * Infrastructure networks.
 *
 * The routing manager works on both Thread interface and infrastructure
 * interface. All ICMPv6 messages are sent/recv on the infrastructure
 * interface.
 *
 */
class RoutingManager : public InstanceLocator
{
    friend class ot::Notifier;

public:
    /**
     * This constructor initializes the routing manager.
     *
     * @param[in]  aInstance  A OpenThread instance.
     *
     */
    explicit RoutingManager(Instance &aInstance);

    /**
     * This method initializes the routing manager on given infrastructure interface.
     *
     * @param[in]  aInfraIfIndex  An infrastructure network interface index.
     *
     * @retval  OT_ERROR_NONE          Successfully started the routing manager.
     * @retval  OT_ERROR_INVALID_ARGS  The index of the infra interface is not valid.
     *
     */
    otError Init(uint32_t aInfraIfIndex);

    /**
     * This method receives an ICMPv6 message on the infrastructure interface.
     *
     * Malformed or undesired messages are dropped silently.
     *
     * @param[in]  aInfraIfIndex  The infrastructure interface index.
     * @param[in]  aSrcAddress    The source address this message is sent from.
     * @param[in]  aBuffer        THe ICMPv6 message buffer.
     * @param[in]  aLength        The length of the ICMPv6 message buffer.
     *
     */
    void RecvIcmp6Message(uint32_t            aInfraIfIndex,
                          const Ip6::Address &aSrcAddress,
                          const uint8_t *     aBuffer,
                          uint16_t            aBufferLength);

private:
    enum : uint16_t
    {
        kMaxRouterAdvMessageLength = 256u, ///< The maximum RA message length we can handle.
    };

    enum : uint32_t
    {
        kDefaultOmrPrefixLifetime    = 1800u, ///< The default OMR prefix valid lifetime. In seconds.
        kDefaultOnLinkPrefixLifetime = 1800u, ///< The default on-link prefix valid lifetime. In seconds.
    };

    enum : uint8_t
    {
        kOmrPrefixLength    = OT_IP6_PREFIX_BITSIZE, ///< The length of an OMR prefix. In bits.
        kOnLinkPrefixLength = OT_IP6_PREFIX_BITSIZE, ///< The length of an On-link prefix. In bits.
    };

    enum : uint32_t
    {
        kMaxRtrAdvInterval       = 600,                    ///< Maximum Router Advertisement Interval. In seconds.
        kMinRtrAdvInterval       = kMaxRtrAdvInterval / 3, ///< Minimum Router Advertisement Interval. In seconds.
        kMaxInitRtrAdvInterval   = 16,  ///< Maximum Initial Router Advertisement Interval. In seconds.
        kMaxRaDelayTime          = 500, ///< The maximum delay of sending RA after receiving RS. In milliseconds.
        kRtrSolicitationInterval = 4,   ///< The interval between Router Solicitations. In seconds.
        kMaxRtrSolicitationDelay = 1,   ///< The maximum delay for initial solicitation. In seconds.
    };

    static_assert(kMinRtrAdvInterval <= 3 * kMaxRtrAdvInterval / 4, "invalid RA intervals");
    static_assert(kDefaultOmrPrefixLifetime >= kMaxRtrAdvInterval, "invalid default OMR prefix lifetime");
    static_assert(kDefaultOnLinkPrefixLifetime >= kMaxRtrAdvInterval, "invalid default on-link prefix lifetime");

    enum : uint32_t
    {
        kMaxInitRtrAdvertisements = 3, ///< The maximum number of initial Router Advertisements.
        kMaxRtrSolicitations = 3, ///< The Maximum number of Router Solicitations before sending Router Advertisements.
    };

    void    Start(void);
    void    Stop(void);
    void    HandleNotifierEvents(Events aEvents);
    bool    IsInitialized(void) const { return mInfraIfIndex != 0; }
    otError LoadOrGenerateRandomOmrPrefix(void);
    otError LoadOrGenerateRandomOnLinkPrefix(void);

    static bool IsValidOmrPrefix(const Ip6::Prefix &aOmrPrefix);
    static bool IsValidOnLinkPrefix(const Ip6::Prefix &aOnLinkPrefix);

    /**
     * This method evaluate the routing policy depends on prefix and route
     * information on Thread Network and infra link. As a result, this
     * method May send RA messages on infra link and publish/unpublish
     * OMR prefix in the Thread network.
     *
     * @see EvaluateOmrPrefix
     * @see EvaluateOnLinkPrefix
     * @see PublishOmrPrefix
     * @see UnpublishOmrPrefix
     *
     */
    void EvaluateRoutingPolicy(void);

    /**
     * This method evaluates the OMR prefix for the Thread Network.
     *
     * @returns  The new OMR prefix should be advertised. An invalid OMR prefix
     *           means we should no longer publish an OMR prefix in the Thread network.
     *
     */
    Ip6::Prefix EvaluateOmrPrefix(void);

    /**
     * This method evaluates the on-link prefix for the infra link.
     *
     * @returns  The new on-link prefix should be advertised. An invalid on-link prefix
     *           means we should no longer advertise an on-link prefix on the infra link.
     *
     */
    Ip6::Prefix EvaluateOnLinkPrefix(void);

    /**
     * This method publishes an OMR prefix in Thread network.
     *
     * @param[in]  aOmrPrefix  An OMR prefix.
     *
     */
    void PublishOmrPrefix(const Ip6::Prefix &aOmrPrefix);

    /**
     * This method unpublishes an OMR prefix if we have done that.
     *
     * @param[in]  aOmrPrefix  An OMR prefix.
     *
     */
    void UnpublishOmrPrefix(const Ip6::Prefix &aOmrPrefix);

    /**
     * This method starts sending Router Solicitations in random delay
     * between 0 and kMaxRtrSolicitationDelay.
     *
     */
    void StartRouterSolicitation();

    /**
     * This method sends Router Solicitation messages to discovery on-link
     * prefix on infra links.
     *
     * @see HandleRouterAdvertisement
     *
     */
    otError SendRouterSolicit(void);

    /**
     * This method sends Router Advertisement messages to advertise
     * on-link prefix and route for OMR prefix.
     *
     * @param[in]  aNewOmrPrefix  The new OMR prefix to be advertised.
     *                            An invalid OMR prefix means we should stop advertising OMR prefix.
     * @param[in]  aOnLinkPrefix  The new on-link prefix to be advertised.
     *                            An invalid on-link prefix means we should stop advertising on-link prefix.
     *
     */
    void SendRouterAdvertisement(const Ip6::Prefix &aNewOmrPrefix, const Ip6::Prefix &aNewOnLinkPrefix);

    static void HandleRouterAdvertisementTimer(Timer &aTimer);
    void        HandleRouterAdvertisementTimer(void);

    static void HandleRouterSolicitTimer(Timer &aTimer);
    void        HandleRouterSolicitTimer(void);

    static void HandleDiscoveredOnLinkPrefixInvalidTimer(Timer &aTimer);
    void        HandleDiscoveredOnLinkPrefixInvalidTimer(void);

    void HandleRouterSolicit(const Ip6::Address &aSrcAddress, const uint8_t *aBuffer, uint16_t aBufferLength);
    void HandleRouterAdvertisement(const Ip6::Address &aSrcAddress, const uint8_t *aBuffer, uint16_t aBufferLength);
    bool HandlePrefixInfoOption(const RouterAdv::PrefixInfoOption &aPio);

    static TimeMilli GetPrefixExpireTime(uint32_t aValidLifetime);

    const char *GetInfraIfName() const { return otPlatInfraIfGetName(mInfraIfIndex); }

    uint32_t mInfraIfIndex;

    /**
     * The OMR prefix loaded from local persistent storage.
     *
     */
    Ip6::Prefix mLocalOmrPrefix;

    /**
     * The OMR prefix selected to be advertised.
     *
     */
    Ip6::Prefix mAdvertisedOmrPrefix;

    /**
     * The on-link prefix created based on the local OMR prefix.
     *
     */
    Ip6::Prefix mLocalOnLinkPrefix;

    /**
     * The on-link prefix selected to be advertised.
     *
     */
    Ip6::Prefix mAdvertisedOnLinkPrefix;

    /**
     * The on-link prefix we discvered on the infra link by
     * sending Router Solicitations.
     *
     */
    Ip6::Prefix mDiscoveredOnLinkPrefix;

    TimerMilli mRouterAdvertisementTimer;
    uint32_t   mRouterAdvertisementCount;

    TimerMilli mRouterSolicitTimer;
    uint8_t    mRouterSolicitCount;

    TimerMilli mDiscoveredOnLinkPrefixInvalidTimer;
};

} // namespace ot

#endif // OPENTHREAD_CONFIG_BORDER_ROUTING_ENABLE

#endif // ROUTING_MANAGER_HPP_
