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

#ifndef POSIX_BORDER_ROUTER_HPP_
#define POSIX_BORDER_ROUTER_HPP_

#include "openthread-posix-config.h"

#include <openthread/error.h>
#include <openthread/instance.h>
#include <openthread/ip6.h>
#include <openthread/netdata.h>

#include <openthread/openthread-system.h>

#include "common/locator.hpp"
#include "common/locator-getters.hpp"

#include "icmp6.hpp"

namespace ot
{

namespace Posix
{

class BorderRouter : public InstanceLocator
{
public:
    BorderRouter(otInstance *aInstance);

    otError Init();
    void Deinit();

    void Update(otSysMainloopContext *aMainLoop);
    void Process(const otSysMainloopContext *aMainLoop);

    // Send Router Solicit messages to discovery on-link prefix
    // on infra links.
    //
    // See `HandleRouterAdvertisement` for response handler.
    otError SendRouterSolicit();

    // Send Router Advertisement messages to advertise on-link prefix
    // and route for on-mesh prefix.
    otError SendRouterAdvertisement();

private:
    // Start the Border Router functionality.
    // Called when current device becomes a Router.
    otError Start();

    // Stop acting as a Border Router.
    void Stop();

    static void HandleStateChanged(otChangedFlags aFlags, void *aBorderRouter);
    void HandleStateChanged(otChangedFlags aFlags);

    // Publish an random-generated on-mesh prefix in Thread Network
    // if non existing such prefix is already in the Thread Network Data.
    //
    // See `PublishOnMeshPrefix` for prefix publishing.
    void PublishOnMeshPrefixIfNoneExisting();

    // Publish on-mesh prefix in Thread Network.
    //
    // Called by `PublishOnMeshPrefixIfNoneExisting`.
    void PublishOnMeshPrefix();

    void HandleRouterSolicit();
    void HandleRouterAdvertisement();

    // The off-mesh reachable on-mesh prefix.
    // The route for this prefix is advertised with RIO
    // in Router Advertisement messages on infra links.
    // This field is valid only when mHasOnMeshPrefix is true.
    //
    // TODO(wgtdkp): We should keep using a single on-mesh prefix.
    // In case there are multiple Border Routers in the Thread
    // Network, there are chances that multiple on-mesh prefixes
    // will present in the Network Data. All Border Routers should
    // use the "lowest" prefix and remove its prefix and deadvertise
    // the route for its prefix.
    otBorderRouterConfig mBorderRouterConfig;

    // Indicates that if we published a Border Router configuration
    // by ourselves.
    bool        mHasBorderRouterConfig;

    Icmp6 mIcmp6;
};

} // namespace Posix

} // namespace ot

#endif // POSIX_BORDER_ROUTER_HPP_
