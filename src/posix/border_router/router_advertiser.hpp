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

#ifndef POSIX_ROUTER_ADVERTISER_HPP_
#define POSIX_ROUTER_ADVERTISER_HPP_

#include "openthread-posix-config.h"

#include <openthread/error.h>

#include "common/locator.hpp"
#include "common/locator-getters.hpp"
#include "CommonCrypto/timer.hpp"
#include "netif.hpp"

namespace ot
{

namespace Posix
{

class RouterAdvertiser : InstanceLocator
{
public:
    RouterAdvertiser(otInstance *aInstance)
        : InstanceLocator(*static_cast<Instance *>(aInstance))
        , mNext(nullptr)
    {
    }

    otError Init(const char *aNetIfName);

    void Deinit();

    otError SendRouterSolicit();

    otError SendRouterAdvertisement();

private:
    Netif mNetIf;

    TimerMilli mRouterSolicitTimer;
    TimerMilli mRouterAdvertisementTimer;

    RouterAdvertiser *mNext;
};

} // namespace Posix

} // namespace ot

#endif // POSIX_ROUTER_ADVERTISER_HPP_
