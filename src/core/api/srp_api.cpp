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
 *  This file defines the OpenThread SRP API.
 */

#include "openthread-core-config.h"

#if OPENTHREAD_CONFIG_SRP_SERVER_ENABLE

#include <openthread/srp.h>

#include "border_router/srp_server.hpp"

#include "common/instance.hpp"
#include "common/locator-getters.hpp"

using namespace ot;

void otSrpServerSetEnabled(otInstance *aInstance, bool aEnabled)
{
    Instance &instance = *static_cast<Instance *>(aInstance);

    instance.Get<BorderRouter::SrpServer>().SetEnabled(aEnabled);
}

otError otSrpServerSetLeaseRange(otInstance *aInstance,
                                 uint32_t    aMinLease,
                                 uint32_t    aMaxLease,
                                 uint32_t    aMinKeyLease,
                                 uint32_t    aMaxKeyLease)
{
    Instance &instance = *static_cast<Instance *>(aInstance);

    return instance.Get<BorderRouter::SrpServer>().SetLeaseRange(aMinLease, aMaxLease, aMinKeyLease, aMaxKeyLease);
}

void otSrpServerSetServiceHandler(otInstance *aInstance, otSrpServiceHandler aServiceHandler, void *aContext)
{
    Instance &instance = *static_cast<Instance *>(aInstance);

    instance.Get<BorderRouter::SrpServer>().SetServiceHandler(aServiceHandler, aContext);
}

void otSrpServiceEventResult(otInstance *aInstance, otError aError, void *aContext)
{
    Instance &instance = *static_cast<Instance *>(aInstance);

    instance.Get<BorderRouter::SrpServer>().HandleServiceEventResult(aError, aContext);
}

const otSrpHost *otSrpServerGetNextHost(otInstance *aInstance, const otSrpHost *aHost)
{
    Instance &instance = *static_cast<Instance *>(aInstance);

    return instance.Get<BorderRouter::SrpServer>().GetNextHost(
        static_cast<const BorderRouter::SrpServer::Host *>(aHost));
}

const char *otSrpServerHostGetFullName(const otSrpHost *aHost)
{
    return static_cast<const BorderRouter::SrpServer::Host *>(aHost)->GetFullName();
}

const otIp6Address *otSrpServerHostGetAddresses(const otSrpHost *aHost, uint8_t *aAddressesNum)
{
    auto host = static_cast<const BorderRouter::SrpServer::Host *>(aHost);

    return host->GetAddresses(*aAddressesNum);
}

const otSrpService *otSrpServerHostGetServices(const otSrpHost *aHost)
{
    return static_cast<const BorderRouter::SrpServer::Host *>(aHost)->GetServices();
}

#endif // OPENTHREAD_CONFIG_SRP_SERVER_ENABLE
