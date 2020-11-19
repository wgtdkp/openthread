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

#include "infra_netif.hpp"

#include <errno.h>
#include <memory.h>

#include <netinet/in.h>
#include <net/if.h>
#include <sys/types.h>
#include <ifaddrs.h>

#include "common/code_utils.hpp"
#include "common/logging.hpp"

namespace ot
{

namespace Posix
{

InfraNetif::InfraNetif()
    : mIndex(0)
    , mAddressNum(0)
{
    memset(mName, 0, sizeof(mName));
    memset(mAddresses, 0, sizeof(mAddresses));
}

otError InfraNetif::Init(const char *aName)
{
    otError error = OT_ERROR_NONE;
    struct ifaddrs *ifaddrs;


    VerifyOrExit(strlen(aName) < sizeof(mName), error = OT_ERROR_INVALID_ARGS);
    strcpy(mName, aName);
    mIndex = if_nametoindex(aName);

    if (getifaddrs(&ifaddrs) < 0)
    {
        otLogWarnPlat("failed to get netif address: %s", strerror(errno));
        ExitNow(error = OT_ERROR_FAILED);
    }

    for (struct ifaddrs *ifaddr = ifaddrs; ifaddr; ifaddr = ifaddr->ifa_next)
    {
        if (ifaddr->ifa_name == nullptr || strcmp(mName, ifaddr->ifa_name) != 0)
        {
            continue;
        }
        if (ifaddr->ifa_addr == nullptr || ifaddr->ifa_addr->sa_family != AF_INET6)
        {
            continue;
        }

        SuccessOrExit(error = AddAddress(*reinterpret_cast<sockaddr_in6 *>(ifaddr->ifa_addr)));
    }


exit:
    return error;
}

void InfraNetif::Deinit()
{
    memset(mName, 0, sizeof(mName));
    mIndex = 0;
}

otError InfraNetif::AddAddress(const struct sockaddr_in6 &aAddr)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(mAddressNum < kMaxAddrNum, error = OT_ERROR_NO_BUFS);
    mAddresses[mAddressNum++] = aAddr;

exit:
    return error;
}

bool InfraNetif::HasUlaOrGuaAddress() const
{
    bool ret = false;
    for (uint8_t i = 0; i < mAddressNum; ++i)
    {
        if (IsUlaAddress(mAddresses[i]) || IsGuaAddress(mAddresses[i]))
        {
            ret = true;
            break;
        }
    }

    return ret;
}

const struct sockaddr_in6 *InfraNetif::GetLinkLocalAddress() const
{
    const struct sockaddr_in6 *linkLocalAddress = nullptr;

    for (uint8_t i = 0; i < mAddressNum; ++i)
    {
        if (IsLinkLocalAddress(mAddresses[i]))
        {
            linkLocalAddress = &mAddresses[i];
            break;
        }
    }

    return linkLocalAddress;
}

bool InfraNetif::IsUlaAddress(const struct sockaddr_in6 &aAddr)
{
    return (aAddr.sin6_addr.s6_addr[0] & 0xfe) == 0xfc;
}

bool InfraNetif::IsGuaAddress(const struct sockaddr_in6 &aAddr)
{
    return (aAddr.sin6_addr.s6_addr[0] & 0xe0) == 0x20;
}

bool InfraNetif::IsLinkLocalAddress(const struct sockaddr_in6 &aAddr)
{
    return (aAddr.sin6_addr.s6_addr[0] & 0xff) == 0xfe &&
           (aAddr.sin6_addr.s6_addr[1] & 0xc0) == 0x80;
}

} // namespace Posix

} // namespace ot
