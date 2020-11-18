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

#define __APPLE_USE_RFC_3542

#include "icmp6.hpp"

#include <errno.h>
#include <netinet/icmp6.h>
#include <unistd.h>

#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "common/logging.hpp"

namespace ot
{

namespace Posix
{

Icmp6::Icmp6()
    : mSocketFd(-1)
{
}

otError Icmp6::Init()
{
    otError error = OT_ERROR_FAILED;
    int sock;
    struct icmp6_filter filter;
    ssize_t rval;
    static const int kTrue = 1;
    static const int kFalse = 0;

    sock = socket(AF_INET6, SOCK_RAW, IPPROTO_ICMPV6);
    VerifyOrExit(sock >= 0);

    // Only accept router advertisements and router solicits.
    ICMP6_FILTER_SETBLOCKALL(&filter);
    ICMP6_FILTER_SETPASS(ND_ROUTER_SOLICIT, &filter);
    ICMP6_FILTER_SETPASS(ND_ROUTER_ADVERT, &filter);
    rval = setsockopt(sock, IPPROTO_ICMPV6, ICMP6_FILTER, &filter, sizeof(filter));
    if (rval < 0)
    {
        otLogWarnPlat("Can't set ICMP6_FILTER: %s", strerror(errno));
        ExitNow();
    }

    // We want a source address and interface index.
    rval = setsockopt(sock, IPPROTO_IPV6, IPV6_RECVPKTINFO, &kTrue, sizeof(kTrue));
    if (rval < 0)
    {
        otLogWarnPlat("Can't set IPV6_RECVPKTINFO: %s", strerror(errno));
        ExitNow();
    }

    // We need to be able to reject RAs arriving from off-link.
    rval = setsockopt(sock, IPPROTO_IPV6, IPV6_RECVHOPLIMIT, &kTrue, sizeof(kTrue));
    if (rval < 0)
    {
        otLogWarnPlat("Can't set IPV6_RECVHOPLIMIT: %s", strerror(errno));
        ExitNow();
    }

    // Prevent our router advertisements from updating our routing table.
    rval = setsockopt(sock, IPPROTO_IPV6, IPV6_MULTICAST_LOOP, &kFalse, sizeof(kFalse));
    if (rval < 0)
    {
        otLogWarnPlat("Can't set IPV6_MULTICAST_LOOP: %s", strerror(errno));
        ExitNow();
    }

    mSocketFd = sock;
    error = OT_ERROR_NONE;

exit:
    if (error != OT_ERROR_NONE)
    {
        if (sock >= 0)
        {
            close(sock);
        }
    }
    return error;
}

void Icmp6::Deinit()
{
    if (mSocketFd >= 0)
    {
        close(mSocketFd);
    }
}

void Icmp6::Update(otSysMainloopContext *aMainloop)
{
    FD_SET(mSocketFd, &aMainloop->mReadFdSet);
    FD_SET(mSocketFd, &aMainloop->mErrorFdSet);

    if (mSocketFd > aMainloop->mMaxFd)
    {
        aMainloop->mMaxFd = mSocketFd;
    }
}

void Icmp6::Process(const otSysMainloopContext *aMainloop)
{
    // TODO(wgtdkp):
    if (FD_ISSET(mSocketFd, &aMainloop->mReadFdSet))
    {
    }
    else if (FD_ISSET(mSocketFd, &aMainloop->mErrorFdSet))
    {
    }
}

otError Icmp6::SendRouterAdvertisement(const otIp6Prefix *aOmrPrefix,
                                       const otIp6Prefix *aOnLinkPrefix,
                                       const InfraNetif &aInfraNetif,
                                       const struct in6_addr &aDest)
{
    uint8_t message[kMaxIcmp6MessageLength];
    uint16_t messageLength = 0;
    RouterAdvMessage advMessage;

    advMessage.SetRouterLifetime(0);
    advMessage.SetReachableTime(0);
    advMessage.SetRetransTimer(0);

    memcpy(message + messageLength, &advMessage, sizeof(advMessage));
    messageLength += sizeof(advMessage);

    // TODO(wgtdkp): append link local address;

    if (aOnLinkPrefix)
    {
        PrefixInfoOption pio;

        pio.SetOnLink(true);
        pio.SetAutoAddrConfig(true);
        pio.SetValidLifetime(1800); // 30 Minutes.
        pio.SetPreferredLifetime(1800); // 30 Minutes.
        pio.SetPrefix(*aOnLinkPrefix);

        memcpy(message + messageLength, &pio, sizeof(pio));
        messageLength += sizeof(pio);
    }

    if (aOmrPrefix)
    {
        RouteInfoOption rio;

        rio.SetRouteLifetime(1800); // 30 Minutes.
        rio.SetPrefix(*aOmrPrefix);

        memcpy(message + messageLength, &rio, rio.GetLengthInBytes());
        messageLength += rio.GetLengthInBytes();
    }

    return Send(message, messageLength, aInfraNetif, aDest);
}

otError Icmp6::Send(uint8_t *aMessage,
                    uint16_t aMessageLength,
                    const InfraNetif &aInfraNetif,
                    const struct in6_addr &aDest)
{
    otError error = OT_ERROR_FAILED;

    struct iovec iov;
    struct in6_pktinfo *packetInfo;
    uint8_t cmsgBuffer[CMSG_SPACE(sizeof(*packetInfo)) + CMSG_SPACE(sizeof (int))];
    struct msghdr msgHeader;
    struct cmsghdr *cmsgPointer;
    static const int kHopLimit = 255;
    ssize_t rval;
    struct sockaddr_in6 dest;

    OT_ASSERT(aMessage != nullptr);
    VerifyOrExit(mSocketFd >= 0);

    // This is critical to get CMSG_NXTHDR work.
    memset(cmsgBuffer, 0, sizeof(cmsgBuffer));

    // Send the message
    memset(&dest, 0, sizeof(dest));
    dest.sin6_family = AF_INET6;
    dest.sin6_scope_id = aInfraNetif.GetIndex();
    dest.sin6_addr = aDest;

    iov.iov_base = aMessage;
    iov.iov_len = aMessageLength;

    msgHeader.msg_namelen = sizeof(dest);
    msgHeader.msg_name = &dest;
    msgHeader.msg_iov = &iov;
    msgHeader.msg_iovlen = 1;
    msgHeader.msg_control = cmsgBuffer;
    msgHeader.msg_controllen = sizeof(cmsgBuffer);

    // Specify the interface
    cmsgPointer = CMSG_FIRSTHDR(&msgHeader);
    cmsgPointer->cmsg_level = IPPROTO_IPV6;
    cmsgPointer->cmsg_type = IPV6_PKTINFO;
    cmsgPointer->cmsg_len = CMSG_LEN(sizeof(*packetInfo));
    packetInfo = (struct in6_pktinfo *)CMSG_DATA(cmsgPointer);
    memset(packetInfo, 0, sizeof(*packetInfo));
    packetInfo->ipi6_ifindex = aInfraNetif.GetIndex();

    // Router advertisements and solicitations have a hop limit of 255
    cmsgPointer = CMSG_NXTHDR(&msgHeader, cmsgPointer);
    cmsgPointer->cmsg_level = IPPROTO_IPV6;
    cmsgPointer->cmsg_type = IPV6_HOPLIMIT;
    cmsgPointer->cmsg_len = CMSG_LEN(sizeof(kHopLimit));
    // FIXME(wgtdkp): byte order?
    memcpy(CMSG_DATA(cmsgPointer), &kHopLimit, sizeof(kHopLimit));

    rval = sendmsg(mSocketFd, &msgHeader, 0);
    if (rval < 0 || static_cast<size_t>(rval) != iov.iov_len)
    {
        otLogWarnPlat("failed to send %s on interface %s, index %u: %s",
            aMessage[0] == ND_ROUTER_SOLICIT ? "Router Solicit" : "Router Advertisement",
            aInfraNetif.GetName(), aInfraNetif.GetIndex(), strerror(errno));
        ExitNow();
    }

    error = OT_ERROR_NONE;

exit:
    return error;
}

} // namespace Posix

} // namespace ot
