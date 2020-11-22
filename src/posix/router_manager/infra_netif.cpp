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

#if OPENTHREAD_CONFIG_DUCKHORN_BORDER_ROUTER_ENABLE

#include <errno.h>
#include <memory.h>

#include <ifaddrs.h>
#if __linux__
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#endif
#include <netinet/in.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>

#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "common/logging.hpp"
#include "platform/address_utils.hpp"

namespace ot
{

namespace Posix
{

InfraNetif::InfraNetif(StateChangedHandler aHandler, void *aContext)
    : mIndex(0)
    , mAddressNum(0)
    , mStateChangedHandler(aHandler)
    , mStateChangedHandlerContext(aContext)
    , mNetlinkFd(-1)
    , mNetlinkSequence(0)
{
    memset(mName, 0, sizeof(mName));
    memset(mAddresses, 0, sizeof(mAddresses));
}

otError InfraNetif::Init(const char *aName)
{
    otError error = OT_ERROR_NONE;

    OT_ASSERT(aName != nullptr);

    VerifyOrExit(strlen(aName) < sizeof(mName), error = OT_ERROR_INVALID_ARGS);
    strcpy(mName, aName);
    mIndex = if_nametoindex(aName);

    InitNetlink();

    RefreshAddresses();

exit:
    return error;
}

void InfraNetif::InitNetlink()
{
    otError error = OT_ERROR_FAILED;
    int netLinkFd = -1;
    struct sockaddr_nl snl;
    static const int kTrue = 1;

    netLinkFd = socket(PF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
    if (netLinkFd < 0)
    {
        otLogWarnPlat("failed to open netlink socket: %s", strerror(errno));
        ExitNow();
    }

#if defined(SOL_NETLINK) && defined(NETLINK_NO_ENOBUFS)
	if (setsockopt(netLinkFd, SOL_NETLINK, NETLINK_NO_ENOBUFS, &kTrue, sizeof(kTrue)) < 0)
    {
		otLogWarnPlat("failed to setsockopt NETLINK_NO_ENOBUFS: %s", strerror(errno));
        ExitNow();
	}
#endif

    memset(&snl, 0, sizeof(snl));
	snl.nl_family = AF_NETLINK;
	snl.nl_groups = RTMGRP_LINK | RTMGRP_IPV6_IFADDR;

    if (bind(netLinkFd, reinterpret_cast<struct sockaddr *>(&snl), sizeof(snl)) < 0)
    {
        otLogWarnPlat("failed to bind netlink socket: %s", strerror(errno));
        ExitNow();
    }

    mNetlinkFd = netLinkFd;
    error = OT_ERROR_NONE;

exit:
    if (error != OT_ERROR_NONE)
    {
        if (netLinkFd >= 0)
        {
            close(netLinkFd);
        }
    }
}

void InfraNetif::Deinit()
{
    memset(mName, 0, sizeof(mName));
    mIndex = 0;

    if (mNetlinkFd >= 0)
    {
        close(mNetlinkFd);
        mNetlinkFd = -1;
    }
}

void InfraNetif::Update(otSysMainloopContext *aMainloop)
{
    VerifyOrExit(mNetlinkFd >= 0);

    FD_SET(mNetlinkFd, &aMainloop->mReadFdSet);
    FD_SET(mNetlinkFd, &aMainloop->mErrorFdSet);

    if (mNetlinkFd > aMainloop->mMaxFd)
    {
        aMainloop->mMaxFd = mNetlinkFd;
    }

exit:
    return;
}

void InfraNetif::Process(const otSysMainloopContext *aMainloop)
{
    VerifyOrExit(mNetlinkFd >= 0);

    if (FD_ISSET(mNetlinkFd, &aMainloop->mReadFdSet))
    {
        RecvNetlinkMessage();
    }

    if (FD_ISSET(mNetlinkFd, &aMainloop->mErrorFdSet))
    {
        otLogWarnPlat("netlink socket errored");
    }

exit:
    return;
}

bool InfraNetif::IsUp() const
{
    struct ifreq ifr;
    int sock;

    memset(&ifr, 0, sizeof(ifr));
    sock = socket(PF_INET6, SOCK_DGRAM, IPPROTO_IP);
    strcpy(ifr.ifr_name, GetName());

    if (ioctl(sock, SIOCGIFFLAGS, &ifr) < 0)
    {
        perror("SIOCGIFFLAGS");
    }

    close(sock);

    return (ifr.ifr_flags & IFF_UP);
}

void InfraNetif::RefreshAddresses()
{
    struct ifaddrs *ifaddrs = nullptr;

    if (getifaddrs(&ifaddrs) < 0)
    {
        otLogWarnPlat("failed to get netif address: %s", strerror(errno));
        ExitNow();
    }

    mAddressNum = 0;

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
        if (mAddressNum >= kMaxAddrNum)
        {
            otLogWarnPlat("too many addresses on interface %s", GetName());
            break;
        }

        mAddresses[mAddressNum++] = *reinterpret_cast<sockaddr_in6 *>(ifaddr->ifa_addr);
    }

exit:
    return;
}

/*
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

bool InfraNetif::IsUlaAddress(const struct sockaddr_in6 &aAddr)
{
    return (aAddr.sin6_addr.s6_addr[0] & 0xfe) == 0xfc;
}

bool InfraNetif::IsGuaAddress(const struct sockaddr_in6 &aAddr)
{
    return (aAddr.sin6_addr.s6_addr[0] & 0xe0) == 0x20;
}
*/

const struct sockaddr_in6 *InfraNetif::GetAllAddresses(uint8_t &aAddressNum) const
{
    aAddressNum = mAddressNum;
    return mAddresses;
}

void InfraNetif::RecvNetlinkMessage()
{

    const size_t kMaxNetifEvent = 8192;
    ssize_t      length;

    union
    {
#if defined(__linux__)
        nlmsghdr nlMsg;
#else
        rt_msghdr rtMsg;
#endif
        char buffer[kMaxNetifEvent];
    } msgBuffer;

    VerifyOrExit(mNetlinkFd >= 0);

    length = recv(mNetlinkFd, msgBuffer.buffer, sizeof(msgBuffer.buffer), 0);
    if (length <= 0)
    {
        otLogWarnPlat("failed to recv netlink messages: %s", strerror(errno));
        ExitNow();
    }

#if defined(__linux__)
    for (struct nlmsghdr *msg = &msgBuffer.nlMsg;
         NLMSG_OK(msg, static_cast<size_t>(length));
         msg = NLMSG_NEXT(msg, length))
    {
#else
    {
        // BSD sends one message per read to routing socket (see route.c, monitor command)
        struct rt_msghdr *msg;

        msg = &msgBuffer.rtMsg;

#define nlmsg_type rtm_type

#endif
        switch (msg->nlmsg_type)
        {
        case RTM_NEWADDR:
        case RTM_DELADDR:
            ProcessNetlinkEvent(reinterpret_cast<struct ifaddrmsg *>(NLMSG_DATA(msg))->ifa_index);
            break;

#if defined(RTM_NEWLINK) && defined(RTM_DELLINK)
        case RTM_NEWLINK:
        case RTM_DELLINK:
            ProcessNetlinkEvent(reinterpret_cast<struct ifinfomsg *>(NLMSG_DATA(msg))->ifi_index);
            break;
#endif

#if !defined(__linux__)
        case RTM_IFINFO:
            ProcessNetlinkEvent(reinterpret_cast<struct if_msghdr *>(msg)->ifi_index);
            break;
#endif

#if defined(RTM_NEWMADDR) && defined(RTM_DELMADDR)
        case RTM_NEWMADDR:
        case RTM_DELMADDR:
            // We don't care for multicast addresses for now.
            break;
#endif

        default:
            break;
        }
    }

exit:
    return;
}

void InfraNetif::ProcessNetlinkEvent(int aNetifIndex)
{
    VerifyOrExit(aNetifIndex > 0 && static_cast<unsigned int>(aNetifIndex) == GetIndex());

    otLogDebgPlat("received new netlink message on interface %s", GetName());

    RefreshAddresses();

// TODO(wgtdkp):
/*
    if (mStateChangedHandler != nullptr)
    {
        mStateChangedHandler(mStateChangedHandlerContext);
    }
*/

exit:
    return;
}

void InfraNetif::UpdateGatewayAddress(const otIp6Prefix &aOnLinkPrefix, bool aIsAdded)
{
    otIp6Address gatewayAddress;
    otIp6AddressInfo gatewayAddressInfo;

    OT_ASSERT(aOnLinkPrefix.mLength == OT_IP6_PREFIX_BITSIZE);

    memset(&gatewayAddressInfo, 0, sizeof(gatewayAddressInfo));
    
    gatewayAddress = aOnLinkPrefix.mPrefix;
    gatewayAddress.mFields.m8[OT_IP6_ADDRESS_SIZE - 1] = 1;    
    gatewayAddressInfo.mAddress = &gatewayAddress;
    gatewayAddressInfo.mPrefixLength = aOnLinkPrefix.mLength;
    gatewayAddressInfo.mScope = 14; // Global scope
    gatewayAddressInfo.mIsAnycast = false;

    otLogInfoPlat("%s gateway adderss %s on interface %s",
        (aIsAdded ? "add" : "remove"), Ip6PrefixString(gatewayAddressInfo).AsCString(), GetName());
    UpdateUnicastAddress(gatewayAddressInfo, aIsAdded);
}

void InfraNetif::UpdateUnicastAddress(const otIp6AddressInfo &aAddressInfo, bool aIsAdded)
{
    struct rtattr *rta;

    struct
    {
        struct nlmsghdr  nh;
        struct ifaddrmsg ifa;
        char             buf[512];
    } req;

    memset(&req, 0, sizeof(req));

    req.nh.nlmsg_len   = NLMSG_LENGTH(sizeof(struct ifaddrmsg));
    req.nh.nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK | NLM_F_CREATE | NLM_F_EXCL;
    req.nh.nlmsg_type  = aIsAdded ? RTM_NEWADDR : RTM_DELADDR;
    req.nh.nlmsg_pid   = 0;
    req.nh.nlmsg_seq   = ++mNetlinkSequence;

    req.ifa.ifa_family    = AF_INET6;
    req.ifa.ifa_prefixlen = aAddressInfo.mPrefixLength;
    req.ifa.ifa_flags     = IFA_F_NODAD;
    req.ifa.ifa_scope     = aAddressInfo.mScope;
    req.ifa.ifa_index     = GetIndex();

    rta           = reinterpret_cast<struct rtattr *>((reinterpret_cast<char *>(&req)) + NLMSG_ALIGN(req.nh.nlmsg_len));
    rta->rta_type = IFA_LOCAL;
    rta->rta_len  = RTA_LENGTH(sizeof(*aAddressInfo.mAddress));

    memcpy(RTA_DATA(rta), aAddressInfo.mAddress, sizeof(*aAddressInfo.mAddress));

    req.nh.nlmsg_len = NLMSG_ALIGN(req.nh.nlmsg_len) + rta->rta_len;

    if (aAddressInfo.mIsAnycast)
    {
        struct ifa_cacheinfo cacheinfo;

        rta           = reinterpret_cast<struct rtattr *>((reinterpret_cast<char *>(rta)) + rta->rta_len);
        rta->rta_type = IFA_CACHEINFO;
        rta->rta_len  = RTA_LENGTH(sizeof(cacheinfo));

        memset(&cacheinfo, 0, sizeof(cacheinfo));
        cacheinfo.ifa_valid = UINT32_MAX;

        memcpy(RTA_DATA(rta), &cacheinfo, sizeof(cacheinfo));

        req.nh.nlmsg_len += rta->rta_len;
    }

    if (send(mNetlinkFd, &req, req.nh.nlmsg_len, 0) != -1)
    {
        otLogInfoPlat("successfully %s new address %s on interface %s",
            (aIsAdded ? "add" : "remove"), Ip6PrefixString(aAddressInfo).AsCString(), GetName());
    }
    else
    {
        otLogWarnPlat("failed to %s new address %s on interface %s",
            (aIsAdded ? "add" : "remove"), Ip6PrefixString(aAddressInfo).AsCString(), GetName());
    }
}

} // namespace Posix

} // namespace ot

#endif // OPENTHREAD_CONFIG_DUCKHORN_BORDER_ROUTER_ENABLE
