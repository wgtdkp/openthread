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

#ifndef POSIX_ICMP6_HPP_
#define POSIX_ICMP6_HPP_

#include "openthread-posix-config.h"

#if OPENTHREAD_CONFIG_DUCKHORN_BORDER_ROUTER_ENABLE

#include <memory.h>
#include <stdint.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/icmp6.h>

#include <openthread/icmp6.h>
#include <openthread/openthread-system.h>
#include <openthread/platform/toolchain.h>

#include "infra_netif.hpp"

#include "common/encoding.hpp"

#ifndef ND_OPT_ROUTE_INFORMATION
#define ND_OPT_ROUTE_INFORMATION 24
#endif

using ot::Encoding::BigEndian::HostSwap16;
using ot::Encoding::BigEndian::HostSwap32;

namespace ot
{

namespace Posix
{

namespace Icmp6 {

static constexpr uint32_t kInfiniteLifetime = 0xffffffff;

OT_TOOL_PACKED_BEGIN
class Option
{
public:
    explicit Option(uint8_t aType, uint8_t aLength=0)
        : mType(aType)
        , mLength(aLength)
    {
    }

    uint8_t GetType() const { return mType; }

    void SetLength(uint16_t aLength) { mLength = aLength / 8; }

    uint16_t GetLength() const { return mLength * 8; }

    const Option *GetNextOption() const
    {
        return reinterpret_cast<const Option *>(reinterpret_cast<const uint8_t *>(this) + GetLength());
    }

private:
    uint8_t mType;
    uint8_t mLength;
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
class PrefixInfoOption : public Option
{
public:
    PrefixInfoOption()
        : Option(ND_OPT_PREFIX_INFORMATION, 4)
        , mPrefixLength(0)
        , mReserved1(0)
        , mValidLifetime(0)
        , mPreferredLifetime(0)
        , mReserved2(0)
    {
        memset(&mPrefix, 0, sizeof(mPrefix));
    }

    void SetOnLink(bool aOnLink)
    {
        if (aOnLink)
        {
            mReserved1 |= ND_OPT_PI_FLAG_ONLINK;
        }
        else
        {
            mReserved1 &= ~ND_OPT_PI_FLAG_ONLINK;
        }
    }

    void SetAutoAddrConfig(bool aAutoAddrConfig)
    {
        if (aAutoAddrConfig)
        {
            mReserved1 |= ND_OPT_PI_FLAG_AUTO;
        }
        else
        {
            mReserved1 &= ~ND_OPT_PI_FLAG_AUTO;
        }
    }

    void SetValidLifetime(uint32_t aValidLifetime)
    {
        mValidLifetime = HostSwap32(aValidLifetime);
    }

    uint32_t GetValidLifetime() const
    {
        return HostSwap32(mValidLifetime);
    }

    void SetPreferredLifetime(uint32_t aPreferredLifetime)
    {
        mPreferredLifetime = HostSwap32(aPreferredLifetime);
    }

    void SetPrefix(const otIp6Prefix &aPrefix)
    {
        mPrefixLength = aPrefix.mLength;
        memcpy(mPrefix, &aPrefix.mPrefix, sizeof(aPrefix.mPrefix));
    }

    uint8_t GetPrefixLength() const { return mPrefixLength; }
    const uint8_t *GetPrefix() const { return mPrefix; }

private:
    uint8_t mPrefixLength;
    uint8_t mReserved1;
    uint32_t mValidLifetime;
    uint32_t mPreferredLifetime;
    uint32_t mReserved2;
    uint8_t  mPrefix[16];
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
class RouteInfoOption : public Option
{
public:
    RouteInfoOption()
        : Option(ND_OPT_ROUTE_INFORMATION, 0)
        , mPrefixLength(0)
        , mReserved(0)
        , mRouteLifetime(0)
    {
        memset(&mPrefix, 0, sizeof(mPrefix));
    }

    void SetRoutePreference(uint8_t aPreference)
    {
        mReserved &= ~0x18;
        mReserved |= (0x03 & aPreference) << 3;
    }

    void SetRouteLifetime(uint32_t aLifetime)
    {
        mRouteLifetime = HostSwap32(aLifetime);
    }

    void SetPrefix(const otIp6Prefix &aPrefix)
    {
        SetLength(((aPrefix.mLength + 63) / 64) * 8 + 8);

        mPrefixLength = aPrefix.mLength;
        memcpy(mPrefix, &aPrefix.mPrefix, sizeof(aPrefix.mPrefix));
    }

private:
    uint8_t mPrefixLength;
    uint8_t mReserved;
    uint32_t mRouteLifetime;
    uint8_t mPrefix[16];

} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
class MessageBase
{
public:
    explicit MessageBase(uint16_t aLength = 0)
        : mLength(aLength)
    {
    }

    void SetLength(uint16_t aLength) { mLength = aLength; }

    uint16_t GetLength() const { return mLength; }

    uint16_t GetTotalLength() const { return sizeof(*this) + GetLength(); }

    uint8_t *GetBuffer() { return mBuffer; }

private:
    uint16_t mLength;
    uint8_t mBuffer[0];
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
class MessageBuffer : public MessageBase
{
public:
    static constexpr uint16_t kMaxLength = 1500;

private:
    uint8_t mBuffer[kMaxLength];
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
class RouterAdvMessage : public MessageBase
{
public:
    RouterAdvMessage()
        : MessageBase(sizeof(*this) - sizeof(MessageBase))
        , mReachableTime(0)
        , mRetransTimer(0)
    {
        memset(&mHeader, 0, sizeof(mHeader));
        mHeader.mType = ND_ROUTER_ADVERT;
        mHeader.mCode = 0;
    }

    void SetCurHopLimit(uint8_t aHopLimit)
    {
        mHeader.mData.m8[0] = aHopLimit;
    }

    void SetRouterLifetime(uint16_t aLifetime)
    {
        mHeader.mData.m16[1] = HostSwap16(aLifetime);
    }

    void SetReachableTime(uint32_t aReachableTime)
    {
        mReachableTime = HostSwap32(aReachableTime);
    }

    void SetRetransTimer(uint32_t aRetransTimer)
    {
        mRetransTimer = HostSwap32(aRetransTimer);
    }

    const PrefixInfoOption *GetNextPrefixInfo(const PrefixInfoOption *aCurPrefixInfo) const;

private:
    const Option *GetNextOption(const Option *aCurOption) const;

    otIcmp6Header mHeader;
    uint32_t      mReachableTime;
    uint32_t      mRetransTimer;
    uint8_t       mOptions[0];
} OT_TOOL_PACKED_END;


/**
 * This class implements ICMPv6 message transceiver.
 *
 */
class Icmp6
{
public:
    typedef void (*RouterSolicitHandler)(unsigned int aIfIndex, void *aContext);
    typedef void (*RouterAdvertisementHandler)(const RouterAdvMessage &aRouterAdv, unsigned int aIfIndex, void *aContext);

    Icmp6(RouterSolicitHandler aRouterSolicitHandler,
          void *aRouterSolicitHandlerContext,
          RouterAdvertisementHandler aRouterAdvertisementHandler,
          void *aRouterAdvertisementHandlerContext);

    otError Init();

    void Deinit();

    void Update(otSysMainloopContext *aMainloop);
    void Process(const otSysMainloopContext *aMainloop);

    otError SendRouterAdvertisement(const otIp6Prefix *aOmrPrefix,
                                    const otIp6Prefix *aOnLinkPrefix,
                                    const InfraNetif &aInfraNetif,
                                    const struct in6_addr &aDest);

    void SendRouterSolicit(const InfraNetif &aInfraNetif,
                           const struct in6_addr &aDest);

private:
    static constexpr uint16_t kMaxIcmp6MessageLength = 1280;

    void HandleRouterSolicit(unsigned int aIfIndex,
                             const MessageBuffer &aBuffer,
                             const struct in6_addr &aSrcAddr,
                             const struct in6_addr &aDstAddr);
    void HandleRouterAdvertisement(unsigned int aIfIndex,
                                   const MessageBuffer &aBuffer,
                                   const struct in6_addr &aSrcAddr,
                                   const struct in6_addr &aDstAddr);

    otError Send(uint8_t *aMessage,
                 uint16_t aMessageLength,
                 const InfraNetif &aInfraNetif,
                 const struct in6_addr &aDest);

    void Recv();

    int mSocketFd;
    RouterSolicitHandler mRouterSolicitHandler;
    void * mRouterSolicitHandlerContext;
    RouterAdvertisementHandler mRouterAdvertisementHandler;
    void * mRouterAdvertisementHandlerContext;
};

} // namespace Icmp6

} // namespace Posix

} // namespace ot

#endif // OPENTHREAD_CONFIG_DUCKHORN_BORDER_ROUTER_ENABLE

#endif // POSIX_ICMP6_HPP_
