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

#ifndef POSIX_INFRA_NETIF_HPP_
#define POSIX_INFRA_NETIF_HPP_

#include <stdint.h>

#include <net/if.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <openthread/error.h>
#include <openthread/openthread-system.h>

namespace ot
{

namespace Posix
{

/**
 * This class represents an infrastructure network interface (e.g. wlan0).
 *
 */
class InfraNetif
{
public:
    typedef void (*StateChangedHandler)(void *aContext);

    InfraNetif(StateChangedHandler aHandler, void *aContext);

    otError Init(const char *aName);
    void Deinit();

    void Update(otSysMainloopContext *aMainloop);
    void Process(const otSysMainloopContext *aMainloop);

    /**
     * This method checks if the netif is UP.
     *
     */
    bool IsUp() const;

    const char *GetName() const { return mName; }
    unsigned int GetIndex() const { return mIndex; }
    bool HasUlaOrGuaAddress() const;

    const struct sockaddr_in6 &GetNextAddr() const;

private:
    static constexpr uint8_t kMaxAddrNum = 32;

    void RefreshAddresses();

    static bool IsUlaAddress(const struct sockaddr_in6 &aAddr);
    static bool IsGuaAddress(const struct sockaddr_in6 &aAddr);

    void RecvNetlinkMessage();

    void InitNetlink();
    void ProcessNetlinkEvent(int aNetifIndex);

    char mName[IFNAMSIZ];
    unsigned int mIndex;

    struct sockaddr_in6 mAddresses[kMaxAddrNum];
    uint8_t mAddressNum;

    StateChangedHandler mStateChangedHandler;
    void *mStateChangedHandlerContext;

    int mNetlinkFd;
};

} // namespace Posix

} // namespace ot

#endif // POSIX_INFRA_NETIF_HPP_
