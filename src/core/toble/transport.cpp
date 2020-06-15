/*
 *  Copyright (c) 2019, The OpenThread Authors.
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
 *   This file contains the implementation for ToBLE transport abstraction.
 */

#include "toble/transport.hpp"

#include "common/code_utils.hpp"
#include "common/locator-getters.hpp"

namespace ot {
namespace Toble {

#if OPENTHREAD_CONFIG_TOBLE_ENABLE
Transport::Transport(Instance &aInstance)
    : InstanceLocator(aInstance)
    , mBtp(aInstance)
#if OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
    , mL2cap(aInstance)
#endif
{
}

void Transport::Start(Connection &aConn)
{
    if (Get<Toble>().IsCentral())
    {
        switch (aConn.mTransport)
        {
        case kUnspecified:
            // On central the `mTransport` should not be unspecified
            assert(false);
            break;

        case kBtp:
            Get<Btp>().Start(aConn);
            break;

#if OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
        case kL2cap:
            Get<L2cap>().Start(aConn);
            break;
#endif
        }
    }
    else
    {
        // On peripheral, we start all supported transports.
        // This allows the central side to select the transport.
        // The transport layer code should determine this based
        // on event/callbacks from platform layer
        Get<Btp>().Start(aConn);
#if OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
        Get<L2cap>().Start(aConn);
#endif
    }
}

void Transport::Stop(Connection &aConn)
{
    if (Get<Toble>().IsCentral())
    {
        switch (aConn.mTransport)
        {
        case kUnspecified:
            // On central the `mTransport` should not be unspecified
            assert(false);
            break;

        case kBtp:
            Get<Btp>().Stop(aConn);
            break;

#if OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
        case kL2cap:
            Get<L2cap>().Stop(aConn);
            break;
#endif
        }
    }
    else
    {
        // On peripheral, always stop all supported transports
        // This ensures all transport layer clear their internal
        // states.
        Get<Btp>().Stop(aConn);
#if OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
        Get<L2cap>().Stop(aConn);
#endif
    }
}

void Transport::Send(Connection &aConn, const uint8_t *aBuf, uint16_t aLength)
{
    aLength -= Mac::Frame::GetFcsSize(); // All ToBLE transports strip MAC FCS.

    switch (aConn.mTransport)
    {
    case kUnspecified:
        // An unspecified transport type is only allowed on peripheral,
        // where the send request is passed to all supported transports
        // This allows the central to decide on the transport.
        assert(!Get<Toble>().IsCentral());
        Get<Btp>().Send(aConn, aBuf, aLength);
#if OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
        Get<L2cap>().Send(aConn, aBuf, aLength);
#endif
        break;

    case kBtp:
        Get<Btp>().Send(aConn, aBuf, aLength);
        break;

#if OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
    case kL2cap:
        Get<L2cap>().Send(aConn, aBuf, aLength);
        break;
#endif
    }
}

void Transport::HandleSendDone(Connection &aConn, otError aError)
{
    if (Get<Toble>().IsCentral())
    {
#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
        Get<Central::Controller>().HandleTransportSendDone(aConn, aError);
#endif
    }
    else
    {
#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
        Get<Peripheral::Controller>().HandleTransportSendDone(aConn, aError);
#endif
    }
}

void Transport::HandleReceiveDone(Connection &aConn, uint8_t *aFrame, uint16_t aLength, otError aError)
{
    aLength += Mac::Frame::GetFcsSize(); // All ToBLE transports strip MAC FCS.

    if (Get<Toble>().IsCentral())
    {
#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
        Get<Central::Controller>().HandleTransportReceiveDone(aConn, aFrame, aLength, aError);
#endif
    }
    else
    {
#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
        Get<Peripheral::Controller>().HandleTransportReceiveDone(aConn, aFrame, aLength, aError);
#endif
    }
}

#endif // #if OPENTHREAD_CONFIG_TOBLE_ENABLE

} // namespace Toble
} // namespace ot
