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
 *   This file contains the implementation for BTP client.
 */

#include "toble/btp.hpp"

#include <openthread/platform/toble.h>

#include "common/debug.hpp"
#include "common/locator-getters.hpp"
#include "common/logging.hpp"

#if OPENTHREAD_CONFIG_TOBLE_ENABLE && OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE

namespace ot {
namespace Toble {

void Btp::HandleConnectionReady(Platform::Connection *aPlatConn)
{
    HandshakeRequest handshakeRequest(Get<Platform>().GetConnMtu(aPlatConn), kWindowSize);
    Connection *     conn = Get<ConnectionTable>().Find(aPlatConn);

    otLogDebgBtp("HandleConnectionReady");
    VerifyOrExit((conn != NULL) && Get<Toble>().IsCentral(), OT_NOOP);

    otLogInfoBtp("Send Handshake Request: Mtu=%d, Window=%d", Get<Platform>().GetConnMtu(aPlatConn), kWindowSize);
    GattSend(*conn, reinterpret_cast<uint8_t *>(&handshakeRequest), sizeof(handshakeRequest));
    conn->mSession.mState = kStateHandshake;

exit:
    return;
}

void Btp::HandleC1WriteDone(Platform::Connection *aPlatConn)
{
    Connection *conn = Get<ConnectionTable>().Find(aPlatConn);

    otLogDebgBtp("HandleC1WriteDone");
    OT_ASSERT(Get<Toble>().IsCentral());

    VerifyOrExit(conn != NULL, OT_NOOP);

    switch (conn->mSession.mState)
    {
    case kStateIdle:
        // fall through

    case kStateSubscribe:
        break;

    case kStateHandshake:
        otLogInfoBtp("GATT Subscribe to C2");
        Get<Platform>().SubscribeC2(aPlatConn, true);
        conn->mSession.mState = kStateSubscribe;
        break;

    case kStateConnected:
        HandleSentData(*conn);
        break;
    }

exit:
    return;
}

void Btp::HandleC2Notification(Platform::Connection *aPlatConn, const uint8_t *aFrame, uint16_t aLength)
{
    Connection *  conn   = Get<ConnectionTable>().Find(aPlatConn);
    const Header *header = reinterpret_cast<const Header *>(aFrame);

    otLogDebgBtp("HandleC2Notification");
    OT_ASSERT(Get<Toble>().IsCentral());

    VerifyOrExit(conn != NULL, OT_NOOP);

    VerifyOrExit(conn->mSession.mState == kStateSubscribe || conn->mSession.mState == kStateConnected, OT_NOOP);

    VerifyOrExit(aLength > 0, OT_NOOP);

    if (header->GetFlag(Header::kHandshakeFlag))
    {
        VerifyOrExit(aLength >= sizeof(HandshakeResponse), OT_NOOP);
        HandleHandshake(*conn, static_cast<const HandshakeResponse &>(*header));
    }
    else
    {
        HandleFrame(*conn, aFrame, aLength);
    }

exit:
    return;
}

void Btp::HandleHandshake(Connection &aConn, const HandshakeResponse &aResponse)
{
    Session &session = aConn.mSession;

    VerifyOrExit(session.mState == kStateSubscribe, OT_NOOP);

    otLogInfoBtp("Receive Handshake Request: FragSize=%d, Window=%d", aResponse.GetSegmentSize(),
                 aResponse.GetWindowSize());

    session.mState = kStateConnected;
    session.mMtu   = aResponse.GetSegmentSize();

    session.mTxSeqnoCurrent = 255;
    session.mTxSeqnoAcked   = 255;
    session.mTxWindow       = aResponse.GetWindowSize();

    session.mRxSeqnoCurrent = 0;
    session.mRxSeqnoAcked   = 255;
    session.mRxWindow       = kWindowSize;

    otLogNoteBtp("BTP connected: MTU=%d, TxWindow=%d, RxWindow=%d", session.mMtu, session.mTxWindow, session.mRxWindow);

    HandleSessionReady(aConn);

exit:
    return;
}

} // namespace Toble
} // namespace ot

#endif // #if OPENTHREAD_CONFIG_TOBLE_ENABLE && OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
