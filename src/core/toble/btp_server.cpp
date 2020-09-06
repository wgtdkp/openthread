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
 *   This file contains the implementation for BTP server.
 */

#include "toble/btp.hpp"
#include "toble/toble.hpp"

#include "common/debug.hpp"
#include "common/locator-getters.hpp"
#include "common/logging.hpp"

#if OPENTHREAD_CONFIG_TOBLE_ENABLE && OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE

namespace ot {
namespace Toble {

void Btp::HandleC2Subscribed(Platform::Connection *aPlatConn, bool aIsSubscribed)
{
    Connection *conn = Get<ConnectionTable>().Find(aPlatConn);

    OT_ASSERT(!Get<Toble>().IsCentral());

    VerifyOrExit(conn != NULL, OT_NOOP);

    if (aIsSubscribed)
    {
        HandshakeResponse handshakeResponse(conn->mSession.mMtu, kWindowSize);

        VerifyOrExit(conn->mSession.mState == kStateHandshake, OT_NOOP);
        otLogDebgBtp("HandleC2Subscribed: subscribed");
        otLogInfoBtp("Send Handshake Response: SegSize=%d, Window=%d", handshakeResponse.GetSegmentSize(),
                     handshakeResponse.GetWindowSize());
        GattSend(*conn, reinterpret_cast<uint8_t *>(&handshakeResponse), sizeof(handshakeResponse));
        conn->mSession.mState = kStateSubscribe;
    }
    else
    {
        VerifyOrExit(conn->mSession.mState != kStateIdle, OT_NOOP);
        otLogDebgBtp("HandleC2Subscribed: unsubscribed");

        // Optional future enhancement: Trigger a BLE disconnect on un-subscribe.

        conn->mSession.mState = kStateIdle;
    }

exit:
    return;
}

void Btp::HandleC2NotificateDone(Platform::Connection *aPlatConn)
{
    Connection *conn = Get<ConnectionTable>().Find(aPlatConn);

    OT_ASSERT(!Get<Toble>().IsCentral());

    otLogDebgBtp("HandleC2NotificateDone");
    VerifyOrExit(conn != NULL, OT_NOOP);

    switch (conn->mSession.mState)
    {
    case kStateIdle:
        // fall through

    case kStateHandshake:
        break;

    case kStateSubscribe:
        otLogInfoBtp("BTP connected: MTU=%d, TxWindow = %d", conn->mSession.mMtu, conn->mSession.mTxWindow);
        conn->mSession.mState = kStateConnected;
        HandleSessionReady(*conn);
        break;

    case kStateConnected:
        HandleSentData(*conn);
        break;
    }

exit:
    return;
}

void Btp::HandleC1Write(Platform::Connection *aPlatConn, const uint8_t *aFrame, uint16_t aLength)
{
    Connection *  conn   = Get<ConnectionTable>().Find(aPlatConn);
    const Header *header = reinterpret_cast<const Header *>(aFrame);

    otLogDebgBtp("HandleC1Write");
    OT_ASSERT(!Get<Toble>().IsCentral());

    VerifyOrExit(aLength > 0, OT_NOOP);

    VerifyOrExit(conn != NULL, OT_NOOP);

    if (header->GetFlag(Header::kHandshakeFlag))
    {
        VerifyOrExit(aLength >= sizeof(HandshakeRequest), OT_NOOP);
        HandleHandshake(*conn, static_cast<const HandshakeRequest &>(*header));
    }
    else
    {
        HandleDataFrame(*conn, aFrame, aLength);
    }

exit:
    return;
}

void Btp::HandleHandshake(Connection &aConn, const HandshakeRequest &aRequest)
{
    Session &session = aConn.mSession;

    VerifyOrExit(session.mState == kStateIdle, OT_NOOP);

    otLogInfoBtp("Receive Handshake Request: Mtu=%d, Window=%d", aRequest.GetMtu(), aRequest.GetWindowSize());

    session.mMtu = aRequest.GetMtu();

    if (session.mMtu == 0)
    {
        session.mMtu = Get<Platform>().GetConnMtu(aConn.mPlatConn);
    }

    if (session.mMtu < kSegmentSizeDefault)
    {
        session.mMtu = kSegmentSizeDefault;
    }
    else
    {
        session.mMtu -= kAttHeaderSize;

        if (session.mMtu > kSegmentSizeMax)
        {
            session.mMtu = kSegmentSizeMax;
        }
    }

    session.mState = kStateHandshake;

    session.mTxSeqnoCurrent = 0;
    session.mTxSeqnoAcked   = 0;
    session.mTxWindow       = aRequest.GetWindowSize();

    session.mRxSeqnoCurrent = 255;
    session.mRxSeqnoAcked   = 255;
    session.mRxWindow       = kWindowSize;

    otLogNoteBtp("BTP connected: MTU=%d, TxWindow=%d, RxWindow=%d", session.mMtu, session.mTxWindow, session.mRxWindow);

    session.SetTimer(kKeepAliveDelay);
    UpdateTimer();

exit:
    return;
}

} // namespace Toble
} // namespace ot

#endif // #if OPENTHREAD_CONFIG_TOBLE_ENABLE && OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE