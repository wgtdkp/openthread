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

void Btp::HandleC1WriteDone(Platform::Connection *aPlatConn)
{
    Connection *conn = Get<ConnectionTable>().Find(aPlatConn);

    OT_ASSERT(Get<Toble>().IsCentral());

    VerifyOrExit(conn != NULL, OT_NOOP);

    switch (conn->mSession.mState)
    {
    case kStateIdle:
        break;

    case kStateHandshake:
        otLogDebgBle("Btp::SubscribeC2");
        Get<Platform>().SubscribeC2(aPlatConn, true);
        break;

    case kStateConnected:
        HandleSentData(*conn);
        break;
    }

exit:
    return;
}

void Btp::HandleC2Indication(Platform::Connection *aPlatConn, const uint8_t *aFrame, uint16_t aLength)
{
    Connection * conn  = Get<ConnectionTable>().Find(aPlatConn);
    const Frame *frame = reinterpret_cast<const Frame *>(aFrame);

    OT_ASSERT(Get<Toble>().IsCentral());

    VerifyOrExit(conn != NULL, OT_NOOP);

    VerifyOrExit(conn->mSession.mState == kStateHandshake || conn->mSession.mState == kStateConnected, OT_NOOP);

    VerifyOrExit(aLength > 0, OT_NOOP);

    if (frame->IsHandshake())
    {
        VerifyOrExit(aLength >= sizeof(HandshakeResponse), OT_NOOP);
        HandleHandshake(*conn, static_cast<const HandshakeResponse &>(*frame));
    }
    else
    {
        HandleDataFrame(*conn, aFrame, aLength);
    }

exit:
    return;
}

void Btp::HandleHandshake(Connection &aConn, const HandshakeResponse &aResponse)
{
    Session &session = aConn.mSession;

    VerifyOrExit(session.mState == kStateHandshake, OT_NOOP);

    otLogDebgBle("Btp::HandleHandshake");

    session.mState = kStateConnected;

    session.mMtu = aResponse.GetSegmentSize();

    session.mTxSeqnoCurrent = 255;
    session.mTxSeqnoAcked   = 255;
    session.mTxWindow       = aResponse.GetWindowSize();

    session.mRxSeqnoCurrent = 0;
    session.mRxSeqnoAcked   = 255;
    session.mRxWindow       = kWindowSize;

    otLogDebgBle("BTP connected: MTU=%d, TxWindow = %d", session.mMtu, session.mTxWindow);

    if ((session.mSendOffset != session.mSendLength) || session.GetRxWindowRemaining() <= 1)
    {
        SendData(aConn);
    }
    else
    {
        session.SetTimer(kKeepAliveDelay);
        UpdateTimer();
    }

exit:
    return;
}

} // namespace Toble
} // namespace ot

#endif // #if OPENTHREAD_CONFIG_TOBLE_ENABLE && OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
