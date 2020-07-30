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
 *   This file contains the implementation for ToBLE BTP transport.
 */

#include "toble/btp.hpp"

#include <openthread/platform/toble.h>

#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "common/locator-getters.hpp"
#include "common/logging.hpp"
#include "utils/wrap_string.h"

#if OPENTHREAD_CONFIG_TOBLE_ENABLE

namespace ot {
namespace Toble {

using ot::Encoding::LittleEndian::ReadUint16;
using ot::Encoding::LittleEndian::WriteUint16;

Btp::Btp(Instance &aInstance)
    : InstanceLocator(aInstance)
    , mTimer(aInstance, &Btp::HandleTimer, this)
{
}

void Btp::Start(Connection &aConn)
{
    otLogNoteBtp("Btp::Start");

    aConn.mSession.mState = kStateIdle;
}

void Btp::HandleConnectionReady(Platform::Connection *aPlatConn)
{
    Connection *conn = Get<ConnectionTable>().Find(aPlatConn);

    otLogNoteBtp("Btp::ConnectionReady (%p -> %p)", aPlatConn, conn);

    VerifyOrExit(conn != NULL, OT_NOOP);
    VerifyOrExit(Get<Toble>().IsCentral(), OT_NOOP);

#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
    otLogNoteBtp("Btp::HandleConnectionReady: WriteC1(CONNECT_REQ)");
    conn->mSession.mRequest.Init(Get<Platform>().GetConnMtu(aPlatConn), kWindowSize);
    Get<Platform>().WriteC1(aPlatConn, &conn->mSession.mRequest, sizeof(conn->mSession.mRequest));
    conn->mSession.mState = kStateHandshake;
#endif

exit:
    return;
}

void Btp::Stop(Connection &aConn)
{
    otLogNoteBtp("Btp::Stop");

    // Optional future enhancement: On central we can consider
    // unsubscribe from C2.

    Reset(aConn.mSession);
}

void Btp::Send(Connection &aConn, const uint8_t *aBuf, uint16_t aLength)
{
    otLogNoteBtp("Btp::Send");

    aConn.mSession.mSendBuf    = aBuf;
    aConn.mSession.mSendLength = aLength;
    aConn.mSession.mSendOffset = 0;

    SendData(aConn);
}

void Btp::SendData(Connection &aConn)
{
    Session &session = aConn.mSession;
    uint8_t *cur     = &session.mTxBuf[1];
    uint16_t frameLength;

    OT_UNUSED_VARIABLE(frameLength);
    VerifyOrExit(!session.mIsSending && session.mState == kStateConnected, OT_NOOP);

    session.mTxBuf[0] = 0;

    otLogNoteBtp("BTP send");

    if (session.mRxSeqnoCurrent == session.mRxSeqnoAcked)
    {
        VerifyOrExit(session.GetTxWindowRemaining() > 1, OT_NOOP);
    }
    else
    {
        session.mTxBuf[0] |= Frame::kAckFlag;
        otLogNoteBtp("  ack=%u", session.mRxSeqnoCurrent);

        *cur++                = session.mRxSeqnoCurrent;
        session.mRxSeqnoAcked = session.mRxSeqnoCurrent;
    }

    *cur++ = ++session.mTxSeqnoCurrent;
    otLogNoteBtp("  seq=%u ", cur[-1]);

    if (session.mSendBuf != NULL)
    {
        uint16_t segmentLength;
        uint16_t segmentRemaining;

        if (session.mSendOffset == 0)
        {
            session.mTxBuf[0] |= Frame::kBeginFlag;
            otLogNoteBtp("  start ------------>");
            otLogNoteBtp("  len=%u", session.mSendLength);

            WriteUint16(session.mSendLength, cur);
            cur += sizeof(uint16_t);
        }
        else
        {
            session.mTxBuf[0] |= Frame::kContinueFlag;
            otLogNoteBtp("  off=%u", session.mSendOffset);
        }

        segmentRemaining = session.mMtu - (cur - session.mTxBuf);
        segmentLength    = session.mSendLength - session.mSendOffset;

        if (segmentLength > segmentRemaining)
        {
            segmentLength = segmentRemaining;
        }

        otLogNoteBtp("  segLen=%u", segmentLength);
        memcpy(cur, session.mSendBuf + session.mSendOffset, segmentLength);
        cur += segmentLength;
        session.mSendOffset += segmentLength;

        if (session.mSendOffset >= session.mSendLength)
        {
            session.mTxBuf[0] |= Frame::kEndFlag;
            // otLogNoteBtp("  end");
            otLogNoteBtp("  end -------------->");
            otLogNoteBtp("BTP::Send Done %d", session.mSendLength);
        }
    }

    frameLength = cur - session.mTxBuf;

    mTimer.Stop();
    session.mIsSending = true;

    GattSend(aConn, session.mTxBuf, frameLength);

exit:
    return;
}

void Btp::HandleSentData(Connection &aConn)
{
    Session &session = aConn.mSession;

    session.mIsSending = false;

    if ((session.mSendOffset != session.mSendLength) || session.GetRxWindowRemaining() <= 1)
    {
        SendData(aConn);
    }
    else
    {
        session.SetTimer(kKeepAliveDelay);
        UpdateTimer();
    }

    if ((session.mSendOffset >= session.mSendLength) && (session.mSendBuf != NULL))
    {
        session.mSendBuf = NULL;

        HandleGattSentDone(aConn, OT_ERROR_NONE);
    }
}

void Btp::HandleDataFrame(Connection &aConn, const uint8_t *aFrame, uint16_t aLength)
{
    Session &      session = aConn.mSession;
    const Frame &  frame(*reinterpret_cast<const Frame *>(aFrame));
    const uint8_t *cur = &aFrame[1];

    VerifyOrExit(session.mState == kStateConnected, OT_NOOP);

    ConnectionTimerRefresh(aConn);

    otLogNoteBtp("BTP receive");

    if (frame.IsBegin())
    {
        otLogNoteBtp("  start <------------");
    }

    if (frame.IsAck())
    {
        session.mTxSeqnoAcked = *cur++;
        otLogNoteBtp("  ack=%d", session.mTxSeqnoAcked);
    }

    // OT_ASSERT(static_cast<uint8_t>(session.mRxSeqnoCurrent + 1) == cur[0]);
    if (static_cast<uint8_t>(session.mRxSeqnoCurrent + 1) != cur[0])
    {
        otLogNoteBtp("  mRxSeqnoCurrent+1=%d cur[0]=%d ---ERROR------------>", (session.mRxSeqnoCurrent + 1), cur[0]);
        return;
    }

    session.mRxSeqnoCurrent = *cur++;
    otLogNoteBtp("  seq=%d ", session.mRxSeqnoCurrent);

    if (frame.IsBegin())
    {
        session.mReceiveOffset = 0;
        session.mReceiveLength = ReadUint16(cur);
        cur += sizeof(uint16_t);
        otLogNoteBtp("  len=%u", session.mReceiveLength);
    }

    aLength -= cur - aFrame;

    memcpy(session.mRxBuf + session.mReceiveOffset, cur, aLength);
    session.mReceiveOffset += aLength;

    if (session.mSendOffset < session.mSendLength || session.GetRxWindowRemaining() <= 1)
    {
        SendData(aConn);
    }
    else
    {
        session.SetTimer(kKeepAliveDelay);
        UpdateTimer();
    }

    if (frame.IsEnd())
    {
        uint16_t length;

        OT_ASSERT(session.mReceiveOffset == session.mReceiveLength);
        otLogNoteBtp("  end <--------------");
        otLogNoteBtp("BTP received %d", session.mReceiveOffset);

        // All ToBLE transports strip MAC FCS.
        length = session.mReceiveLength + Mac::Frame::GetFcsSize();

        HandleGattReceiveDone(aConn, session.mRxBuf, length, OT_ERROR_NONE);
    }

exit:
    return;
}

void Btp::HandleTimer(Timer &aTimer)
{
    aTimer.GetOwner<Btp>().HandleTimer();
}

void Btp::HandleTimer(void)
{
    for (Connection *conn = Get<ConnectionTable>().GetFirst(); conn != NULL;
         conn             = Get<ConnectionTable>().GetNext(conn))
    {
        Session &session = conn->mSession;

        if (!session.mIsTimerSet || !session.IsTimerExpired())
        {
            continue;
        }

        otLogNoteBtp("BTP timer fired");

        session.mIsTimerSet = false;

        switch (session.mState)
        {
        case kStateIdle:
            break;

        case kStateHandshake:
        case kStateSubscribe:
            // did not receive handshake response, close the connection
            Reset(session);
            break;

        case kStateConnected:
            if (session.GetRxWindowRemaining())
            {
                SendData(*conn);
            }
            else
            {
                // did not receive an ACK, close the connection
                Reset(session);
            }
            break;
        }
    }

    UpdateTimer();
}

void Btp::UpdateTimer(void)
{
    Session *nextEvent = NULL;

    for (Connection *conn = Get<ConnectionTable>().GetFirst(); conn != NULL;
         conn             = Get<ConnectionTable>().GetNext(conn))
    {
        Session &session = conn->mSession;

        if (!session.mIsTimerSet)
        {
            continue;
        }

        if ((nextEvent == NULL) || (session.GetTimer() < nextEvent->GetTimer()))
        {
            nextEvent = &session;
        }
    }

    if (nextEvent != NULL)
    {
        mTimer.Start(nextEvent->GetTimer());
    }
}

void Btp::Reset(Session &aSession)
{
    otLogNoteBtp("BTP::Reset");

    aSession.mSendBuf    = NULL;
    aSession.mSendOffset = 0;
    aSession.mSendLength = 0;
    aSession.mIsSending  = false;
    aSession.mIsTimerSet = false;
    aSession.mState      = kStateIdle;

    UpdateTimer();
}

void Btp::GattSend(Connection &aConn, uint8_t *aBuffer, uint16_t aLength)
{
    ConnectionTimerRefresh(aConn);

    if (Get<Toble>().IsCentral())
    {
#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
        otLogNoteBtp("Btp::SendData: WriteC1");
        Get<Platform>().WriteC1(aConn.mPlatConn, aBuffer, aLength);
#endif
    }
    else
    {
#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
        Get<Platform>().IndicateC2(aConn.mPlatConn, aBuffer, aLength);
#endif
    }
}

void Btp::HandleGattSentDone(Connection &aConn, otError aError)
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

void Btp::HandleGattReceiveDone(Connection &aConn, uint8_t *aBuffer, uint16_t aLength, otError aError)
{
    if (Get<Toble>().IsCentral())
    {
#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
        Get<Central::Controller>().HandleTransportReceiveDone(aConn, aBuffer, aLength, aError);
#endif
    }
    else
    {
#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
        Get<Peripheral::Controller>().HandleTransportReceiveDone(aConn, aBuffer, aLength, aError);
#endif
    }
}

void Btp::ConnectionTimerRefresh(Connection &aConn)
{
    if (Get<Toble>().IsCentral())
    {
#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
        Get<Central::Controller>().ConnectionTimerRefresh(aConn);
#endif
    }
    else
    {
#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
        Get<Peripheral::Controller>().ConnectionTimerRefresh(aConn);
#endif
    }
}

} // namespace Toble
} // namespace ot

#endif // #if OPENTHREAD_CONFIG_TOBLE_ENABLE