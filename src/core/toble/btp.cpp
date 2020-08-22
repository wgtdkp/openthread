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
    , mTxTask(aInstance, &Btp::HandleTxTask, this)
{
}

void Btp::Start(Connection &aConn)
{
    otLogNoteBtp("Btp::Start");

    aConn.mSession.mState = kStateIdle;
}

void Btp::Stop(Connection &aConn)
{
    otLogNoteBtp("Btp::Stop");

    Get<Platform>().SubscribeC2(aConn.mPlatConn, false);
    Reset(aConn.mSession);
}

void Btp::Send(Connection &aConn, const uint8_t *aBuf, uint16_t aLength)
{
    otLogNoteBtp("Btp::Send");

    aConn.mSession.mSendBuf    = aBuf;
    aConn.mSession.mSendLength = aLength;
    aConn.mSession.mSendOffset = 0;

    TransmitTaskPost(aConn.mSession);
}

void Btp::HandleSessionReady(Connection &aConn)
{
    ConnectionTimerRefresh(aConn);
    TransmitTaskPost(aConn.mSession);
}

void Btp::SendData(Connection &aConn)
{
    Session &       session = aConn.mSession;
    Frame &         frame   = mTxFrame;
    Frame::Iterator iterator;
    uint16_t        payloadLength = 0;

    VerifyOrExit(session.mState == kStateConnected, OT_NOOP);

    otLogNoteBtp("BTP send");

    frame.Init(iterator);

    if (session.mRxSeqnoCurrent != session.mRxSeqnoAcked)
    {
        frame.AppendAck(iterator, session.mRxSeqnoCurrent);
    }

    frame.AppendSeqNum(iterator, session.mTxSeqnoCurrent + 1);

    if (session.HasFragmentToSend())
    {
        frame.AppendPayload(iterator, session.mSendBuf, session.mSendOffset, session.mSendLength, session.mMtu,
                            payloadLength);
    }

    VerifyOrExit(frame.IsValid(), OT_NOOP);
    VerifyOrExit(GattSend(aConn, reinterpret_cast<uint8_t *>(&frame), iterator) == OT_ERROR_NONE, OT_NOOP);

    otLogNoteBtp("Send BTP msg, %s", frame.ToString().AsCString());

    if (frame.IsAck())
    {
        session.mRxSeqnoAcked         = session.mRxSeqnoCurrent;
        session.mTransmitKeepAliveAck = false;
    }

    if (payloadLength != 0)
    {
        session.mSendOffset += payloadLength;
    }

    session.mTxSeqnoCurrent += 1;

    TransmitTaskPost(session);

exit:
    return;
}

void Btp::TransmitTaskPost(Session &aSession)
{
    aSession.mTransmitRequest = true;
    mTxTask.Post();
}

void Btp::HandleTxTask(Tasklet &aTasklet)
{
    aTasklet.GetOwner<Btp>().SendNextFragment();
}

void Btp::SendNextFragment(void)
{
    for (Connection *conn = Get<ConnectionTable>().GetFirst(); conn != NULL;
         conn             = Get<ConnectionTable>().GetNext(conn))
    {
        Session &session = conn->mSession;

        if (session.mTransmitRequest == true)
        {
            session.mTransmitRequest = false;

            // a local peer SHALL not send packets if the remote peer's receive window has one slot
            // open and the local peer does not have a pending packet acknowledgement.

            if ((session.HasFragmentToSend() &&
                 (((session.GetTxWindowRemaining() > 1) && !session.HasPendingAckToSend()) ||
                  session.HasPendingAckToSend())) ||
                session.GetRxWindowRemaining() <= 1 || session.mTransmitKeepAliveAck)
            {
                SendData(*conn);
            }
            else
            {
                StartTimer(session, kKeepAliveDelay);
            }
        }
    }
}

void Btp::HandleSentData(Connection &aConn)
{
    Session &session = aConn.mSession;

    if ((session.mSendOffset >= session.mSendLength) && (session.mSendBuf != NULL))
    {
        session.mSendBuf = NULL;

        HandleGattSentDone(aConn, OT_ERROR_NONE);
    }

    TransmitTaskPost(session);
}

void Btp::HandleDataFrame(Connection &aConn, const uint8_t *aFrame, uint16_t aLength)
{
    Session &      session = aConn.mSession;
    const Frame &  frame(*reinterpret_cast<const Frame *>(aFrame));
    const uint8_t *cur = &aFrame[1];

    VerifyOrExit(session.mState == kStateConnected, OT_NOOP);
    VerifyOrExit(frame.IsValid(), OT_NOOP);

    otLogNoteBtp("Received BTP msg, %s", frame.ToString().AsCString());

    ConnectionTimerRefresh(aConn);

    if (frame.IsAck())
    {
        session.mTxSeqnoAcked = *cur++;
    }

    OT_ASSERT(static_cast<uint8_t>(session.mRxSeqnoCurrent + 1) == cur[0]);
    session.mRxSeqnoCurrent = *cur++;

    if (frame.IsData())
    {
        if (frame.GetFlag(Header::kBeginFlag))
        {
            session.mReceiveOffset = 0;
            session.mReceiveLength = ReadUint16(cur);
            cur += sizeof(uint16_t);
        }

        VerifyOrExit(session.mReceiveLength != 0, OT_NOOP);

        aLength -= cur - aFrame;

        memcpy(session.mRxBuf + session.mReceiveOffset, cur, aLength);
        session.mReceiveOffset += aLength;

        if (frame.GetFlag(Header::kEndFlag))
        {
            uint16_t length;

            OT_ASSERT(session.mReceiveOffset == session.mReceiveLength);

            // all ToBLE transports strip MAC FCS.
            length                 = session.mReceiveLength + Mac::Frame::GetFcsSize();
            session.mReceiveLength = 0;

            HandleGattReceiveDone(aConn, session.mRxBuf, length, OT_ERROR_NONE);
        }
    }

    TransmitTaskPost(session);

exit:
    return;
}

void Btp::HandleTimer(Timer &aTimer)
{
    aTimer.GetOwner<Btp>().HandleTimer();
}

void Btp::HandleTimer(void)
{
    otLogNoteBtp("BTP timer fired");

    for (Connection *conn = Get<ConnectionTable>().GetFirst(); conn != NULL;
         conn             = Get<ConnectionTable>().GetNext(conn))
    {
        Session &session = conn->mSession;

        if (!session.mIsTimerSet || !session.IsTimerExpired())
        {
            continue;
        }

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
                session.mTransmitKeepAliveAck = true;
                TransmitTaskPost(conn->mSession);
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

    aSession.mSendBuf              = NULL;
    aSession.mSendOffset           = 0;
    aSession.mSendLength           = 0;
    aSession.mReceiveLength        = 0;
    aSession.mIsTimerSet           = false;
    aSession.mTransmitKeepAliveAck = false;
    aSession.mState                = kStateIdle;

    UpdateTimer();
}

otError Btp::GattSend(Connection &aConn, const uint8_t *aBuffer, uint16_t aLength)
{
    otError error = OT_ERROR_FAILED;

    ConnectionTimerRefresh(aConn);

    if (Get<Toble>().IsCentral())
    {
#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
        otLogNoteBtp("Btp::SendData: WriteC1");
        error = Get<Platform>().WriteC1(aConn.mPlatConn, aBuffer, aLength);
#endif
    }
    else
    {
#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
        otLogNoteBtp("Btp::SendData: NotifyC2");
        error = Get<Platform>().NotifyC2(aConn.mPlatConn, aBuffer, aLength);
#endif
    }

    return error;
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
