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
#include "common/random.hpp"
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
    otLogInfoBtp("Start");

    aConn.mSession.mState = kStateIdle;
}

void Btp::Stop(Connection &aConn)
{
    otLogInfoBtp("Stop");

    Get<Platform>().SubscribeC2(aConn.mPlatConn, false);
    Reset(aConn.mSession);
}

void Btp::Send(Connection &aConn, const uint8_t *aBuffer, uint16_t aLength)
{
    otLogInfoBtp("Send");

    aConn.mSession.mSendBuf    = aBuffer;
    aConn.mSession.mSendLength = aLength;
    aConn.mSession.mSendOffset = 0;
    aConn.mSession.mIsSending  = false;

    SendData(aConn);
}

void Btp::HandleSessionReady(Connection &aConn)
{
    Session &session = aConn.mSession;

    if (session.HasFragmentToSend() || session.GetRxWindowRemaining() <= 1)
    {
        SendData(aConn);
    }
    else
    {
        StartTimer(session, kKeepAliveDelay);
    }

    ConnectionTimerRefresh(aConn);
}

void Btp::SendData(Connection &aConn)
{
    Session &session = aConn.mSession;
    Frame &  frame   = mTxFrame;
    uint8_t  frameLength;
    uint16_t segmentLength;

    VerifyOrExit(session.mState == kStateConnected, OT_NOOP);

    frameLength = frame.Init();

    if ((session.mRxSeqnoCurrent != session.mRxSeqnoAcked) || session.mSendAck)
    {
        frameLength = frame.AppendAck(session.mRxSeqnoCurrent);
    }

    if (session.HasFragmentToSend() && (session.GetTxWindowRemaining() > 1))
    {
        frameLength = frame.AppendPayload(frameLength, session.mTxSeqnoCurrent + 1, session.mSendBuf,
                                          session.mSendOffset, session.mSendLength, session.mMtu, segmentLength);
    }

    VerifyOrExit(!frame.IsEmpty(), OT_NOOP);

    if (frame.IsAck())
    {
        otLogInfoBtp("SendData(ackNum=%d)", session.mRxSeqnoCurrent);
    }
    else if (frame.IsData())
    {
        otLogInfoBtp("SendData(seqNum=%d offset=%d length=%d)", session.mTxSeqnoCurrent + 1, session.mSendOffset,
                     segmentLength);
    }
    else
    {
        otLogInfoBtp("SendData(ackNum=%d seqNum=%d offset=%d length=%d)", session.mRxSeqnoCurrent,
                     session.mTxSeqnoCurrent + 1, session.mSendOffset, segmentLength);
    }

    VerifyOrExit(GattSend(aConn, reinterpret_cast<uint8_t *>(&frame), frameLength) == OT_ERROR_NONE, OT_NOOP);

    otLogNoteBtp("Send BTP msg, %s", frame.ToString().AsCString());

    if (frame.IsAck())
    {
        session.mRxSeqnoAcked = session.mRxSeqnoCurrent;
        session.mSendAck      = false;
    }

    if (frame.IsData())
    {
        if (!session.mIsSending)
        {
            session.mIsSending  = true;
            session.mIsTimerSet = false;
        }

        session.mQueue.Push(session.mTxSeqnoCurrent + 1, session.mSendOffset);

        session.mSendOffset += segmentLength;
        session.mTxSeqnoCurrent += 1;

        if (!session.mIsTimerSet)
        {
            otLogInfoBtp("StartTimer(kAckTimeout)");
            StartTimer(session, kAckTimeout);
        }
    }

    mTxTask.Post();

exit:
    return;
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

        if ((session.HasFragmentToSend() && (session.GetTxWindowRemaining() > 1)) ||
            session.GetRxWindowRemaining() <= 1 || session.mSendAck)
        {
            SendData(*conn);
        }
    }
}

void Btp::HandleSentData(Connection &aConn)
{
    OT_UNUSED_VARIABLE(aConn);
    mTxTask.Post();
}

void Btp::HandleFrame(Connection &aConn, const uint8_t *aFrame, uint16_t aLength)
{
    const Frame &  frame(*reinterpret_cast<const Frame *>(aFrame));
    Session &      session = aConn.mSession;
    const uint8_t *cur     = &aFrame[1];
    uint8_t        ackNum  = 0;
    uint8_t        seqNum  = 0;

#if 1
    //    if (Get<Toble>().IsCentral())
    {
        if (Random::NonCrypto::GetUint8() % 10 == 0)
        {
            if (frame.IsAck())
            {
                ackNum = *cur++;
            }

            if (frame.IsData())
            {
                seqNum = cur[0];
            }

            otLogInfoBtp("Drop (seqNum=%d, AckNum=%d)~~~~~~~~~~~~~~~~~~~~~~~~~~~", seqNum, ackNum);
            ExitNow();
        }
    }
#endif

    VerifyOrExit(session.mState == kStateConnected, OT_NOOP);
    VerifyOrExit(frame.IsValid(), OT_NOOP);

    otLogNoteBtp("Received BTP msg, %s", frame.ToString().AsCString());

    ConnectionTimerRefresh(aConn);

    if (frame.IsAck())
    {
        ackNum = *cur++;

        if (CompareUint8(ackNum, session.mTxSeqnoAcked) < 0)
        {
            // the received ACK Number is smaller than Acked sequence number.
            ExitNow();
        }
        else if (ackNum == session.mTxSeqnoAcked)
        {
            // receives a duplicated ACK frame.
            session.mAckCount++;
            if (session.mAckCount >= 2)
            {
                uint16_t offset = 0;

                session.mAckCount = 0;

                // find the message offset based on the sequence number.
                VerifyOrExit(session.mQueue.Find(ackNum + 1, offset) == OT_ERROR_NONE, OT_NOOP);
                session.mSendOffset     = offset;
                session.mTxSeqnoCurrent = ackNum;

                session.mQueue.FreeAll();
                StartTimer(session, kAckTimeout);
                otLogInfoBtp("mAckCount >= 2: ackNum=%d mTxSeqnoCurrent=%d offset=%d", ackNum, session.mTxSeqnoCurrent,
                             offset);

                mTxTask.Post();
                ExitNow();
            }
        }
        else
        {
            session.mTxSeqnoAcked    = ackNum;
            session.mAckCount        = 0;
            session.mTransmitRetries = 0;
            session.mQueue.Free(ackNum);

            otLogInfoBtp("Start(kAckTimeout) 1");
            StartTimer(session, kAckTimeout);
        }

        if ((session.mQueue.Size() == 0) && (session.mSendOffset >= session.mSendLength) && (session.mSendBuf != NULL))
        {
            session.mSendBuf = NULL;
            // session.mState   = kStateConnected;
            session.mIsSending = false;

            otLogInfoBtp("HandleGattSentDone");
            otLogInfoBtp("Start(kKeepAliveDelay) 2");
            StartTimer(session, kKeepAliveDelay);

            HandleGattSentDone(aConn, OT_ERROR_NONE);
        }
    }

    if (frame.IsData())
    {
        // Received a data frame.
        seqNum = *cur++;

        if (static_cast<uint8_t>(session.mRxSeqnoCurrent + 1) != seqNum)
        {
            // Receive a wrong BTP fragment, send ACK to the peer to notify it.
            session.mSendAck = true;

            otLogInfoBtp("SendAck: mRxSeqnoCurrent=%d, seqNum=%d", session.mRxSeqnoCurrent, seqNum);
            mTxTask.Post();
            ExitNow();
        }

        if (frame.GetFlag(Header::kBeginFlag))
        {
            otLogInfoBtp("BtpReceiveStart");
            session.mReceiveOffset = 0;
            session.mReceiveLength = ReadUint16(cur);
            cur += sizeof(uint16_t);
        }

        VerifyOrExit(session.mReceiveLength != 0, OT_NOOP);

        session.mRxSeqnoCurrent = seqNum;

        aLength -= cur - aFrame;
        otLogInfoBtp("ReceiveFrag: seqNum:%d offset:%d length:%d", seqNum, session.mReceiveOffset, aLength);

        memcpy(session.mRxBuf + session.mReceiveOffset, cur, aLength);
        session.mReceiveOffset += aLength;

        if (frame.GetFlag(Header::kEndFlag))
        {
            uint16_t length;

            otLogInfoBtp("kEndFlag: mReceiveOffset=%d session.mReceiveLength=%d", session.mReceiveOffset,
                         session.mReceiveLength);

            VerifyOrExit(session.mReceiveOffset == session.mReceiveLength, session.mReceiveLength = 0);

            // All ToBLE transports strip MAC FCS.
            length = session.mReceiveLength + Mac::Frame::GetFcsSize();

            session.mReceiveLength = 0;
            session.mSendAck       = true;

            otLogInfoBtp("HandleGattReceiveDone");
            HandleGattReceiveDone(aConn, session.mRxBuf, length, OT_ERROR_NONE);
        }
    }

    mTxTask.Post();

exit:
    return;
}

void Btp::HandleTimer(Timer &aTimer)
{
    aTimer.GetOwner<Btp>().HandleTimer();
}

void Btp::HandleTimer(void)
{
    otLogInfoBtp("BTP timer fired");

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
            if (session.mIsSending)
            {
                // ACK timeout, retransmit the fragment.
                if (session.mTransmitRetries < kMaxTransmitRetries)
                {
                    otLogInfoBtp("mTransmitRetries=%d", session.mTransmitRetries);
                    session.mTransmitRetries++;
                    session.mQueue.Front(session.mTxSeqnoCurrent, session.mSendOffset);
                    session.mTxSeqnoCurrent--;
                    session.mQueue.FreeAll();
                    SendData(*conn);
                }
                else
                {
                    Reset(session);
                }
            }
            else
            {
                if (session.GetRxWindowRemaining())
                {
                    otLogInfoBtp("SendData(*conn)");
                    session.mSendAck = true;
                    SendData(*conn);
                    StartTimer(session, kKeepAliveDelay);
                }
                else
                {
                    // did not receive an ACK, close the connection
                    Reset(session);
                }
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
    aSession.mIsTimerSet = false;
    aSession.mIsSending  = false;
    aSession.mState      = kStateIdle;

    UpdateTimer();
}

otError Btp::GattSend(Connection &aConn, const uint8_t *aBuffer, uint16_t aLength)
{
    otError error = OT_ERROR_FAILED;

    ConnectionTimerRefresh(aConn);

    if (Get<Toble>().IsCentral())
    {
#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
        otLogDebgBtp("WriteC1");
        error = Get<Platform>().WriteC1(aConn.mPlatConn, aBuffer, aLength);
#endif
    }
    else
    {
#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
        otLogDebgBtp("IndicateC2");
        error = Get<Platform>().IndicateC2(aConn.mPlatConn, aBuffer, aLength);
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

void Btp::Queue::Push(uint8_t aSeqNum, uint16_t aOffset)
{
    mEntries[mSize].mSeqNum = aSeqNum;
    mEntries[mSize].mOffset = aOffset;
    mSize++;

    otLogInfoBtp("%s: i=%d, Offset=%d, SeqNum=%d", __func__, mSize - 1, aOffset, aSeqNum);
}

void Btp::Queue::Front(uint8_t &aSeqNum, uint16_t &aOffset)
{
    OT_ASSERT(mSize != 0);
    aSeqNum = mEntries[0].mSeqNum;
    aOffset = mEntries[0].mOffset;
}

otError Btp::Queue::Find(uint8_t aSeqNum, uint16_t &aOffset)
{
    otError error = OT_ERROR_NOT_FOUND;

    for (size_t i = 0; i < mSize; i++)
    {
        if (aSeqNum == mEntries[i].mSeqNum)
        {
            aOffset = mEntries[i].mOffset;
            ExitNow(error = OT_ERROR_NONE);
        }
    }

exit:
    return error;
}

void Btp::Queue::Free(uint8_t aAckNum)
{
    Entry *entry = NULL;

    for (size_t i = 0; i < mSize; i++)
    {
        if (CompareUint8(aAckNum, mEntries[i].mSeqNum) >= 0)
        {
            otLogInfoBtp("%s: i=%d, Offset=%d, SeqNum=%d", __func__, i, mEntries[i].mOffset, mEntries[i].mSeqNum);
            entry = &mEntries[i];
        }
        else
        {
            break;
        }
    }

    if (entry != NULL)
    {
        mSize = mEntries + mSize - (entry + 1);

        if (mSize != 0)
        {
            memmove(mEntries, entry + 1, mSize * sizeof(Entry));
            otLogInfoBtp("%s: memove size=%d", __func__, mSize * sizeof(Entry));
        }
    }
}
} // namespace Toble
} // namespace ot

#endif // #if OPENTHREAD_CONFIG_TOBLE_ENABLE
