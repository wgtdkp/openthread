
#include "btp.hpp"

#include <openthread/platform/toble.h>

#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "common/locator-getters.hpp"
#include "common/logging.hpp"
#include "utils/wrap_string.h"

using ot::Encoding::LittleEndian::ReadUint16;
using ot::Encoding::LittleEndian::WriteUint16;

namespace ot {
namespace Toble {

#if OPENTHREAD_CONFIG_ENABLE_TOBLE

Btp::Btp(Instance &aInstance)
    : InstanceLocator(aInstance)
    , mTimer(aInstance, &Btp::HandleTimer, this)
{
}

void Btp::Start(Connection &aConn)
{
    otLogDebgBle("Btp::Start");

    aConn.mSession.mState = kStateIdle;
}

void Btp::HandleConnectionReady(Platform::Connection *aPlatConn)
{
    Connection *conn = Get<ConnectionTable>().Find(aPlatConn);

    otLogDebgBle("Btp::ConnectionReady (%p -> %p)", aPlatConn, conn);

    VerifyOrExit(conn != NULL);

    VerifyOrExit(Get<Toble>().IsCentral());

#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
    conn->mSession.mRequest.Init(Get<Platform>().GetConnMtu(aPlatConn), kWindowSize);
    Get<Platform>().WriteC1(aPlatConn, &conn->mSession.mRequest, sizeof(conn->mSession.mRequest));
    conn->mSession.mState = kStateHandshake;
#endif

exit:
    return;
}

void Btp::Stop(Connection &aConn)
{
    otLogDebgBle("Btp::Stop");

    // Optional future enhancement: On central we can consider
    // unsubscribe from C2.

    Reset(aConn.mSession);
}

void Btp::Send(Connection &aConn, const uint8_t *aBuf, uint16_t aLength)
{
    otLogDebgBle("Btp::Send");

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

    VerifyOrExit(!session.mIsSending && session.mState == kStateConnected);

    session.mTxBuf[0] = 0;

    otLogDebgBle("BTP send");

    if (session.mRxSeqnoCurrent == session.mRxSeqnoAcked)
    {
        VerifyOrExit(session.GetTxWindowRemaining() > 1);
    }
    else
    {
        session.mTxBuf[0] |= Frame::kAckFlag;
        otLogDebgBle("  ack=%u", session.mRxSeqnoCurrent);

        *cur++                = session.mRxSeqnoCurrent;
        session.mRxSeqnoAcked = session.mRxSeqnoCurrent;
    }

    *cur++ = ++session.mTxSeqnoCurrent;
    otLogDebgBle("  seq=%u ", cur[-1]);

    if (session.mSendBuf != NULL)
    {
        uint16_t segmentLength;
        uint16_t segmentRemaining;

        if (session.mSendOffset == 0)
        {
            session.mTxBuf[0] |= Frame::kBeginFlag;
            otLogDebgBle("  len=%u", session.mSendLength);

            WriteUint16(session.mSendLength, cur);
            cur += sizeof(uint16_t);
        }
        else
        {
            session.mTxBuf[0] |= Frame::kContinueFlag;
            otLogDebgBle("  off=%u", session.mSendOffset);
        }

        segmentRemaining = session.mMtu - (cur - session.mTxBuf);
        segmentLength    = session.mSendLength - session.mSendOffset;

        if (segmentLength > segmentRemaining)
        {
            segmentLength = segmentRemaining;
        }

        memcpy(cur, session.mSendBuf + session.mSendOffset, segmentLength);
        cur += segmentLength;
        session.mSendOffset += segmentLength;

        if (session.mSendOffset >= session.mSendLength)
        {
            session.mTxBuf[0] |= Frame::kEndFlag;
            otLogDebgBle("  end");
        }
    }

    frameLength = cur - session.mTxBuf;

    mTimer.Stop();
    session.mIsSending = true;

    if (Get<Toble>().IsCentral())
    {
#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
        Get<Platform>().WriteC1(aConn.mPlatConn, session.mTxBuf, frameLength);
#endif
    }
    else
    {
#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
        Get<Platform>().IndicateC2(aConn.mPlatConn, session.mTxBuf, frameLength);
#endif
    }

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
        Get<Transport>().HandleSendDone(aConn, OT_ERROR_NONE);
    }
}

void Btp::HandleDataFrame(Connection &aConn, const uint8_t *aFrame, uint16_t aLength)
{
    Session &      session = aConn.mSession;
    const Frame &  frame(*reinterpret_cast<const Frame *>(aFrame));
    const uint8_t *cur = &aFrame[1];

    VerifyOrExit(session.mState == kStateConnected);

    otLogDebgBle("BTP receive");

    if (frame.IsAck())
    {
        session.mTxSeqnoAcked = *cur++;
        otLogDebgBle("  ack=%d", session.mTxSeqnoAcked);
    }

    assert(static_cast<uint8_t>(session.mRxSeqnoCurrent + 1) == cur[0]);
    session.mRxSeqnoCurrent = *cur++;
    otLogDebgBle("  seq=%d ", session.mRxSeqnoCurrent);

    if (frame.IsBegin())
    {
        session.mReceiveOffset = 0;
        session.mReceiveLength = ReadUint16(cur);
        cur += sizeof(uint16_t);
        otLogDebgBle("  len=%u", session.mReceiveLength);
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
        assert(session.mReceiveOffset == session.mReceiveLength);
        otLogDebgBle("BTP received %d", session.mReceiveOffset);

        Get<Transport>().HandleReceiveDone(aConn, session.mRxBuf, session.mReceiveLength, OT_ERROR_NONE);
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

        otLogDebgBle("BTP timer fired");

        session.mIsTimerSet = false;

        switch (session.mState)
        {
        case kStateIdle:
            break;

        case kStateHandshake:
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
    otLogDebgBle("BTP::Reset");

    aSession.mSendBuf    = NULL;
    aSession.mSendOffset = 0;
    aSession.mSendLength = 0;
    aSession.mIsSending  = false;
    aSession.mIsTimerSet = false;
    aSession.mState      = kStateIdle;

    UpdateTimer();
}

#endif // #if OPENTHREAD_CONFIG_ENABLE_TOBLE

} // namespace Toble
} // namespace ot
