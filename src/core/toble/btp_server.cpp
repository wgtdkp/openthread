
#include "toble/btp.hpp"

#include "common/locator-getters.hpp"
#include "common/logging.hpp"

namespace ot {
namespace Toble {

#if OPENTHREAD_CONFIG_TOBLE_ENABLE && OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE

void Btp::HandleC2Subscribed(Platform::ConnectionId aPlatConn, bool aIsSubscribed)
{
    Connection *conn = Get<ConnectionTable>().Find(aPlatConn);

    assert(!Get<Toble>().IsCentral());

    VerifyOrExit(conn != NULL, OT_NOOP);

    if (aIsSubscribed)
    {
        VerifyOrExit(conn->mSession.mState == kStateHandshake, OT_NOOP);
        otLogDebgBle("BTP subscribed");
        Get<Platform>().IndicateC2(aPlatConn, &conn->mSession.mResponse, sizeof(HandshakeResponse));
    }
    else
    {
        VerifyOrExit(conn->mSession.mState != kStateIdle, OT_NOOP);
        otLogDebgBle("BTP unsubscribed");

        // Optional future enhancement: Trigger a BLE disconnect on un-subscribe.

        mTimer.Stop();

        conn->mSession.mState = kStateIdle;
    }

exit:
    return;
}

void Btp::HandleC2IndicateDone(Platform::ConnectionId aPlatConn)
{
    Connection *conn = Get<ConnectionTable>().Find(aPlatConn);

    assert(!Get<Toble>().IsCentral());

    VerifyOrExit(conn != NULL, OT_NOOP);

    switch (conn->mSession.mState)
    {
    case kStateIdle:
        break;
    case kStateHandshake:
        conn->mSession.mState = kStateConnected;

        if ((conn->mSession.mSendOffset != conn->mSession.mSendLength) || conn->mSession.GetRxWindowRemaining() <= 1)
        {
            SendData(*conn);
        }
        else
        {
            conn->mSession.SetTimer(kKeepAliveDelay);
            UpdateTimer();
        }

        break;
    case kStateConnected:
        HandleSentData(*conn);
        break;
    }

exit:
    return;
}

void Btp::HandleC1Write(Platform::ConnectionId aPlatConn, const uint8_t *aFrame, uint16_t aLength)
{
    Connection * conn  = Get<ConnectionTable>().Find(aPlatConn);
    const Frame *frame = reinterpret_cast<const Frame *>(aFrame);

    assert(!Get<Toble>().IsCentral());

    VerifyOrExit(aLength > 0, OT_NOOP);

    VerifyOrExit(conn != NULL, OT_NOOP);

    if (frame->IsHandshake())
    {
        VerifyOrExit(aLength >= sizeof(HandshakeRequest), OT_NOOP);
        HandleHandshake(*conn, static_cast<const HandshakeRequest &>(*frame));
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

    otLogDebgBle("BTP handshake receive");

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

    session.mResponse.Init(session.mMtu, kWindowSize);

    session.mState = kStateHandshake;

    session.mTxSeqnoCurrent = 0;
    session.mTxSeqnoAcked   = 0;
    session.mTxWindow       = aRequest.GetWindowSize();

    session.mRxSeqnoCurrent = 255;
    session.mRxSeqnoAcked   = 255;
    session.mRxWindow       = kWindowSize;

    session.SetTimer(kKeepAliveDelay);
    UpdateTimer();

exit:
    return;
}

#endif // #if OPENTHREAD_CONFIG_TOBLE_ENABLE && OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE

} // namespace Toble
} // namespace ot
