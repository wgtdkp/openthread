
#include "btp.hpp"

#include <openthread/platform/toble.h>

#include "common/locator-getters.hpp"
#include "common/logging.hpp"

namespace ot {
namespace Toble {

#if OPENTHREAD_CONFIG_ENABLE_TOBLE && OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE

void Btp::HandleC1WriteDone(Platform::Connection *aPlatConn)
{
    Connection *conn = Get<ConnectionTable>().Find(aPlatConn);

    assert(Get<Toble>().IsCentral());

    VerifyOrExit(conn != NULL);

    switch (conn->mSession.mState)
    {
    case kStateIdle:
        break;

    case kStateHandshake:
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

    assert(Get<Toble>().IsCentral());

    VerifyOrExit(conn != NULL);

    VerifyOrExit(conn->mSession.mState == kStateHandshake || conn->mSession.mState == kStateConnected);

    VerifyOrExit(aLength > 0);

    if (frame->IsHandshake())
    {
        VerifyOrExit(aLength >= sizeof(HandshakeResponse));
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

    VerifyOrExit(session.mState == kStateHandshake);

    otLogDebgBle("BTP handshake receive");

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

#endif // #if OPENTHREAD_CONFIG_ENABLE_TOBLE && OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE

} // namespace Toble
} // namespace ot
