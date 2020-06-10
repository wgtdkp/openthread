
#ifndef BTP_HPP_
#define BTP_HPP_

#include "openthread-core-config.h"

#include <openthread/platform/radio.h>

#include "common/locator.hpp"
#include "common/timer.hpp"
#include "toble/btp_frame.hpp"
#include "toble/platform.hpp"
#include "utils/wrap_stdint.h"

namespace ot {
namespace Toble {

#if OPENTHREAD_CONFIG_ENABLE_TOBLE

class Connection;

class Btp : public InstanceLocator
{
    friend class Platform::Callbacks;
    friend class Connection;

public:
    explicit Btp(Instance &aInstance);

    void Start(Connection &aConn);
    void Stop(Connection &aConn);
    void Send(Connection &aConn, const uint8_t *aBuf, uint16_t aLength);

private:
    enum
    {
        kWindowSize     = 4,
        kKeepAliveDelay = 2500,                                         // milliseconds
        kSegmentSizeMax = OPENTHREAD_CONFIG_TOBLE_BTP_MAX_SEGMENT_SIZE, // bytes
    };

    enum State
    {
        kStateIdle,
        kStateHandshake,
        kStateConnected,
    };

    class Session
    {
    public:
        bool    IsTimerExpired(void) const { return static_cast<int32_t>(mTimerExpire - TimerMilli::GetNow()) <= 0; }
        int32_t GetTimer(void) const { return static_cast<int32_t>(mTimerExpire - TimerMilli::GetNow()); }
        void    SetTimer(uint32_t aDelay)
        {
            mTimerExpire = TimerMilli::GetNow() + aDelay;
            mIsTimerSet  = true;
        }

        uint8_t GetRxWindowRemaining(void) const { return mRxWindow - (mRxSeqnoCurrent - mRxSeqnoAcked); }
        uint8_t GetTxWindowRemaining(void) const { return mTxWindow - (mTxSeqnoCurrent - mTxSeqnoAcked); }

#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
        HandshakeRequest mRequest;
#endif

#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
        HandshakeResponse mResponse;
#endif

        State mState;

        uint32_t mTimerExpire;

        uint16_t mMtu;

        uint8_t mTxSeqnoCurrent;
        uint8_t mTxSeqnoAcked;
        uint8_t mTxWindow;

        uint8_t mRxSeqnoCurrent;
        uint8_t mRxSeqnoAcked;
        uint8_t mRxWindow;

        const uint8_t *mSendBuf;
        uint16_t       mSendLength;
        uint16_t       mSendOffset;

        uint16_t mReceiveLength;
        uint16_t mReceiveOffset;

        uint8_t mTxBuf[kSegmentSizeMax];
        uint8_t mRxBuf[OT_RADIO_TOBLE_FRAME_MAX_SIZE];

        bool mIsSending : 1;
        bool mIsTimerSet : 1;
    };

    void Reset(Session &aSession);

    void HandleConnectionReady(Platform::Connection *aPlatConn);

#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
    void HandleHandshake(Connection &aConn, const HandshakeResponse &aResponse);

    // Callbacks from platform
    void HandleC1WriteDone(Platform::Connection *aConn);
    void HandleC2Indication(Platform::Connection *aConn, const uint8_t *aFrame, uint16_t aLength);
#endif

#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
    enum
    {
        kAttHeaderSize      = 3,
        kSegmentSizeDefault = 20,
    };

    void HandleHandshake(Connection &aConn, const HandshakeRequest &aRequest);

    // Callbacks from platform
    void HandleC1Write(Platform::Connection *aPlatConn, const uint8_t *aFrame, uint16_t aLength);
    void HandleC2Subscribed(Platform::Connection *aPlatConn, bool aIsSubscribed);
    void HandleC2IndicateDone(Platform::Connection *aPlatConn);
#endif

    void HandleSentData(Connection &aConn);
    void HandleDataFrame(Connection &aConn, const uint8_t *aFrame, uint16_t aLength);

    static void HandleTimer(Timer &aTimer);
    void        HandleTimer(void);

    void UpdateTimer(void);

    void SendData(Connection &aConn);

    TimerMilli mTimer;
};

#endif // #if OPENTHREAD_CONFIG_ENABLE_TOBLE

} // namespace Toble
} // namespace ot

#endif // BTP_HPP_
