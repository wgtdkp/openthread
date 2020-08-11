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
 *   This file contains the definition for ToBLE BTP transport.
 */

#ifndef BTP_HPP_
#define BTP_HPP_

#include "openthread-core-config.h"

#include <openthread/platform/radio.h>

#include "common/locator.hpp"
#include "common/timer.hpp"
#include "toble/btp_frame.hpp"
#include "toble/platform.hpp"

#if OPENTHREAD_CONFIG_TOBLE_ENABLE

namespace ot {
namespace Toble {

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
        kWindowSize     = 5,
        kKeepAliveDelay = 2500,                                         // milliseconds
        kSegmentSizeMax = OPENTHREAD_CONFIG_TOBLE_BTP_MAX_SEGMENT_SIZE, // bytes
    };

    enum State
    {
        kStateIdle,
        kStateSubscribe,
        kStateHandshake,
        kStateConnected,
    };

    class Session
    {
    public:
        bool    IsTimerExpired(void) const { return (mTimerExpire <= TimerMilli::GetNow()); }
        int32_t GetTimer(void) const { return mTimerExpire - TimerMilli::GetNow(); }
        void    SetTimer(uint32_t aDelay)
        {
            mTimerExpire = TimerMilli::GetNow() + aDelay;
            mIsTimerSet  = true;
        }

        uint8_t GetRxWindowRemaining(void) const { return mRxWindow - (mRxSeqnoCurrent - mRxSeqnoAcked); }
        uint8_t GetTxWindowRemaining(void) const { return mTxWindow - (mTxSeqnoCurrent - mTxSeqnoAcked); }

        DataFrame *NewDataFrame(void)
        {
            DataFrame *frame = NULL;
            for (size_t i = 0; i < OT_ARRAY_LENGTH(mTxFrames); i++)
            {
                if (mTxFrames[i].GetDataLength() == 0)
                {
                    frame = &mTxFrames[i];
                    break;
                }
            }

            return frame;
        }

        void FreeDataFrame(DataFrame *aFrame) { aFrame->SetDataLength(0); }

#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
        HandshakeRequest mRequest;
#endif

#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
        HandshakeResponse mResponse;
#endif

        enum
        {
            kNumFrames = 4,
        };

        State mState;

        TimeMilli mTimerExpire;

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

        DataFrame mTxFrames[kNumFrames];
        bool      mIsSending : 1;
        bool      mIsTimerSet : 1;
    };

    void Reset(Session &aSession);

    otError GattSend(Connection &aConn, const uint8_t *aBuffer, uint16_t aLength);
    void    HandleGattSentDone(Connection &aConn, otError aError);
    void    HandleGattReceiveDone(Connection &aConn, uint8_t *aBuffer, uint16_t aLength, otError aError);
    void    ConnectionTimerRefresh(Connection &aConn);

    void HandleConnectionReady(Platform::Connection *aPlatConn);

#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
    void HandleHandshake(Connection &aConn, const HandshakeResponse &aResponse);

    // Callbacks from platform
    void HandleC1WriteDone(Platform::Connection *aPlatConn, const uint8_t *aFrame, uint16_t aLength);
    void HandleC2Indication(Platform::Connection *aPlatConn, const uint8_t *aFrame, uint16_t aLength);
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
    void HandleC2IndicateDone(Platform::Connection *aPlatConn, const uint8_t *aFrame, uint16_t aLength);
#endif

    void HandleSentData(Connection &aConn, const uint8_t *aFrame, uint16_t aLength);
    void HandleDataFrame(Connection &aConn, const uint8_t *aFrame, uint16_t aLength);

    static void HandleTimer(Timer &aTimer);
    void        HandleTimer(void);

    static void HandleTxTask(Tasklet &aTasklet);
    void        SendNextFragment(void);

    void UpdateTimer(void);

    void SendData(Connection &aConn);

    TimerMilli mTimer;
    Tasklet    mTxTask;
};

} // namespace Toble
} // namespace ot

#endif // #if OPENTHREAD_CONFIG_TOBLE_ENABLE
#endif // BTP_HPP_
