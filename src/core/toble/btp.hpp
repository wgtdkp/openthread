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

#include <string.h>

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
    /**
     * This constructor initializes the object.
     *
     */
    explicit Btp(Instance &aInstance);

    /**
     * This function enables the BTP connection.
     *
     * @param[in]  aConn  A reference to ToBLE connection.
     *
     */
    void Start(Connection &aConn);

    /**
     * This function disables the BTP connection.
     *
     * @param[in]  aConn  A reference to ToBLE connection.
     *
     */
    void Stop(Connection &aConn);

    /**
     * This function sends bytes over the BTP .
     *
     * @param[in]  aConn       A reference to ToBLE connection.
     * @param[in]  aBuffer     A pointer to the data buffer.
     * @param[in]  aBufLength  Number of bytes to transmit.
     */
    void Send(Connection &aConn, const uint8_t *aBuffer, uint16_t aLength);

private:
    enum
    {
        kWindowSize     = 5,                                            ///< Receive window size.
        kKeepAliveDelay = 2500,                                         ///< Keep alive delay, in milliseconds.
        kSegmentSizeMax = OPENTHREAD_CONFIG_TOBLE_BTP_MAX_SEGMENT_SIZE, ///< Maximum segment size.
        kTobleFrameSize = OT_RADIO_TOBLE_FRAME_MAX_SIZE,                ///< ToBLE frame size.
    };

    enum State
    {
        kStateIdle,      ///< BTP is idle.
        kStateSubscribe, ///< BTP is in subscribe state.
        kStateHandshake, ///< BTP is in handshake state.
        kStateConnected, ///< BTP connection has established.
    };

    /**
     * This class defines the BTP session object.
     *
     */
    class Session
    {
    public:
        /**
         * This method indicates if the timer expires.
         *
         * @retval TRUE   The timer expires.
         * @retval FALSE  The timer dosen't expire.
         *
         */
        bool IsTimerExpired(void) const { return (mTimerExpire <= TimerMilli::GetNow()); }

        /**
         * This method gets the remaining time of the timer.
         *
         * @retval the remaining time.
         *
         */
        int32_t GetTimer(void) const { return mTimerExpire - TimerMilli::GetNow(); }

        /**
         * This method starts the timer.
         *
         * @param[in]  aDelay  Delay time, in milliseconds.
         *
         */
        void SetTimer(uint32_t aDelay)
        {
            mTimerExpire = TimerMilli::GetNow() + aDelay;
            mIsTimerSet  = true;
        }

        /**
         * This method gets the remaining receive window size.
         *
         * @retval the remaining receive window size.
         *
         */
        uint8_t GetRxWindowRemaining(void) const { return mRxWindow - (mRxSeqnoCurrent - mRxSeqnoAcked); }

        /**
         * This method gets the remaining transmit window size.
         *
         * @retval the remaining transmit window size.
         *
         */
        uint8_t GetTxWindowRemaining(void) const { return mTxWindow - (mTxSeqnoCurrent - mTxSeqnoAcked); }

        /**
         * This method indicates if there is a fragment need to be sent.
         *
         * @retval TRUE   Has fragment to send.
         * @retval FALSE  Dosen't have fragment to send.
         *
         */
        bool HasFragmentToSend(void) { return (mSendBuf != NULL) && (mSendOffset < mSendLength); }

        /**
         * This method indicates if there is an ACK need to be sent.
         *
         * @retval TRUE   Has ACK to send.
         * @retval FALSE  Dosen't have ACK to send.
         *
         */
        bool HasPendingAckToSend(void) { return (mRxSeqnoCurrent != mRxSeqnoAcked); }

        State mState;

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
        uint8_t  mRxBuf[kTobleFrameSize];

        bool      mTransmitRequest : 1;
        bool      mTransmitKeepAliveAck : 1;
        bool      mIsTimerSet : 1;
        TimeMilli mTimerExpire;
    };

    void StartTimer(Session &aSession, uint32_t aDelay)
    {
        aSession.SetTimer(aDelay);
        UpdateTimer();
    }

    void    Reset(Session &aSession);
    void    SendData(Connection &aConn);
    void    TransmitTaskPost(Session &aSession);
    otError GattSend(Connection &aConn, const uint8_t *aBuffer, uint16_t aLength);
    void    HandleGattSentDone(Connection &aConn, otError aError);
    void    HandleGattReceiveDone(Connection &aConn, uint8_t *aBuffer, uint16_t aLength, otError aError);
    void    ConnectionTimerRefresh(Connection &aConn);
    void    HandleSessionReady(Connection &aConn);
    void    HandleSentData(Connection &aConn);
    void    HandleDataFrame(Connection &aConn, const uint8_t *aFrame, uint16_t aLength);

    static void HandleTimer(Timer &aTimer);
    void        HandleTimer(void);

    static void HandleTxTask(Tasklet &aTasklet);
    void        SendNextFragment(void);

    void UpdateTimer(void);

#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
    void HandleConnectionReady(Platform::Connection *aPlatConn);
    void HandleHandshake(Connection &aConn, const HandshakeResponse &aResponse);

    // Callbacks from platform
    void HandleC1WriteDone(Platform::Connection *aPlatConn);
    void HandleC2Notification(Platform::Connection *aPlatConn, const uint8_t *aFrame, uint16_t aLength);
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
    void HandleC2NotificateDone(Platform::Connection *aPlatConn);
#endif

    TimerMilli mTimer;
    Tasklet    mTxTask;
    Frame      mTxFrame;
};

} // namespace Toble
} // namespace ot

#endif // #if OPENTHREAD_CONFIG_TOBLE_ENABLE
#endif // BTP_HPP_