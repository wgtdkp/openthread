/*
 *  Copyright (c) 2019, The OpenThread Authors.
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
 *   This file implements ToBLE Controller for peripheral.
 */

#include "controller_peripheral.hpp"

#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "common/encoding.hpp"
#include "common/instance.hpp"
#include "common/locator-getters.hpp"
#include "common/logging.hpp"
#include "common/random.hpp"
#include "mac/mac.hpp"
#include "mac/mac_frame.hpp"

#if OPENTHREAD_CONFIG_ENABLE_TOBLE && OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE

#define OT_TOBLE_DEFAULT_RSSI 0

namespace ot {
namespace Toble {
namespace Peripheral {

Controller::Controller(Instance &aInstance)
    : InstanceLocator(aInstance)
    , mConn(NULL)
    , mTimer(aInstance, &Controller::HandleTimer, this)
    , mState(kStateSleep)
    , mTxFrame(NULL)
{
}

void Controller::SetState(State aState)
{
    if (aState != mState)
    {
        otLogNoteBle("PeriCtrl::State: %s -> %s", StateToString(mState), StateToString(aState));
        mState = aState;
    }
}

otError Controller::Sleep(void)
{
    otError error = OT_ERROR_NONE;

    otLogInfoBle("PeriCtrl::Sleep()");

    switch (mState)
    {
    case kStateSleep:
        ExitNow();
        break;

    case kStateRx:
        break;

    case kStateTxSending:
    case kStateTxAdvertising:
        ExitNow(error = OT_ERROR_BUSY);
        break;
    }

    if (mConn != NULL)
    {
        mTimer.Start(kSleepDisconnectTimeout);
    }
    else
    {
        mTimer.Stop();
    }

    Get<Platform>().StopAdv();

    SetState(kStateSleep);

exit:
    return error;
}

otError Controller::Receive(void)
{
    otError error = OT_ERROR_NONE;

    otLogInfoBle("PeriCtrl::Receive()");

    switch (mState)
    {
    case kStateSleep:
        break;

    case kStateRx:
        ExitNow();
        break;

    case kStateTxSending:
        ExitNow(error = OT_ERROR_INVALID_STATE);
        break;

    case kStateTxAdvertising:
        Get<Platform>().StopAdv();
        break;
    }

    StartReceive();

exit:
    return error;
}

void Controller::StartReceive(void)
{
    SetState(kStateRx);
    mTimer.Stop();

    // If we already have a connection, keep it open till parent closes it from its side
    // or the long rx mode timeout passes.
    if (mConn != NULL)
    {
        mTimer.Start(kRxModeConnTimeout);
    }
    else
    {
        // Add a small delay before starting the advertisements.
        mTimer.Start(kRxModeAdvStartDelay);
    }
}

void Controller::StartRxModeAdv(void)
{
    AdvData             advData(mAdvDataBuffer, sizeof(mAdvDataBuffer));
    AdvData::Info       info;
    Platform::AdvConfig config;

#if OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
    info.mL2capSupport = true;
    info.mL2capPsm     = Get<Platform>().GetL2capPsm();
#else
    info.mL2capSupport = false;
#endif

    info.mFramePending = false;
    info.mDataPolling  = false;
    info.mPanId        = Get<Mac::Mac>().GetPanId();
    info.mSrcShort     = Get<Mac::Mac>().GetShortAddress();
    info.mSrcExtended  = Get<Mac::Mac>().GetExtAddress();
    info.mDest.SetNone();

    advData.Populate(info);

    config.mType     = OT_TOBLE_ADV_TYPE_UNDIRECTED_CONNECTABLE_NONSCANNABLE;
    config.mInterval = kRxModeAdvInterval;
    config.mData     = advData.GetData();
    config.mLength   = advData.GetLength();

    otLogInfoBle("PeriCtrl::StartRxModeAdv(AdvInfo:[%s])", info.ToString().AsCString());
    otLogInfoBle("PeriCtrl: AdvData: %s", advData.ToString().AsCString());

    Get<Platform>().StartAdv(config);
}

otError Controller::Transmit(Mac::Frame &aFrame)
{
    otError error = OT_ERROR_NONE;

    otLogInfoBle("PeriCtrl::Transmit([%s])", aFrame.ToInfoString().AsCString());

    switch (mState)
    {
    case kStateSleep:
    case kStateTxSending:
    case kStateTxAdvertising:
        ExitNow(error = OT_ERROR_INVALID_STATE);
        break;

    case kStateRx:
        Get<Platform>().StopAdv();
        break;
    }

    mTimer.Stop();
    mTxFrame = &aFrame;

    if (mTxFrame->GetChannel() != Get<Mac::Mac>().GetPanChannel())
    {
        otLogNoteBle("PeriCtrl: Skip frame tx on channel %d (pan-channel: %d)", mTxFrame->GetChannel(),
                     Get<Mac::Mac>().GetPanChannel());
        StartReceive();
        InvokeRadioTxDone(OT_ERROR_NONE);
        ExitNow();
    }

    if (mConn == NULL)
    {
        SetState(kStateTxAdvertising);
        StartTxModeAdv();
        mTimer.Start(kTxConnectTimeout);
    }
    else
    {
        // If we already have a connection, send it using BTP and wait for callback
        // or a timeout.
        SetState(kStateTxSending);
        mTimer.Start(kTxTimeout);
        Get<Transport>().Send(*mConn, mTxFrame->GetPsdu(), mTxFrame->GetPsduLength());
    }

exit:
    return error;
}

void Controller::StartTxModeAdv(void)
{
    otError             error;
    AdvData             advData(mAdvDataBuffer, sizeof(mAdvDataBuffer));
    AdvData::Info       info;
    Platform::AdvConfig config;

    assert(mState == kStateTxAdvertising);
    assert(mTxFrame != NULL);

#if OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
    info.mL2capSupport = true;
    info.mL2capPsm     = Get<Platform>().GetL2capPsm();
#else
    info.mL2capSupport = false;
#endif

    info.mFramePending = true; // indicating "tx mode"
    info.mDataPolling  = false;
    info.mSrcShort     = Get<Mac::Mac>().GetShortAddress();
    info.mSrcExtended  = Get<Mac::Mac>().GetExtAddress();

    // Get PanId and destination from the current tx frame

    error = mTxFrame->GetDstPanId(info.mPanId);

    if (error != OT_ERROR_NONE)
    {
        info.mPanId = Get<Mac::Mac>().GetPanId();
    }

    if (info.mPanId == OT_PANID_BROADCAST)
    {
        otLogNoteBle("PeriCtrl: Skip sending to broadcast PAN-Id");
        StartReceive();
        InvokeRadioTxDone(OT_ERROR_NONE);
        ExitNow();
    }

    error = mTxFrame->GetDstAddr(info.mDest);
    assert(error == OT_ERROR_NONE);

    advData.Populate(info);

    config.mType     = OT_TOBLE_ADV_TYPE_UNDIRECTED_CONNECTABLE_NONSCANNABLE;
    config.mInterval = kTxModeAdvInterval;
    config.mData     = advData.GetData();
    config.mLength   = advData.GetLength();

    otLogInfoBle("PeriCtrl::StartTxModeAdv(AdvInfo:[%s])", info.ToString().AsCString());
    otLogInfoBle("PeriCtrl: AdvData: %s", advData.ToString().AsCString());

    Get<Platform>().StartAdv(config);

exit:
    return;
}

void Controller::HandleTimer(Timer &aTimer)
{
    aTimer.GetOwner<Controller>().HandleTimer();
}

void Controller::HandleTimer(void)
{
    otLogInfoBle("PeriCtrl::HandleTimer()");

    switch (mState)
    {
    case kStateSleep:
        if (mConn != NULL)
        {
            otLogDebgBle("PeriCtrl::HandleTimer: TIMEOUT EXCEEDED: kSleepDisconnectTimeout");
            Get<Transport>().Stop(*mConn);
            Get<Platform>().Disconnect(mConn->mPlatConn);
            Get<ConnectionTable>().Remove(*mConn);
            mConn = NULL;
        }
        break;

    case kStateRx:
        if (mConn == NULL)
        {
            // Delay time before starting advertisement has expired
            StartRxModeAdv();
        }
        else
        {
            // Connection timeout, disconnect and start adv again.
            otLogDebgBle("PeriCtrl::HandleTimer: TIMEOUT EXCEEDED: kRxModeConnTimeout");
            Get<Transport>().Stop(*mConn);
            Get<Platform>().Disconnect(mConn->mPlatConn);
            Get<ConnectionTable>().Remove(*mConn);
            mConn = NULL;
            StartRxModeAdv();
        }
        break;

    case kStateTxSending:
        // Timed out waiting for BTP send done callback.
        otLogDebgBle("PeriCtrl::HandleTimer: TIMEOUT EXCEEDED: kTxTimeout");
        Get<Transport>().Stop(*mConn);
        Get<Platform>().Disconnect(mConn->mPlatConn);
        Get<ConnectionTable>().Remove(*mConn);
        mConn = NULL;
        StartReceive();
        InvokeRadioTxDone(mTxFrame->GetAckRequest() ? OT_ERROR_NO_ACK : OT_ERROR_NONE);
        break;

    case kStateTxAdvertising:
        // Timed out waiting for connection to be established while in tx mode
        Get<Platform>().StopAdv();
        StartReceive();
        InvokeRadioTxDone(mTxFrame->GetAckRequest() ? OT_ERROR_NO_ACK : OT_ERROR_NONE);
        break;
    }
}

void Controller::HandleConnected(Platform::Connection *aPlatConn)
{
    otLogInfoBle("PeriCtrl::HandleConnected()");

    switch (mState)
    {
    case kStateSleep:
        Get<Platform>().Disconnect(aPlatConn);
        break;

    case kStateRx:
        if (mConn != NULL)
        {
            otLogNoteBle("PeriCtrl: Got a new connection while already connected in Rx mode");
            Get<Transport>().Stop(*mConn);
            Get<Platform>().Disconnect(mConn->mPlatConn);
            Get<ConnectionTable>().Remove(*mConn);
            mConn = NULL;
        }

        mTimer.Stop();
        Get<Platform>().StopAdv();
        mConn = Get<ConnectionTable>().GetNew();
        VerifyOrExit(mConn != NULL, StartRxModeAdv());
        mConn->mPlatConn = aPlatConn;
        Get<Transport>().Start(*mConn);
        mTimer.Start(kTransportStartTimeout + kRxModeConnTimeout);
        break;

    case kStateTxSending:
        // Disconnect the new connection while actively sending on an existing connection
        Get<Platform>().Disconnect(aPlatConn);
        break;

    case kStateTxAdvertising:
        // Got connected while advertising in Tx mode
        mTimer.Stop();
        Get<Platform>().StopAdv();
        mConn = Get<ConnectionTable>().GetNew();
        assert(mConn != NULL);
        mConn->mPlatConn = aPlatConn;
        Get<Transport>().Start(*mConn);
        SetState(kStateTxSending);
        mTimer.Start(kTransportStartTimeout + kTxTimeout);
        Get<Transport>().Send(*mConn, mTxFrame->GetPsdu(), mTxFrame->GetPsduLength());
        break;
    }

exit:
    return;
}

void Controller::HandleDisconnected(Platform::Connection *aPlatConn)
{
    Connection *conn = Get<ConnectionTable>().Find(aPlatConn);

    VerifyOrExit((conn != NULL) && (conn == mConn));

    otLogInfoBle("PeriCtrl::Disconnected()");

    Get<Transport>().Stop(*mConn);
    Get<ConnectionTable>().Remove(*mConn);
    mConn = NULL;

    switch (mState)
    {
    case kStateSleep:
        mTimer.Stop(); // Stopping `kSleepDisconnectTimeout`
        break;

    case kStateRx:
        StartReceive(); // start advertising!
        break;

    case kStateTxSending:
        StartReceive();
        InvokeRadioTxDone(OT_ERROR_ABORT); // map disconnect during tx to abort
        break;

    case kStateTxAdvertising:
        // Cannot happen (ignore).
        break;
    }

exit:
    return;
}

// This is callback from transport layer (BTP or L2CAP) after `SendTransport()`
// aError should OT_ERROR_NONE or NO_ACK.
void Controller::HandleTransportSendDone(Connection &aConn, otError aError)
{
    otLogInfoBle("PeriCtrl::HandleTransportSendDone(err:%s)", otThreadErrorToString(aError));

    VerifyOrExit(mState == kStateTxSending);
    VerifyOrExit(mConn == &aConn);

    StartReceive();
    InvokeRadioTxDone(aError);

exit:
    return;
}

// This is callback from transport layer (BTP or L2CAP) to indicate received frame.
void Controller::HandleTransportReceiveDone(Connection &aConn, uint8_t *aFrame, uint16_t aLength, otError aError)
{
    Mac::Frame rxFrame;

    if ((mState == kStateRx) && (&aConn == mConn))
    {
        // Push back the timeout for this connection
        mTimer.Start(kRxModeConnTimeout);
    }

    // We should be able to pass up the buffer pointers we received from BTP to next layer.
    // OT core can possibly modify the frame content (up to length), e.g., in-place decryption.
    rxFrame.mPsdu    = aFrame;
    rxFrame.mLength  = aLength;
    rxFrame.mChannel = Get<Mac::Mac>().GetRadioChannel();
    rxFrame.mIeInfo  = NULL;

    rxFrame.mInfo.mRxInfo.mAckedWithFramePending = true;

    rxFrame.mInfo.mRxInfo.mMsec = TimerMilli::GetNow();
    rxFrame.mInfo.mRxInfo.mUsec = 0;
    rxFrame.mInfo.mRxInfo.mRssi = OT_TOBLE_DEFAULT_RSSI;
    rxFrame.mInfo.mRxInfo.mLqi  = OT_RADIO_LQI_NONE;

    otLogInfoBle("PeriCtrl::HandleBtpReceiveDone(err:%s, [%s])", otThreadErrorToString(aError),
                 rxFrame.ToInfoString().AsCString());

    Get<Radio::Callbacks>().HandleReceiveDone(&rxFrame, aError);
}

void Controller::InvokeRadioTxDone(otError aError)
{
    if ((aError == OT_ERROR_NONE) && mTxFrame->GetAckRequest())
    {
        uint8_t    ackPsdu[kAckFrameLength];
        Mac::Frame ackFrame;

        memset(ackPsdu, 0, sizeof(ackPsdu));

        ackFrame.mPsdu   = ackPsdu;
        ackFrame.mLength = kAckFrameLength;

        ackFrame.mChannel = mTxFrame->GetChannel();
        ackFrame.mIeInfo  = NULL;

        ackFrame.mInfo.mRxInfo.mAckedWithFramePending = false;
        ackFrame.mInfo.mRxInfo.mMsec                  = TimerMilli::GetNow();
        ackFrame.mInfo.mRxInfo.mUsec                  = 0;
        ackFrame.mInfo.mRxInfo.mRssi                  = OT_RADIO_RSSI_INVALID;
        ackFrame.mInfo.mRxInfo.mLqi                   = OT_RADIO_LQI_NONE;

        ackFrame.InitMacHeader(Mac::Frame::kFcfFrameAck | Mac::Frame::kFcfDstAddrNone | Mac::Frame::kFcfSrcAddrNone,
                               Mac::Frame::kSecNone);

        // We need to set "frame pending" flag so that on a child data
        // poll tx works properly, otherwise MAC layer would put a
        // sleepy child back to sleep after a data poll.

        ackFrame.SetFramePending(true);
        ackFrame.SetSequence(mTxFrame->GetSequence());

        otLogInfoBle("PeriCtrl::InvokeRadioTxDone(err:%s, ack:[%s])", otThreadErrorToString(aError),
                     ackFrame.ToInfoString().AsCString());

        Get<Radio::Callbacks>().HandleTransmitDone(*mTxFrame, &ackFrame, aError);
    }
    else
    {
        otLogInfoBle("PeriCtrl::InvokeRadioTxDone(err:%s)", otThreadErrorToString(aError));

        Get<Radio::Callbacks>().HandleTransmitDone(*mTxFrame, NULL, aError);
    }
}

const char *Controller::StateToString(State aState)
{
    const char *str = "Unknown";

    switch (aState)
    {
    case kStateSleep:
        str = "Sleep";
        break;

    case kStateRx:
        str = "Rx";
        break;

    case kStateTxSending:
        str = "TxSending";
        break;

    case kStateTxAdvertising:
        str = "TxAdvertising";
        break;
    }

    return str;
}

} // namespace Peripheral
} // namespace Toble
} // namespace ot

#endif // OPENTHREAD_CONFIG_ENABLE_TOBLE && OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
