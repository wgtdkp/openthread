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
 *   This file implements ToBLE Controller for peripheral.
 */

#include "toble/controller_peripheral.hpp"

#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "common/encoding.hpp"
#include "common/instance.hpp"
#include "common/locator-getters.hpp"
#include "common/logging.hpp"
#include "common/random.hpp"
#include "mac/mac.hpp"
#include "mac/mac_frame.hpp"

#if OPENTHREAD_CONFIG_TOBLE_ENABLE && OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE

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
    , mJoiningPermitted(false)
    , mBorderAgentEnabled(false)
    , mDtcEnabled(false)
    , mTobleRole(AdvData::kRoleEndDevice)
{
    mSteeringData.Init();
    mSteeringData.Clear();
}

void Controller::test(void)
{
    otLogNoteToblePeri("%s: mJoiningPermitted   =%d", __func__, mJoiningPermitted);
    otLogNoteToblePeri("%s: mBorderAgentEnabled =%d", __func__, mBorderAgentEnabled);
    otLogNoteToblePeri("%s: mTobleRole          =%d", __func__, mTobleRole);
}

void Controller::SetJoiningPermitted(bool aEnabled, otSteeringData *aSteeringData)
{
    mJoiningPermitted = aEnabled;

    if (aSteeringData != NULL)
    {
        mSteeringData.Init(*aSteeringData);
    }
    UpdateAdvertising();

    otLogNoteToblePeri("%s: mJoiningPermitted=%d ~~~", __func__, mJoiningPermitted);
}

void Controller::SetBoarderAgent(bool aEnabled)
{
    mBorderAgentEnabled = aEnabled;
    UpdateAdvertising();

    otLogNoteToblePeri("%s: mBorderAgentEnabled=%d ~~~", __func__, mBorderAgentEnabled);
}

void Controller::SetDtc(bool aEnabled)
{
    mDtcEnabled = aEnabled;
    UpdateAdvertising();

    otLogNoteToblePeri("%s: mDtcEnabled=%d ~~~", __func__, mDtcEnabled);
}

void Controller::SetTobleRole(uint8_t aRole)
{
    mTobleRole = static_cast<AdvData::TobleRole>(aRole);
    UpdateAdvertising();

    otLogNoteToblePeri("%s: mTobleRole=%d ~~~", __func__, mTobleRole);
}

void Controller::UpdateAdvertising(void)
{
    if (mState == kStateRxAdvertising)
    {
        StartAdvertising(false);
    }
    else if (mState == kStateTxAdvertising)
    {
        StartAdvertising(true);
    }
}

void Controller::SetState(State aState)
{
    if (aState != mState)
    {
        otLogInfoToblePeri("State: %s -----------------> %s", StateToString(mState), StateToString(aState));
        mState = aState;
    }
}

otError Controller::Sleep(void)
{
    otError error = OT_ERROR_NONE;

    otLogInfoToblePeri("Sleep()");

    switch (mState)
    {
    case kStateSleep:
        ExitNow();
        break;

    case kStateConnected:
        otLogNoteToblePeri("%s: kStateConnected, TimerStart(%d)", __func__, kSleepDisconnectTimeout);
        TimerStart(kSleepDisconnectTimeout);
        break;

    case kStateRxAdvertising:
        Get<Platform>().StopAdv();
        break;

    case kStateTxSending:
    case kStateTxAdvertising:
        ExitNow(error = OT_ERROR_BUSY);
        break;
    }

    SetState(kStateSleep);

exit:
    return error;
}

otError Controller::Receive(void)
{
    otError error = OT_ERROR_NONE;

    otLogInfoToblePeri("Receive()");

    switch (mState)
    {
    case kStateSleep:
    case kStateRxAdvertising:
        break;

    case kStateConnected:
        ExitNow();
        break;

    case kStateTxSending:
    case kStateTxAdvertising:
        ExitNow(error = OT_ERROR_INVALID_STATE);
        break;
    }

    StartAdvertising(false /* aTransmit */);

exit:
    return error;
}

void Controller::StartAdvertising(bool aTransmit)
{
    Advertisement       advData(mAdvDataBuffer, sizeof(mAdvDataBuffer));
    ScanResponse        scanRspData(mScanRspDataBuffer, sizeof(mScanRspDataBuffer));
    Advertisement::Info info;
    Platform::AdvConfig config;

    info.mL2capTransport     = false;
    info.mJoiningPermitted   = mJoiningPermitted;
    info.mDtcEnabled         = mDtcEnabled;
    info.mBorderAgentEnabled = mBorderAgentEnabled;
    info.mTobleRole          = mTobleRole;

    info.mSrcShort    = Get<Mac::Mac>().GetShortAddress();
    info.mSrcExtended = Get<Mac::Mac>().GetExtAddress();

    if (mJoiningPermitted)
    {
        info.mSteeringData = mSteeringData;
    }

    if (!aTransmit)
    {
        info.mLinkState = Advertisement::kRxReady;
        info.mPanId     = Get<Mac::Mac>().GetPanId();
        info.mDest.SetNone();
        mDestAddr.SetNone();

        SetState(kStateRxAdvertising);
    }
    else
    {
        OT_ASSERT(mTxFrame != NULL);

        // Get PanId from the current tx frame
        if (mTxFrame->GetDstPanId(info.mPanId) != OT_ERROR_NONE)
        {
            info.mPanId = Get<Mac::Mac>().GetPanId();
        }

        info.mDest      = mDestAddr;
        info.mLinkState = info.mDest.IsShort() ? Advertisement::kTxReadyToShort : Advertisement::kTxReadyToExtended;

        SetState(kStateTxAdvertising);
        otLogNoteToblePeri("info.mLinkState=%d info.mDest=%s", info.mLinkState, info.mDest.ToString().AsCString());
    }

    advData.Populate(info);

    config.mType     = OT_TOBLE_ADV_IND;
    config.mInterval = kAdvInterval;
    config.mData     = advData.GetData();
    config.mLength   = advData.GetLength();

    if (info.mJoiningPermitted || info.mBorderAgentEnabled)
    {
        scanRspData.Populate(info);

        config.mScanRspData       = scanRspData.GetData();
        config.mScanRspDataLength = scanRspData.GetLength();
    }
    else
    {
        config.mScanRspData       = NULL;
        config.mScanRspDataLength = 0;
    }

    otLogNoteToblePeri("StartAdvertising(AdvInfo:[%s])", info.ToString().AsCString());

    Get<Platform>().StopAdv();
    Get<Platform>().StartAdv(config);
}

void Controller::Disconnect(void)
{
    OT_ASSERT(mConn != NULL);

    otLogNoteToblePeri("%s", __func__);

    Get<Btp>().Stop(*mConn);
    Get<Platform>().Disconnect(mConn->mPlatConn);
    Get<ConnectionTable>().Remove(*mConn);
    mConn = NULL;
}

otError Controller::Transmit(Mac::TxFrame &aFrame)
{
    otError error = OT_ERROR_NONE;

    otLogInfoToblePeri("Transmit([%s]) -------------->\r\n", aFrame.ToInfoString().AsCString());

    VerifyOrExit(mState != kStateSleep, error = OT_ERROR_INVALID_STATE);
    VerifyOrExit(mTxFrame == NULL, error = OT_ERROR_INVALID_STATE);

    mTxFrame = &aFrame;

    if (mTxFrame->mInfo.mTxInfo.mSubType == OT_RADIO_SUB_TYPE_MLE_DISCOVERY_REQUEST)
    {
        otLogNoteToblePeri("CentCtrl: Send OT_RADIO_SUB_TYPE_MLE_DISCOVERY_REQUEST");
    }

    if (mTxFrame->mInfo.mTxInfo.mSubType == OT_RADIO_SUB_TYPE_MLE_PARENT_REQUEST)
    {
        otLogNoteToblePeri("CentCtrl: Send OT_RADIO_SUB_TYPE_MLE_PARENT_REQUEST");
    }

    OT_ASSERT(aFrame.GetDstAddr(mDestAddr) == OT_ERROR_NONE);

    if (mTxFrame->GetChannel() != Get<Mac::Mac>().GetPanChannel())
    {
        otLogNoteToblePeri("Skip frame tx on channel %d (pan-channel: %d)", mTxFrame->GetChannel(),
                           Get<Mac::Mac>().GetPanChannel());
        InvokeRadioTxDone(OT_ERROR_NONE);
        ExitNow();
    }

    if ((mTobleRole == AdvData::kRoleInactiveRouter) || (mTobleRole == AdvData::kRoleActiveRouter))
    {
        {
            Mac::PanId panId;
            // Get PanId from the current tx frame
            if (mTxFrame->GetDstPanId(panId) != OT_ERROR_NONE)
            {
                panId = Get<Mac::Mac>().GetPanId();
            }

            if (panId == OT_PANID_BROADCAST)
            {
                otLogNoteToblePeri("Skip sending to broadcast PAN-Id");
                InvokeRadioTxDone(OT_ERROR_NONE);
                ExitNow();
            }
        }

        if (mDestAddr.IsNone())
        {
            otLogNoteToblePeri("TxFrame has no destination address");
            InvokeRadioTxDone(OT_ERROR_NONE);
            ExitNow();
        }

        if (mDestAddr.IsBroadcast())
        {
            otLogNoteToblePeri("TxFrame is broadcast - not supported yet!");
            InvokeRadioTxDone(OT_ERROR_NONE);
            ExitNow();
        }
    }

    if (mConn == NULL)
    {
        otLogNoteToblePeri("Transmit: No existing connection");
        // No existing connection, start to send TX advertisement.
        StartAdvertising(true);
        otLogNoteToblePeri("%s: mConn==NULL, TimerStart(kWaitBleConnectionTimeout=%d)", __func__,
                           kWaitBleConnectionTimeout);
        TimerStart(kWaitBleConnectionTimeout);
    }
    else if (((mDestAddr.GetType() == Mac::Address::kTypeShort) &&
              (mDestAddr.GetShort() == mConn->mPeerAddr.GetShort())) ||
             ((mDestAddr.GetType() == Mac::Address::kTypeExtended) &&
              (mDestAddr.GetExtended() == mConn->mPeerAddr.GetExtended())))
    {
        // All ToBLE transports strip MAC FCS.
        uint16_t length = mTxFrame->GetPsduLength() - Mac::Frame::GetFcsSize();

        // If we already have a connection, send it using BTP and wait for callback
        // or a timeout.

        SetState(kStateTxSending);
        otLogNoteToblePeri("%s: mConn!=NULL, TimerStart(kConnectionTimeout=%d)", __func__, kConnectionTimeout);
        TimerStart(kConnectionTimeout);

        otLogNoteToblePeri("Transmit: Found an existing connection");
        Get<Btp>().Send(*mConn, mTxFrame->GetPsdu(), length);
    }
    else
    {
        // The existing connection doesn't match the destination address of the packet.
        // Disconnect existing connection and wait for `HandleDisconnected()` to start
        // sending TX advertisement.

        otLogNoteToblePeri("Transmit: Disconnect the existing connection. mDestAddr=%d, mConn->mPeerAddr=%d ",
                           mDestAddr.ToString().AsCString(), mConn->mPeerAddr.ToString().AsCString());
        Disconnect();
        otLogNoteToblePeri("%s: mConn!=NULL, TimerStart(kWaitBleConnectionTimeout=%d)", __func__,
                           kWaitBleConnectionTimeout);
        TimerStart(kWaitBleConnectionTimeout);
    }

exit:
    OT_UNUSED_VARIABLE(error);
    if (error != OT_ERROR_NONE)
    {
        otLogNoteToblePeri("%s: ERROR=%d --------------------!!!!!!", error);
    }

    return OT_ERROR_NONE; // error;
}

void Controller::TimerStart(uint32_t aTimeout)
{
    mTimer.Start(aTimeout);
}

void Controller::HandleTimer(Timer &aTimer)
{
    aTimer.GetOwner<Controller>().HandleTimer();
}

void Controller::HandleTimer(void)
{
    otLogDebgToblePeri("HandleTimer()");

    switch (mState)
    {
    case kStateSleep:
        if (mConn != NULL)
        {
            otLogNoteToblePeri("HandleTimer: TIMEOUT EXCEEDED: kSleepDisconnectTimeout");
            Disconnect();
        }
        break;

    case kStateConnected:
        // Connection timeout, disconnect and wait for `HandleDisconnected()`
        // to start advertising again.
        otLogNoteToblePeri("HandleTimer: CONN TIMEOUT EXCEEDED: kConnectionTimeout");
        Disconnect();
        break;

    case kStateTxSending:
        // Timed out waiting for BTP send done callback.
        otLogNoteToblePeri("HandleTimer: TX TIMEOUT EXCEEDED: kConnectionTimeout");
        Disconnect();
        InvokeRadioTxDone(mTxFrame->GetAckRequest() ? OT_ERROR_NO_ACK : OT_ERROR_NONE);
        break;

    case kStateTxAdvertising:
        otLogNoteToblePeri("HandleTimer: TX ADV TIMEOUT EXCEEDED: kWaitBleConnectionTimeout");
        StartAdvertising(false /* aTransmit */);
        InvokeRadioTxDone(mTxFrame->GetAckRequest() ? OT_ERROR_NO_ACK : OT_ERROR_NONE);
        break;

    case kStateRxAdvertising:
        OT_ASSERT(false);
        break;
    }
}

void Controller::HandleConnected(Platform::Connection *aPlatConn)
{
    otLogNoteToblePeri("%s: aPlatConn=%p ~~~~~~~~~~~~~~", __func__, aPlatConn);

    switch (mState)
    {
    case kStateSleep:
    case kStateConnected:
    case kStateTxSending:

        otLogNoteToblePeri("%s: Disconnect(aPlatConn=%p) !!!!!!!!", __func__, aPlatConn);
        Get<Platform>().Disconnect(aPlatConn);
        break;

    case kStateTxAdvertising:
    case kStateRxAdvertising:
        // VerifyOrExit(mConn == NULL, Get<Platform>().Disconnect(aPlatConn));

        if (mConn != NULL)
        {
            otLogNoteToblePeri("%s: Disconnect(aPlatConn=%p) !!!!!!!!", __func__, aPlatConn);
            Get<Platform>().Disconnect(aPlatConn);
            ExitNow();
        }

        Get<Platform>().StopAdv();
        OT_ASSERT((mConn = Get<ConnectionTable>().GetNew()) != NULL);
        mConn->mPlatConn = aPlatConn;
        if (mState == kStateTxAdvertising)
        {
            mConn->mPeerAddr = mDestAddr;
        }

        Get<Btp>().Start(*mConn);
        SetState(kStateConnected);

        if (mTxFrame != NULL)
        {
            // All ToBLE transports strip MAC FCS.
            uint16_t length = mTxFrame->GetPsduLength() - Mac::Frame::GetFcsSize();

            otLogNoteToblePeri("%s: mTxFrame!=NULL, TimerStart(kWaitTobleConnectionTimeout=%d)", __func__,
                               kWaitTobleConnectionTimeout);
            TimerStart(kWaitTobleConnectionTimeout);

            SetState(kStateTxSending);
            Get<Btp>().Send(*mConn, mTxFrame->GetPsdu(), length);
        }
        else
        {
            otLogNoteToblePeri("%s: mTxFrame==NULL, TimerStart(kWaitTobleConnectionTimeout=%d)", __func__,
                               kWaitTobleConnectionTimeout);
            TimerStart(kWaitTobleConnectionTimeout);
        }

        break;
    }

exit:
    return;
}

void Controller::HandleTransportConnected(Connection &aConn)
{
    OT_UNUSED_VARIABLE(aConn);
    if (mState == kStateConnected)
    {
        otLogNoteToblePeri("Refresh=%d", kConnectionTimeout);
        TimerStart(kConnectionTimeout);
    }
}

void Controller::ConnectionTimerRefresh(Connection &aConn)
{
    OT_UNUSED_VARIABLE(aConn);

    if ((mState == kStateConnected) || (mState == kStateTxSending))
    {
        // Push back the timeout for this connection.
        otLogNoteToblePeri("Refresh=%d", kConnectionTimeout);
        TimerStart(kConnectionTimeout);
    }
}

void Controller::HandleDisconnected(Platform::Connection *aPlatConn)
{
    Connection *conn = Get<ConnectionTable>().Find(aPlatConn);

    otLogNoteToblePeri("%s: aPlatConn=%p !!!!!!!!", __func__, aPlatConn);

    VerifyOrExit((conn != NULL) && (conn == mConn), OT_NOOP);

    Get<Btp>().Stop(*mConn);
    Get<ConnectionTable>().Remove(*mConn);
    mConn = NULL;

    mTimer.Stop();

exit:

    if (mState != kStateSleep)
    {
        StartAdvertising(mTxFrame != NULL);
    }

    return;
}

// This is callback from transport layer (BTP or L2CAP) after `SendTransport()`
// aError should OT_ERROR_NONE or NO_ACK.
void Controller::HandleTransportSendDone(Connection &aConn, otError aError)
{
    otLogInfoToblePeri("HandleTransportSendDone(err:%s)", otThreadErrorToString(aError));

    VerifyOrExit(mState == kStateTxSending, OT_NOOP);
    VerifyOrExit(mConn == &aConn, OT_NOOP);

    SetState(kStateConnected);

    InvokeRadioTxDone(aError);

exit:
    return;
}

// This is callback from transport layer (BTP or L2CAP) to indicate received frame.
void Controller::HandleTransportReceiveDone(Connection &aConn, uint8_t *aFrame, uint16_t aLength, otError aError)
{
    Mac::RxFrame rxFrame;

    VerifyOrExit(&aConn == mConn, OT_NOOP);

    // We should be able to pass up the buffer pointers we received from BTP to next layer.
    // OT core can possibly modify the frame content (up to length), e.g., in-place decryption.
    rxFrame.mPsdu    = aFrame;
    rxFrame.mLength  = aLength;
    rxFrame.mChannel = Get<Mac::Mac>().GetPanChannel();

    rxFrame.mInfo.mRxInfo.mAckedWithFramePending = true;

    rxFrame.mInfo.mRxInfo.mTimestamp = TimerMilli::GetNow().GetValue() * 1000;
    rxFrame.mInfo.mRxInfo.mRssi      = OT_TOBLE_DEFAULT_RSSI;
    rxFrame.mInfo.mRxInfo.mLqi       = OT_RADIO_LQI_NONE;

    if (mState == kStateConnected)
    {
        Mac::Address addr;

        if ((rxFrame.GetSrcAddr(addr) == OT_ERROR_NONE) && !addr.IsNone() && !addr.IsBroadcast())
        {
            mConn->mPeerAddr = addr;
            otLogInfoToblePeri("%s:Update PeerAddr=%s ", __func__, mConn->mPeerAddr.ToString().AsCString());
        }
    }

    otLogInfoToblePeri("HandleBtpReceiveDone(err:%s, [%s])", otThreadErrorToString(aError),
                       rxFrame.ToInfoString().AsCString());

    Get<Radio::Callbacks>().HandleReceiveDone(&rxFrame, aError);

exit:
    return;
}

void Controller::InvokeRadioTxDone(otError aError)
{
    Mac::TxFrame *txFrame = mTxFrame;

    mTxFrame = NULL;

    otLogNoteToblePeri("%s: --------------TX DONE ------->\r\n", __func__);
    if ((aError == OT_ERROR_NONE) && txFrame->GetAckRequest())
    {
        uint8_t      ackPsdu[kAckFrameLength];
        Mac::RxFrame ackFrame;

        memset(ackPsdu, 0, sizeof(ackPsdu));

        ackFrame.mPsdu   = ackPsdu;
        ackFrame.mLength = kAckFrameLength;

        ackFrame.mChannel = txFrame->GetChannel();

        ackFrame.mInfo.mRxInfo.mAckedWithFramePending = false;
        ackFrame.mInfo.mRxInfo.mTimestamp             = TimerMilli::GetNow().GetValue() * 1000;
        ackFrame.mInfo.mRxInfo.mRssi                  = OT_RADIO_RSSI_INVALID;
        ackFrame.mInfo.mRxInfo.mLqi                   = OT_RADIO_LQI_NONE;

        ackFrame.InitMacHeader(Mac::Frame::kFcfFrameAck | Mac::Frame::kFcfDstAddrNone | Mac::Frame::kFcfSrcAddrNone,
                               Mac::Frame::kSecNone);

        // We need to set "frame pending" flag so that on a child data
        // poll tx works properly, otherwise MAC layer would put a
        // sleepy child back to sleep after a data poll.

        ackFrame.SetFramePending(true);
        ackFrame.SetSequence(txFrame->GetSequence());

        otLogInfoToblePeri("InvokeRadioTxDone(err:%s, ack:[%s])", otThreadErrorToString(aError),
                           ackFrame.ToInfoString().AsCString());

        Get<Radio::Callbacks>().HandleTransmitDone(*txFrame, &ackFrame, aError);
    }
    else
    {
        otLogInfoToblePeri("InvokeRadioTxDone(err:%s)", otThreadErrorToString(aError));

        Get<Radio::Callbacks>().HandleTransmitDone(*txFrame, NULL, aError);
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

    case kStateConnected:
        str = "Connected";
        break;

    case kStateTxSending:
        str = "TxSending";
        break;

    case kStateTxAdvertising:
        str = "TxAdvertising";
        break;

    case kStateRxAdvertising:
        str = "RxAdvertising";
        break;
    }

    return str;
}

} // namespace Peripheral
} // namespace Toble
} // namespace ot

#endif // #if OPENTHREAD_CONFIG_TOBLE_ENABLE && OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
