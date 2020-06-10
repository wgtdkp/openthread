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
 *   This file implements ToBLE Controller for central mode.
 */

#include "controller_central.hpp"

#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "common/encoding.hpp"
#include "common/instance.hpp"
#include "common/locator-getters.hpp"
#include "common/logging.hpp"
#include "common/random.hpp"
#include "mac/mac.hpp"
#include "mac/mac_frame.hpp"

#if OPENTHREAD_CONFIG_ENABLE_TOBLE && OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE

#define DEBUG_SHOW_ALL_ADVERTISEMENTS 0

namespace ot {
namespace Toble {
namespace Central {

Controller::Controller(Instance &aInstance)
    : InstanceLocator(aInstance)
    , mState(kStateSleep)
    , mTxFrame(NULL)
    , mTxDest()
    , mTxConn(NULL)
    , mRxConn(NULL)
    , mConnTimer(aInstance, &Controller::HandleConnTimer, this)
    , mTxTimer(aInstance, &Controller::HandleTxTimer, this)
{
}

void Controller::SetState(State aState)
{
    if (aState != mState)
    {
        otLogNoteBle("CentCtrl::State %s -> %s", StateToString(mState), StateToString(aState));
        mState = aState;
    }
}

otError Controller::Sleep(void)
{
    otError     error = OT_ERROR_NONE;
    Connection *conn;

    otLogInfoBle("CentCtrl::Sleep()");

    switch (mState)
    {
    case kStateSleep:
        ExitNow();
        break;
    case kStateRxScanning:
        Get<Platform>().StopScan();
        break;
    case kStateRxConnecting:
        mRxConn = NULL;
        break;

    case kStateTxPending:
    case kStateTxScanning:
    case kStateTxConnecting:
    case kStateTxSending:
    case kStateTxSendingRxConnecting:
        ExitNow(error = OT_ERROR_BUSY);
        break;
    }

    for (conn = Get<ConnectionTable>().GetFirst(); conn != NULL; conn = Get<ConnectionTable>().GetNext(conn))
    {
        switch (conn->mState)
        {
        case Connection::kConnected:
        case Connection::kSending:
            Get<Transport>().Stop(*conn);

            // Fall through

        case Connection::kConnecting:
            Get<Platform>().Disconnect(conn->mPlatConn);
            Get<ConnectionTable>().Remove(*conn);
            break;
        }
    }

    SetState(kStateSleep);

exit:
    return error;
}

otError Controller::Receive(void)
{
    otError error = OT_ERROR_NONE;

    otLogInfoBle("CentCtrl::Receive()");

    switch (mState)
    {
    case kStateSleep:
        StartRxScanning();
        break;

    case kStateRxScanning:
    case kStateRxConnecting:
        break;

    case kStateTxPending:
    case kStateTxScanning:
    case kStateTxConnecting:
    case kStateTxSending:
    case kStateTxSendingRxConnecting:
        error = OT_ERROR_INVALID_STATE;
        break;
    }

    return error;
}

void Controller::StartRxScanning(void)
{
    otError error;

    otLogInfoBle("CentCtrl::StartRxScanning(int:%u, wind:%d)", kRxScanInterval, kRxScanWindow);

    error = Get<Platform>().StartScan(kRxScanInterval, kRxScanWindow);
    assert(error == OT_ERROR_NONE);

    SetState(kStateRxScanning);
}

otError Controller::Transmit(Mac::Frame &aFrame)
{
    otError error = OT_ERROR_NONE;

    otLogInfoBle("CentCtrl::Transmit([%s])", aFrame.ToInfoString().AsCString());

    switch (mState)
    {
    case kStateSleep:
    case kStateTxPending:
    case kStateTxScanning:
    case kStateTxConnecting:
    case kStateTxSending:
    case kStateTxSendingRxConnecting:
        ExitNow(error = OT_ERROR_INVALID_STATE);
        break;

    case kStateRxScanning:
        Get<Platform>().StopScan();
        break;

    case kStateRxConnecting:
        break;
    }

    mTxFrame = &aFrame;
    aFrame.GetDstAddr(mTxDest);

    mTxTimer.Start(kTxTimeout);

    StartTransmit();

exit:
    return error;
}

void Controller::StartTransmit(void)
{
    assert(mTxFrame != NULL);

    if (mTxFrame->GetChannel() != Get<Mac::Mac>().GetPanChannel())
    {
        otLogNoteBle("CentCtrl: Skip frame tx on channel %d (pan-channel: %d)", mTxFrame->GetChannel(),
                     Get<Mac::Mac>().GetPanChannel());
        InvokeRadioTxDone(OT_ERROR_NONE);
        ExitNow();
    }

    if (mTxDest.IsNone())
    {
        otLogNoteBle("CentCtrl: TxFrame has no destination address");
        InvokeRadioTxDone(OT_ERROR_NONE);
        ExitNow();
    }

    if (mTxDest.IsBroadcast())
    {
        otLogNoteBle("CentCtrl: TxFrame is broadcast - not supported yet!");
        // For now we don't support broadcast frame tx from central. Ideas for
        // future enhancements: Send the frame over all existing (connected)
        // connections if device is attached. If device is detached, scan and
        // connect to any device on our pan and send the broadcast frame to it.
        InvokeRadioTxDone(OT_ERROR_NONE);
        ExitNow();
    }

    // Destination is a unicast.

    // Check if we have a connection with this destination already.

    mTxConn = Get<ConnectionTable>().Find(mTxDest);

    if ((mTxConn == NULL) && mTxDest.IsShort())
    {
        // If the `mTxDest` address is short, check if we can find a connection
        // using extended address associated with same short address. This handles
        // the case where at the time the connection was established, the child/peer
        // did not yet know its short address, so the connection table entry recorded
        // it with its extended address only.

        Neighbor *neighbor = Get<Mle::MleRouter>().GetNeighbor(mTxDest.GetShort());

        if (neighbor != NULL)
        {
            Mac::Address addr;

            addr.SetExtended(neighbor->GetExtAddress());
            mTxConn = Get<ConnectionTable>().Find(addr);

            if (mTxConn != NULL)
            {
                mTxConn->mShortAddr = mTxDest.GetShort();
                otLogNoteBle("CentCtrl: Updated short addr, conn:[%s]", mTxConn->ToString().AsCString());
            }
        }
    }

    if (mTxConn == NULL)
    {
        // If there is no existing connection, start scanning and look
        // for advertisement from the `mTxDest`. We cannot scan if
        // a connection is being established for rx. In that case,
        // we switch to `TxPending` state and wait for `mRxConn` to
        // either connect or time out.

        if (mState == kStateRxConnecting)
        {
            SetState(kStateTxPending);
        }
        else
        {
            StartTxScanning();
        }

        ExitNow();
    }

    // We have a connection to the peer.

    switch (mTxConn->mState)
    {
    case Connection::kConnecting:
        if (mState == kStateRxConnecting)
        {
            // Current design only allows a single connection being
            // established at a time (to accommodate platforms not
            // capable of handling parallel connection requests).
            // Therefore, if we are in `RxConnecting` state and
            // `mTxConn` is also in `kConnecting` we expect the
            // `mTxConn` to be same as `mRxConn`.
            assert(mRxConn == mTxConn);
            mRxConn = NULL;
        }
        SetState(kStateTxConnecting);
        // Wait for either `HandleConnected` callback or a tx timeout.
        break;

    case Connection::kConnected:
        // If we already have a `mTxConn` in `kConnected` state, we
        // pass the frame to transport layer to be sent out. We allow
        // the frame tx to start even in the case where currently
        // waiting for a connection to be established for rx operation
        // (we switch to `TxSendingRxConnecting` state to remember
        // that there is a pending rx connection).

        SetState((mState == kStateRxConnecting) ? kStateTxSendingRxConnecting : kStateTxSending);
        mTxConn->mState = Connection::kSending;
        Get<Transport>().Send(*mTxConn, mTxFrame->GetPsdu(), mTxFrame->GetPsduLength());
        mTxConn->mDisconnectTime = TimerMilli::GetNow() + kTxDisconnectTimeout;
        UpdateConnTimer();
        // Wait for `TransportSendDone` callback, a tx timeout and/or a connection timeout.
        break;

    case Connection::kSending:
        // Connection can be in `kSending` state if from an earlier tx
        // we happen to get a tx timeout waiting for `TxDone` callback
        // before the connection itself was timed out. We abort the new
        // tx and wait for connection disconnect timeout or a late
        // `TransportSendDone` to disconnect or clear the connection's state.
        otLogNoteBle("Abort tx due to conn being busy from earlier tx, conn:[%s]", mTxConn->ToString().AsCString());
        InvokeRadioTxDone(OT_ERROR_ABORT);
        break;
    }

exit:
    return;
}

void Controller::StartTxScanning(void)
{
    otError error;

    otLogInfoBle("CentCtrl::StartTxScanning(int:%u, wind:%d)", kTxScanInterval, kTxScanWindow);

    error = Get<Platform>().StartScan(kTxScanInterval, kTxScanWindow);
    assert(error == OT_ERROR_NONE);

    SetState(kStateTxScanning);
}

void Controller::HandleTxTimer(Timer &aTimer)
{
    aTimer.GetOwner<Controller>().HandleTxTimer();
}

void Controller::HandleTxTimer(void)
{
    otLogInfoBle("CentCtrl::HandleTxTimer()");

    switch (mState)
    {
    case kStateSleep:
    case kStateRxScanning:
    case kStateRxConnecting:
        break;

    case kStateTxPending:
        // Timed out waiting for `mRxConn` to be connected.
        InvokeRadioTxDone(OT_ERROR_CHANNEL_ACCESS_FAILURE);
        break;

    case kStateTxScanning:
        // Timed out while scanning to find peer.
        InvokeRadioTxDone(mTxFrame->GetAckRequest() ? OT_ERROR_NO_ACK : OT_ERROR_NONE);
        break;

    case kStateTxConnecting:
        // Timed out while actively trying to establish a connection
        // with the peer. We let the connection attempt to proceed. This
        // way we can potentially have connection ready for a re-tx of
        // the frame. If the connection does not get established in
        // time, the `mConnTimer` callback will handle it.
        assert((mTxConn != NULL) && (mRxConn == NULL));
        mRxConn = mTxConn;
        mTxConn = NULL;
        InvokeRadioTxDone(mTxFrame->GetAckRequest() ? OT_ERROR_NO_ACK : OT_ERROR_NONE);
        break;

    case kStateTxSending:
    case kStateTxSendingRxConnecting:
        // Timed out while sending over the connection. We let either
        // a late `TransportSendDone()` callback or the connection
        // timeout callback clear the `mTxConn->mState`.
        assert(mTxConn != NULL);
        mTxConn = NULL;
        InvokeRadioTxDone(mTxFrame->GetAckRequest() ? OT_ERROR_NO_ACK : OT_ERROR_NONE);
        break;
    }
}

void Controller::HandleAdv(Platform::AdvType aAdvType,
                           const Address &   aSource,
                           const uint8_t *   aData,
                           uint16_t          aLength,
                           int8_t            aRssi)
{
    AdvData                    advData(aData, aLength);
    AdvData::Info              advInfo;
    Mac::Address               srcAddr;
    Connection *               conn;
    Platform::ConnectionConfig config;

    VerifyOrExit(aAdvType & OT_TOBLE_ADV_TYPE_CONNECTABLE);

#if DEBUG_SHOW_ALL_ADVERTISEMENTS
    otLogNoteBle("CentCtrl::HandleAdv(src:%s, rssi:%d, data:%s)", aSource.ToString().AsCString(), aRssi,
                 advData.ToString().AsCString());
#endif // DEBUG_SHOW_ALL_ADVERTISEMENTS

    switch (mState)
    {
    case kStateSleep:
    case kStateRxConnecting:
    case kStateTxConnecting:
    case kStateTxSending:
    case kStateTxSendingRxConnecting:
    case kStateTxPending:
        // We ignore any new received adv while actively trying to
        // connect. We may be able to relax this later if platform can
        // handle parallel connection requests.
        ExitNow();
        break;

    case kStateRxScanning:
    case kStateTxScanning:
        break;
    }

    SuccessOrExit(advData.Parse(advInfo));

    otLogNoteBle("CentCtrl::HandleAdv(AdvInfo:[%s])", advInfo.ToString().AsCString());

    // Decide based on the current state and the received adv info
    // whether to establish a connection with the peer or not.

    VerifyOrExit(advInfo.mPanId == Get<Mac::Mac>().GetPanId());

    switch (mState)
    {
    case kStateRxScanning:

        // Check that adv data indicates there is "frame pending" on peer (i.e.,
        // peer is trying to transmit), and that the destination from adv data
        // matches our address or is broadcast.

        VerifyOrExit(advInfo.mFramePending);

        switch (advInfo.mDest.GetType())
        {
        case Mac::Address::kTypeNone:
            ExitNow();
            break;

        case Mac::Address::kTypeShort:
            VerifyOrExit((advInfo.mDest.GetShort() == Get<Mac::Mac>().GetShortAddress()) ||
                         advInfo.mDest.IsBroadcast());
            break;

        case Mac::Address::kTypeExtended:
            VerifyOrExit(advInfo.mDest.GetExtended() == Get<Mac::Mac>().GetExtAddress());
            break;
        }

        break;

    case kStateTxScanning:

        // Check that the source address in adv data matches `mTxDest`.

        switch (mTxDest.GetType())
        {
        case Mac::Address::kTypeNone:
            ExitNow();
            break;

        case Mac::Address::kTypeShort:
            VerifyOrExit(advInfo.mSrcShort == mTxDest.GetShort());
            break;

        case Mac::Address::kTypeExtended:
            VerifyOrExit(advInfo.mSrcExtended == mTxDest.GetExtended());
            break;
        }

        break;

    default:
        assert(false);
        break;
    }

    // Verify there is no existing connection to this device
    // First search using short address, then based on extended
    // address.

    conn = NULL;

    if (advInfo.mSrcShort != Mac::kShortAddrInvalid)
    {
        srcAddr.SetShort(advInfo.mSrcShort);
        conn = Get<ConnectionTable>().Find(srcAddr);
    }

    if (conn == NULL)
    {
        srcAddr.SetExtended(advInfo.mSrcExtended);
        conn = Get<ConnectionTable>().Find(srcAddr);

        if ((conn != NULL) && (advInfo.mSrcShort != Mac::kShortAddrInvalid))
        {
            conn->mShortAddr = advInfo.mSrcShort;
        }
    }

    VerifyOrExit(conn == NULL);

    // Create a new connection to the device.

    conn = Get<ConnectionTable>().GetNew();

    if (conn == NULL)
    {
        otLogNoteBle("CentCtrl: Connection table is full - could not create new connection");
        ExitNow();
    }

    config.mInterval     = kConnectionInterval;
    config.mScanInterval = kConnectionScanInterval;
    config.mScanWindow   = kConnectionScanWindow;

    Get<Platform>().StopScan();
    conn->mPlatConn = Get<Platform>().CreateConnection(aSource, config);

    if (conn->mPlatConn == NULL)
    {
        otLogNoteBle("CentCtrl: Platform could not create new connection - restart scanning");

        Get<ConnectionTable>().Remove(*conn);

        switch (mState)
        {
        case kStateRxScanning:
            StartRxScanning();
            break;

        case kStateTxScanning:
            StartTxScanning();
            break;

        default:
            assert(false);
            break;
        }

        ExitNow();
    }

    conn->mExtAddr   = advInfo.mSrcExtended;
    conn->mShortAddr = advInfo.mSrcShort;
    conn->mRssi      = aRssi;
    conn->mState     = Connection::kConnecting;

#if OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
    if (advInfo.mL2capSupport)
    {
        // Use L2CAP transport if it is supported by both
        // peripheral and central
        conn->mTransport = Transport::kL2cap;
        conn->mL2capPsm  = advInfo.mL2capPsm;
    }
    else
#endif
    {
        conn->mTransport = Transport::kBtp;
    }

    switch (mState)
    {
    case kStateRxScanning:
        conn->mDisconnectTime = TimerMilli::GetNow() + kRxWaitToConnectTimeout;
        SetState(kStateRxConnecting);
        mRxConn = conn;
        break;

    case kStateTxScanning:
        conn->mDisconnectTime = TimerMilli::GetNow() + kTxWaitToConnectTimeout;
        SetState(kStateTxConnecting);
        mTxConn = conn;

        break;

    default:
        assert(false);
        break;
    }

    otLogNoteBle("CentCtrl: Created conn:[%s]", conn->ToString().AsCString());
    UpdateConnTimer();

exit:
    return;
}

void Controller::UpdateConnTimer(void)
{
    Connection *conn = Get<ConnectionTable>().FindEarliestDisconnectTime();

    VerifyOrExit(conn != NULL, mConnTimer.Stop());
    mConnTimer.StartAt(conn->mDisconnectTime, 0);

exit:
    otLogInfoBle("CentCtrl::UpdateConnTimer(), con:[%s]", (conn == NULL) ? "none" : conn->ToString().AsCString());
}

void Controller::HandleConnTimer(Timer &aTimer)
{
    aTimer.GetOwner<Controller>().HandleConnTimer();
}

void Controller::HandleConnTimer(void)
{
    uint32_t    now = TimerMilli::GetNow();
    Connection *conn;

    otLogInfoBle("CentCtrl::HandleConnTimer()");

    for (conn = Get<ConnectionTable>().GetFirst(); conn != NULL; conn = Get<ConnectionTable>().GetNext(conn))
    {
        if (!TimerScheduler::IsStrictlyBefore(now, conn->mDisconnectTime))
        {
            otLogNoteBle("Timed out - disconnecting conn:[%s]", conn->ToString().AsCString());

            switch (conn->mState)
            {
            case Connection::kSending:
            case Connection::kConnected:
                otLogDebgBle("CentCtrl::HandleConnTimer: TIMEOUT EXCEEDED: %s",
                             (conn == mTxConn) ? "kTxDisconnectTimeout" : "kRxDisconnectTimeout");
                Get<Transport>().Stop(*conn);

                // Fall through

            case Connection::kConnecting:
                otLogDebgBle("CentCtrl::HandleConnTimer: TIMEOUT EXCEEDED: %s",
                             (conn == mTxConn) ? "kTxWaitToConnectTimeout" : "kRxWaitToConnectTimeout");
                Get<Platform>().Disconnect(conn->mPlatConn);
                Get<ConnectionTable>().Remove(*conn);
                break;
            }
        }
    }

    // Clear `mRxConn` or `mTxConn` if timed out and got disconnected.

    if ((mRxConn != NULL) && !mRxConn->IsInUse())
    {
        mRxConn = NULL;
    }

    if ((mTxConn != NULL) && !mTxConn->IsInUse())
    {
        mTxConn = NULL;
    }

    switch (mState)
    {
    case kStateSleep:
    case kStateRxScanning:
    case kStateTxScanning:
        break;

    case kStateRxConnecting:
        if (mRxConn == NULL)
        {
            // `mRxConn` timed out and got disconnected while
            // establishing connection, we (re)start rx scanning.
            StartRxScanning();
        }
        break;

    case kStateTxPending:
        if (mRxConn == NULL)
        {
            // Tx operation is pending and waiting for `mRxConn` to be
            // established. Check if the `mRxConn` is disconnected and
            // start the transmission.
            StartTransmit();
        }
        break;

    case kStateTxConnecting:
    case kStateTxSending:
        if (mTxConn == NULL)
        {
            // `mTxConn` timed out and got disconnected while trying
            // to connect or sending. We (re)start scanning (waiting
            // to connect again or `TxTimer` to expire).
            StartTxScanning();
        }
        break;

    case kStateTxSendingRxConnecting:
        if (mTxConn == NULL)
        {
            // `mTxConn` timed out and got disconnect while we were
            // sending a frame. We check if there is a still a
            // pending rx connection being established, and if so we
            // move to `TxPending` state otherwise we restart tx
            // scanning.

            if (mRxConn == NULL)
            {
                StartTxScanning();
            }
            else
            {
                SetState(kStateTxPending);
            }
        }
        else
        {
            // `mTxConn` is still active and sending. We check if
            // the pending rx connection `mRxConn` has timed out.

            if (mRxConn == NULL)
            {
                SetState(kStateTxSending);
            }
        }
    }

    UpdateConnTimer();
}

void Controller::HandleConnected(Platform::Connection *aPlatConn)
{
    Connection *conn;

    conn = Get<ConnectionTable>().Find(aPlatConn);
    VerifyOrExit(conn != NULL);

    conn->mState = Connection::kConnected;

    // If the connection matches `mTxConn` (i.e., this is the connection
    // we intend to send on) use a different connection disconnect
    // timeout interval.
    conn->mDisconnectTime = TimerMilli::GetNow() + ((conn == mTxConn) ? kTxDisconnectTimeout : kRxDisconnectTimeout);

    otLogNoteBle("CentCtrl::HandleConnected(conn:[%s])", conn->ToString().AsCString());

    UpdateConnTimer();

    Get<Transport>().Start(*conn);

    switch (mState)
    {
    case kStateSleep:
    case kStateRxScanning:
    case kStateTxScanning:
    case kStateTxSending:
        break;

    case kStateRxConnecting:
        VerifyOrExit(conn == mRxConn);
        mRxConn = NULL;
        StartRxScanning();
        break;

    case kStateTxPending:
        if (conn == mRxConn)
        {
            mRxConn = NULL;
            StartTransmit();
        }
        break;

    case kStateTxSendingRxConnecting:
        VerifyOrExit(conn == mRxConn);
        mRxConn = NULL;
        SetState(kStateTxSending);
        break;

    case kStateTxConnecting:
        VerifyOrExit(conn == mTxConn);
        SetState(kStateTxSending);
        mTxConn->mState = Connection::kSending;
        Get<Transport>().Send(*mTxConn, mTxFrame->GetPsdu(), mTxFrame->GetPsduLength());
        // Wait for `TransportSendDone` callback, or a tx timeout and/or
        // a connection timeout.
        break;
    }

exit:
    return;
}

void Controller::HandleDisconnected(Platform::Connection *aPlatConn)
{
    Connection *conn;

    conn = Get<ConnectionTable>().Find(aPlatConn);
    VerifyOrExit(conn != NULL);

    otLogNoteBle("CentCtrl::HandleDisconnected(conn:[%s])", conn->ToString().AsCString());

    Get<Transport>().Stop(*conn);
    Get<ConnectionTable>().Remove(*conn);
    UpdateConnTimer();

    switch (mState)
    {
    case kStateSleep:
    case kStateRxScanning:
    case kStateTxScanning:
        break;

    case kStateRxConnecting:
        VerifyOrExit(conn == mRxConn);
        mRxConn = NULL;
        StartRxScanning();
        break;

    case kStateTxPending:
        VerifyOrExit(conn == mRxConn);
        mRxConn = NULL;
        StartTransmit();
        break;

    case kStateTxConnecting:
        VerifyOrExit(conn == mTxConn);
        mTxConn = NULL;
        StartTxScanning();
        break;

    case kStateTxSending:
        VerifyOrExit(conn == mTxConn);
        // Peer disconnected while sending a frame to it. Restart
        // scanning and wait for a new connection or a tx timeout.
        mTxConn = NULL;
        StartTxScanning();
        break;

    case kStateTxSendingRxConnecting:
        if (conn == mRxConn)
        {
            // The connecting `mRxConn` got disconnected.
            mRxConn = NULL;
            SetState(kStateTxSending);
        }
        else if (conn == mTxConn)
        {
            // Peer disconnected while sending a frame to it. Go back
            // to `TxPending` state and wait for `mRxConn` before
            // restarting scan for tx.
            mTxConn = NULL;
            SetState(kStateTxPending);
        }
        break;
    }

exit:
    return;
}

// This is callback from transport layer after `Get<Transport>().Send()`
// aError should OT_ERROR_NONE or NO_ACK.
void Controller::HandleTransportSendDone(Connection &aConn, otError aError)
{
    aConn.mState = Connection::kConnected;

    // If there is an error in tx, use a short timeout to disconnect quickly.
    // Otherwise, push back the disconnect time of this connection.

    aConn.mDisconnectTime = TimerMilli::GetNow();
    aConn.mDisconnectTime += ((aError == OT_ERROR_NONE) ? kRxDisconnectTimeout : kTxErrorDisconnectTimeout);

    otLogInfoBle("CentCtrl::HandleTransportSendDone(err:%s, conn:[%s])", otThreadErrorToString(aError),
                 aConn.ToString().AsCString());

    UpdateConnTimer();

    switch (mState)
    {
    case kStateTxSending:
    case kStateTxSendingRxConnecting:
        break;
    default:
        ExitNow();
    }

    VerifyOrExit(mTxConn == &aConn);

    mTxConn = NULL;
    InvokeRadioTxDone(aError);

exit:
    return;
}

void Controller::HandleTransportReceiveDone(Connection &aConn, uint8_t *aFrame, uint16_t aLength, otError aError)
{
    Mac::Frame rxFrame;

    otLogInfoBle("CentCtrl::HandleTransportReceiveDone(err:%s, conn:[%s])", otThreadErrorToString(aError),
                 aConn.ToString().AsCString());

    switch (aConn.mState)
    {
    case Connection::kConnecting:
    case Connection::kSending:
        // If we happen to receive on a connection which we are also
        // sending and waiting for `TxDone` callback, we do not push back
        // the timeout. This ensures that we time out and disconnect from
        // a possibly one-way (rx only) connection.
        break;

    case Connection::kConnected:
        // Push back the disconnect timeout for this connection.
        aConn.mDisconnectTime = TimerMilli::GetNow() + kRxDisconnectTimeout;
        UpdateConnTimer();
        break;
    }

    // We should be able to pass up the buffer pointers we received from
    // transport to next layer. OT core can possibly modify the frame content
    // (up to length), e.g., in-place decryption.
    rxFrame.mPsdu    = aFrame;
    rxFrame.mLength  = aLength;
    rxFrame.mChannel = Get<Mac::Mac>().GetRadioChannel();
    rxFrame.mIeInfo  = NULL;

    rxFrame.mInfo.mRxInfo.mAckedWithFramePending = true;

    rxFrame.mInfo.mRxInfo.mMsec = TimerMilli::GetNow();
    rxFrame.mInfo.mRxInfo.mUsec = 0;
    rxFrame.mInfo.mRxInfo.mRssi = aConn.mRssi;
    rxFrame.mInfo.mRxInfo.mLqi  = OT_RADIO_LQI_NONE;

    if (aConn.mShortAddr == Mac::kShortAddrInvalid)
    {
        // If the `conn` does not have a valid short address check if the
        // received frame has one and update the connection. This handles the
        // case where at the time the `Connection` was established, the
        // child/peer did not yet know its short address, so `mConTable` entry
        // recorded it with its extended address only.

        Mac::Address srcAddr;

        rxFrame.GetSrcAddr(srcAddr);

        if (srcAddr.IsShort())
        {
            aConn.mShortAddr = srcAddr.GetShort();

            otLogNoteBle("CentCtrl: Updated short addr,conn:[%s]", aConn.ToString().AsCString());
        }
    }

    otLogInfoBle("CentCtrl::HandleTransportReceiveDone(err:%s, frame:[%s])", otThreadErrorToString(aError),
                 rxFrame.ToInfoString().AsCString());

    Get<Radio::Callbacks>().HandleReceiveDone(&rxFrame, aError);
}

void Controller::InvokeRadioTxDone(otError aError)
{
    mTxTimer.Stop();

    // Start receive operation before notifying end of tx operation.

    if (mRxConn != NULL)
    {
        SetState(kStateRxConnecting);
    }
    else
    {
        StartRxScanning();
    }

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

        // We need to set "frame pending" flag so that on a child, data
        // poll tx works properly, otherwise MAC layer would put a
        // sleepy child back to sleep after a data poll.

        ackFrame.SetFramePending(true);
        ackFrame.SetSequence(mTxFrame->GetSequence());

        otLogInfoBle("CentCtrl::InvokeRadioTxDone(err:%s, ack:[%s])", otThreadErrorToString(aError),
                     ackFrame.ToInfoString().AsCString());

        Get<Radio::Callbacks>().HandleTransmitDone(*mTxFrame, &ackFrame, aError);
    }
    else
    {
        otLogInfoBle("CentCtrl::InvokeRadioTxDone(err:%s)", otThreadErrorToString(aError));

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

    case kStateRxScanning:
        str = "RxScanning";
        break;

    case kStateRxConnecting:
        str = "RxConnecting";
        break;

    case kStateTxPending:
        str = "TxPending";
        break;

    case kStateTxScanning:
        str = "TxScanning";
        break;

    case kStateTxConnecting:
        str = "TxConnecting";
        break;

    case kStateTxSending:
        str = "TxSending";
        break;

    case kStateTxSendingRxConnecting:
        str = "TxSendingRxConnecting";
        break;
    }

    return str;
}

} // namespace Central
} // namespace Toble
} // namespace ot

#endif // OPENTHREAD_CONFIG_ENABLE_TOBLE && OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
