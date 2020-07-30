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
 / @file
 *   This file implements ToBLE Controller for central mode.
 */

#include "toble/controller_central.hpp"

#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "common/encoding.hpp"
#include "common/instance.hpp"
#include "common/locator-getters.hpp"
#include "common/logging.hpp"
#include "common/random.hpp"
#include "mac/mac.hpp"
#include "mac/mac_frame.hpp"

#if OPENTHREAD_CONFIG_TOBLE_ENABLE && OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE

namespace ot {
namespace Toble {
namespace Central {

Controller::Controller(Instance &aInstance)
    : InstanceLocator(aInstance)
    , mState(kStateSleep)
    , mTxFrame(NULL)
    , mTxDest()
    , mTxConn(NULL)
    , mConnTimer(aInstance, &Controller::HandleConnTimer, this)
    , mTxTimer(aInstance, &Controller::HandleTxTimer, this)
    , mWaitForCreatingTxConnection(false)
    , mSendToPeers(false)
    , mDiscoverTarget(Mle::Mle::kDiscoverTargetAll)
    , mDiscoverEnableFiltering(false)
    , mDiscoverCcittIndex(0)
    , mDiscoverAnsiIndex(0)
    , mTxFrameType(OT_RADIO_SUB_TYPE_NONE)
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
    otLogNoteTobleCent("%s: mJoiningPermitted   =%d", __func__, mJoiningPermitted);
    otLogNoteTobleCent("%s: mBorderAgentEnabled =%d", __func__, mBorderAgentEnabled);
    otLogNoteTobleCent("%s: mTobleRole          =%d", __func__, mTobleRole);
}

void Controller::SetMleDiscoverRequestParameters(uint8_t  aDiscoverTarget,
                                                 bool     aEnableFiltering,
                                                 uint16_t aDiscoverCcittIndex,
                                                 uint16_t aDiscoverAnsiIndex)
{
    mDiscoverTarget          = aDiscoverTarget;
    mDiscoverEnableFiltering = aEnableFiltering;
    mDiscoverCcittIndex      = aDiscoverCcittIndex;
    mDiscoverAnsiIndex       = aDiscoverAnsiIndex;
}

void Controller::SetJoiningPermitted(bool aEnabled, otSteeringData *aSteeringData)
{
    mJoiningPermitted = aEnabled;
    if (aSteeringData != NULL)
    {
        mSteeringData.Init(*aSteeringData);
    }

    otLogNoteTobleCent("%s: mJoiningPermitted=%d ~~~", __func__, mJoiningPermitted);
}

void Controller::SetBoarderAgent(bool aEnabled)
{
    mBorderAgentEnabled = aEnabled;
    otLogNoteTobleCent("%s: mBorderAgentEnabled=%d ~~~", __func__, mBorderAgentEnabled);
}

void Controller::SetDtc(bool aEnabled)
{
    mDtcEnabled = aEnabled;
    otLogNoteTobleCent("%s: mDtcEnabled=%d ~~~", __func__, mDtcEnabled);
}

void Controller::SetTobleRole(uint8_t aRole)
{
    mTobleRole = aRole;
    otLogNoteTobleCent("%s: mTobleRole=%d ~~~", __func__, mTobleRole);
}

void Controller::ClearPeers(void)
{
    Peer *peer;

    mPeerIndex = 0;

    for (peer = &mPeers[0]; peer < OT_ARRAY_END(mPeers); peer++)
    {
        peer->mRssi = kRssiMin; // kRssiMin means peer is not in-use.
    }
}

void Controller::SavePeer(Platform::AdvPacket &aAdvPacket, Advertisement::Info &aAdvInfo)
{
    Peer *end = OT_ARRAY_END(mPeers);
    Peer *peer;

    // VerifyOrExit(aAdvInfo.mLinkState == Advertisement::kRxReady, OT_NOOP);
    VerifyOrExit(aAdvInfo.mSrcShort != Mac::kShortAddrInvalid, OT_NOOP);

    // Check if the peer info has been stored.

    for (peer = &mPeers[0]; peer < end; peer++)
    {
        if (peer->mRssi != kRssiMin)
        {
            VerifyOrExit(memcmp(&aAdvPacket.mSrcAddress, &peer->mBleAddress, sizeof(otTobleAddress)) != 0, OT_NOOP);
        }
    }

    // We keep the list sorted based on Rssi. Find the place to
    // add the new result.

    for (peer = &mPeers[0]; peer < end; peer++)
    {
        if (peer->mRssi < aAdvPacket.mRssi)
        {
            break;
        }
    }

    VerifyOrExit(peer < end, OT_NOOP);

    // Shift elements in array to make room for the new one.
    memmove(peer + 1, peer,
            static_cast<size_t>(reinterpret_cast<uint8_t *>(end - 1) - reinterpret_cast<uint8_t *>(peer)));

    peer->mRssi       = aAdvPacket.mRssi;
    peer->mBleAddress = aAdvPacket.mSrcAddress;
    peer->mSrcShort   = aAdvInfo.mSrcShort;

    otLogNoteTobleCent("%s: BleAddr=%02x%02x%02x%02x%02x%02x Rssi=%d", __func__, aAdvPacket.mSrcAddress.mAddress[0],
                       aAdvPacket.mSrcAddress.mAddress[1], aAdvPacket.mSrcAddress.mAddress[2],
                       aAdvPacket.mSrcAddress.mAddress[3], aAdvPacket.mSrcAddress.mAddress[4],
                       aAdvPacket.mSrcAddress.mAddress[5], aAdvPacket.mRssi);

exit:
    return;
}

void Controller::ProcessAdvertisement(Platform::AdvPacket &aAdvPacket, Advertisement::Info &aAdvInfo)
{
    if (mTxFrameType == OT_RADIO_SUB_TYPE_MLE_PARENT_REQUEST)
    {
        otLogNoteTobleCent("%s: aAdvInfo.mTobleRole=%d", aAdvInfo.mTobleRole);
        VerifyOrExit(aAdvInfo.mTobleRole == AdvData::kRoleActiveRouter, OT_NOOP);
        SavePeer(aAdvPacket, aAdvInfo);
    }
    else if (mTxFrameType == OT_RADIO_SUB_TYPE_MLE_DISCOVERY_REQUEST)
    {
        switch (mDiscoverTarget)
        {
        case Mle::Mle::kDiscoverTargetJoinerRouter:
            VerifyOrExit(aAdvInfo.mJoiningPermitted, OT_NOOP);

            if (mDiscoverEnableFiltering)
            {
                MeshCoP::SteeringDataTlv &steeringData = aAdvInfo.mSteeringData;

                VerifyOrExit(steeringData.IsValid(), OT_NOOP);
                VerifyOrExit((steeringData.GetBit(mDiscoverCcittIndex % steeringData.GetNumBits()) &&
                              steeringData.GetBit(mDiscoverAnsiIndex % steeringData.GetNumBits())),
                             OT_NOOP);
            }
            break;

        case Mle::Mle::kDiscoverTargetBorderAgent:
            VerifyOrExit(aAdvInfo.mBorderAgentEnabled, OT_NOOP);
            break;

        case Mle::Mle::kDiscoverTargetAll:
            break;
        }

        SavePeer(aAdvPacket, aAdvInfo);
    }

exit:
    return;
}

void Controller::SetState(State aState)
{
    if (aState != mState)
    {
        otLogNoteTobleCent("State %s ----------------------> %s", StateToString(mState), StateToString(aState));
        mState = aState;
    }
}

void Controller::SetTxState(TxState aTxState)
{
    if (aTxState != mTxState)
    {
        otLogNoteTobleCent("TxState %s ----------------------> %s", TxStateToString(mTxState),
                           TxStateToString(aTxState));
        mTxState = aTxState;
    }
}

otError Controller::Sleep(void)
{
    otError     error = OT_ERROR_NONE;
    Connection *conn;

    otLogInfoTobleCent("Sleep()");

    switch (mState)
    {
    case kStateSleep:
        ExitNow();
        break;
    case kStateScanning:
        StopScanning();
        break;
    case kStateNotScanning:
        break;
    }

    for (conn = Get<ConnectionTable>().GetFirst(); conn != NULL; conn = Get<ConnectionTable>().GetNext(conn))
    {
        switch (conn->mState)
        {
        case Connection::kConnected:
            Get<Btp>().Stop(*conn);

            // Fall through

        case Connection::kConnecting:
            otLogNoteTobleCent("%s: Disconnect(conn->mPlatConn=%p) !!!!!!!!", __func__, conn->mPlatConn);
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

    otLogInfoTobleCent("Receive()");

    switch (mState)
    {
    case kStateSleep:
        StartScanning();
        break;

    case kStateNotScanning:
    case kStateScanning:
        break;
    }

    return error;
}

void Controller::StopScanning(void)
{
    otLogNoteTobleCent("StopScanning()");

    OT_ASSERT(Get<Platform>().StopScan() == OT_ERROR_NONE);

    SetState(kStateNotScanning);
}

void Controller::StartScanning(void)
{
    otLogNoteTobleCent("StartScanning(int:%u, wind:%d)", kScanInterval, kScanWindow);

    OT_ASSERT(Get<Platform>().StartScan(kScanInterval, kScanWindow) == OT_ERROR_NONE);

    SetState(kStateScanning);
}

otError Controller::ScanPeers(void)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(mState != kStateSleep, error = OT_ERROR_INVALID_STATE);

    otLogInfoTobleCent("Scan()");

    ClearPeers();
    SetTxState(kStatePeerScanning);

exit:
    return error;
}

void Controller::ScanPeersDone(void)
{
    otLogNoteTobleCent("OT_RADIO_SUB_TYPE_MLE_PARENT_REQUEST  ScanDone");

    if (SendToNextPeer())
    {
        mSendToPeers = true;
    }
    else
    {
        // No parent peer was found.
        InvokeRadioTxDone(OT_ERROR_NONE);
    }
}

bool Controller::SendToNextPeer(void)
{
    bool sendToPeer = false;

    if (mPeerIndex < OT_ARRAY_LENGTH(mPeers))
    {
        Peer &peer = mPeers[mPeerIndex];

        VerifyOrExit(peer.mRssi != kRssiMin, OT_NOOP);

        mTxDest.SetShort(peer.mSrcShort);

        otLogNoteTobleCent("%s: mTxDest=%s", __func__, mTxDest.ToString().AsCString());
        StartTransmit();

        sendToPeer = true;
        mPeerIndex++;
    }

exit:
    return sendToPeer;
}

otError Controller::Transmit(Mac::TxFrame &aFrame)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(mState != kStateSleep, error = OT_ERROR_INVALID_STATE);
    VerifyOrExit(mTxFrame == NULL, error = OT_ERROR_INVALID_STATE);
    VerifyOrExit(mTxConn == NULL, error = OT_ERROR_INVALID_STATE);

    otLogInfoTobleCent("Transmit([%s])", aFrame.ToInfoString().AsCString());

    mTxFrame = &aFrame;
    SetTxState(kStateIdle);
    OT_ASSERT(aFrame.GetDstAddr(mTxDest) == OT_ERROR_NONE);

    if (mTxFrame->GetChannel() != Get<Mac::Mac>().GetPanChannel())
    {
        otLogNoteTobleCent("Skip frame tx on channel %d (pan-channel: %d)", mTxFrame->GetChannel(),
                           Get<Mac::Mac>().GetPanChannel());
        InvokeRadioTxDone(OT_ERROR_NONE);
        ExitNow();
    }

    if (mTxDest.IsNone())
    {
        otLogNoteTobleCent("TxFrame has no destination address");
        InvokeRadioTxDone(OT_ERROR_NONE);
        ExitNow();
    }

    mTxFrameType = mTxFrame->mInfo.mTxInfo.mSubType;
    if ((mTobleRole == AdvData::kRoleJoiner) || (mTobleRole == AdvData::kRoleEndDevice))
    {
        if ((mTxFrame->mInfo.mTxInfo.mSubType == OT_RADIO_SUB_TYPE_MLE_PARENT_REQUEST) ||
            (mTxFrame->mInfo.mTxInfo.mSubType == OT_RADIO_SUB_TYPE_MLE_DISCOVERY_REQUEST))
        {
            Connection *conn = Get<ConnectionTable>().GetFirst();

            if (conn != NULL)
            {
                otLogNoteTobleCent("Found an existing connection: ShortAddr=0x%04x", conn->mShortAddr);
                mTxDest.SetShort(conn->mShortAddr);
            }
            else
            {
                if (mTxFrameType == OT_RADIO_SUB_TYPE_MLE_PARENT_REQUEST)
                {
                    otLogNoteTobleCent("Send OT_RADIO_SUB_TYPE_MLE_PARENT_REQUEST");
                }
                else
                {
                    otLogNoteTobleCent("Send OT_RADIO_SUB_TYPE_MLE_DISCOVERY_REQUEST");
                }

                otLogNoteTobleCent("Scan potential parents");

                if (ScanPeers() == OT_ERROR_NONE)
                {
                    otLogNoteTobleCent("%s: mTxTimer.Start(kScanPeersTimeout=%d)", __func__, kScanPeersTimeout);
                    mTxTimer.Start(kScanPeersTimeout);
                }
                else
                {
                    InvokeRadioTxDone(OT_ERROR_NONE);
                }
            }

            ExitNow();
        }
    }

    if (mTxDest.IsBroadcast())
    {
        otLogNoteTobleCent("TxFrame is broadcast - not supported yet!");
        // For now we don't support broadcast frame tx from central. Ideas for
        // future enhancements: Send the frame over all existing (connected)
        // connections if device is attached. If device is detached, scan and
        // connect to any device on our pan and send the broadcast frame to it.
        InvokeRadioTxDone(OT_ERROR_NONE);
        ExitNow();
    }

    StartTransmit();

exit:
    return error;
}

void Controller::StartTransmit(void)
{
    OT_ASSERT(mTxFrame != NULL);

    otLogNoteTobleCent("%s: --------------TX------------->\r\n", __func__);

    // Destination is a unicast.

    // Check if we have a connection with this destination already.

    mTxConn = Get<ConnectionTable>().Find(mTxDest);

    otLogNoteTobleCent("mTxDest=%s", mTxDest.ToString().AsCString());

    if ((mTxConn == NULL) && mTxDest.IsShort())
    {
        // If the `mTxDest` address is short, check if we can find a connection
        // using extended address associated with same short address. This handles
        // the case where at the time the connection was established, the child/peer
        // did not yet know its short address, so the connection table entry recorded
        // it with its extended address only.

        Neighbor *neighbor = Get<Mle::MleRouter>().GetNeighbor(mTxDest);

        if (neighbor != NULL)
        {
            Mac::Address addr;

            addr.SetExtended(neighbor->GetExtAddress());
            mTxConn = Get<ConnectionTable>().Find(addr);

            if (mTxConn != NULL)
            {
                otLogNoteTobleCent("%s: Step 3", __func__);
                mTxConn->mShortAddr = mTxDest.GetShort();
                otLogNoteTobleCent("Updated short addr, conn:[%s]", mTxConn->ToString().AsCString());
            }
        }
    }

    if (mTxConn == NULL)
    {
        // If there is no existing connection, wait for "HandleAdv" callback to
        // find the the 'mTxDest'. In that case, we will create a connection with
        // the peer.

        mWaitForCreatingTxConnection = true;
        SetTxState(kStateTxScanning);

        otLogNoteTobleCent("%s: No existing connection", __func__);
        otLogNoteTobleCent("%s: mTxTimer.Start(kWaitBleConnectionTimeout=%d)", __func__, kWaitBleConnectionTimeout);
        mTxTimer.Start(kWaitBleConnectionTimeout);
        ExitNow();
    }

    // We have a connection to the peer.
    if (mTxConn->mState == Connection::kConnected)
    {
        // All ToBLE transports strip MAC FCS.
        uint16_t length = mTxFrame->GetPsduLength() - Mac::Frame::GetFcsSize();

        otLogNoteTobleCent("%s: Found an existing connection.", __func__);

        if (mState == kStateScanning)
        {
            // Stop scanning to reduce the packet loss rate of the connection.
            StopScanning();
        }

        SetTxState(kStateTxSending);
        Get<Btp>().Send(*mTxConn, mTxFrame->GetPsdu(), length);

        otLogNoteTobleCent("%s: mTxTimer.Start(kConnectionTimeout=%d)", __func__, kConnectionTimeout);
        mTxTimer.Start(kConnectionTimeout);
        mTxConn->mDisconnectTime = TimerMilli::GetNow() + kConnectionTimeout;
        UpdateConnTimer();
    }

    // Wait for `HandleConnected` callback, a tx timeout and/or a connection timeout.

exit:
    return;
}

void Controller::HandleTxTimer(Timer &aTimer)
{
    aTimer.GetOwner<Controller>().HandleTxTimer();
}

void Controller::HandleTxTimer(void)
{
    otLogNoteTobleCent("HandleTxTimer() mTxState=%d", mTxState);

    mTxTimer.Stop();

    switch (mTxState)
    {
    case kStateIdle:
        otLogNoteTobleCent("%s: ------------ERROR----------------->");
        //        OT_ASSERT(false);
        break;

    case kStatePeerScanning:
        ScanPeersDone();
        break;

    case kStateTxScanning:
    case kStateTxSending:
        if (mWaitForCreatingTxConnection)
        {
            mWaitForCreatingTxConnection = false;

            otLogNoteTobleCent("%s: mWaitForCreatingTxConnection = false", __func__);
            // Timed out waiting for `mTxConn` to be connected.
            InvokeRadioTxDone(OT_ERROR_CHANNEL_ACCESS_FAILURE);
        }
        else
        {
            // Timed out while sending over the connection. We let `HandleConnTimer`
            // callback close the connection.
            OT_ASSERT(mTxConn != NULL);

            mTxConn = NULL;
            otLogNoteTobleCent("%s: mTxConn = NULL", __func__);
            InvokeRadioTxDone(mTxFrame->GetAckRequest() ? OT_ERROR_NO_ACK : OT_ERROR_NONE);
        }
        break;
    }
}

bool Controller::IsAdvFromDest(Advertisement::Info &aAdvInfo, Mac::Address &aAddress)
{
    bool advFromDest = false;

    // VerifyOrExit((aAdvInfo.mLinkState == Advertisement::kRxReady), OT_NOOP);

    switch (aAddress.GetType())
    {
    case Mac::Address::kTypeNone:
        ExitNow();
        break;

    case Mac::Address::kTypeShort:
        VerifyOrExit(aAdvInfo.mSrcShort == mTxDest.GetShort(), OT_NOOP);
        advFromDest = true;
        break;

    case Mac::Address::kTypeExtended:
        VerifyOrExit(aAdvInfo.mSrcExtended == mTxDest.GetExtended(), OT_NOOP);
        advFromDest = true;
        break;
    }

exit:
    return advFromDest;
}

bool Controller::IsAdvSendToUs(Advertisement::Info &aAdvInfo)
{
    bool advSendToUs = false;

    // Check that adv data indicates there is "frame pending" on peer (i.e.,
    // peer is trying to transmit), and that the destination from adv data
    // matches our address or is broadcast.
    // otLogNoteTobleCent("info.mLinkState=%d info.mDest=%s", aAdvInfo.mLinkState,
    // aAdvInfo.mDest.ToString().AsCString());
    VerifyOrExit((aAdvInfo.mLinkState == Advertisement::kTxReadyToShort) ||
                     (aAdvInfo.mLinkState == Advertisement::kTxReadyToExtended),
                 OT_NOOP);

    switch (aAdvInfo.mDest.GetType())
    {
    case Mac::Address::kTypeNone:
        ExitNow();
        break;

    case Mac::Address::kTypeShort:

        if ((mTobleRole == AdvData::kRoleJoiner) || (mTobleRole == AdvData::kRoleEndDevice))
        {
            VerifyOrExit((aAdvInfo.mDest.GetShort() == Get<Mac::Mac>().GetShortAddress()) ||
                             (aAdvInfo.mDest.IsBroadcast() &&
                              (Get<Mle::MleRouter>().GetNeighbor(aAdvInfo.mSrcExtended) != NULL)),
                         OT_NOOP);
        }
        else
        {
            VerifyOrExit((aAdvInfo.mDest.GetShort() == Get<Mac::Mac>().GetShortAddress()) ||
                             aAdvInfo.mDest.IsBroadcast(),
                         OT_NOOP);
        }

        advSendToUs = true;
        break;

    case Mac::Address::kTypeExtended:
        VerifyOrExit(aAdvInfo.mDest.GetExtended() == Get<Mac::Mac>().GetExtAddress(), OT_NOOP);
        advSendToUs = true;
        break;
    }

exit:
    return advSendToUs;
}

void Controller::HandleScanResponse(Platform::AdvPacket &aAdvPacket)
{
    ScanResponse scanRespone(const_cast<uint8_t *>(aAdvPacket.mData), aAdvPacket.mLength);

    VerifyOrExit(mTxState == kStatePeerScanning, OT_NOOP);

    SuccessOrExit(scanRespone.Parse(mAdvInfo));

    ProcessAdvertisement(aAdvPacket, mAdvInfo);

exit:
    return;
}

void Controller::HandleAdv(Platform::AdvType aAdvType, Platform::AdvPacket &aAdvPacket)
{
    Advertisement              advData(const_cast<uint8_t *>(aAdvPacket.mData), aAdvPacket.mLength);
    Mac::Address               srcAddr;
    Connection *               conn;
    Platform::ConnectionConfig config;
    bool                       createTxConnection = false;
    bool                       createRxConnection = false;

    VerifyOrExit((aAdvType == OT_TOBLE_ADV_IND) || (aAdvType == OT_TOBLE_ADV_DIRECT_IND), OT_NOOP);
    VerifyOrExit(mState != kStateSleep, OT_NOOP);

    SuccessOrExit(advData.Parse(mAdvInfo));
    if (mAdvInfo.mSrcShort == 0xaabb)
    {
        // Drop Diag message
        ExitNow();
    }

    // otLogDebgTobleCent("HandleAdv(AdvInfo:[%s])", mAdvInfo.ToString().AsCString());

    // Decide based on the current state and the received adv info
    // whether to establish a connection with the peer or not.
#if 0
    otLogNoteTobleCent(
        "%s: SrcShort=0x%04x mWaitForCreatingTxConnection=%d Info.mPanid=0x%04x GetPanId()=%04x ---------------> ",
        __func__, mAdvInfo.mSrcShort, mWaitForCreatingTxConnection, mAdvInfo.mPanId, Get<Mac::Mac>().GetPanId());
#endif

    // In case a specific PAN ID of a Thread Network to be discovered is not known, Discovery
    // Request messages MUST have the Destination PAN ID in the IEEE 802.15.4 MAC header set
    // to be the Broadcast PAN ID (0xFFFF) and the Source PAN ID set to a randomly generated
    // value. In this case, the PAN ID of the MAC layer also be updated to a random value. When
    // we are sending the Disocver Request, we don't check the PAN ID.
    if ((mTobleRole == AdvData::kRoleJoiner) || (mTobleRole == AdvData::kRoleEndDevice))
    {
        if (!((mTxFrame != NULL) && (mTxFrameType == OT_RADIO_SUB_TYPE_MLE_DISCOVERY_REQUEST)))
        {
            VerifyOrExit((Get<Mac::Mac>().GetPanId() == Mac::kPanIdBroadcast) ||
                             (mAdvInfo.mPanId == Get<Mac::Mac>().GetPanId()),
                         OT_NOOP);
        }
    }

    if (mTxState == kStatePeerScanning)
    {
        ProcessAdvertisement(aAdvPacket, mAdvInfo);
    }

    if (mWaitForCreatingTxConnection && IsAdvFromDest(mAdvInfo, mTxDest))
    {
        createTxConnection = true;
        otLogNoteTobleCent("%s: createTxConnection ---------------> ", __func__);
    }

    if (IsAdvSendToUs(mAdvInfo))
    {
        createRxConnection = true;
        otLogNoteTobleCent("%s: createRxConnection <--------------- ", __func__);
    }

    VerifyOrExit(createTxConnection || createRxConnection, OT_NOOP);

    // Verify there is no existing connection to this device
    // First search using short address, then based on extended
    // address.

    conn = NULL;

    if (mAdvInfo.mSrcShort != Mac::kShortAddrInvalid)
    {
        srcAddr.SetShort(mAdvInfo.mSrcShort);
        conn = Get<ConnectionTable>().Find(srcAddr);
    }

    if (conn == NULL)
    {
        srcAddr.SetExtended(mAdvInfo.mSrcExtended);
        conn = Get<ConnectionTable>().Find(srcAddr);

        if ((conn != NULL) && (mAdvInfo.mSrcShort != Mac::kShortAddrInvalid))
        {
            conn->mShortAddr = mAdvInfo.mSrcShort;
        }
    }

    VerifyOrExit(conn == NULL, OT_NOOP);

    // Create a new connection to the device.

    conn = Get<ConnectionTable>().GetNew();

    if (conn == NULL)
    {
        otLogNoteTobleCent("Connection table is full - could not create new connection");
        ExitNow();
    }

    config.mInterval     = kConnectionInterval;
    config.mScanInterval = kConnectionScanInterval;
    config.mScanWindow   = kConnectionScanWindow;
    config.mLinkType     = kConnectionLinkTypeGatt;

    StopScanning();
    conn->mPlatConn = Get<Platform>().CreateConnection(*static_cast<const Address *>(&aAdvPacket.mSrcAddress), config);

    otLogNoteTobleCent("CreateConnection() conn->mPlatConn=%p", conn->mPlatConn);

    if (conn->mPlatConn == NULL)
    {
        otLogNoteTobleCent("Platform could not create new connection - restart scanning");

        Get<ConnectionTable>().Remove(*conn);
        StartScanning();
        ExitNow();
    }

    if (createTxConnection)
    {
        mWaitForCreatingTxConnection = false;
        mTxConn                      = conn;
    }

    conn->mExtAddr        = mAdvInfo.mSrcExtended;
    conn->mShortAddr      = mAdvInfo.mSrcShort;
    conn->mRssi           = aAdvPacket.mRssi;
    conn->mState          = Connection::kConnecting;
    conn->mDisconnectTime = TimerMilli::GetNow() + kWaitBleConnectionTimeout;

    UpdateConnTimer();

    otLogNoteTobleCent("Created conn:[%s] ShortAddr=0x%04x   <------------>", conn->ToString().AsCString(),
                       mAdvInfo.mSrcShort);

exit:
    return;
}

void Controller::UpdateConnTimer(void)
{
    Connection *conn = Get<ConnectionTable>().FindEarliestDisconnectTime();

    VerifyOrExit(conn != NULL, mConnTimer.Stop());
    mConnTimer.StartAt(conn->mDisconnectTime, 0);

exit:
    return;
}

void Controller::HandleConnTimer(Timer &aTimer)
{
    aTimer.GetOwner<Controller>().HandleConnTimer();
}

void Controller::HandleConnTimer(void)
{
    TimeMilli   now = TimerMilli::GetNow();
    Connection *conn;
    bool        scanning = false;

    otLogInfoTobleCent("HandleConnTimer()");

    for (conn = Get<ConnectionTable>().GetFirst(); conn != NULL; conn = Get<ConnectionTable>().GetNext(conn))
    {
        if (now >= conn->mDisconnectTime)
        {
            otLogNoteTobleCent("Timed out - disconnecting conn:[%s]", conn->ToString().AsCString());

            if (conn->mState == Connection::kConnecting)
            {
                // The scanning has been stopped in state `kConnecting`, in case if
                // timeout when creating a connection.
                scanning = true;
                otLogNoteTobleCent("%s: scanning = true", __func__);
            }

            switch (conn->mState)
            {
            case Connection::kConnected:
                otLogDebgTobleCent("HandleConnTimer: TIMEOUT EXCEEDED: %s",
                                   (conn == mTxConn) ? "kTxDisconnectTimeout" : "kRxDisconnectTimeout");
                Get<Btp>().Stop(*conn);

                // Fall through

            case Connection::kConnecting:
                otLogDebgTobleCent("HandleConnTimer: TIMEOUT EXCEEDED: %s",
                                   (conn == mTxConn) ? "kTxWaitToConnectTimeout" : "kRxWaitToConnectTimeout");
                otLogNoteTobleCent("%s: Disconnect(conn->mPlatConn=%p) !!!!!!!!", __func__, conn->mPlatConn);
                Get<Platform>().Disconnect(conn->mPlatConn);
                Get<ConnectionTable>().Remove(*conn);
                break;
            }
        }
    }

    // Clear `mTxConn` if timed out and got disconnected.
    // Waitting for "HandleAdv()" to create a connection for transmitting.
    if ((mTxConn != NULL) && !mTxConn->IsInUse())
    {
        mTxConn                      = NULL;
        mWaitForCreatingTxConnection = true;
    }

    if (scanning)
    {
        StartScanning();
    }

    UpdateConnTimer();
}

void Controller::HandleConnected(Platform::Connection *aPlatConn)
{
    Connection *conn;

    otLogNoteTobleCent("%s: aPlatConn=%p ~~~~~~~~~~~~~~", __func__, aPlatConn);
    VerifyOrExit((conn = Get<ConnectionTable>().Find(aPlatConn)) != NULL, OT_NOOP);
    VerifyOrExit(conn->mState == Connection::kConnecting, OT_NOOP);

    conn->mState          = Connection::kConnected;
    conn->mDisconnectTime = TimerMilli::GetNow() + kWaitTobleConnectionTimeout;

    otLogNoteTobleCent("HandleConnected(conn:[%s])", conn->ToString().AsCString());

    UpdateConnTimer();

    Get<Btp>().Start(*conn);

    if (mTxConn == conn)
    {
        // All ToBLE transports strip MAC FCS.
        uint16_t length = mTxFrame->GetPsduLength() - Mac::Frame::GetFcsSize();

        SetTxState(kStateTxSending);
        Get<Btp>().Send(*conn, mTxFrame->GetPsdu(), length);
    }

exit:
    if (mTxConn != conn)
    {
        StartScanning();
    }

    return;
}

void Controller::HandleDisconnected(Platform::Connection *aPlatConn)
{
    Connection *conn;

    otLogNoteTobleCent("%s: aPlatConn=%p !!!!!!!!", __func__, aPlatConn);

    conn = Get<ConnectionTable>().Find(aPlatConn);
    VerifyOrExit(conn != NULL, OT_NOOP);

    otLogNoteTobleCent("HandleDisconnected(conn:[%s]) !!!!!!!!", conn->ToString().AsCString());

    Get<Btp>().Stop(*conn);
    Get<ConnectionTable>().Remove(*conn);
    UpdateConnTimer();

exit:
    return;
}

void Controller::HandleTransportConnected(Connection &aConn)
{
    if (aConn.mState == Connection::kConnected)
    {
        aConn.mDisconnectTime = TimerMilli::GetNow() + kConnectionTimeout;
        UpdateConnTimer();
        otLogNoteTobleCent("Refresh=%d", kConnectionTimeout);
        mTxTimer.Start(kConnectionTimeout);
    }
}

void Controller::ConnectionTimerRefresh(Connection &aConn)
{
    if (aConn.mState == Connection::kConnected)
    {
        aConn.mDisconnectTime = TimerMilli::GetNow() + kConnectionTimeout;
        UpdateConnTimer();
        otLogNoteTobleCent("Refresh=%d", kConnectionTimeout);
        mTxTimer.Start(kConnectionTimeout);
    }
}

// This is callback from transport layer after `Get<Btp>().Send()`
// aError should OT_ERROR_NONE or NO_ACK.
void Controller::HandleTransportSendDone(Connection &aConn, otError aError)
{
    otLogInfoTobleCent("HandleTransportSendDone(err:%s, conn:[%s])", otThreadErrorToString(aError),
                       aConn.ToString().AsCString());

    if (mState == kStateNotScanning)
    {
        StartScanning();
    }

    VerifyOrExit(mTxConn == &aConn, OT_NOOP);

    mTxConn = NULL;

    if (mSendToPeers)
    {
        if (!SendToNextPeer())
        {
            mSendToPeers = false;
            InvokeRadioTxDone(aError);
        }
    }
    else
    {
        InvokeRadioTxDone(aError);
    }

exit:
    return;
}

void Controller::HandleTransportReceiveDone(Connection &aConn, uint8_t *aFrame, uint16_t aLength, otError aError)
{
    Mac::RxFrame rxFrame;

    otLogInfoTobleCent("HandleTransportReceiveDone(err:%s, conn:[%s])", otThreadErrorToString(aError),
                       aConn.ToString().AsCString());

    // We should be able to pass up the buffer pointers we received from
    // transport to next layer. OT core can possibly modify the frame content
    // (up to length), e.g., in-place decryption.
    rxFrame.mPsdu    = aFrame;
    rxFrame.mLength  = aLength;
    rxFrame.mChannel = Get<Mac::Mac>().GetPanChannel();

    rxFrame.mInfo.mRxInfo.mAckedWithFramePending = true;

    rxFrame.mInfo.mRxInfo.mTimestamp = TimerMilli::GetNow().GetValue() * 1000;
    rxFrame.mInfo.mRxInfo.mRssi      = aConn.mRssi;
    rxFrame.mInfo.mRxInfo.mLqi       = OT_RADIO_LQI_NONE;

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

            otLogNoteTobleCent("Updated short addr,conn:[%s]", aConn.ToString().AsCString());
        }
    }

    otLogInfoTobleCent("HandleTransportReceiveDone(err:%s, frame:[%s])", otThreadErrorToString(aError),
                       rxFrame.ToInfoString().AsCString());

    Get<Radio::Callbacks>().HandleReceiveDone(&rxFrame, aError);
}

void Controller::InvokeRadioTxDone(otError aError)
{
    Mac::TxFrame *txFrame = mTxFrame;

    otLogNoteTobleCent("%s: --------------TX DONE ------->\r\n", __func__);

    mTxFrame = NULL;
    mTxTimer.Stop();
    SetTxState(kStateIdle);

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

        // We need to set "frame pending" flag so that on a child, data
        // poll tx works properly, otherwise MAC layer would put a
        // sleepy child back to sleep after a data poll.

        ackFrame.SetFramePending(true);
        ackFrame.SetSequence(txFrame->GetSequence());

        otLogInfoTobleCent("InvokeRadioTxDone(err:%s, ack:[%s])", otThreadErrorToString(aError),
                           ackFrame.ToInfoString().AsCString());

        Get<Radio::Callbacks>().HandleTransmitDone(*txFrame, &ackFrame, aError);
    }
    else
    {
        otLogInfoTobleCent("InvokeRadioTxDone(err:%s)", otThreadErrorToString(aError));

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

    case kStateNotScanning:
        str = "NotScanning";
        break;

    case kStateScanning:
        str = "Scanning";
        break;
    }

    return str;
}

const char *Controller::TxStateToString(TxState aTxState)
{
    const char *str = "Unknown";

    switch (aTxState)
    {
    case kStateIdle:
        str = "Idle";
        break;

    case kStateTxSending:
        str = "TxSending";
        break;

    case kStatePeerScanning:
        str = "PeerScanning";
        break;

    case kStateTxScanning:
        str = "TxScanning";
        break;
    }

    return str;
}

} // namespace Central
} // namespace Toble
} // namespace ot

#endif // #if OPENTHREAD_CONFIG_TOBLE_ENABLE && OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
