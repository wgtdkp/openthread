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
 *   This file contains implementation of ToBLE controller for central.
 */

#ifndef TOBLE_CONTROLLER_CENTRAL_HPP_
#define TOBLE_CONTROLLER_CENTRAL_HPP_

#include "openthread-core-config.h"

#include <openthread/platform/toble.h>

#include "common/locator.hpp"
#include "common/timer.hpp"
#include "mac/mac_frame.hpp"
#include "thread/mle.hpp"
#include "toble/adv_data.hpp"
#include "toble/platform.hpp"

#if OPENTHREAD_CONFIG_TOBLE_ENABLE && OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE

namespace ot {
namespace Toble {

class Toble;
class Connection;

namespace Central {

class Controller : public InstanceLocator
{
    friend class Platform::Callbacks;
    friend class ot::Toble::Toble;

public:
    Controller(Instance &aInstance);

    void    test(void);
    otError Sleep(void);
    otError Receive(void);
    otError Transmit(Mac::TxFrame &aFrame);

    // Callbacks from `Transport`
    void HandleTransportSendDone(Connection &aConn, otError aError);
    void HandleTransportReceiveDone(Connection &aConn, uint8_t *aFrame, uint16_t aLength, otError aError);

    void SetMleDiscoverRequestParameters(uint8_t  aDiscoverTarget,
                                         bool     aEnableFiltering,
                                         uint16_t aDiscoverCcittIndex,
                                         uint16_t aDiscoverAnsiIndex);

    void HandleTransportConnected(Connection &aConn);
    void ConnectionTimerRefresh(Connection &aConn);

    void SetJoiningPermitted(bool aEnabled, otSteeringData *aSteeringData);
    void SetDtc(bool aEnabled);
    void SetBoarderAgent(bool aEnabled);
    void SetTobleRole(uint8_t aRole);

private:
    enum State
    {
        kStateSleep,       // Sleep (Radio off).
        kStateNotScanning, // Not scanning.
        kStateScanning,    // Scanning.
    };

    enum
    {
#if 0
         kRxScanInterval = 40, // Scan interval while in rx mode (msec, same as kConnectionInterval).
         kRxScanWindow   = 30, // Scan window while in rx mode (msec, larger than peripheral's kTxModeAdvInterval).

         kTxScanInterval = 40, // Scan interval while in tx mode (msec, same as kConnectionInterval).
         kTxScanWindow = 30,   // Scan window while in tx mode (msec, larger than peripheral's kRxModeAdvInterval).

         kRxWaitToConnectTimeout = 1600, // Wait time to establish a connection for rx.
         kTxWaitToConnectTimeout = 1600, // Wait time to establish a connection for tx.

         kTxTimeout = 10000, // 5000, // Tx timeout interval (max time waiting for entire tx operation to finish.

         kRxDisconnectTimeout      = 5000, // Timeout to disconnect from an idle connected connection.
         kTxDisconnectTimeout      = 5000, // Timeout to disconnect from a connection in sending/tx state.
         kTxErrorDisconnectTimeout = 5,    // Timeout to disconnect after a tx error happens.
#endif

        kConnectionInterval     = 30,                  // The connection data interval (msec).
        kConnectionScanInterval = kConnectionInterval, // Scan interval when trying to establish a connection
                                                       // (msec, same as kConnectionInterval).
        kConnectionScanWindow = 20,                    // Scan window when trying to establish a connection (msec,
                                                       // larger than peripheral's kAdvInterval).

        kScanInterval               = kConnectionInterval,
        kScanWindow                 = kConnectionScanWindow,
        kWaitBleConnectionTimeout   = 10 * kConnectionInterval,
        kWaitTobleConnectionTimeout = (7 + 2) * 2 * kConnectionInterval,
        kScanPeersTimeout           = 10 * kConnectionInterval,
        kConnectionTimeout          = (kWaitBleConnectionTimeout + kWaitTobleConnectionTimeout), // Timer::kMaxDelay,

        kAckFrameLength = 5,
        kMaxPeers       = 4,
        kRssiMin        = -127,
    };

    typedef enum
    {
        kStateIdle,
        kStateTxSending,
        kStatePeerScanning,
        kStateTxScanning,
    } TxState;

    void SetState(State aState);
    void SetTxState(TxState aTxState);
    void StartScanning(void);
    void StopScanning(void);
    void StartTransmit(void);
    void InvokeRadioTxDone(otError aError);

    bool IsAdvSendToUs(Advertisement::Info &aAdvInfo);
    bool IsAdvFromDest(Advertisement::Info &aAdvInfo, Mac::Address &aAddress);
    void ProcessAdvertisement(Platform::AdvPacket &aAdvPacket, Advertisement::Info &aAdvInfo);

    void UpdateConnTimer(void);

    void HandleAdv(Platform::AdvType aAdvType, Platform::AdvPacket &aAdvPacket);
    void HandleScanResponse(Platform::AdvPacket &aAdvPacket);

    void HandleConnected(Platform::Connection *aPlatConn);
    void HandleDisconnected(Platform::Connection *aPlatConn);

    static void HandleConnTimer(Timer &aTimer);
    void        HandleConnTimer(void);

    static void HandleTxTimer(Timer &aTimer);
    void        HandleTxTimer(void);

    void    ClearPeers(void);
    void    SavePeer(Platform::AdvPacket &aAdvPacket, Advertisement::Info &aAdvInfo);
    otError ScanPeers(void);
    void    ScanPeersDone(void);
    bool    SendToNextPeer(void);

    static const char *StateToString(State aState);
    static const char *TxStateToString(TxState aTxState);

    struct Peer
    {
        int8_t            mRssi;
        otTobleAddress    mBleAddress;
        Mac::ShortAddress mSrcShort;
    };

    State         mState;
    TxState       mTxState;
    Mac::TxFrame *mTxFrame;
    Mac::Address  mTxDest;
    Connection *  mTxConn; // Connection being used for tx.
    TimerMilli    mConnTimer;
    TimerMilli    mTxTimer;
    bool          mWaitForCreatingTxConnection;

    Advertisement::Info mAdvInfo;

    Peer    mPeers[kMaxPeers];
    uint8_t mPeerIndex;
    bool    mSendToPeers;

    uint8_t  mDiscoverTarget;
    bool     mDiscoverEnableFiltering;
    uint16_t mDiscoverCcittIndex;
    uint16_t mDiscoverAnsiIndex;

    uint8_t mTxFrameType;
    bool    mJoiningPermitted;
    bool    mBorderAgentEnabled;
    bool    mDtcEnabled;
    uint8_t mTobleRole;

    MeshCoP::SteeringDataTlv mSteeringData;
};

} // namespace Central
} // namespace Toble
} // namespace ot

#endif // #if OPENTHREAD_CONFIG_TOBLE_ENABLE && OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
#endif // TOBLE_CONTROLLER_CENTRAL_HPP_
