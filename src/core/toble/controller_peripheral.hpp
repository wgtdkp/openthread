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
 *   This file contains implementation of ToBLE controller for peripheral.
 */

#ifndef TOBLE_CONTROLLER_PERIPHERAL_HPP_
#define TOBLE_CONTROLLER_PERIPHERAL_HPP_

#include "openthread-core-config.h"

#include <openthread/platform/toble.h>

#include "common/locator.hpp"
#include "common/timer.hpp"
#include "mac/mac_frame.hpp"
#include "toble/adv_data.hpp"
#include "toble/platform.hpp"

#if OPENTHREAD_CONFIG_TOBLE_ENABLE && OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE

namespace ot {
namespace Toble {

class Toble;
class Connection;

namespace Peripheral {

class Controller : public InstanceLocator
{
    friend class Platform::Callbacks;
    friend class ot::Toble::Toble;

public:
    explicit Controller(Instance &aInstance);

    void    test(void);
    otError Sleep(void);
    otError Receive(void);
    otError Transmit(Mac::TxFrame &aFrame);

    // Callbacks from `Transport`
    void HandleTransportSendDone(Connection &aConn, otError aError);
    void HandleTransportReceiveDone(Connection &aConn, uint8_t *aFrame, uint16_t aLength, otError aError);
    void HandleTransportConnected(Connection &aConn);
    void ConnectionTimerRefresh(Connection &aConn);

    void SetJoiningPermitted(bool aEnabled, otSteeringData *aSteeringData);
    void SetDtc(bool aEnabled);
    void SetBoarderAgent(bool aEnabled);
    void SetTobleRole(uint8_t aRole);

private:
    enum State
    {
        kStateSleep,
        kStateTxAdvertising,
        kStateRxAdvertising,
        kStateConnected,
        kStateTxSending,
    };

    enum
    {
        kSleepDisconnectTimeout = 90, // Time to wait after `Sleep() to disconnect if already connected
#if 0
        kTransportStartTimeout  = 3000,  // 6000, // 3000, // Maximum time waiting for transport to be setup.
        kRxModeAdvInterval      = 20,    // Adv interval during Rx Mode
        kRxModeAdvStartDelay    = 2,     // Delay when entering receive before starting BLE advertisements.
        kRxModeConnTimeout      = 10000, // 5000, // 2000, // Timeout keeping connection open while in rx mode
                                         // (kicked on recvd frame).
        kTxConnectTimeout =
            5000, // 1000, // Maximum time waiting for connection to be established while in tx mode.
        kTxTimeout         = 4000, // 1000, // Tx timeout interval (max time waiting for tx to finish).
        kTxModeAdvInterval = 20,   // Adv interval during Tx Mode
#endif
        kAckFrameLength = 5,

        kAdvInterval                = 20,
        kConnectionInterval         = 30,
        kWaitBleConnectionTimeout   = 10 * kConnectionInterval,
        kWaitTobleConnectionTimeout = (7 + 2) * 2 * kConnectionInterval,
        kConnectionTimeout          = Timer::kMaxDelay, //(kWaitBleConnectionTimeout + kWaitTobleConnectionTimeout),
    };

    void SetState(State aState);
    void InvokeRadioTxDone(otError aError);
    void StartAdvertising(bool aTransmit);
    void UpdateAdvertising(void);
    void Disconnect(void);

    // Callbacks from platform
    void HandleConnected(Platform::Connection *aPlatConn);
    void HandleDisconnected(Platform::Connection *aPlatConn);

    static void HandleTimer(Timer &aTimer);
    void        HandleTimer(void);
    void        TimerStart(uint32_t aTimeout);

    static const char *StateToString(State aState);

    Mac::Address       mDestAddr;
    Connection *       mConn;
    TimerMilli         mTimer;
    State              mState;
    Mac::TxFrame *     mTxFrame;
    bool               mJoiningPermitted;
    bool               mBorderAgentEnabled;
    bool               mDtcEnabled;
    AdvData::TobleRole mTobleRole;

    MeshCoP::SteeringDataTlv mSteeringData;

    uint8_t mAdvDataBuffer[AdvData::kMaxSize];
    uint8_t mScanRspDataBuffer[AdvData::kMaxSize];
};
} // namespace Peripheral
} // namespace Toble
} // namespace ot

#endif // #if OPENTHREAD_CONFIG_TOBLE_ENABLE && OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
#endif // TOBLE_CONTROLLER_PERIPHERAL_HPP_
