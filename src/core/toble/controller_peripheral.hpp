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

#if OPENTHREAD_CONFIG_ENABLE_TOBLE && OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE

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
    Controller(Instance &aInstance);

    otError Sleep(void);
    otError Receive(void);
    otError Transmit(Mac::Frame &aFrame);

    // Callbacks from `Transport`
    void HandleTransportSendDone(Connection &aConn, otError aError);
    void HandleTransportReceiveDone(Connection &aConn, uint8_t *aFrame, uint16_t aLength, otError aError);

private:
    enum State
    {
        kStateSleep,
        kStateRx,
        kStateTxSending,
        kStateTxAdvertising,
    };

    enum
    {
        kSleepDisconnectTimeout = 90,   // Time to wait after `Sleep() to disconnect if already connected
        kTransportStartTimeout  = 3000, // Maximum time waiting for transport to be setup.
        kRxModeAdvInterval      = 20,   // Adv interval during Rx Mode
        kRxModeAdvStartDelay    = 2,    // Delay when entering receive before starting BLE advertisements.
        kRxModeConnTimeout      = 2000, // Timeout keeping connection open while in rx mode (kicked on recvd frame).
        kTxConnectTimeout       = 1000, // Maximum time waiting for connection to be established while in tx mode.
        kTxTimeout              = 1000, // Tx timeout interval (max time waiting for tx to finish).
        kTxModeAdvInterval      = 20,   // Adv interval during Tx Mode
        kAckFrameLength         = 5,
    };

    void SetState(State aState);
    void StartReceive(void);
    void StartRxModeAdv(void);
    void StartTxModeAdv(void);
    void InvokeRadioTxDone(otError aError);

    // Callbacks from platform
    void HandleConnected(Platform::Connection *aPlatConn);
    void HandleDisconnected(Platform::Connection *aPlatConn);

    static void HandleTimer(Timer &aTimer);
    void        HandleTimer(void);

    static const char *StateToString(State aState);

    Connection *mConn;
    TimerMilli  mTimer;
    State       mState;
    Mac::Frame *mTxFrame;
    uint8_t     mAdvDataBuffer[AdvData::kMaxSize];
};

} // namespace Peripheral
} // namespace Toble
} // namespace ot

#endif // OPENTHREAD_CONFIG_ENABLE_TOBLE && OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE

#endif // TOBLE_CONTROLLER_PERIPHERAL_HPP_
