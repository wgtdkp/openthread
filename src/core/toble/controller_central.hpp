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
 *   This file contains implementation of ToBLE controller for central.
 */

#ifndef TOBLE_CONTROLLER_CENTRAL_HPP_
#define TOBLE_CONTROLLER_CENTRAL_HPP_

#include "openthread-core-config.h"

#include <openthread/platform/toble.h>

#include "common/locator.hpp"
#include "common/timer.hpp"
#include "mac/mac_frame.hpp"
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

    otError Sleep(void);
    otError Receive(void);
    otError Transmit(Mac::TxFrame &aFrame);

    // Callbacks from `Transport`
    void HandleTransportSendDone(Connection &aConn, otError aError);
    void HandleTransportReceiveDone(Connection &aConn, uint8_t *aFrame, uint16_t aLength, otError aError);

private:
    enum State
    {
        kStateSleep,                 // Sleep (not scanning).
        kStateRxScanning,            // Scanning to rx.
        kStateRxConnecting,          // Establishing a connection to rx (`mRxConn`).
        kStateTxPending,             // Pending Tx, waiting for a mRxConn to be established or timeout to scan for tx.
        kStateTxScanning,            // Scanning to tx (looking for peer).
        kStateTxConnecting,          // Establishing a connection for tx (`mTxConn`).
        kStateTxSending,             // Sending a frame over `mTxConn`.
        kStateTxSendingRxConnecting, // Sending a frame over `mTxConn` while establishing a `mRxConn`.
    };

    enum
    {
        kRxScanInterval = 40, // Scan interval while in rx mode (msec, same as kConnectionInterval).
        kRxScanWindow   = 30, // Scan window while in rx mode (msec, larger than peripheral's kTxModeAdvInterval).

        kTxScanInterval = 40, // Scan interval while in tx mode (msec, same as kConnectionInterval).
        kTxScanWindow   = 30, // Scan window while in tx mode (msec, larger than peripheral's kRxModeAdvInterval).

        kRxWaitToConnectTimeout = 1600, // Wait time to establish a connection for rx.
        kTxWaitToConnectTimeout = 1600, // Wait time to establish a connection for tx.

        kTxTimeout = 1000, // Tx timeout interval (max time waiting for entire tx operation to finish.

        kRxDisconnectTimeout      = 5000, // Timeout to disconnect from an idle connected connection.
        kTxDisconnectTimeout      = 5000, // Timeout to disconnect from a connection in sending/tx state.
        kTxErrorDisconnectTimeout = 5,    // Timeout to disconnect after a tx error happens.

        kConnectionInterval     = 40, // The connection data interval (msec).
        kConnectionScanInterval = 40, // Scan interval when trying to establish a connection (msec, same as
                                      // kConnectionInterval).
        kConnectionScanWindow = 30,   // Scan window when trying to establish a connection (msec, larger than
                                      // peripheral's kTxModeAdvInterval).

        kAckFrameLength = 5,
    };

    void SetState(State aState);
    void StartRxScanning(void);
    void StartTransmit(void);
    void StartTxScanning(void);
    void InvokeRadioTxDone(otError aError);

    void UpdateConnTimer(void);

    void HandleAdv(Platform::AdvType aAdvType,
                   const Address &   aSource,
                   const uint8_t *   aData,
                   uint16_t          aLength,
                   int8_t            aRssi);

    void HandleConnected(Platform::ConnectionId aPlatConn);
    void HandleDisconnected(Platform::ConnectionId aPlatConn);

    static void HandleConnTimer(Timer &aTimer);
    void        HandleConnTimer(void);

    static void HandleTxTimer(Timer &aTimer);
    void        HandleTxTimer(void);

    static const char *StateToString(State aState);

    State         mState;
    Mac::TxFrame *mTxFrame;
    Mac::Address  mTxDest;
    Connection *  mTxConn; // Connection being used for tx.
    Connection *  mRxConn; // Connection we are connecting (while in `kStateRxConnecting` or `kStateTxPending`).
    TimerMilli    mConnTimer;
    TimerMilli    mTxTimer;
};

} // namespace Central
} // namespace Toble
} // namespace ot

#endif // OPENTHREAD_CONFIG_TOBLE_ENABLE && OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE

#endif // TOBLE_CONTROLLER_CENTRAL_HPP_
