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
 *   This file contains implementation of ToBLE.
 */

#ifndef TOBLE_HPP_
#define TOBLE_HPP_

#include "openthread-core-config.h"

#include <openthread/toble.h>
#include <openthread/platform/radio.h>
#include <openthread/platform/toble.h>

#include "common/locator.hpp"
#include "mac/mac_frame.hpp"
#include "toble/btp.hpp"
#include "toble/conn_table.hpp"
#include "toble/controller_central.hpp"
#include "toble/controller_peripheral.hpp"
#include "toble/l2cap.hpp"
#include "toble/platform.hpp"
#include "toble/transport.hpp"

#if OPENTHREAD_CONFIG_ENABLE_TOBLE

namespace ot {

class Radio;

namespace Toble {

class Toble : public InstanceLocator
{
    friend class ot::Instance;
    friend class ot::Radio;
    friend class Platform::Callbacks;

public:
    Toble(Instance &aInstance);

#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE && OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
    otError         SetMode(otTobleLinkMode aMode);
    otTobleLinkMode GetMode(void) { return mMode; }

    bool IsCentral(void) const { return (mMode == OT_TOBLE_LINK_MODE_CENTRAL); }
#else
    bool IsCentral(void) const { return OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE; }
#endif

private:
    enum
    {
        kDefaulteceiveSensitivity = -100,
    };

    otInstance *GetInstancePtr(void) { return reinterpret_cast<otInstance *>(&GetInstance()); }

    // Radio APIs
    otError     Sleep(void);
    otError     Receive(uint8_t aChannel);
    otError     Transmit(Mac::Frame &aFrame);
    otError     Enable(void) { return (mEnabled = true, OT_ERROR_NONE); }
    otError     Disable(void) { return (mEnabled = false, OT_ERROR_NONE); }
    bool        IsEnabled(void) { return mEnabled; }
    otRadioCaps GetCaps(void) { return (otRadioCaps)(OT_RADIO_CAPS_ACK_TIMEOUT | OT_RADIO_CAPS_CSMA_BACKOFF); }
    const char *GetVersionString(void) { return otPlatRadioGetVersionString(GetInstancePtr()); }
    int8_t      GetReceiveSensitivity(void) { return kDefaulteceiveSensitivity; }
    void        GetIeeeEui64(Mac::ExtAddress &aIeeeEui64) { otPlatRadioGetIeeeEui64(GetInstancePtr(), aIeeeEui64.m8); }
    void        SetPanId(Mac::PanId) {}
    void        SetExtendedAddress(const Mac::ExtAddress &) {}
    void        SetShortAddress(Mac::ShortAddress) {}
    otError     GetTransmitPower(int8_t &) { return OT_ERROR_NOT_IMPLEMENTED; }
    otError     SetTransmitPower(int8_t) { return OT_ERROR_NOT_IMPLEMENTED; }
    bool        GetPromiscuous(void) { return false; }
    void        SetPromiscuous(bool) {}
    Mac::Frame &GetTransmitBuffer(void) { return mTxFrame; }
    int8_t      GetRssi(void) { return OT_RADIO_RSSI_INVALID; }
    otError     EnergyScan(uint8_t, uint16_t) { return OT_ERROR_NOT_IMPLEMENTED; }
    void        EnableSrcMatch(bool) {}
    otError     AddSrcMatchShortEntry(Mac::ShortAddress) { return OT_ERROR_NONE; }
    otError     AddSrcMatchExtEntry(const Mac::ExtAddress &) { return OT_ERROR_NONE; }
    otError     ClearSrcMatchShortEntry(Mac::ShortAddress) { return OT_ERROR_NONE; }
    otError     ClearSrcMatchExtEntry(const Mac::ExtAddress &) { return OT_ERROR_NONE; }
    void        ClearSrcMatchShortEntries(void) {}
    void        ClearSrcMatchExtEntries(void) {}
    uint32_t    GetSupportedChannelMask(void) { return OT_RADIO_2P4GHZ_OQPSK_CHANNEL_MASK; }
    uint32_t    GetPreferredChannelMask(void) { return OT_RADIO_2P4GHZ_OQPSK_CHANNEL_MASK; }

    // Callbacks from Toble::Platform
    void HandleConnected(Platform::Connection *aPlatConn);
    void HandleDisconnected(Platform::Connection *aPlatConn);

    bool            mEnabled;
    Mac::Frame      mTxFrame;
    uint8_t         mTxPsdu[OT_RADIO_TOBLE_FRAME_MAX_SIZE];
    Platform        mPlatform;
    ConnectionTable mConnTable;
    Transport       mTransport;

#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
    Central::Controller mCentralController;
#endif
#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
    Peripheral::Controller mPeripheralController;
#endif
#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE && OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
    otTobleLinkMode mMode;
#endif
};

} // namespace Toble
} // namespace ot

#endif // OPENTHREAD_CONFIG_ENABLE_TOBLE

#endif // TOBLE_CONTROLLER_HPP_
