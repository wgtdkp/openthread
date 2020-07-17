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
 *   This file implements ToBLE.
 */

#include "toble/toble.hpp"

#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "common/encoding.hpp"
#include "common/instance.hpp"
#include "common/locator-getters.hpp"
#include "common/logging.hpp"
#include "common/random.hpp"
#include "mac/mac_frame.hpp"

#if OPENTHREAD_CONFIG_TOBLE_ENABLE

namespace ot {
namespace Toble {

Toble::Toble(ot::Instance &aInstance)
    : InstanceLocator(aInstance)
    , mEnabled(false)
    , mTxFrame()
    , mPlatform(aInstance)
    , mConnTable()
    , mTransport(aInstance)
#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
    , mCentralController(aInstance)
#endif
#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
    , mPeripheralController(aInstance)
#endif
#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE && OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
    , mMode(OT_TOBLE_LINK_MODE_PERIPHERAL)
#endif
{
    mPlatform.Init();
    mTxFrame.mPsdu = mTxPsdu;
}

otError Toble::Sleep(void)
{
    otError error = OT_ERROR_NONE;

    if (IsCentral())
    {
#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
        error = Get<Central::Controller>().Sleep();
#endif
    }
    else
    {
#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
        error = Get<Peripheral::Controller>().Sleep();
#endif
    }

    return error;
}

otError Toble::Receive(uint8_t aChannel)
{
    otError error = OT_ERROR_NONE;

    if (IsCentral())
    {
#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
        error = Get<Central::Controller>().Receive();
#endif
    }
    else
    {
#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
        error = Get<Peripheral::Controller>().Receive();
#endif
    }

    OT_UNUSED_VARIABLE(aChannel);

    return error;
}

otError Toble::Transmit(Mac::TxFrame &aFrame)
{
    otError error = OT_ERROR_NONE;
    OT_UNUSED_VARIABLE(aFrame);
    if (IsCentral())
    {
#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
        error = Get<Central::Controller>().Transmit(aFrame);
#endif
    }
    else
    {
#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
        error = Get<Peripheral::Controller>().Transmit(aFrame);
#endif
    }

    return error;
}

void Toble::HandleConnected(Platform::Connection *aPlatConn)
{
    OT_UNUSED_VARIABLE(aPlatConn);
    if (IsCentral())
    {
#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
        Get<Central::Controller>().HandleConnected(aPlatConn);
#endif
    }
    else
    {
#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
        Get<Peripheral::Controller>().HandleConnected(aPlatConn);
#endif
    }
}

void Toble::HandleDisconnected(Platform::Connection *aPlatConn)
{
    OT_UNUSED_VARIABLE(aPlatConn);
    if (IsCentral())
    {
#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
        Get<Central::Controller>().HandleDisconnected(aPlatConn);
#endif
    }
    else
    {
#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
        Get<Peripheral::Controller>().HandleDisconnected(aPlatConn);
#endif
    }
}

#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE && OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
otError Toble::SetMode(otTobleLinkMode aMode)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(Get<Mle::MleRouter>().GetRole() == Mle::kRoleDisabled, error = OT_ERROR_INVALID_STATE);
    mMode = aMode;

    otLogNoteBle("Toble mode set to %s", IsCentral() ? "central" : "peripheral");

exit:
    return error;
}
#endif

void Toble::PrintHex(const char *aName, const uint8_t *aData, uint8_t aLength)
{
    char    string[300] = {0};
    char *  start       = string;
    char *  end         = string + sizeof(string) - 1;
    uint8_t i;

    for (i = 0; i < aLength; i++)
    {
        start += snprintf(start, static_cast<uint32_t>(end - start), "%02x ", aData[i]);
    }

    otLogCritBle("%s: %s", aName, string);
}

void Toble::Test(void)
{
    uint8_t data[OT_TOBLE_ADV_DATA_MAX_LENGTH];
    uint8_t ScanRespData[OT_TOBLE_ADV_DATA_MAX_LENGTH];

    Advertisement advertisement(data, sizeof(data));
    ScanResponse  scanRespone(ScanRespData, sizeof(ScanRespData));
    AdvData::Info info;
    AdvData::Info parsedInfo;
    uint8_t       extSrcAddress[OT_EXT_ADDRESS_SIZE] = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18};
    uint8_t       extDstAddress[OT_EXT_ADDRESS_SIZE] = {0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28};
    Mac::NameData networkName("OpenThread", 11);

    info.mL2capTransport     = false;
    info.mJoiningPermitted   = false;
    info.mDtcEnabled         = true;
    info.mBorderAgentEnabled = true;
    info.mLinkState          = AdvData::kTxReadyToExtended;
    info.mTobleRole          = AdvData::kBedPeripheral;
    info.mL2capPsm           = 0;
    info.mPanId              = 0x1234;
    info.mSrcShort           = 0x2b01;
    info.mSrcExtended.Set(extSrcAddress);
    info.mDest.SetExtended(extDstAddress);

    info.mNetworkName.Init();
    info.mNetworkName.SetNetworkName(networkName);

    info.mSteeringData.Init();
    info.mSteeringData.Set();

    otLogCritBle("%s", info.ToString().AsCString());

    advertisement.Populate(info);
    PrintHex("ADV", advertisement.GetData(), advertisement.GetLength());

    scanRespone.Populate(info);
    PrintHex("RSP", scanRespone.GetData(), scanRespone.GetLength());

    advertisement.Parse(parsedInfo);
    scanRespone.Parse(parsedInfo);
    otLogCritBle("%s", parsedInfo.ToString().AsCString());
}

} // namespace Toble
} // namespace ot

#endif // #if OPENTHREAD_CONFIG_TOBLE_ENABLE
