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
 *   This file implements the radio callbacks.
 */

#include "radio.hpp"

#include "common/code_utils.hpp"
#include "common/instance.hpp"
#include "common/locator-getters.hpp"
#include "common/logging.hpp"
#include "toble/toble.hpp"

namespace ot {

#if OPENTHREAD_CONFIG_TOBLE_ENABLE

#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
void Radio::SetLinkType(LinkType aLinkType)
{
    if (mLinkType != aLinkType)
    {
        otLogNoteMac("Radio::LinkType %s -> %s", LinkTypeToString(mLinkType), LinkTypeToString(aLinkType));
        mLinkType = aLinkType;
    }
}

const char *Radio::LinkTypeToString(LinkType aLinkType)
{
    const char *str = "Unknown";

    switch (aLinkType)
    {
    case kLinkTypeIeee802154:
        str = "IEEE802.15.4";
        break;
    case kLinkTypeToble:
        str = "ToBLE";
        break;
    }

    return str;
}
#endif // OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE

#if !OPENTHREAD_RADIO && !OPENTHREAD_CONFIG_LINK_RAW_ENABLE
otRadioCaps Radio::GetCaps(void)
{
    return
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
        Is154LinkEnabled() ? otPlatRadioGetCaps(GetInstancePtr()) :
#endif
                           Get<ot::Toble::Toble>().GetCaps();
}

const char *Radio::GetVersionString(void)
{
    return
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
        Is154LinkEnabled() ? otPlatRadioGetVersionString(GetInstancePtr()) :
#endif
                           Get<ot::Toble::Toble>().GetVersionString();
}

int8_t Radio::GetReceiveSensitivity(void)
{
    return
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
        Is154LinkEnabled() ? otPlatRadioGetReceiveSensitivity(GetInstancePtr()) :
#endif
                           Get<ot::Toble::Toble>().GetReceiveSensitivity();
}

otRadioState Radio::GetState(void)
{
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
    if (Is154LinkEnabled())
    {
        return otPlatRadioGetState(GetInstance());
    }
    else
#endif
    {
        return Get<ot::Toble::Toble>().GetState();
    }
}

void Radio::GetIeeeEui64(Mac::ExtAddress &aIeeeEui64)
{
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
    if (Is154LinkEnabled())
    {
        otPlatRadioGetIeeeEui64(GetInstancePtr(), aIeeeEui64.m8);
    }
    else
#endif
    {
        Get<ot::Toble::Toble>().GetIeeeEui64(aIeeeEui64);
    }
}

void Radio::SetPanId(Mac::PanId aPanId)
{
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
    if (Is154LinkEnabled())
    {
        otPlatRadioSetPanId(GetInstancePtr(), aPanId);
    }
    else
#endif
    {
        Get<ot::Toble::Toble>().SetPanId(aPanId);
    }
}

void Radio::SetExtendedAddress(const Mac::ExtAddress &aExtAddress)
{
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
    if (Is154LinkEnabled())
    {
        otPlatRadioSetExtendedAddress(GetInstancePtr(), &aExtAddress);
    }
    else
#endif
    {
        Get<ot::Toble::Toble>().SetExtendedAddress(aExtAddress);
    }
}

void Radio::SetShortAddress(Mac::ShortAddress aShortAddress)
{
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
    if (Is154LinkEnabled())
    {
        otPlatRadioSetShortAddress(GetInstancePtr(), aShortAddress);
    }
    else
#endif
    {
        Get<ot::Toble::Toble>().SetShortAddress(aShortAddress);
    }
}

void Radio::SetMacKey(uint8_t         aKeyIdMode,
                      uint8_t         aKeyId,
                      const Mac::Key &aPrevKey,
                      const Mac::Key &aCurrKey,
                      const Mac::Key &aNextKey)
{
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
    Is154LinkEnabled() ? otPlatRadioSetMacKey(GetInstancePtr(), aKeyIdMode, aKeyId, &aPrevKey, &aCurrKey, &aNextKey) :
#endif
                       Get<ot::Toble::Toble>().SetMacKey(aKeyIdMode, aKeyId, aPrevKey, aCurrKey, aNextKey);
}

otError Radio::GetTransmitPower(int8_t &aPower)
{
    return
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
        Is154LinkEnabled() ? otPlatRadioGetTransmitPower(GetInstancePtr(), &aPower) :
#endif
                           Get<ot::Toble::Toble>().GetTransmitPower(aPower);
}

otError Radio::SetTransmitPower(int8_t aPower)
{
    return
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
        Is154LinkEnabled() ? otPlatRadioSetTransmitPower(GetInstancePtr(), aPower) :
#endif
                           Get<ot::Toble::Toble>().SetTransmitPower(aPower);
}

bool Radio::GetPromiscuous(void)
{
    return
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
        Is154LinkEnabled() ? otPlatRadioGetPromiscuous(GetInstancePtr()) :
#endif
                           Get<ot::Toble::Toble>().GetPromiscuous();
}

void Radio::SetPromiscuous(bool aEnable)
{
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
    if (Is154LinkEnabled())
    {
        otPlatRadioSetPromiscuous(GetInstancePtr(), aEnable);
    }
#endif
    {
        Get<ot::Toble::Toble>().SetPromiscuous(aEnable);
    }
}

otError Radio::Enable(void)
{
    return
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
        Is154LinkEnabled() ? otPlatRadioEnable(GetInstancePtr()) :
#endif
                           Get<ot::Toble::Toble>().Enable();
}

otError Radio::Disable(void)
{
    return
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
        Is154LinkEnabled() ? otPlatRadioDisable(GetInstancePtr()) :
#endif
                           Get<ot::Toble::Toble>().Disable();
}

bool Radio::IsEnabled(void)
{
    return
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
        Is154LinkEnabled() ? otPlatRadioIsEnabled(GetInstancePtr()) :
#endif
                           Get<ot::Toble::Toble>().IsEnabled();
}

otError Radio::Sleep(void)
{
    return
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
        Is154LinkEnabled() ? otPlatRadioSleep(GetInstancePtr()) :
#endif
                           Get<ot::Toble::Toble>().Sleep();
}

otError Radio::Receive(uint8_t aChannel)
{
    return
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
        Is154LinkEnabled() ? otPlatRadioReceive(GetInstancePtr(), aChannel) :
#endif
                           Get<ot::Toble::Toble>().Receive(aChannel);
}

Mac::TxFrame &Radio::GetTransmitBuffer(void)
{
    return
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
        Is154LinkEnabled() ? *static_cast<Mac::Frame *>(otPlatRadioGetTransmitBuffer(GetInstancePtr())) :
#endif
                           Get<ot::Toble::Toble>().GetTransmitBuffer();
}

otError Radio::Transmit(Mac::TxFrame &aFrame)
{
    return
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
        Is154LinkEnabled() ? otPlatRadioTransmit(GetInstancePtr(), &aFrame) :
#endif
                           Get<ot::Toble::Toble>().Transmit(aFrame);
}

int8_t Radio::GetRssi(void)
{
    return
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
        Is154LinkEnabled() ? otPlatRadioGetRssi(GetInstancePtr()) :
#endif
                           Get<ot::Toble::Toble>().GetRssi();
}

otError Radio::EnergyScan(uint8_t aScanChannel, uint16_t aScanDuration)
{
    return
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
        Is154LinkEnabled() ? otPlatRadioEnergyScan(GetInstancePtr(), aScanChannel, aScanDuration) :
#endif
                           Get<ot::Toble::Toble>().EnergyScan(aScanChannel, aScanDuration);
}

void Radio::EnableSrcMatch(bool aEnable)
{
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
    if (Is154LinkEnabled())
    {
        otPlatRadioEnableSrcMatch(GetInstancePtr(), aEnable);
    }
    else
#endif
    {
        Get<ot::Toble::Toble>().EnableSrcMatch(aEnable);
    }
}

otError Radio::AddSrcMatchShortEntry(Mac::ShortAddress aShortAddress)
{
    return
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
        Is154LinkEnabled() ? otPlatRadioAddSrcMatchShortEntry(GetInstancePtr(), aShortAddress) :
#endif
                           Get<ot::Toble::Toble>().AddSrcMatchShortEntry(aShortAddress);
}

otError Radio::AddSrcMatchExtEntry(const Mac::ExtAddress &aExtAddress)
{
    return
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
        Is154LinkEnabled() ? otPlatRadioAddSrcMatchExtEntry(GetInstancePtr(), &aExtAddress) :
#endif
                           Get<ot::Toble::Toble>().AddSrcMatchExtEntry(aExtAddress);
}
otError Radio::ClearSrcMatchShortEntry(Mac::ShortAddress aShortAddress)
{
    return
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
        Is154LinkEnabled() ? otPlatRadioClearSrcMatchShortEntry(GetInstancePtr(), aShortAddress) :
#endif
                           Get<ot::Toble::Toble>().ClearSrcMatchShortEntry(aShortAddress);
}

otError Radio::ClearSrcMatchExtEntry(const Mac::ExtAddress &aExtAddress)
{
    return
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
        Is154LinkEnabled() ? otPlatRadioClearSrcMatchExtEntry(GetInstancePtr(), &aExtAddress) :
#endif
                           Get<ot::Toble::Toble>().ClearSrcMatchExtEntry(aExtAddress);
}
void Radio::ClearSrcMatchShortEntries(void)
{
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
    if (Is154LinkEnabled())
    {
        otPlatRadioClearSrcMatchShortEntries(GetInstancePtr());
    }
    else
#endif
    {
        Get<ot::Toble::Toble>().ClearSrcMatchShortEntries();
    }
}

void Radio::ClearSrcMatchExtEntries(void)
{
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
    if (Is154LinkEnabled())
    {
        otPlatRadioClearSrcMatchExtEntries(GetInstancePtr());
    }
    else
#endif
    {
        Get<ot::Toble::Toble>().ClearSrcMatchExtEntries();
    }
}

uint32_t Radio::GetSupportedChannelMask(void)
{
    return
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
        Is154LinkEnabled() ? otPlatRadioGetSupportedChannelMask(GetInstancePtr()) :
#endif
                           Get<ot::Toble::Toble>().GetSupportedChannelMask();
}
uint32_t Radio::GetPreferredChannelMask(void)
{
    return
#if OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE
        Is154LinkEnabled() ? otPlatRadioGetPreferredChannelMask(GetInstancePtr()) :
#endif
                           Get<ot::Toble::Toble>().GetPreferredChannelMask();
}
#else
otRadioCaps Radio::GetCaps(void)
{
    return (otRadioCaps)(OT_RADIO_CAPS_ACK_TIMEOUT | OT_RADIO_CAPS_CSMA_BACKOFF);
}

const char *Radio::GetVersionString(void)
{
    return "NULL";
}

int8_t Radio::GetReceiveSensitivity(void)
{
    return -100;
}

void Radio::GetIeeeEui64(Mac::ExtAddress &aIeeeEui64)
{
    OT_UNUSED_VARIABLE(aIeeeEui64);
}

void Radio::SetPanId(Mac::PanId aPanId)
{
    OT_UNUSED_VARIABLE(aPanId);
}

void Radio::SetExtendedAddress(const Mac::ExtAddress &aExtAddress)
{
    OT_UNUSED_VARIABLE(aExtAddress);
}

void Radio::SetShortAddress(Mac::ShortAddress aShortAddress)
{
    OT_UNUSED_VARIABLE(aShortAddress);
}

void Radio::SetMacKey(uint8_t         aKeyIdMode,
                      uint8_t         aKeyId,
                      const Mac::Key &aPrevKey,
                      const Mac::Key &aCurrKey,
                      const Mac::Key &aNextKey)
{
    OT_UNUSED_VARIABLE(aKeyIdMode);
    OT_UNUSED_VARIABLE(aKeyId);
    OT_UNUSED_VARIABLE(aPrevKey);
    OT_UNUSED_VARIABLE(aCurrKey);
    OT_UNUSED_VARIABLE(aNextKey);
}

otError Radio::GetTransmitPower(int8_t &aPower)
{
    OT_UNUSED_VARIABLE(aPower);
    return OT_ERROR_NONE;
}

otError Radio::SetTransmitPower(int8_t aPower)
{
    OT_UNUSED_VARIABLE(aPower);
    return OT_ERROR_NONE;
}

bool Radio::GetPromiscuous(void)
{
    return false;
}

void Radio::SetPromiscuous(bool aEnable)
{
    OT_UNUSED_VARIABLE(aEnable);
}

otError Radio::Enable(void)
{
    return OT_ERROR_NONE;
}

otError Radio::Disable(void)
{
    return OT_ERROR_NONE;
}

bool Radio::IsEnabled(void)
{
    return false;
}

otError Radio::Sleep(void)
{
    return OT_ERROR_NONE;
}

otError Radio::Receive(uint8_t aChannel)
{
    OT_UNUSED_VARIABLE(aChannel);
    return OT_ERROR_NONE;
}

Mac::TxFrame &Radio::GetTransmitBuffer(void)
{
    static Mac::TxFrame frame;
    return frame;
}

otError Radio::Transmit(Mac::TxFrame &aFrame)
{
    OT_UNUSED_VARIABLE(aFrame);
    return OT_ERROR_NONE;
}

int8_t Radio::GetRssi(void)
{
    return 127;
}

otError Radio::EnergyScan(uint8_t aScanChannel, uint16_t aScanDuration)
{
    OT_UNUSED_VARIABLE(aScanChannel);
    OT_UNUSED_VARIABLE(aScanDuration);
    return OT_ERROR_NONE;
}

void Radio::EnableSrcMatch(bool aEnable)
{
    OT_UNUSED_VARIABLE(aEnable);
}

otError Radio::AddSrcMatchShortEntry(Mac::ShortAddress aShortAddress)
{
    OT_UNUSED_VARIABLE(aShortAddress);
    return OT_ERROR_NONE;
}

otError Radio::AddSrcMatchExtEntry(const Mac::ExtAddress &aExtAddress)
{
    OT_UNUSED_VARIABLE(aExtAddress);
    return OT_ERROR_NONE;
}

otError Radio::ClearSrcMatchShortEntry(Mac::ShortAddress aShortAddress)
{
    OT_UNUSED_VARIABLE(aShortAddress);
    return OT_ERROR_NONE;
}

otError Radio::ClearSrcMatchExtEntry(const Mac::ExtAddress &aExtAddress)
{
    OT_UNUSED_VARIABLE(aExtAddress);
    return OT_ERROR_NONE;
}

void Radio::ClearSrcMatchShortEntries(void)
{
}

void Radio::ClearSrcMatchExtEntries(void)
{
}

uint32_t Radio::GetSupportedChannelMask(void)
{
    return 0x00;
}

uint32_t Radio::GetPreferredChannelMask(void)
{
    return 0x00;
}
#endif

#endif // OPENTHREAD_CONFIG_TOBLE_ENABLE

} // namespace ot
