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
 *   This file implements ToBLE Advertisement Data parsing and generation logic.
 */

#include "toble/adv_data.hpp"
#include "common/code_utils.hpp"
#include "toble/ads.hpp"

#if OPENTHREAD_CONFIG_TOBLE_ENABLE

namespace ot {
namespace Toble {

AdvData::Info::InfoString Advertisement::Info::ToString(void) const
{
    InfoString str("L2:%d State:%d BA:%d D:%d J:%d Role:%d, PANID:0x%04x, SRC16:0x%04x ", mL2capTransport, mLinkState,
                   mBorderAgentEnabled, mDtcEnabled, mJoiningPermitted, mTobleRole, mPanId, mSrcShort);

    SuccessOrExit(str.Append(", SRC64:%s", mSrcExtended.ToString().AsCString()));

    if (mL2capTransport)
    {
        SuccessOrExit(str.Append(", PSM:%d", mL2capPsm));
    }

    if (mLinkState == kTxReadyToShort || mLinkState == kTxReadyToExtended)
    {
        SuccessOrExit(str.Append(", DST:%s", mDest.ToString().AsCString()));
    }

    if (mJoiningPermitted)
    {
        const uint8_t *value = mSteeringData.GetValue();

        SuccessOrExit(str.Append(", SteeringData:"));
        for (uint8_t i = 0; i < mSteeringData.GetSteeringDataLength(); i++)
        {
            SuccessOrExit(str.Append("%02X ", value[i]));
        }
    }

    if (mBorderAgentEnabled)
    {
        SuccessOrExit(str.Append(", NetworkName:%s", mNetworkName.GetNetworkName().GetBuffer()));
    }

exit:
    return str;
}

otError Advertisement::Populate(const Info &aInfo)
{
    otError      error = OT_ERROR_NONE;
    Ads *        ads   = reinterpret_cast<Ads *>(mData);
    ControlField controlField;
    BitsField    connectionFlags;
    BitsField    capabilitiesFlags;

    ads->Init(kBleAdvDataTypeFlags);
    SuccessOrExit(error = ads->AppendUint8(kBleFlagsLength, kBleFlagsUuid));

    mLength = ads->GetSize();

    ads = ads->GetNext();
    ads->Init(kBleAdvDataTypeServiceData);
    SuccessOrExit(error = ads->AppendUint16(kBleServiceDataMaxLength, kBleServiceDataUuid));

    controlField.SetVersion(ControlField::kThreadVersion);
    controlField.SetOpCode(ControlField::kOpCodeAdvertisement);
    SuccessOrExit(error = ads->AppendUint8(kBleServiceDataMaxLength, controlField.GetValue()));

    connectionFlags.SetBit(kL2capTransportOffset, aInfo.mL2capTransport);
    connectionFlags.SetBits(kLinkStateOffset, kLinkStateMask, aInfo.mLinkState);
    SuccessOrExit(error = ads->AppendUint8(kBleServiceDataMaxLength, connectionFlags.GetValue()));

    capabilitiesFlags.SetBit(kBorderAgentEnabledOffset, aInfo.mBorderAgentEnabled);
    capabilitiesFlags.SetBit(kDtcEnabledOffset, aInfo.mDtcEnabled);
    capabilitiesFlags.SetBit(kJoiningPermittedOffset, aInfo.mJoiningPermitted);
    capabilitiesFlags.SetBits(kTobleRoleOffset, kTobleRoleMask, aInfo.mTobleRole);
    SuccessOrExit(error = ads->AppendUint8(kBleServiceDataMaxLength, capabilitiesFlags.GetValue()));

    SuccessOrExit(error = ads->AppendUint16(kBleServiceDataMaxLength, aInfo.mPanId));
    SuccessOrExit(error = ads->AppendShortAddress(kBleServiceDataMaxLength, aInfo.mSrcShort));
    SuccessOrExit(error = ads->AppendExtAddress(kBleServiceDataMaxLength, aInfo.mSrcExtended));

    if (aInfo.mL2capTransport)
    {
        SuccessOrExit(error = ads->AppendUint8(kBleServiceDataMaxLength, aInfo.mL2capPsm));
    }

    if (aInfo.mLinkState == kTxReadyToShort)
    {
        VerifyOrExit(aInfo.mDest.IsShort(), error = OT_ERROR_INVALID_ARGS);
        SuccessOrExit(error = ads->AppendShortAddress(kBleServiceDataMaxLength, aInfo.mDest.GetShort()));
    }
    else if (aInfo.mLinkState == kTxReadyToExtended)
    {
        VerifyOrExit(aInfo.mDest.IsExtended(), error = OT_ERROR_INVALID_ARGS);
        SuccessOrExit(error = ads->AppendExtAddress(kBleServiceDataMaxLength, aInfo.mDest.GetExtended()));
    }

    mLength += ads->GetSize();

exit:
    if (error != OT_ERROR_NONE)
    {
        mLength = 0;
    }

    return error;
}

otError Advertisement::Parse(Info &aInfo) const
{
    otError       error    = OT_ERROR_NONE;
    const Ads *   ads      = reinterpret_cast<const Ads *>(mData);
    Ads::Iterator iterator = Ads::kIteratorInit;
    ControlField  controlField;
    BitsField     connectionFlags;
    BitsField     capabilitiesFlags;
    uint8_t       uint8Value;
    uint16_t      uint16Value;

    VerifyOrExit(ads->GetType() == kBleAdvDataTypeFlags, error = OT_ERROR_PARSE);
    VerifyOrExit(ads->GetLength() == kBleFlagsLength, error = OT_ERROR_PARSE);
    SuccessOrExit(error = ads->GetUint8(iterator, uint8Value));
    VerifyOrExit(uint8Value == kBleFlagsUuid, error = OT_ERROR_PARSE);

    ads      = ads->GetNext();
    iterator = Ads::kIteratorInit;

    VerifyOrExit(ads->GetType() == kBleAdvDataTypeServiceData, error = OT_ERROR_PARSE);
    VerifyOrExit(ads->GetLength() >= kBleServiceDataMinLength && ads->GetLength() <= kBleServiceDataMaxLength,
                 error = OT_ERROR_PARSE);
    SuccessOrExit(error = ads->GetUint16(iterator, uint16Value));
    VerifyOrExit(uint16Value == kBleServiceDataUuid, error = OT_ERROR_PARSE);

    SuccessOrExit(error = ads->GetUint8(iterator, uint8Value));
    controlField.Init(uint8Value);
    VerifyOrExit(controlField.GetVersion() == ControlField::kThreadVersion, error = OT_ERROR_PARSE);
    VerifyOrExit(controlField.GetOpCode() == ControlField::kOpCodeAdvertisement, error = OT_ERROR_PARSE);

    SuccessOrExit(error = ads->GetUint8(iterator, uint8Value));
    connectionFlags.Init(uint8Value);
    aInfo.mL2capTransport = connectionFlags.GetBit(kL2capTransportOffset);
    aInfo.mLinkState      = static_cast<LinkState>(connectionFlags.GetBits(kLinkStateOffset, kLinkStateMask));

    SuccessOrExit(error = ads->GetUint8(iterator, uint8Value));

    capabilitiesFlags.Init(uint8Value);
    aInfo.mBorderAgentEnabled = capabilitiesFlags.GetBit(kBorderAgentEnabledOffset);
    aInfo.mDtcEnabled         = capabilitiesFlags.GetBit(kDtcEnabledOffset);
    aInfo.mJoiningPermitted   = capabilitiesFlags.GetBit(kJoiningPermittedOffset);
    aInfo.mTobleRole          = static_cast<TobleRole>(capabilitiesFlags.GetBits(kTobleRoleOffset, kTobleRoleMask));

    SuccessOrExit(error = ads->GetUint16(iterator, aInfo.mPanId));
    SuccessOrExit(error = ads->GetShortAddress(iterator, aInfo.mSrcShort));

    if (aInfo.mL2capTransport)
    {
        SuccessOrExit(error = ads->GetUint8(iterator, aInfo.mL2capPsm));
    }

    if (aInfo.mLinkState == kTxReadyToShort)
    {
        Mac::ShortAddress address;

        SuccessOrExit(error = ads->GetShortAddress(iterator, address));
        aInfo.mDest.SetShort(address);
    }
    else if (aInfo.mLinkState == kTxReadyToExtended)
    {
        Mac::ExtAddress address;

        SuccessOrExit(error = ads->GetExtAddress(iterator, address));
        aInfo.mDest.SetExtended(address);
    }

exit:
    return error;
}

otError ScanResponse::Populate(const Info &aInfo)
{
    otError      error = OT_ERROR_NONE;
    Ads *        ads   = reinterpret_cast<Ads *>(mData);
    ControlField controlField;
    BitsField    connectionFlags;
    BitsField    capabilitiesFlags;

    ads->Init(kBleAdvDataTypeServiceData);
    SuccessOrExit(error = ads->AppendUint16(kBleServiceDataMaxLength, kBleServiceDataUuid));

    controlField.SetVersion(ControlField::kThreadVersion);
    controlField.SetOpCode(ControlField::kOpCodeScanRespone);
    SuccessOrExit(error = ads->AppendUint8(kBleServiceDataMaxLength, controlField.GetValue()));

    if (aInfo.mBorderAgentEnabled)
    {
        SuccessOrExit(error = ads->AppendTlv(kBleServiceDataMaxLength, aInfo.mNetworkName));
    }

    if (aInfo.mJoiningPermitted)
    {
        SuccessOrExit(error = ads->AppendTlv(kBleServiceDataMaxLength, aInfo.mSteeringData));
    }

    mLength = ads->GetSize();

exit:
    if (error != OT_ERROR_NONE)
    {
        mLength = 0;
    }

    return error;
}

otError ScanResponse::Parse(Info &aInfo) const
{
    otError       error    = OT_ERROR_NONE;
    const Ads *   ads      = reinterpret_cast<const Ads *>(mData);
    Ads::Iterator iterator = Ads::kIteratorInit;
    ControlField  controlField;
    uint8_t       uint8Value;
    uint16_t      uint16Value;

    VerifyOrExit(ads->GetType() == kBleAdvDataTypeServiceData, error = OT_ERROR_PARSE);
    VerifyOrExit(ads->GetLength() >= kBleServiceDataMinLength && ads->GetLength() <= kBleServiceDataMaxLength,
                 error = OT_ERROR_PARSE);
    SuccessOrExit(error = ads->GetUint16(iterator, uint16Value));
    VerifyOrExit(uint16Value == kBleServiceDataUuid, error = OT_ERROR_PARSE);

    SuccessOrExit(error = ads->GetUint8(iterator, uint8Value));
    controlField.Init(uint8Value);
    VerifyOrExit(controlField.GetVersion() == ControlField::kThreadVersion, error = OT_ERROR_PARSE);
    VerifyOrExit(controlField.GetOpCode() == ControlField::kOpCodeScanRespone, error = OT_ERROR_PARSE);

    if (aInfo.mBorderAgentEnabled)
    {
        SuccessOrExit(error = ads->GetTlv(iterator, aInfo.mNetworkName));
    }

    if (aInfo.mJoiningPermitted)
    {
        SuccessOrExit(error = ads->GetTlv(iterator, aInfo.mSteeringData));
    }

exit:
    return error;
}
} // namespace Toble
} // namespace ot

#endif // OPENTHREAD_CONFIG_TOBLE_ENABLE
