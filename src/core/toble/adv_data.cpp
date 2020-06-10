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
#include "toble/ltv.hpp"

#if OPENTHREAD_CONFIG_TOBLE_ENABLE

namespace ot {
namespace Toble {

ConnectBeacon::Info::InfoString ConnectBeacon::Info::ToString(void) const
{
    InfoString str("PANID:0x%04x, FP:%d DP:%d DST:%d EXT:%d L2:%s, SRC16:0x%04x ", mPanId, mFramePending, mDataPending,
                   mDestAddress, mExtendedAddress, mL2capTransport, mSrcShort);

    if (mExtendedAddress)
    {
        SuccessOrExit(str.Append(", SRC64:%s", mSrcExtended.ToString().AsCString()));
    }

    if (mL2capTransport)
    {
        SuccessOrExit(str.Append(", PSM:%d", mL2capPsm));
    }

    if (mDestAddress)
    {
        SuccessOrExit(str.Append(", DST:%s", mDest.ToString().AsCString()));
    }

exit:
    return str;
}

otError ConnectBeacon::Populate(const Info &aInfo)
{
    otError      error = OT_ERROR_NONE;
    Ltv *        ltv   = reinterpret_cast<Ltv *>(mData);
    ControlField controlField;
    BitField     connectFlags;

    ltv->Init();
    ltv->SetType(kBleAdvDataTypeFlags);
    SuccessOrExit(error = ltv->AppendUint8(kBleFlagsLength, kBleFlagsUuid));

    mLength = ltv->GetSize();

    ltv = ltv->GetNext();
    ltv->Init();
    ltv->SetType(kBleAdvDataTypeServiceData);
    SuccessOrExit(error = ltv->AppendUint16(kBleServiceDataMaxLength, kBleServiceDataUuid));

    controlField.SetVersion(ControlField::kBeaconVersion);
    controlField.SetOpCode(ControlField::kOpCodeConnect);
    SuccessOrExit(error = ltv->AppendUint8(kBleServiceDataMaxLength, controlField.GetValue()));

    connectFlags.SetBit(kFramePendingRequestedOffset, aInfo.mFramePending);
    connectFlags.SetBit(kDataPendingOffset, aInfo.mDataPending);
    connectFlags.SetBit(kDestAddressOffset, aInfo.mDestAddress);
    connectFlags.SetBit(kExtendedAddressOffset, aInfo.mExtendedAddress);
    connectFlags.SetBit(kL2capTransportOffset, aInfo.mL2capTransport);

    SuccessOrExit(error = ltv->AppendUint8(kBleServiceDataMaxLength, connectFlags.GetValue()));
    SuccessOrExit(error = ltv->AppendUint16(kBleServiceDataMaxLength, aInfo.mPanId));
    SuccessOrExit(error = ltv->AppendShortAddress(kBleServiceDataMaxLength, aInfo.mSrcShort));

    if (aInfo.mExtendedAddress)
    {
        SuccessOrExit(error = ltv->AppendExtAddress(kBleServiceDataMaxLength, aInfo.mSrcExtended));
    }

    if (aInfo.mL2capTransport)
    {
        SuccessOrExit(error = ltv->AppendUint8(kBleServiceDataMaxLength, aInfo.mL2capPsm));
    }

    if (aInfo.mDestAddress)
    {
        if (aInfo.mExtendedAddress && (aInfo.mDest.GetType() == Mac::Address::kTypeExtended))
        {
            SuccessOrExit(error = ltv->AppendExtAddress(kBleServiceDataMaxLength, aInfo.mDest.GetExtended()));
        }
        else if (!aInfo.mExtendedAddress && (aInfo.mDest.GetType() == Mac::Address::kTypeShort))
        {
            SuccessOrExit(error = ltv->AppendShortAddress(kBleServiceDataMaxLength, aInfo.mDest.GetShort()));
        }
        else
        {
            ExitNow(error = OT_ERROR_INVALID_ARGS);
        }
    }

    mLength += ltv->GetSize();

exit:
    if (error != OT_ERROR_NONE)
    {
        mLength = 0;
    }

    return error;
}

otError ConnectBeacon::Parse(Info &aInfo) const
{
    otError       error    = OT_ERROR_NONE;
    const Ltv *   ltv      = reinterpret_cast<const Ltv *>(mData);
    Ltv::Iterator iterator = Ltv::kIteratorInit;
    ControlField  controlField;
    BitField      connectFlags;
    uint8_t       uint8Value;
    uint16_t      uint16Value;

    VerifyOrExit(ltv->GetType() == kBleAdvDataTypeFlags, error = OT_ERROR_PARSE);
    VerifyOrExit(ltv->GetLength() == kBleFlagsLength, error = OT_ERROR_PARSE);
    SuccessOrExit(error = ltv->GetUint8(iterator, uint8Value));
    VerifyOrExit(uint8Value == kBleFlagsUuid, error = OT_ERROR_PARSE);

    ltv      = ltv->GetNext();
    iterator = Ltv::kIteratorInit;

    VerifyOrExit(ltv->GetType() == kBleAdvDataTypeServiceData, error = OT_ERROR_PARSE);
    VerifyOrExit(ltv->GetLength() >= kBleServiceDataMinLength && ltv->GetLength() <= kBleServiceDataMaxLength,
                 error = OT_ERROR_PARSE);
    SuccessOrExit(error = ltv->GetUint16(iterator, uint16Value));
    VerifyOrExit(uint16Value == kBleServiceDataUuid, error = OT_ERROR_PARSE);

    SuccessOrExit(error = ltv->GetUint8(iterator, uint8Value));
    controlField.Init(uint8Value);
    VerifyOrExit(controlField.GetVersion() == ControlField::kBeaconVersion, error = OT_ERROR_PARSE);
    VerifyOrExit(controlField.GetOpCode() == ControlField::kOpCodeConnect, error = OT_ERROR_PARSE);

    SuccessOrExit(error = ltv->GetUint8(iterator, uint8Value));
    connectFlags.Init(uint8Value);

    aInfo.mFramePending    = connectFlags.GetBit(kFramePendingRequestedOffset);
    aInfo.mDataPending     = connectFlags.GetBit(kDataPendingOffset);
    aInfo.mDestAddress     = connectFlags.GetBit(kDestAddressOffset);
    aInfo.mExtendedAddress = connectFlags.GetBit(kExtendedAddressOffset);
    aInfo.mL2capTransport  = connectFlags.GetBit(kL2capTransportOffset);

    SuccessOrExit(error = ltv->GetUint16(iterator, aInfo.mPanId));
    SuccessOrExit(error = ltv->GetShortAddress(iterator, aInfo.mSrcShort));

    if (aInfo.mExtendedAddress)
    {
        SuccessOrExit(error = ltv->GetExtAddress(iterator, aInfo.mSrcExtended));
    }

    if (aInfo.mL2capTransport)
    {
        SuccessOrExit(error = ltv->GetUint8(iterator, aInfo.mL2capPsm));
    }

    if (aInfo.mDestAddress)
    {
        if (aInfo.mExtendedAddress)
        {
            Mac::ExtAddress address;
            SuccessOrExit(error = ltv->GetExtAddress(iterator, address));
            aInfo.mDest.SetExtended(address);
        }
        else
        {
            Mac::ShortAddress address;

            SuccessOrExit(error = ltv->GetShortAddress(iterator, address));
            aInfo.mDest.SetShort(address);
        }
    }

exit:
    return error;
}

DiscoveryBeacon::Info::InfoString DiscoveryBeacon::Info::ToString(void) const
{
    InfoString str("B:%d D:%d U:%d J:%d A:%d R:%d S:%d C:%d FP:%d L2:%d, Id:%016x ", mBorderAgentEnabled, mDtcEnabled,
                   mUnconfigured, mJoiningPermitted, mActiveRouter, mRouterCapable, mScanCapableRouter,
                   mConnectionRequested, mFramePendingRequested, mL2capTransport, mDiscoveryId);

    if (mL2capTransport)
    {
        SuccessOrExit(str.Append(", PSM:%d", mL2capPsm));
    }

    if (mUnconfigured)
    {
        SuccessOrExit(str.Append(", JoinerUdpPort:%d", mJoinerUdpPort.GetUdpPort()));
    }

    if (mBorderAgentEnabled)
    {
        SuccessOrExit(str.Append(", CommissionerUdpPort:%d", mCommissionerUdpPort.GetUdpPort()));
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
    else
    {
        SuccessOrExit(str.Append(", NetworkName:%s", mNetworkName.GetNetworkName().GetBuffer()));
    }

exit:
    return str;
}

otError DiscoveryBeacon::Populate(const Info &aInfo)
{
    otError      error = OT_ERROR_NONE;
    Ltv *        ltv   = reinterpret_cast<Ltv *>(mData);
    ControlField controlField;
    BitField     roleFlags;
    BitField     transportFlags;

    ltv->Init();
    ltv->SetType(kBleAdvDataTypeFlags);
    SuccessOrExit(error = ltv->AppendUint8(kBleFlagsLength, kBleFlagsUuid));

    mLength = ltv->GetSize();

    ltv = ltv->GetNext();
    ltv->Init();
    ltv->SetType(kBleAdvDataTypeServiceData);
    SuccessOrExit(error = ltv->AppendUint16(kBleServiceDataMaxLength, kBleServiceDataUuid));

    controlField.SetVersion(ControlField::kBeaconVersion);
    controlField.SetOpCode(ControlField::kOpCodeDiscovery);
    SuccessOrExit(error = ltv->AppendUint8(kBleServiceDataMaxLength, controlField.GetValue()));

    roleFlags.SetBit(kScanCapableRouterOffset, aInfo.mScanCapableRouter);
    roleFlags.SetBit(kRouterCapableOffset, aInfo.mRouterCapable);
    roleFlags.SetBit(kActiveRouterOffset, aInfo.mActiveRouter);
    roleFlags.SetBit(kJoiningPermittedOffset, aInfo.mJoiningPermitted);
    roleFlags.SetBit(kUnconfiguredOffset, aInfo.mUnconfigured);
    roleFlags.SetBit(kDtcEnabledOffset, aInfo.mDtcEnabled);
    roleFlags.SetBit(kBorderAgentEnabledOffset, aInfo.mBorderAgentEnabled);
    SuccessOrExit(error = ltv->AppendUint8(kBleServiceDataMaxLength, roleFlags.GetValue()));

    transportFlags.SetBit(kL2capTransportOffset, aInfo.mL2capTransport);
    transportFlags.SetBit(kFramePendingRequestedOffset, aInfo.mFramePendingRequested);
    transportFlags.SetBit(kConnectionRequestedOffset, aInfo.mConnectionRequested);
    SuccessOrExit(error = ltv->AppendUint8(kBleServiceDataMaxLength, transportFlags.GetValue()));

    SuccessOrExit(error = ltv->AppendUint64(kBleServiceDataMaxLength, aInfo.mDiscoveryId));

    if (aInfo.mL2capTransport)
    {
        SuccessOrExit(error = ltv->AppendUint8(kBleServiceDataMaxLength, aInfo.mL2capPsm));
    }

    if (aInfo.mUnconfigured)
    {
        SuccessOrExit(error = ltv->AppendTlv(kBleServiceDataMaxLength, aInfo.mJoinerUdpPort));
    }

    if (aInfo.mBorderAgentEnabled)
    {
        SuccessOrExit(error = ltv->AppendTlv(kBleServiceDataMaxLength, aInfo.mCommissionerUdpPort));
    }

    mLength += ltv->GetSize();

exit:
    return error;
}

otError DiscoveryBeacon::Parse(Info &aInfo) const
{
    otError       error    = OT_ERROR_NONE;
    const Ltv *   ltv      = reinterpret_cast<const Ltv *>(mData);
    Ltv::Iterator iterator = Ltv::kIteratorInit;
    ControlField  controlField;
    BitField      roleFlags;
    BitField      transportFlags;
    uint8_t       uint8Value;
    uint16_t      uint16Value;

    VerifyOrExit(ltv->GetType() == kBleAdvDataTypeFlags, error = OT_ERROR_PARSE);
    VerifyOrExit(ltv->GetLength() == kBleFlagsLength, error = OT_ERROR_PARSE);
    SuccessOrExit(error = ltv->GetUint8(iterator, uint8Value));
    VerifyOrExit(uint8Value == kBleFlagsUuid, error = OT_ERROR_PARSE);

    ltv      = ltv->GetNext();
    iterator = Ltv::kIteratorInit;

    VerifyOrExit(ltv->GetType() == kBleAdvDataTypeServiceData, error = OT_ERROR_PARSE);
    VerifyOrExit(ltv->GetLength() >= kBleServiceDataMinLength && ltv->GetLength() <= kBleServiceDataMaxLength,
                 error = OT_ERROR_PARSE);
    SuccessOrExit(error = ltv->GetUint16(iterator, uint16Value));
    VerifyOrExit(uint16Value == kBleServiceDataUuid, error = OT_ERROR_PARSE);

    SuccessOrExit(error = ltv->GetUint8(iterator, uint8Value));
    controlField.Init(uint8Value);
    VerifyOrExit(controlField.GetVersion() == ControlField::kBeaconVersion, error = OT_ERROR_PARSE);
    VerifyOrExit(controlField.GetOpCode() == ControlField::kOpCodeDiscovery, error = OT_ERROR_PARSE);

    SuccessOrExit(error = ltv->GetUint8(iterator, uint8Value));
    roleFlags.Init(uint8Value);
    aInfo.mScanCapableRouter  = roleFlags.GetBit(kScanCapableRouterOffset);
    aInfo.mRouterCapable      = roleFlags.GetBit(kRouterCapableOffset);
    aInfo.mActiveRouter       = roleFlags.GetBit(kActiveRouterOffset);
    aInfo.mJoiningPermitted   = roleFlags.GetBit(kJoiningPermittedOffset);
    aInfo.mUnconfigured       = roleFlags.GetBit(kUnconfiguredOffset);
    aInfo.mDtcEnabled         = roleFlags.GetBit(kDtcEnabledOffset);
    aInfo.mBorderAgentEnabled = roleFlags.GetBit(kBorderAgentEnabledOffset);

    SuccessOrExit(error = ltv->GetUint8(iterator, uint8Value));
    transportFlags.Init(uint8Value);
    aInfo.mL2capTransport        = transportFlags.GetBit(kL2capTransportOffset);
    aInfo.mFramePendingRequested = transportFlags.GetBit(kFramePendingRequestedOffset);
    aInfo.mConnectionRequested   = transportFlags.GetBit(kConnectionRequestedOffset);

    SuccessOrExit(error = ltv->GetUint64(iterator, aInfo.mDiscoveryId));

    if (aInfo.mL2capTransport)
    {
        SuccessOrExit(ltv->GetUint8(iterator, aInfo.mL2capPsm));
    }

    if (aInfo.mUnconfigured)
    {
        SuccessOrExit(error = ltv->GetTlv(iterator, aInfo.mJoinerUdpPort));
    }

    if (aInfo.mBorderAgentEnabled)
    {
        SuccessOrExit(error = ltv->GetTlv(iterator, aInfo.mCommissionerUdpPort));
    }

exit:
    return error;
}

otError DiscoveryScanResponse::Populate(const DiscoveryBeacon::Info &aInfo)
{
    otError error = OT_ERROR_NONE;
    Ltv *   ltv   = reinterpret_cast<Ltv *>(mData);

    ltv->Init();
    ltv->SetType(kBleAdvDataTypeServiceData);
    SuccessOrExit(error = ltv->AppendUint16(kBleServiceDataMaxLength, kBleServiceDataUuid));

    if (aInfo.mJoiningPermitted)
    {
        SuccessOrExit(error = ltv->AppendTlv(kBleServiceDataMaxLength, aInfo.mSteeringData));
    }
    else
    {
        SuccessOrExit(error = ltv->AppendTlv(kBleServiceDataMaxLength, aInfo.mNetworkName));
    }

exit:
    return error;
}

otError DiscoveryScanResponse::Parse(DiscoveryBeacon::Info &aInfo) const
{
    otError       error    = OT_ERROR_NONE;
    const Ltv *   ltv      = reinterpret_cast<const Ltv *>(mData);
    Ltv::Iterator iterator = Ltv::kIteratorInit;
    uint16_t      uint16Value;

    VerifyOrExit(ltv->GetType() == kBleAdvDataTypeServiceData, error = OT_ERROR_PARSE);
    VerifyOrExit(ltv->GetLength() >= kBleServiceDataMinLength && ltv->GetLength() <= kBleServiceDataMaxLength,
                 error = OT_ERROR_PARSE);
    SuccessOrExit(error = ltv->GetUint16(iterator, uint16Value));
    VerifyOrExit(uint16Value == kBleServiceDataUuid, error = OT_ERROR_PARSE);

    if (aInfo.mJoiningPermitted)
    {
        SuccessOrExit(error = ltv->GetTlv(iterator, aInfo.mSteeringData));
    }
    else
    {
        SuccessOrExit(error = ltv->GetTlv(iterator, aInfo.mNetworkName));
    }

exit:
    return error;
}

} // namespace Toble
} // namespace ot
#endif // OPENTHREAD_CONFIG_TOBLE_ENABLE
