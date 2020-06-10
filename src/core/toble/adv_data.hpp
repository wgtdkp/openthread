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
 *   This file contains the definition related to parsing/generation of ToBLE Advertisement Data.
 */

#ifndef TOBLE_ADV_DATA_HPP_
#define TOBLE_ADV_DATA_HPP_

#include "openthread-core-config.h"

#include "common/encoding.hpp"
#include "common/string.hpp"
#include "mac/mac_frame.hpp"
#include "meshcop/meshcop_tlvs.hpp"

#if OPENTHREAD_CONFIG_TOBLE_ENABLE

namespace ot {
namespace Toble {

OT_TOOL_PACKED_BEGIN
class ControlField
{
public:
    enum
    {
        kBeaconVersion   = 3,
        kOpCodeDiscovery = 0,
        kOpCodeConnect   = 1,
    };

    ControlField(void)
        : mControlField(0)
    {
    }

    void Init(uint8_t aControlField) { mControlField = aControlField; }

    uint8_t GetVersion(void) const { return (mControlField & kVersionMask) >> kVersionOffset; }
    uint8_t GetOpCode(void) const { return (mControlField & kOpCodeMask) >> kOpCodeOffset; }
    void    SetVersion(uint8_t aVersion) { mControlField |= (aVersion << kVersionOffset) & kVersionMask; }
    void    SetOpCode(uint8_t aOpCode) { mControlField |= (aOpCode << kOpCodeOffset) & kOpCodeMask; }
    uint8_t GetValue(void) { return mControlField; }

private:
    enum
    {
        kOpCodeOffset  = 0,
        kOpCodeMask    = 0x0f << kOpCodeOffset,
        kVersionOffset = 4,
        kVersionMask   = 0x0f << kVersionOffset,
    };

    uint8_t mControlField;
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
class BitField
{
public:
    BitField(void)
        : mBitField(0)
    {
    }

    void Init(uint8_t aBitField) { mBitField = aBitField; }
    void SetBit(uint8_t aOffset, bool aValue) { mBitField |= static_cast<uint8_t>(aValue) << aOffset; }
    bool GetBit(uint8_t aOffset) { return (mBitField &= (1 << aOffset)) != 0; }

    uint8_t GetValue(void) const { return mBitField; }

private:
    uint8_t mBitField;
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
class Beacon
{
public:
    Beacon(void)
        : mLength(0)
    {
    }

    const uint8_t *GetData(void) const { return mData; }
    uint8_t        GetLength(void) const { return mLength; }

protected:
    enum
    {
        kBlevDataLength = 31,
    };

    enum
    {
        kBleAdvDataTypeFlags       = 0x01,
        kBleAdvDataTypeServiceData = 0x16,
        kBleFlagsUuid              = 0x06,
        kBleServiceDataUuid        = 0xfffb,
    };

    uint8_t mData[kBlevDataLength];
    uint8_t mLength;
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
class ConnectBeacon : public Beacon
{
public:
    struct Info
    {
        enum
        {
            kInfoStringSize = 100,
        };

        typedef String<kInfoStringSize> InfoString;

        InfoString ToString(void) const;

        bool              mL2capTransport;
        bool              mExtendedAddress;
        bool              mDestAddress;
        bool              mDataPending;
        bool              mFramePending;
        Mac::PanId        mPanId;
        Mac::ShortAddress mSrcShort;
        Mac::ExtAddress   mSrcExtended;
        Mac::Address      mDest;
        uint8_t           mL2capPsm;
    };

    otError Populate(const Info &aInfo);
    otError Parse(Info &aInfo) const;

private:
    enum
    {
        kFramePendingRequestedOffset = 0,
        kDataPendingOffset           = 1,
        kDestAddressOffset           = 4,
        kExtendedAddressOffset       = 5,
        kL2capTransportOffset        = 7,
    };

    enum
    {
        kBleFlagsLength          = 2,
        kBleServiceDataMinLength = 13,
        kBleServiceDataMaxLength = 25,
    };
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
class DiscoveryBeacon : public Beacon
{
public:
    struct Info
    {
        enum
        {
            kInfoStringSize = 100,
        };

        typedef String<kInfoStringSize> InfoString;

        InfoString ToString(void) const;

        bool     mBorderAgentEnabled;
        bool     mDtcEnabled;
        bool     mUnconfigured;
        bool     mJoiningPermitted;
        bool     mActiveRouter;
        bool     mRouterCapable;
        bool     mScanCapableRouter;
        bool     mConnectionRequested;
        bool     mFramePendingRequested;
        bool     mL2capTransport;
        uint64_t mDiscoveryId;
        uint8_t  mL2capPsm;

        MeshCoP::JoinerUdpPortTlv       mJoinerUdpPort;
        MeshCoP::CommissionerUdpPortTlv mCommissionerUdpPort;
        MeshCoP::NetworkNameTlv         mNetworkName;
        MeshCoP::SteeringDataTlv        mSteeringData;
    };

    otError Populate(const Info &aInfo);
    otError Parse(Info &aInfo) const;

private:
    enum
    {
        kScanCapableRouterOffset     = 1,
        kRouterCapableOffset         = 2,
        kActiveRouterOffset          = 3,
        kJoiningPermittedOffset      = 4,
        kUnconfiguredOffset          = 5,
        kDtcEnabledOffset            = 6,
        kBorderAgentEnabledOffset    = 7,
        kL2capTransportOffset        = 0,
        kFramePendingRequestedOffset = 6,
        kConnectionRequestedOffset   = 7,
    };

    enum
    {
        kBleFlagsLength          = 2,
        kBleServiceDataMinLength = 13,
        kBleServiceDataMaxLength = 25,
    };
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
class DiscoveryScanResponse : public Beacon
{
public:
    otError Populate(const DiscoveryBeacon::Info &aInfo);
    otError Parse(DiscoveryBeacon::Info &aInfo) const;

private:
    enum
    {
        kBleServiceDataMinLength = 11,
        kBleServiceDataMaxLength = 19,
    };
} OT_TOOL_PACKED_END;

} // namespace Toble
} // namespace ot

#endif // OPENTHREAD_CONFIG_TOBLE_ENABLE

#endif // TOBLE_ADV_DATA_HPP_
