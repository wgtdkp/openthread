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
class BitField
{
public:
    BitField(void)
        : mBitField(0)
    {
    }

    void Init(uint8_t aBitField) { mBitField = aBitField; }
    void SetBit(uint8_t aOffset, bool aValue) { mBitField |= static_cast<uint8_t>(aValue) << aOffset; }
    bool GetBit(uint8_t aOffset) const { return (mBitField & (1 << aOffset)) != 0; }
    void SetBits(uint8_t aOffset, uint8_t aValue, uint8_t aMask)
    {
        mBitField = (mBitField & ~aMask) | ((aValue << aOffset) & aMask);
    }
    uint8_t GetBits(uint8_t aOffset, uint8_t aMask) const { return ((mBitField & aMask) >> aOffset); }

    uint8_t GetValue(void) const { return mBitField; }

private:
    uint8_t mBitField;
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
class ControlField : public BitField
{
public:
    enum
    {
        kBeaconVersion       = 3, ///< Version of Thread protocol.
        kOpCodeAdvertisement = 0, ///< Toble Advertisement.
        kOpCodeScanRespone   = 1, ///< Toble Scan response.
    };

    ControlField(void)
        : BitField()
    {
    }

    uint8_t GetVersion(void) const { return GetBits(kVersionOffset, kVersionMask); }
    uint8_t GetOpCode(void) const { return GetBits(kOpCodeOffset, kOpCodeMask); }
    void    SetVersion(uint8_t aVersion) { SetBits(kVersionOffset, kVersionMask, aVersion); }
    void    SetOpCode(uint8_t aOpCode) { SetBits(kOpCodeOffset, kOpCodeMask, aOpCode); }

private:
    enum
    {
        kOpCodeOffset  = 0,
        kOpCodeMask    = 0x0f << kOpCodeOffset,
        kVersionOffset = 4,
        kVersionMask   = 0x0f << kVersionOffset,
    };
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
class AdvData
{
public:
    enum
    {
        kMaxSize = 31,
    };

    AdvData(uint8_t *aData, uint8_t aLength)
        : mData(aData)
        , mLength(aLength)
    {
    }

    const uint8_t *GetData(void) const { return mData; }
    uint8_t        GetLength(void) const { return mLength; }

protected:
    enum
    {
        kBleAdvDataTypeFlags       = 0x01,
        kBleAdvDataTypeServiceData = 0x16,
        kBleFlagsUuid              = 0x06,
        kBleServiceDataUuid        = 0xfffb,
    };

    uint8_t *mData;
    uint8_t  mLength;
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
class Advertisement : public AdvData
{
public:
    /**
     * This enumeration type represents the radio link state of ToBLE Peripheral.
     *
     */
    typedef enum
    {
        kRxReady           = 0, ///< Rx ready.
        kTxReadyToShort    = 1, ///< TX ready to RLOC16 destination address.
        kTxReadyToExtended = 2, ///< TX ready to Extended destination address.
    } LinkState;

    /**
     * This enumeration type represents the role of the BLE Peripheral sending the ToBLE Advertisement.
     *
     */
    typedef enum
    {
        kJoiner                 = 0, ///< Joiner (Unconfigured).
        kBedPeripheral          = 1, ///< BED-P.
        kInactiveBlerPeripheral = 2, ///< BLER-P (Inactive — REED, disconnected, or not taking children).
        kActiveBlerPeripheral   = 3, ///< BLER-P (Active).
    } TobleRole;

    struct Info
    {
        enum
        {
            kInfoStringSize = 100,
        };

        typedef String<kInfoStringSize> InfoString;

        InfoString ToString(void) const;

        bool      mL2capTransport;
        bool      mJoiningPermitted;
        bool      mDtcEnabled;
        bool      mBorderAgentEnabled;
        LinkState mLinkState;
        TobleRole mTobleRole;

        uint8_t           mL2capPsm;
        Mac::PanId        mPanId;
        Mac::ShortAddress mSrcShort;
        Mac::ExtAddress   mSrcExtended;
        Mac::Address      mDest;

        MeshCoP::NetworkNameTlv  mNetworkName;
        MeshCoP::SteeringDataTlv mSteeringData;
    };

    Advertisement(uint8_t *aData, uint8_t aLength)
        : AdvData(aData, aLength)
    {
    }

    otError Populate(const Info &aInfo);
    otError Parse(Info &aInfo) const;

private:
    enum
    {
        kLinkStateOffset          = 0,
        kL2capTransportOffset     = 7,
        kTobleRoleOffset          = 0,
        kJoiningPermittedOffset   = 5,
        kDtcEnabledOffset         = 6,
        kBorderAgentEnabledOffset = 7,
        kLinkStateMask            = 0x03 << kLinkStateOffset,
        kTobleRoleMask            = 0x03 << kTobleRoleOffset,
    };

    enum
    {
        kBleFlagsLength          = 2,
        kBleServiceDataMinLength = 18,
        kBleServiceDataMaxLength = 27,
    };
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
class ScanResponse : public AdvData
{
public:
    ScanResponse(uint8_t *aData, uint8_t aLength)
        : AdvData(aData, aLength)
    {
    }

    otError Populate(const Advertisement::Info &aInfo);
    otError Parse(Advertisement::Info &aInfo) const;

private:
    enum
    {
        kBleServiceDataMinLength = 4,
        kBleServiceDataMaxLength = 22,
    };
} OT_TOOL_PACKED_END;
} // namespace Toble
} // namespace ot

#endif // OPENTHREAD_CONFIG_TOBLE_ENABLE

#endif // TOBLE_ADV_DATA_HPP_
