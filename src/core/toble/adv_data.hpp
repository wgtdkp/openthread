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

/**
 * This class represents a 8 bits field.
 *
 */
OT_TOOL_PACKED_BEGIN
class BitsField
{
public:
    /**
     * This constructor initializes a bits field.
     *
     */
    BitsField(void)
        : mBitsField(0)
    {
    }

    /**
     * This method initializes the BitsField.
     *
     * @param[in]  aBitsField  The bits field value.
     *
     */
    void Init(uint8_t aBitsField) { mBitsField = aBitsField; }

    /**
     * This method sets the specified bit to the given value.
     *
     * @param[in]  aOffset  The bit offset in the bits field.
     * @param[in]  aValue   The value of the the bit.
     *
     */
    void SetBit(uint8_t aOffset, bool aValue) { mBitsField |= static_cast<uint8_t>(aValue) << aOffset; }

    /**
     * This method gets the value of the specified bit.
     *
     * @param[in]  aOffset  The bit offset in the bits field.
     *
     * @returns TRUE if the bit is a set, FALSE otherwise.
     *
     */
    bool GetBit(uint8_t aOffset) const { return (mBitsField & (1 << aOffset)) != 0; }

    /**
     * This method sets the specified bits to the given value.
     *
     * @param[in]  aOffset  The bit offset in the bits field.
     * @param[in]  aMask    The bits mask of the bits field.
     * @param[in]  aValue   The value.
     *
     */
    void SetBits(uint8_t aOffset, uint8_t aMask, uint8_t aValue)
    {
        mBitsField = (mBitsField & ~aMask) | ((aValue << aOffset) & aMask);
    }

    /**
     * This method gets the value of the specified bits.
     *
     * @param[in]  aOffset  The bit offset in the bits field.
     * @param[in]  aMask    The bits mask of the bits field.
     *
     * @returns The bits value.
     *
     */
    uint8_t GetBits(uint8_t aOffset, uint8_t aMask) const { return ((mBitsField & aMask) >> aOffset); }

    /**
     * This method gets the whole bits field value.
     *
     * @returns The bits field value.
     *
     */
    uint8_t GetValue(void) const { return mBitsField; }

private:
    uint8_t mBitsField;
} OT_TOOL_PACKED_END;

/**
 * This class represents a BLE advertisement control field.
 *
 */
OT_TOOL_PACKED_BEGIN
class ControlField : public BitsField
{
public:
    /**
     * This enumeration specifies the Operation Code.
     *
     */
    enum
    {
        kOpCodeAdvertisement = 0, ///< Toble Advertisement.
        kOpCodeScanRespone   = 1, ///< Toble Scan response.
    };

    /**
     * This enumeration specifies the Version of Thread protocol.
     *
     */
    enum
    {
        kThreadVersion = OT_THREAD_VERSION_1_2, ///< Version of Thread protocol.
    };

    /**
     * This constructor initializes an Control field.
     *
     */
    ControlField(void)
        : BitsField()
    {
    }

    /**
     * This method gets the Version value of control field.
     *
     * @returns The Version value.
     *
     */
    uint8_t GetVersion(void) const { return GetBits(kVersionOffset, kVersionMask); }

    /**
     * This method gets the Operation Code value of control field.
     *
     * @returns The Operation Code value.
     *
     */
    uint8_t GetOpCode(void) const { return GetBits(kOpCodeOffset, kOpCodeMask); }

    /**
     * This method sets the Version value of control field.
     *
     * @param[in]  aVersion  The Version value.
     *
     */
    void SetVersion(uint8_t aVersion) { SetBits(kVersionOffset, kVersionMask, aVersion); }

    /**
     * This method sets the Operation Code value of control field.
     *
     * @param[in]  aOpCode  The Operation Code value.
     *
     */
    void SetOpCode(uint8_t aOpCode) { SetBits(kOpCodeOffset, kOpCodeMask, aOpCode); }

private:
    enum
    {
        kOpCodeOffset  = 0,
        kOpCodeMask    = 0x0f << kOpCodeOffset,
        kVersionOffset = 4,
        kVersionMask   = 0x0f << kVersionOffset,
    };
} OT_TOOL_PACKED_END;

/**
 * This class represents a BLE advertising data.
 *
 */
OT_TOOL_PACKED_BEGIN
class AdvData
{
public:
    enum
    {
        kMaxSize = 31, ///< The maximum length of the advertising data.
    };

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

    typedef struct Info
    {
        enum
        {
            kInfoStringSize = 200, ///< Recommended buffer size to use with `ToString()`.
        };

        /**
         * This type defines the fixed-length `String` object returned from `ToString()`.
         *
         */
        typedef String<kInfoStringSize> InfoString;

        /**
         * This method converts the advertising info into a human-readable string.
         *
         * @returns  An `InfoString` object representing the advertising info.
         *
         */
        InfoString ToString(void) const;

        bool      mL2capTransport;     ///< Indicates the device supports the optional L2CAP transport mode.
        bool      mJoiningPermitted;   ///< Indicates that if joining is permitted.
        bool      mDtcEnabled;         ///< Indicates that DTC commands are enabled.
        bool      mBorderAgentEnabled; ///< Indicates that the Border Agent is enabled.
        LinkState mLinkState;          ///< Indicates the radio link state of ToBLE Peripheral.
        TobleRole mTobleRole;          ///< Indicates the role of the BLE Peripheral sending the ToBLE Advertisement.

        uint8_t           mL2capPsm;    ///< The dynamic LE_PSM channel ID.
        Mac::PanId        mPanId;       ///< The destination PANID of the network.
        Mac::ShortAddress mSrcShort;    ///< The short source address of the peripheral device.
        Mac::ExtAddress   mSrcExtended; ///< The extended source address of the peripheral device.
        Mac::Address      mDest;        ///< The destination address of the central device.

        MeshCoP::NetworkNameTlv  mNetworkName;  ///< Network Name TLV.
        MeshCoP::SteeringDataTlv mSteeringData; ///< Steering Data TLV.
    } Info;

    enum
    {
        kStringSize = 120, ///< Recommended buffer size to use with `ToString()`.
    };

    typedef String<kStringSize> HexString;

    HexString ToString(void) const
    {
        HexString str;
        uint8_t   i;

        for (i = 0; i < mLength; i++)
        {
            SuccessOrExit(str.Append("%02X", mData[i]));
        }

    exit:
        return str;
    }

    /**
     * This constructor initializes the advertising data.
     *
     * @param[in]  aData    A pointer to a buffer containing advertising data.
     * @param[in]  aLength  The number of bytes in @p aData.
     *
     */
    AdvData(uint8_t *aData, uint8_t aLength)
        : mData(aData)
        , mLength(aLength)
    {
    }

    /**
     * This method returns a pointer to the first data byte of the advertising data.
     *
     * @returns A pointer to the first data byte.
     *
     */
    const uint8_t *GetData(void) const { return mData; }

    /**
     * This method returns the Length of advertising data.
     *
     * @returns The length of advertising data.
     *
     */
    uint8_t GetLength(void) const { return mLength; }

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

/**
 * This class represents a ToBLE advertisement.
 *
 */
OT_TOOL_PACKED_BEGIN
class Advertisement : public AdvData
{
public:
    /**
     * This constructor initializes the ToBle Advertisement.
     *
     * @param[in]  aData    A pointer to a buffer containing Advertisement.
     * @param[in]  aLength  The number of bytes in @p aData.
     *
     */
    Advertisement(uint8_t *aData, uint8_t aLength)
        : AdvData(aData, aLength)
    {
    }

    /**
     * This method populates the Advertisement based on the given advertisement information.
     *
     * @param[in]  aInfo  A reference to the advertisement infomantion.
     *
     * @retval OT_ERROR_NONE          Successfully populated the Advertisement.
     * @retval OT_ERROR_NO_BUFS       Insufficient available buffers.
     * @retval OT_ERROR_INVALID_ARGS  The infomation indicated by @p aInfo is invalid.
     *
     */
    otError Populate(const Info &aInfo);

    /**
     * This method parses the Advertisement to the advertisement information.
     *
     * @param[out]  aInfo  A reference to the advertisement infomantion.
     *
     * @retval OT_ERROR_NONE    Successfully parsed the Advertisement.
     * @retval OT_ERROR_PARSE   Failed to parse the Advertisement.
     *
     */
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

/**
 * This class represents a ToBLE Scan Sesponse.
 *
 */
OT_TOOL_PACKED_BEGIN
class ScanResponse : public AdvData
{
public:
    /**
     * This constructor initializes the ToBle Scan Response.
     *
     * @param[in]  aData    A pointer to a buffer containing Scan Response.
     * @param[in]  aLength  The number of bytes in @p aData.
     *
     */
    ScanResponse(uint8_t *aData, uint8_t aLength)
        : AdvData(aData, aLength)
    {
    }

    /**
     * This method populates the Scan Response based on the given advertisement information.
     *
     * @param[in]  aInfo  A reference to the advertisement infomantion.
     *
     * @retval OT_ERROR_NONE          Successfully populated the Scan Response.
     * @retval OT_ERROR_NO_BUFS       Insufficient available buffers.
     * @retval OT_ERROR_INVALID_ARGS  The infomation indicated by @p aInfo is invalid.
     *
     */
    otError Populate(const Info &aInfo);

    /**
     * This method parses the Scan Response to the advertisement information.
     *
     * @param[out]  aInfo  A reference to the advertisement infomantion.
     *
     * @retval OT_ERROR_NONE    Successfully parsed the Scan Response.
     * @retval OT_ERROR_PARSE   Failed to parse the Scan Response.
     *
     */
    otError Parse(Info &aInfo) const;

private:
    enum
    {
        kBleServiceDataMinLength = 4,
        kBleServiceDataMaxLength = 22,
    };
} OT_TOOL_PACKED_END;
} // namespace Toble
} // namespace ot

#endif // #if OPENTHREAD_CONFIG_TOBLE_ENABLE
#endif // TOBLE_ADV_DATA_HPP_
