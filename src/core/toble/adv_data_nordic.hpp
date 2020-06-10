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
 *   This file contains the definition related to parsing/generation of ToBLE Advertisement Data.
 */

#ifndef TOBLE_ADV_DATA_HPP_
#define TOBLE_ADV_DATA_HPP_

#include "openthread-core-config.h"

#include "common/encoding.hpp"
#include "common/string.hpp"
#include "mac/mac_frame.hpp"

#if OPENTHREAD_CONFIG_ENABLE_TOBLE

namespace ot {
namespace Toble {

class AdvData
{
public:
    enum
    {
        kMaxSize        = 32,
        kInfoStringSize = 120,
    };

    typedef String<kInfoStringSize> InfoString;

    struct Info
    {
        enum
        {
            kInfoStringSize = 100,
        };

        typedef String<kInfoStringSize> InfoString;

        InfoString ToString(void) const;

        bool              mFramePending;
        bool              mDataPolling;
        bool              mL2capSupport;
        Mac::PanId        mPanId;
        Mac::ShortAddress mSrcShort;
        Mac::ExtAddress   mSrcExtended;
        Mac::Address      mDest;     // Applicable only when mFramePending flag is `true`.
        uint8_t           mL2capPsm; // Applicable only when mL2capSupport flag is `true`.
    };

    AdvData(uint8_t *aBuffer, uint16_t aLength);
    AdvData(const uint8_t *aBuffer, uint16_t aLength);

    const uint8_t *GetData(void) const { return mBuffer; }
    uint16_t       GetLength(void) const { return mLength; }

    otError Parse(Info &aInfo) const;
    otError Populate(const Info &aInfo);

    InfoString ToString(void) const;

private:
    OT_TOOL_PACKED_BEGIN
    class Struct
    {
    public:
        bool    IsValid(void) const { return (mLength >= sizeof(mType)); }
        Struct *GetNext(void) { return reinterpret_cast<Struct *>(reinterpret_cast<uint8_t *>(this) + GetSize()); }
        uint8_t GetSize(void) { return mLength + sizeof(uint8_t); }

    protected:
        void Populate(uint8_t aType);

        uint8_t mLength;
        uint8_t mType;
    } OT_TOOL_PACKED_END;

    OT_TOOL_PACKED_BEGIN
    class BleFlagsStruct : public Struct
    {
    public:
        enum
        {
            kType  = 0x01,
            kFlags = 0x06,
        };

        uint8_t GetFlags(void) const { return mFlags; }
        void    Populate(uint8_t aFlags);

    private:
        uint8_t mFlags;
    } OT_TOOL_PACKED_END;

    OT_TOOL_PACKED_BEGIN
    class ServiceUuid16Struct : public Struct
    {
    protected:
        enum
        {
            kType = 0x16,
        };

        uint16_t GetUuid(void) const { return Encoding::LittleEndian::HostSwap16(mUuid16); }
        void     Populate(uint16_t aUuid);

    private:
        uint16_t mUuid16;
    } OT_TOOL_PACKED_END;

    OT_TOOL_PACKED_BEGIN
    class TobleStruct : public ServiceUuid16Struct
    {
    protected:
        enum
        {
            kBeaconVersion   = 3,
            kOpCodeDiscovery = 0,
            kOpCodeData      = 1,
        };

        bool    IsValid(void) const;
        uint8_t GetVersion(void) const { return (mControlField & kVersionMask) >> kVersionOffset; }
        uint8_t GetOpCode(void) const { return (mControlField & kOpCodeMask) >> kOpCodeOffset; }
        void    Populate(uint8_t aVersion, uint8_t aOpCode);

    private:
        enum
        {
            kUuid16Thread  = 0xfffb,
            kOpCodeOffset  = 0,
            kOpCodeMask    = 0x0f << kOpCodeOffset,
            kVersionOffset = 4,
            kVersionMask   = 0x0f << kVersionOffset,
        };

        uint8_t mControlField;
    } OT_TOOL_PACKED_END;

    OT_TOOL_PACKED_BEGIN
    class TobleDataStruct : public TobleStruct
    {
    public:
        static uint8_t CalculateSize(const Info &aInfo);

        void    Populate(const Info &aInfo);
        otError Parse(Info &aInfo) const;

    private:
        enum
        {
            kFlagFramePending = 1 << 0,
            kFlagDataPolling  = 1 << 1,
            kFlagL2capSupport = 1 << 2,
            kFlagHasDest      = 1 << 4,
            kFlagExtendedDest = 1 << 5,
        };

        const uint8_t *GetDestPtr(void) const
        {
            return reinterpret_cast<const uint8_t *>(this) + sizeof(TobleDataStruct);
        }
        uint8_t *GetDestPtr(void) { return reinterpret_cast<uint8_t *>(this) + sizeof(TobleDataStruct); }

        uint8_t  mFlags;
        uint16_t mPanId;
        uint16_t mSrcShort;
        uint8_t  mSrcExtended[OT_EXT_ADDRESS_SIZE];
        // Dest address (optional) is after `mSrcExtended (presence requires `kFlagHasDest`)
        // L2CAP PSM (optional) is after src/dest address (presence requires `kFlagL2capSupport).
    } OT_TOOL_PACKED_END;

    uint8_t *mBuffer; // Buffer containing advertisement data
    uint16_t mLength; // Length of data
    uint16_t mSize;   // Total/max buffer size
};

} // namespace Toble
} // namespace ot

#endif // OPENTHREAD_CONFIG_ENABLE_TOBLE

#endif // TOBLE_ADV_DATA_HPP_
