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
 *   This file implements ToBLE Advertisement Data parsing and generation logic.
 */

#include "adv_data.hpp"

#include "common/code_utils.hpp"
#include "common/logging.hpp"

#if OPENTHREAD_CONFIG_ENABLE_TOBLE

namespace ot {
namespace Toble {

//---------------------------------------------------------------------------------------------------------------------
// AdvData::Struct

void AdvData::Struct::Populate(uint8_t aType)
{
    mLength = sizeof(mType);
    mType   = aType;
}

//---------------------------------------------------------------------------------------------------------------------
// AdvData::BleFlagsStruct

void AdvData::BleFlagsStruct::Populate(uint8_t aFlags)
{
    Struct::Populate(kType);
    mFlags = aFlags;
    mLength += sizeof(mFlags);
}

//---------------------------------------------------------------------------------------------------------------------
// AdvData::ServiceUuid16Struct

void AdvData::ServiceUuid16Struct::Populate(uint16_t aUuid)
{
    Struct::Populate(kType);
    mUuid16 = Encoding::LittleEndian::HostSwap16(aUuid);
    mLength += sizeof(mUuid16);
}

//---------------------------------------------------------------------------------------------------------------------
// AdvData::TobleStruct

bool AdvData::TobleStruct::IsValid() const
{
    return (mLength >= sizeof(this) - 1) && (mType == kType) && (GetUuid() == kUuid16Thread);
}

void AdvData::TobleStruct::Populate(uint8_t aVersion, uint8_t aOpCode)
{
    ServiceUuid16Struct::Populate(kUuid16Thread);
    mControlField = (aVersion << kVersionOffset) & kVersionMask;
    mControlField |= (aOpCode << kOpCodeOffset) & kOpCodeMask;
    mLength += sizeof(mControlField);
}

//---------------------------------------------------------------------------------------------------------------------
// AdvData::TobleDataStruct

otError AdvData::TobleDataStruct::Parse(Info &aInfo) const
{
    otError        error  = OT_ERROR_PARSE;
    uint8_t        minLen = sizeof(TobleDataStruct) - 1;
    const uint8_t *ptr;

    VerifyOrExit((mLength >= minLen) && TobleStruct::IsValid());
    VerifyOrExit((GetVersion() == kBeaconVersion) && (GetOpCode() == kOpCodeData));

    aInfo.mFramePending = (mFlags & kFlagFramePending);
    aInfo.mDataPolling  = (mFlags & kFlagDataPolling);
    aInfo.mL2capSupport = (mFlags & kFlagL2capSupport);
    aInfo.mPanId        = Encoding::LittleEndian::HostSwap16(mPanId);
    aInfo.mSrcShort     = Encoding::LittleEndian::HostSwap16(mSrcShort);

    for (uint8_t i = 0; i < sizeof(Mac::ExtAddress); i++)
    {
        aInfo.mSrcExtended.m8[i] = mSrcExtended[sizeof(Mac::ExtAddress) - 1 - i];
    }

    ptr = GetDestPtr();

    if (mFlags & kFlagHasDest)
    {
        if (mFlags & kFlagExtendedDest)
        {
            minLen += sizeof(Mac::ExtAddress);
            VerifyOrExit(mLength >= minLen);
            aInfo.mDest.SetExtended(ptr, /* aReverse */ true);
            ptr += sizeof(Mac::ExtAddress);
        }
        else
        {
            minLen += sizeof(Mac::ShortAddress);
            VerifyOrExit(mLength >= minLen);
            aInfo.mDest.SetShort(Encoding::LittleEndian::ReadUint16(ptr));
            ptr += sizeof(Mac::ShortAddress);
        }
    }
    else
    {
        aInfo.mDest.SetNone();
    }

    if (mFlags & kFlagL2capSupport)
    {
        minLen += sizeof(uint8_t);
        VerifyOrExit(mLength >= minLen);
        aInfo.mL2capPsm = *ptr;
        ptr++;
    }

    error = OT_ERROR_NONE;

exit:
    return error;
}

uint8_t AdvData::TobleDataStruct::CalculateSize(const Info &aInfo)
{
    uint8_t size = sizeof(TobleDataStruct);

    switch (aInfo.mDest.GetType())
    {
    case Mac::Address::kTypeNone:
        break;
    case Mac::Address::kTypeShort:
        size += sizeof(Mac::ShortAddress);
        break;
    case Mac::Address::kTypeExtended:
        size += sizeof(Mac::ExtAddress);
        break;
    }

    if (aInfo.mL2capSupport)
    {
        size += sizeof(uint8_t);
    }

    return size;
}

void AdvData::TobleDataStruct::Populate(const Info &aInfo)
{
    uint8_t *ptr;

    TobleStruct::Populate(kBeaconVersion, kOpCodeData);

    mFlags = 0;
    mFlags |= (aInfo.mFramePending) ? kFlagFramePending : 0;
    mFlags |= (aInfo.mDataPolling) ? kFlagDataPolling : 0;
    mFlags |= (aInfo.mL2capSupport) ? kFlagL2capSupport : 0;
    mLength += sizeof(mFlags);

    mPanId = Encoding::LittleEndian::HostSwap16(aInfo.mPanId);
    mLength += sizeof(Mac::PanId);

    mSrcShort = Encoding::LittleEndian::HostSwap16(aInfo.mSrcShort);
    mLength += sizeof(Mac::ShortAddress);

    for (uint8_t i = 0; i < sizeof(Mac::ExtAddress); i++)
    {
        mSrcExtended[i] = aInfo.mSrcExtended.m8[sizeof(Mac::ExtAddress) - 1 - i];
    }

    mLength += sizeof(Mac::ExtAddress);

    ptr = GetDestPtr();

    switch (aInfo.mDest.GetType())
    {
    case Mac::Address::kTypeNone:
        break;

    case Mac::Address::kTypeShort:
        mFlags |= kFlagHasDest;
        Encoding::LittleEndian::WriteUint16(aInfo.mDest.GetShort(), ptr);
        mLength += sizeof(Mac::ShortAddress);
        ptr += sizeof(Mac::ShortAddress);
        break;

    case Mac::Address::kTypeExtended:
        mFlags |= (kFlagHasDest | kFlagExtendedDest);

        for (uint8_t i = 0; i < sizeof(Mac::ExtAddress); i++)
        {
            ptr[i] = aInfo.mDest.GetExtended().m8[sizeof(Mac::ExtAddress) - 1 - i];
        }

        mLength += sizeof(Mac::ExtAddress);
        ptr += sizeof(Mac::ExtAddress);
        break;
    }

    if (aInfo.mL2capSupport)
    {
        *ptr = aInfo.mL2capPsm;
        ptr += sizeof(uint8_t);
        mLength += sizeof(uint8_t);
    }
}

//---------------------------------------------------------------------------------------------------------------------
// AdvData::Info

AdvData::Info::InfoString AdvData::Info::ToString(void) const
{
    InfoString str("pan-id:0x%04x, fp:%s, poll:%s, l2c:%s, src:0x%04x[%s], dst:%s", mPanId,
                   mFramePending ? "yes" : "no", mDataPolling ? "yes" : "no", mL2capSupport ? "yes" : "no", mSrcShort,
                   mSrcExtended.ToString().AsCString(), mDest.ToString().AsCString());

    if (mL2capSupport)
    {
        str.Append(", psm:%d", mL2capPsm);
    }

    return str;
}

//---------------------------------------------------------------------------------------------------------------------
// AdvData

AdvData::AdvData(uint8_t *aBuffer, uint16_t aLength)
    : mBuffer(aBuffer)
    , mLength(aLength)
    , mSize(aLength)
{
}

AdvData::AdvData(const uint8_t *aBuffer, uint16_t aLength)
    : mBuffer(const_cast<uint8_t *>(aBuffer))
    , mLength(aLength)
    , mSize(0)
{
}

otError AdvData::Parse(Info &aInfo) const
{
    otError error = OT_ERROR_PARSE;
    Struct *cur   = reinterpret_cast<Struct *>(mBuffer);
    Struct *end   = reinterpret_cast<Struct *>(mBuffer + mLength);

    while (cur < end)
    {
        VerifyOrExit(cur->IsValid());

        // We only search for the ToBLE Data structure
        if (static_cast<TobleDataStruct *>(cur)->Parse(aInfo) == OT_ERROR_NONE)
        {
            error = OT_ERROR_NONE;
            ExitNow();
        }

        cur = cur->GetNext();
    }

exit:
    return error;
}

otError AdvData::Populate(const Info &aInfo)
{
    otError error = OT_ERROR_NONE;
    Struct *cur   = reinterpret_cast<Struct *>(mBuffer);

    mLength = 0;

    // BLE Flags structure

    VerifyOrExit(mSize >= mLength + sizeof(BleFlagsStruct), error = OT_ERROR_NO_BUFS);
    static_cast<BleFlagsStruct *>(cur)->Populate(BleFlagsStruct::kFlags);

    mLength += cur->GetSize();
    cur = cur->GetNext();

    // ToBLE Service Data Structure

    VerifyOrExit(mSize >= mLength + TobleDataStruct::CalculateSize(aInfo), error = OT_ERROR_NO_BUFS);
    static_cast<TobleDataStruct *>(cur)->Populate(aInfo);

    mLength += cur->GetSize();
    cur = cur->GetNext();

exit:
    return error;
}

AdvData::InfoString AdvData::ToString(void) const
{
    InfoString str;

    SuccessOrExit(str.Append("len:%d [ ", mLength));

    for (uint8_t i = 0; i < mLength; i++)
    {
        SuccessOrExit(str.Append("%02X ", mBuffer[i]));
    }

    str.Append("]");

exit:
    return str;
}

} // namespace Toble
} // namespace ot

#endif // OPENTHREAD_CONFIG_ENABLE_TOBLE
