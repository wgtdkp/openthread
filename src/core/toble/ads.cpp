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
 *   This file implements BLE Advertisement Data Structure parsing and generation logic.
 */

#include "toble/ads.hpp"

#include "common/code_utils.hpp"

#if OPENTHREAD_CONFIG_TOBLE_ENABLE

namespace ot {
namespace Toble {
otError Ads::AppendUint8(uint8_t aMaxLength, uint8_t aValue)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(GetLength() + sizeof(uint8_t) <= aMaxLength, error = OT_ERROR_NO_BUFS);

    *(GetValue() + GetValueLength()) = aValue;
    mLength += sizeof(uint8_t);

exit:
    return error;
}

otError Ads::AppendUint16(uint8_t aMaxLength, uint16_t aValue)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(GetLength() + sizeof(uint16_t) <= aMaxLength, error = OT_ERROR_NO_BUFS);

    *reinterpret_cast<uint16_t *>(reinterpret_cast<void *>(GetValue() + GetValueLength())) = HostSwap16(aValue);
    mLength += sizeof(uint16_t);

exit:
    return error;
}

otError Ads::AppendUint32(uint8_t aMaxLength, uint32_t aValue)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(GetLength() + sizeof(uint32_t) <= aMaxLength, error = OT_ERROR_NO_BUFS);

    *reinterpret_cast<uint32_t *>(reinterpret_cast<void *>(GetValue() + GetValueLength())) = HostSwap32(aValue);
    mLength += sizeof(uint32_t);

exit:
    return error;
}

otError Ads::AppendExtAddress(uint8_t aMaxLength, const Mac::ExtAddress &aAddress)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(GetLength() + OT_EXT_ADDRESS_SIZE <= aMaxLength, error = OT_ERROR_NO_BUFS);

    aAddress.CopyTo(GetValue() + GetValueLength(), Mac::ExtAddress::kReverseByteOrder);
    mLength += OT_EXT_ADDRESS_SIZE;

exit:
    return error;
}

otError Ads::AppendTlv(uint8_t aMaxLength, const Tlv &aTlv)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(GetLength() + aTlv.GetSize() <= aMaxLength, error = OT_ERROR_NO_BUFS);

    memcpy(GetValue() + GetValueLength(), &aTlv, aTlv.GetSize());
    mLength += aTlv.GetSize();

exit:
    return error;
}

otError Ads::GetUint8(Iterator &aIterator, uint8_t &aValue) const
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(aIterator + sizeof(uint8_t) <= GetValueLength(), error = OT_ERROR_PARSE);

    aValue = *reinterpret_cast<const uint8_t *>(GetValue() + aIterator);
    aIterator += sizeof(uint8_t);

exit:
    return error;
}

otError Ads::GetUint16(Iterator &aIterator, uint16_t &aValue) const
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(aIterator + sizeof(uint16_t) <= GetValueLength(), error = OT_ERROR_PARSE);

    aValue = HostSwap16(*reinterpret_cast<const uint16_t *>(reinterpret_cast<const void *>(GetValue() + aIterator)));
    aIterator += sizeof(uint16_t);

exit:
    return error;
}

otError Ads::GetUint32(Iterator &aIterator, uint32_t &aValue) const
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(aIterator + sizeof(uint32_t) <= GetValueLength(), error = OT_ERROR_PARSE);

    aValue = HostSwap32(*reinterpret_cast<const uint32_t *>(reinterpret_cast<const void *>(GetValue() + aIterator)));
    aIterator += sizeof(uint32_t);

exit:
    return error;
}

otError Ads::GetExtAddress(Iterator &aIterator, Mac::ExtAddress &aAddress) const
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(aIterator + OT_EXT_ADDRESS_SIZE <= GetValueLength(), error = OT_ERROR_PARSE);
    aAddress.Set(GetValue() + aIterator, Mac::ExtAddress::kReverseByteOrder);
    aIterator += OT_EXT_ADDRESS_SIZE;

exit:
    return error;
}

otError Ads::GetTlv(Iterator &aIterator, Tlv &aTlv) const
{
    otError    error = OT_ERROR_NONE;
    const Tlv *tlv;

    VerifyOrExit(aIterator + sizeof(Tlv) <= GetValueLength(), error = OT_ERROR_PARSE);
    tlv = reinterpret_cast<const Tlv *>(GetValue() + aIterator);

    VerifyOrExit(aIterator + tlv->GetSize() <= GetValueLength(), error = OT_ERROR_PARSE);

    memcpy(&aTlv, tlv, tlv->GetSize());
    aIterator += tlv->GetSize();

exit:
    return error;
}

} // namespace Toble
} // namespace ot

#endif // OPENTHREAD_CONFIG_TOBLE_ENABLE
