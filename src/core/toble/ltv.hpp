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
 *   This file includes definitions for generating and processing BLE advertising data LTV (Length Type Value).
 */

#ifndef LTV_HPP_
#define LTV_HPP_

#include "openthread-core-config.h"

#include "common/encoding.hpp"
#include "common/tlvs.hpp"

namespace ot {
namespace Toble {

using ot::Encoding::BigEndian::HostSwap16;
using ot::Encoding::BigEndian::HostSwap32;

/**
 * This class implements LTV generation and parsing.
 *
 */
OT_TOOL_PACKED_BEGIN
class Ltv
{
public:
    typedef uint8_t Iterator; ///< Used to iterate through variables of Ltv Value field.

    enum
    {
        kIteratorInit = 0, ///< Initializer for Ltv::Iterator.
    };

    void Init(void) { mLength = 1; }

    /**
     * This method returns the Type value.
     *
     * @returns The Type value.
     *
     */
    uint8_t GetType(void) const { return mType; }

    /**
     * This method sets the Type value.
     *
     * @param[in]  aType  The Type value.
     *
     */
    void SetType(uint8_t aType) { mType = aType; }

    /**
     * This method returns the Length value.
     *
     * @note The Length value includes the length of the Type.
     *
     * @returns The Length value.
     *
     */
    uint8_t GetLength(void) const { return mLength; }

    /**
     * This method returns the length of Value.
     *
     * @returns The length of the Value.
     *
     */
    uint8_t GetValueLength(void) const { return mLength - sizeof(mType); }

    /**
     * This method returns the LTV's total size (number of bytes) including Length, Type, and Value fields.
     *
     * @returns The total size include Length, Type, and Value fields.
     *
     */
    uint32_t GetSize(void) const { return sizeof(mLength) + GetLength(); }

    /**
     * This method returns a pointer to the Value.
     *
     * @returns A pointer to the value.
     *
     */
    uint8_t *GetValue(void) { return reinterpret_cast<uint8_t *>(this) + sizeof(Ltv); }

    /**
     * This method returns a pointer to the Value.
     *
     * @returns A pointer to the value.
     *
     */
    const uint8_t *GetValue(void) const { return reinterpret_cast<const uint8_t *>(this) + sizeof(Ltv); }

    /**
     * This method returns a pointer to the next LTV.
     *
     * @returns A pointer to the next LTV.
     *
     */
    Ltv *GetNext(void) { return reinterpret_cast<Ltv *>(reinterpret_cast<uint8_t *>(this) + GetSize()); }

    /**
     * This method returns a pointer to the next LTV.
     *
     * @returns A pointer to the next LTV.
     *
     */
    const Ltv *GetNext(void) const
    {
        return reinterpret_cast<const Ltv *>(reinterpret_cast<const uint8_t *>(this) + GetSize());
    }

    /**
     * This method appends an `uint8_t` value to LTV.
     *
     * On success this method grows the Length by the size of the `uint8_t`.
     *
     * @param[in]  aMaxLength   The max value of the Length field.
     * @param[in]  aValue       The value (`uint8_t`).
     *
     * @retval OT_ERROR_NONE     Successfully appended the `uint8_t` value to the LTV.
     * @retval OT_ERROR_NO_BUFS  Insufficient available buffers to grow the LTV.
     *
     */
    otError AppendUint8(uint8_t aMaxLength, uint8_t aValue)
    {
        otError error = OT_ERROR_NONE;
        VerifyOrExit(GetLength() + sizeof(uint8_t) <= aMaxLength, error = OT_ERROR_NO_BUFS);

        *(GetValue() + GetValueLength()) = aValue;
        mLength += sizeof(uint8_t);

    exit:
        return error;
    }

    /**
     * This method appends an `uint16_t` value to LTV.
     *
     * On success this method grows the Length by the size of the `uint16_t`.
     *
     * @param[in]  aMaxLength   The max value of the Length field.
     * @param[in]  aValue       The value (`uint16_t`).
     *
     * @retval OT_ERROR_NONE     Successfully appended the `uint16_t` value to the LTV.
     * @retval OT_ERROR_NO_BUFS  Insufficient available buffers to grow the LTV.
     *
     */
    otError AppendUint16(uint8_t aMaxLength, uint16_t aValue)
    {
        otError error = OT_ERROR_NONE;
        VerifyOrExit(GetLength() + sizeof(uint16_t) <= aMaxLength, error = OT_ERROR_NO_BUFS);

        *reinterpret_cast<uint16_t *>(GetValue() + GetValueLength()) = ot::Encoding::LittleEndian::HostSwap16(aValue);
        mLength += sizeof(uint16_t);

    exit:
        return error;
    }

    /**
     * This method appends an `uint32_t` value to LTV.
     *
     * On success this method grows the Length by the size of the `uint32_t`.
     *
     * @param[in]  aMaxLength   The max value of the Length field.
     * @param[in]  aValue       The value (`uint32_t`).
     *
     * @retval OT_ERROR_NONE     Successfully appended the `uint32_t` value to the LTV.
     * @retval OT_ERROR_NO_BUFS  Insufficient available buffers to grow the LTV.
     *
     */
    otError AppendUint32(uint8_t aMaxLength, uint32_t aValue)
    {
        otError error = OT_ERROR_NONE;
        VerifyOrExit(GetLength() + sizeof(uint32_t) <= aMaxLength, error = OT_ERROR_NO_BUFS);

        *reinterpret_cast<uint32_t *>(GetValue() + GetValueLength()) = ot::Encoding::LittleEndian::HostSwap32(aValue);
        mLength += sizeof(uint32_t);

    exit:
        return error;
    }

    /**
     * This method appends an `uint64_t` value to LTV.
     *
     * On success this method grows the Length by the size of the `uint64_t`.
     *
     * @param[in]  aMaxLength   The max value of the Length field.
     * @param[in]  aValue       The value (`uint64_t`).
     *
     * @retval OT_ERROR_NONE     Successfully appended the `uint64_t` value to the LTV.
     * @retval OT_ERROR_NO_BUFS  Insufficient available buffers to grow the LTV.
     *
     */
    otError AppendUint64(uint8_t aMaxLength, uint64_t aValue)
    {
        otError error = OT_ERROR_NONE;
        VerifyOrExit(GetLength() + sizeof(uint64_t) <= aMaxLength, error = OT_ERROR_NO_BUFS);

        *reinterpret_cast<uint64_t *>(GetValue() + GetValueLength()) = ot::Encoding::LittleEndian::HostSwap64(aValue);
        mLength += sizeof(uint64_t);

    exit:
        return error;
    }

    /**
     * This method appends an `Mac::ShortAddress` value to LTV.
     *
     * On success this method grows the Length by the size of the `Mac::ShortAddress`.
     *
     * @param[in]  aMaxLength   The max value of the Length field.
     * @param[in]  aAddress     The MAC short address.
     *
     * @retval OT_ERROR_NONE     Successfully appended the MAC short address to the LTV.
     * @retval OT_ERROR_NO_BUFS  Insufficient available buffers to grow the LTV.
     *
     */
    otError AppendShortAddress(uint8_t aMaxLength, const Mac::ShortAddress &aAddress)
    {
        return AppendUint16(aMaxLength, static_cast<uint16_t>(aAddress));
    }

    /**
     * This method appends an `Mac::ExtAddress` value to LTV.
     *
     * On success this method grows the Length by the size of the `Mac::ExtAddress`.
     *
     * @param[in]  aMaxLength   The max value of the Length field.
     * @param[in]  aAddress     The MAC extended address.
     *
     * @retval OT_ERROR_NONE     Successfully appended the MAC extended address to the LTV.
     * @retval OT_ERROR_NO_BUFS  Insufficient available buffers to grow the LTV.
     *
     */
    otError AppendExtAddress(uint8_t aMaxLength, const Mac::ExtAddress &aAddress)
    {
        return AppendUint64(aMaxLength, *reinterpret_cast<const uint64_t *>(&aAddress.m8));
    }

    /**
     * This method appends an `Tlv` value to LTV.
     *
     * On success this method grows the Length by the size of the `Tlv`.
     *
     * @param[in]  aMaxLength   The max value of the Length field.
     * @param[in]  aTlv         The Tlv.
     *
     * @retval OT_ERROR_NONE     Successfully appended the TLV to the LTV.
     * @retval OT_ERROR_NO_BUFS  Insufficient available buffers to grow the LTV.
     *
     */
    otError AppendTlv(uint8_t aMaxLength, const Tlv &aTlv)
    {
        otError error = OT_ERROR_NONE;
        VerifyOrExit(GetLength() + aTlv.GetSize() <= aMaxLength, error = OT_ERROR_NO_BUFS);

        memcpy(GetValue() + GetValueLength(), &aTlv, aTlv.GetSize());
        mLength += aTlv.GetSize();

    exit:
        return error;
    }

    /**
     * This method reads an `uint8_t` value from LTV.
     *
     * On success this method grows the @p aIterator by the size of the `uint8_t`.
     *
     * @param[inout]   aIterator   A reference to the iterator.
     * @param[out]     aValue      A reference to an `uint8_t` value.
     *
     * @retval OT_ERROR_NONE    Successfully parsed the @p aValue.
     * @retval OT_ERROR_PARSE   The `uint8_t` value could not be parsed.
     *
     */
    otError GetUint8(Iterator &aIterator, uint8_t &aValue) const
    {
        otError error = OT_ERROR_NONE;

        VerifyOrExit(aIterator + sizeof(uint8_t) <= GetValueLength(), error = OT_ERROR_PARSE);

        aValue = *reinterpret_cast<const uint8_t *>(GetValue() + aIterator);
        aIterator += sizeof(uint8_t);

    exit:
        return error;
    }

    /**
     * This method reads an `uint16_t` value from LTV.
     *
     * On success this method grows the @p aIterator by the size of the `uint16_t`.
     *
     * @param[inout]   aIterator   A reference to the iterator.
     * @param[out]     aValue      A reference to an `uint16_t` value.
     *
     * @retval OT_ERROR_NONE    Successfully parsed the @p aValue.
     * @retval OT_ERROR_PARSE   The `uint16_t` value could not be parsed.
     *
     */
    otError GetUint16(Iterator &aIterator, uint16_t &aValue) const
    {
        otError error = OT_ERROR_NONE;

        VerifyOrExit(aIterator + sizeof(uint16_t) <= GetValueLength(), error = OT_ERROR_PARSE);

        aValue = *reinterpret_cast<const uint16_t *>(GetValue() + aIterator);
        aIterator += sizeof(uint16_t);

    exit:
        return error;
    }

    /**
     * This method reads an `uint32_t` value from LTV.
     *
     * On success this method grows the @p aIterator by the size of the `uint16_t`.
     *
     * @param[inout]   aIterator   A reference to the iterator.
     * @param[out]     aValue      A reference to an `uint32_t` value.
     *
     * @retval OT_ERROR_NONE    Successfully parsed the @p aValue.
     * @retval OT_ERROR_PARSE   The `uint32_t` value could not be parsed.
     *
     */
    otError GetUint32(Iterator &aIterator, uint32_t &aValue) const
    {
        otError error = OT_ERROR_NONE;

        VerifyOrExit(aIterator + sizeof(uint32_t) <= GetValueLength(), error = OT_ERROR_PARSE);

        aValue = *reinterpret_cast<const uint32_t *>(GetValue() + aIterator);
        aIterator += sizeof(uint32_t);

    exit:
        return error;
    }

    /**
     * This method reads an `uint64_t` value from LTV.
     *
     * On success this method grows the @p aIterator by the size of the `uint64_t`.
     *
     * @param[inout]   aIterator   A reference to the iterator.
     * @param[out]     aValue      A reference to an `uint64_t` value.
     *
     * @retval OT_ERROR_NONE    Successfully parsed the @p aValue.
     * @retval OT_ERROR_PARSE   The `uint64_t` value could not be parsed.
     *
     */
    otError GetUint64(Iterator &aIterator, uint64_t &aValue) const
    {
        otError error = OT_ERROR_NONE;

        VerifyOrExit(aIterator + sizeof(uint64_t) <= GetValueLength(), error = OT_ERROR_PARSE);

        aValue = *reinterpret_cast<const uint64_t *>(GetValue() + aIterator);
        aIterator += sizeof(uint64_t);

    exit:
        return error;
    }

    /**
     * This method reads a MAC short address from LTV.
     *
     * On success this method grows the @p aIterator by the size of the `Mac::ShortAddress`.
     *
     * @param[inout]   aIterator   A reference to the iterator.
     * @param[out]     aAddress    A reference to a MAC short address.
     *
     * @retval OT_ERROR_NONE    Successfully parsed the MAC short address.
     * @retval OT_ERROR_PARSE   The MAC short address could not be parsed.
     *
     */
    otError GetShortAddress(Iterator aIterator, Mac::ShortAddress &aAddress) const
    {
        return GetUint16(aIterator, aAddress);
    }

    /**
     * This method reads a MAC extended address from LTV.
     *
     * On success this method grows the @p aIterator by the size of the `Mac::ExtAddress`.
     *
     * @param[inout]   aIterator   A reference to the iterator.
     * @param[out]     aAddress    A reference to a MAC extended address.
     *
     * @retval OT_ERROR_NONE    Successfully parsed the MAC extended address.
     * @retval OT_ERROR_PARSE   The MAC extended address could not be parsed.
     *
     */
    otError GetExtAddress(Iterator aIterator, Mac::ExtAddress &aAddress) const
    {
        otError  error = OT_ERROR_NONE;
        uint64_t address;

        SuccessOrExit(error = GetUint64(aIterator, address));
        aAddress.Set(reinterpret_cast<const uint8_t *>(&address));
    exit:
        return error;
    }

    /**
     * This method reads a TLV from LTV.
     *
     * On success this method grows the @p aIterator by the size of the `TLV`.
     *
     * @param[inout]   aIterator   A reference to the iterator.
     * @param[out]     aTlv        A reference to a TLV.
     *
     * @retval OT_ERROR_NONE    Successfully parsed the TLV.
     * @retval OT_ERROR_PARSE   The TLV could not be parsed.
     *
     */
    otError GetTlv(Iterator aIterator, Tlv &aTlv) const
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

private:
    uint8_t mLength;
    uint8_t mType;
} OT_TOOL_PACKED_END;

} // namespace Toble
} // namespace ot

#endif // LTV_HPP_
