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
 *   This file includes definitions for generating and processing BLE Advertising Data Structure (ADS).
 */

#ifndef ADS_HPP_
#define ADS_HPP_

#include "openthread-core-config.h"

#include "common/encoding.hpp"
#include "common/tlvs.hpp"
#include "mac/mac_types.hpp"

#if OPENTHREAD_CONFIG_TOBLE_ENABLE

namespace ot {
namespace Toble {

using ot::Encoding::BigEndian::HostSwap16;
using ot::Encoding::BigEndian::HostSwap32;

/**
 * This class implements ADS generation and parsing.
 *
 */
OT_TOOL_PACKED_BEGIN
class Ads
{
public:
    typedef uint8_t Iterator; ///< Used to iterate through variables of Ads Value field.

    enum
    {
        kIteratorInit = 0, ///< Initializer for Ads::Iterator.
    };

    /**
     * This method initializes the ADS.
     *
     * @param[in]  aType  The Type value.
     *
     */
    void Init(uint8_t aType)
    {
        mType   = aType;
        mLength = 1;
    }

    /**
     * This method returns the Type value.
     *
     * @returns The Type value.
     *
     */
    uint8_t GetType(void) const { return mType; }

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
     * This method returns the ADS's total size (number of bytes) including Length, Type, and Value fields.
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
    uint8_t *GetValue(void) { return reinterpret_cast<uint8_t *>(this) + sizeof(Ads); }

    /**
     * This method returns a pointer to the Value.
     *
     * @returns A pointer to the value.
     *
     */
    const uint8_t *GetValue(void) const { return reinterpret_cast<const uint8_t *>(this) + sizeof(Ads); }

    /**
     * This method returns a pointer to the next ADS.
     *
     * @returns A pointer to the next ADS.
     *
     */
    Ads *GetNext(void) { return reinterpret_cast<Ads *>(reinterpret_cast<uint8_t *>(this) + GetSize()); }

    /**
     * This method returns a pointer to the next ADS.
     *
     * @returns A pointer to the next ADS.
     *
     */
    const Ads *GetNext(void) const
    {
        return reinterpret_cast<const Ads *>(reinterpret_cast<const uint8_t *>(this) + GetSize());
    }

    /**
     * This method appends an `uint8_t` value to ADS.
     *
     * On success this method grows the Length by the size of the `uint8_t`.
     *
     * @param[in]  aMaxLength   The max value of the Length field.
     * @param[in]  aValue       The value (`uint8_t`).
     *
     * @retval OT_ERROR_NONE     Successfully appended the `uint8_t` value to the ADS.
     * @retval OT_ERROR_NO_BUFS  Insufficient available buffers to grow the ADS.
     *
     */
    otError AppendUint8(uint8_t aMaxLength, uint8_t aValue);

    /**
     * This method appends an `uint16_t` value to ADS.
     *
     * On success this method grows the Length by the size of the `uint16_t`.
     *
     * @param[in]  aMaxLength   The max value of the Length field.
     * @param[in]  aValue       The value (`uint16_t`).
     *
     * @retval OT_ERROR_NONE     Successfully appended the `uint16_t` value to the ADS.
     * @retval OT_ERROR_NO_BUFS  Insufficient available buffers to grow the ADS.
     *
     */
    otError AppendUint16(uint8_t aMaxLength, uint16_t aValue);

    /**
     * This method appends an `uint32_t` value to ADS.
     *
     * On success this method grows the Length by the size of the `uint32_t`.
     *
     * @param[in]  aMaxLength   The max value of the Length field.
     * @param[in]  aValue       The value (`uint32_t`).
     *
     * @retval OT_ERROR_NONE     Successfully appended the `uint32_t` value to the ADS.
     * @retval OT_ERROR_NO_BUFS  Insufficient available buffers to grow the ADS.
     *
     */
    otError AppendUint32(uint8_t aMaxLength, uint32_t aValue);

    /**
     * This method appends an `Mac::ShortAddress` value to ADS.
     *
     * On success this method grows the Length by the size of the `Mac::ShortAddress`.
     *
     * @param[in]  aMaxLength   The max value of the Length field.
     * @param[in]  aAddress     The MAC short address.
     *
     * @retval OT_ERROR_NONE     Successfully appended the MAC short address to the ADS.
     * @retval OT_ERROR_NO_BUFS  Insufficient available buffers to grow the ADS.
     *
     */
    otError AppendShortAddress(uint8_t aMaxLength, const Mac::ShortAddress &aAddress)
    {
        return AppendUint16(aMaxLength, static_cast<uint16_t>(aAddress));
    }

    /**
     * This method appends an `Mac::ExtAddress` value to ADS.
     *
     * On success this method grows the Length by the size of the `Mac::ExtAddress`.
     *
     * @param[in]  aMaxLength   The max value of the Length field.
     * @param[in]  aAddress     The MAC extended address.
     *
     * @retval OT_ERROR_NONE     Successfully appended the MAC extended address to the ADS.
     * @retval OT_ERROR_NO_BUFS  Insufficient available buffers to grow the ADS.
     *
     */
    otError AppendExtAddress(uint8_t aMaxLength, const Mac::ExtAddress &aAddress);

    /**
     * This method appends an `Tlv` value to ADS.
     *
     * On success this method grows the Length by the size of the `Tlv`.
     *
     * @param[in]  aMaxLength   The max value of the Length field.
     * @param[in]  aTlv         The Tlv.
     *
     * @retval OT_ERROR_NONE     Successfully appended the TLV to the ADS.
     * @retval OT_ERROR_NO_BUFS  Insufficient available buffers to grow the ADS.
     *
     */
    otError AppendTlv(uint8_t aMaxLength, const Tlv &aTlv);

    /**
     * This method reads an `uint8_t` value from ADS.
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
    otError GetUint8(Iterator &aIterator, uint8_t &aValue) const;

    /**
     * This method reads an `uint16_t` value from ADS.
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
    otError GetUint16(Iterator &aIterator, uint16_t &aValue) const;

    /**
     * This method reads an `uint32_t` value from ADS.
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
    otError GetUint32(Iterator &aIterator, uint32_t &aValue) const;

    /**
     * This method reads a MAC short address from ADS.
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
    otError GetShortAddress(Iterator &aIterator, Mac::ShortAddress &aAddress) const
    {
        return GetUint16(aIterator, aAddress);
    }

    /**
     * This method reads a MAC extended address from ADS.
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
    otError GetExtAddress(Iterator &aIterator, Mac::ExtAddress &aAddress) const;

    /**
     * This method reads a TLV from ADS.
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
    otError GetTlv(Iterator &aIterator, Tlv &aTlv) const;

private:
    uint8_t mLength;
    uint8_t mType;
} OT_TOOL_PACKED_END;

} // namespace Toble
} // namespace ot

#endif // #if OPENTHREAD_CONFIG_TOBLE_ENABLE
#endif // ADS_HPP_