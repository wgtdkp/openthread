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
 *   This file contains the definition related to parsing/generation of BTP frame.
 */

#ifndef BTP_FRAME_HPP_
#define BTP_FRAME_HPP_

#include "openthread-core-config.h"

#include <openthread/platform/toolchain.h>

#include "common/encoding.hpp"
#include "common/string.hpp"

#if OPENTHREAD_CONFIG_TOBLE_ENABLE

namespace ot {
namespace Toble {

using ot::Encoding::LittleEndian::HostSwap16;
using ot::Encoding::LittleEndian::HostSwap32;

/**
 * This class implements BTP frame header generation and parsing.
 *
 */
OT_TOOL_PACKED_BEGIN
class Header
{
public:
    enum
    {
        kHandshakeFlag  = 1 << 6,
        kManagementFlag = 1 << 5,
        kAckFlag        = 1 << 3,
        kEndFlag        = 1 << 2,
        kContinueFlag   = 1 << 1,
        kBeginFlag      = 1 << 0,
    };

    /**
     * This method indicates whether the frame contains ACK fields.
     *
     * @retval TRUE  The frame contains ACK fields.
     * @retval FALSE The frame doesn't contain ACK fields
     *
     */
    bool IsAck(void) const { return (mFlags & kAckFlag) != 0; }

    /**
     * This method indicates whether the frame contains PDU payload.
     *
     * @retval TRUE  The frame contains PDU payload.
     * @retval FALSE The frame doesn't contain PDU payload.
     *
     */
    bool IsData(void) const { return (mFlags & (kBeginFlag | kContinueFlag | kEndFlag)) != 0; }

    /**
     * This method sets the flag.
     *
     * @param[in]  aFlag  The flag.
     *
     */
    bool GetFlag(uint8_t aFlag) const { return (mFlags & aFlag) != 0; }

    /**
     * This method indicates whether or not the flag is set.
     *
     * @retval TRUE   If the flag present.
     * @retval FALSE  If no flag present.
     *
     */
    void SetFlag(uint8_t aFlag) { mFlags |= aFlag; }

protected:
    void SetFlags(uint8_t aFlags) { mFlags = aFlags; }

    uint8_t mFlags;
} OT_TOOL_PACKED_END;

/**
 * This class implements BTP handshake request frame generation and parsing.
 *
 */
OT_TOOL_PACKED_BEGIN
class HandshakeRequest : public Header
{
public:
    /**
     * This constructor initializes the object.
     *
     */
    HandshakeRequest(uint16_t aMtu, uint8_t aWindowSize)
        : mOpcode(kOpcode)
        , mVersions(HostSwap32(kVersions))
        , mMtu(HostSwap16(aMtu))
        , mWindowSize(aWindowSize)
    {
        SetFlags(kFlags);
    }

    /**
     * This method returns the MTU.
     *
     * @returns the MTU.
     *
     */
    uint16_t GetMtu(void) const { return HostSwap16(mMtu); }

    /**
     * This method returns the window size.
     *
     * @returns the window size.
     *
     */
    uint8_t GetWindowSize(void) const { return mWindowSize; }

private:
    enum
    {
        kFlags    = 0x6e,
        kOpcode   = 0x6c,
        kVersions = 0x3 << 28,
    };

    uint8_t  mOpcode;
    uint32_t mVersions;
    uint16_t mMtu;
    uint8_t  mWindowSize;
} OT_TOOL_PACKED_END;

/**
 * This class implements BTP handshake response frame generation and parsing.
 *
 */
OT_TOOL_PACKED_BEGIN
class HandshakeResponse : public Header
{
public:
    /**
     * This constructor initializes the object.
     *
     */
    HandshakeResponse(uint16_t aSegmentSize, uint8_t aWindowSize)
        : mOpcode(kOpcode)
        , mVersion(kVersion)
        , mSegmentSize(HostSwap16(aSegmentSize))
        , mWindowSize(aWindowSize)
    {
        SetFlags(kFlags);
    }

    /**
     * This method returns the segment size.
     *
     * @returns the segment size.
     *
     */
    uint16_t GetSegmentSize(void) const { return HostSwap16(mSegmentSize); }

    /**
     * This method returns the window size.
     *
     * @returns the window size.
     *
     */
    uint8_t GetWindowSize(void) const { return mWindowSize; }

private:
    enum
    {
        kFlags   = 0x6e,
        kOpcode  = 0x6c,
        kVersion = 0x4,
    };

    uint8_t  mOpcode;
    uint8_t  mVersion;
    uint16_t mSegmentSize;
    uint8_t  mWindowSize;
} OT_TOOL_PACKED_END;

/**
 * This class implements BTP frame generation and parsing.
 *
 */
OT_TOOL_PACKED_BEGIN
class Frame : public Header
{
public:
    typedef uint16_t Iterator; ///< Used to iterate through bytes of the frame.

    enum
    {
        kInfoStringSize = 70, ///< Recommended buffer size to use with `ToString()`.
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

    /**
     * This method indicates whether the frame is valid.
     *
     * @retval TRUE  The frame is valid.
     * @retval FALSE The frame is not valid.
     *
     */
    bool IsValid(void) const { return ((mFlags & (kHandshakeFlag | kManagementFlag)) == 0) && (IsAck() || IsData()); }

    /**
     * This method initializes the frame.
     *
     * @param[inout]  aIterator  A reference to iterator. After the method returns, the iterator will
     *                           be updated to the length of the frame.
     *
     */
    void Init(Iterator &aIterator);

    /**
     * This method appends ACK fields to the frame.
     *
     * @param[inout]  aIterator  A reference to iterator. After the method returns, the iterator will
     *                           be updated to the length of the frame.
     * @param[in]     aAckNum    The ACK number.
     *
     */
    void AppendAck(Iterator &aIterator, uint8_t aAckNum);

    /**
     * This method appends sequence number to the frame.
     *
     * @param[inout]  aIterator  A reference to iterator. After the method returns, the iterator will
     *                           be updated to the length of the frame.
     * @param[in]     aSeqNum    Sequence number.
     *
     */
    void AppendSeqNum(Iterator &aIterator, uint8_t aSeqNum);

    /**
     * This method appends PDU payload to the frame.
     *
     * @param[inout]  aIterator      A reference to iterator. After the method returns, the iterator will
     *                               be updated to the length of the frame.
     * @param[in]     aBuffer        A pointer to a buffer contianing the toble frame.
     * @param[in]     aBufferOffset  Byte offset within the buffer to fill the frame.
     * @param[in]     aBufferLength  Length of buffer.
     * @param[in]     aMtu           The maximum transmit unit of the frame.
     * @param[out]    aPayloadLength Payload length.
     *
     */
    void AppendPayload(Iterator &     aIterator,
                       const uint8_t *aBuffer,
                       uint16_t       aBufferOffset,
                       uint16_t       aBufferLength,
                       uint16_t       aMtu,
                       uint16_t &     aPayloadLength);

private:
    uint8_t *GetFrameStart(void) { return mBuffer - sizeof(mFlags); }

    enum
    {
        kFrameSize = OPENTHREAD_CONFIG_TOBLE_BTP_MAX_SEGMENT_SIZE,
    };

    uint8_t mBuffer[kFrameSize - sizeof(mFlags)];
} OT_TOOL_PACKED_END;

} // namespace Toble
} // namespace ot

#endif // #if OPENTHREAD_CONFIG_TOBLE_ENABLE
#endif // BTP_FRAME_HPP_
