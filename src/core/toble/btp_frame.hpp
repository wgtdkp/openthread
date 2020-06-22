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

#if OPENTHREAD_CONFIG_TOBLE_ENABLE

namespace ot {
namespace Toble {

using ot::Encoding::LittleEndian::HostSwap16;
using ot::Encoding::LittleEndian::HostSwap32;

OT_TOOL_PACKED_BEGIN
class Frame
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

    void Init(void) { mFlags = 0; }

    bool IsHandshake(void) const { return (mFlags & kHandshakeFlag) != 0; }

    bool IsAck(void) const { return (mFlags & kAckFlag) != 0; }

    bool IsBegin(void) const { return (mFlags & kBeginFlag) != 0; }

    bool IsEnd(void) const { return (mFlags & kEndFlag) != 0; }

protected:
    void SetFlags(uint8_t aFlags) { mFlags = aFlags; }

    uint8_t mFlags;
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
class HandshakeRequest : public Frame
{
public:
    void Init(uint16_t aMtu, uint8_t aWindowSize)
    {
        SetFlags(kFlags);
        mOpcode     = kOpcode;
        mVersions   = HostSwap32(kVersions);
        mMtu        = HostSwap16(aMtu);
        mWindowSize = aWindowSize;
    }

    uint16_t GetMtu(void) const { return HostSwap16(mMtu); }

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

OT_TOOL_PACKED_BEGIN
class HandshakeResponse : public Frame
{
public:
    void Init(uint16_t aSegmentSize, uint8_t aWindowSize)
    {
        SetFlags(kFlags);
        mOpcode      = kOpcode;
        mVersion     = kVersion;
        mSegmentSize = HostSwap16(aSegmentSize);
        mWindowSize  = aWindowSize;
    }

    uint16_t GetSegmentSize(void) const { return HostSwap16(mSegmentSize); }

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

} // namespace Toble
} // namespace ot

#endif // #if OPENTHREAD_CONFIG_TOBLE_ENABLE
#endif // BTP_FRAME_HPP_
