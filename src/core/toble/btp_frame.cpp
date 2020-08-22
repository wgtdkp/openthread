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
 *   This file implements ToBLE Advertisement Data parsing and generation logic.
 */

#include <string.h>

#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "common/logging.hpp"
#include "toble/btp_frame.hpp"

#if OPENTHREAD_CONFIG_TOBLE_ENABLE

namespace ot {
namespace Toble {

using ot::Encoding::LittleEndian::ReadUint16;
using ot::Encoding::LittleEndian::WriteUint16;

Frame::InfoString Frame::ToString(void) const
{
    const uint8_t *cur = mBuffer;
    InfoString     str("H:%d M:%d A:%d E:%d C:%d B:%d", GetFlag(kHandshakeFlag), GetFlag(kManagementFlag),
                   GetFlag(kAckFlag), GetFlag(kEndFlag), GetFlag(kContinueFlag), GetFlag(kBeginFlag));

    if (GetFlag(kAckFlag))
    {
        SuccessOrExit(str.Append(", AckNum:%d", *cur++));
    }

    SuccessOrExit(str.Append(", SeqNum:%d", *cur++));

    if (GetFlag(kBeginFlag))
    {
        SuccessOrExit(str.Append(", MsgLen:%d", ReadUint16(cur)));
    }

exit:
    return str;
}

void Frame::Init(Iterator &aIterator)
{
    SetFlags(0);
    aIterator = 1;
}

void Frame::AppendAck(Iterator &aIterator, uint8_t aAckNum)
{
    uint8_t *cur = GetFrameStart() + aIterator;

    OT_ASSERT(aIterator < (kFrameSize - 1));

    SetFlag(kAckFlag);
    *cur++ = aAckNum;

    aIterator = static_cast<Iterator>(cur - GetFrameStart());
}

void Frame::AppendSeqNum(Iterator &aIterator, uint8_t aSeqNum)
{
    uint8_t *cur = GetFrameStart() + aIterator;

    OT_ASSERT(aIterator < (kFrameSize - 1));

    *cur++ = aSeqNum;

    aIterator = static_cast<Iterator>(cur - GetFrameStart());
}

void Frame::AppendPayload(Iterator &     aIterator,
                          const uint8_t *aBuffer,
                          uint16_t       aBufferOffset,
                          uint16_t       aBufferLength,
                          uint16_t       aMtu,
                          uint16_t &     aPayloadLength)
{
    uint8_t *cur = GetFrameStart() + aIterator;
    uint16_t segmentRemaining;

    if (aBufferOffset == 0)
    {
        SetFlag(kBeginFlag);

        WriteUint16(aBufferLength, cur);
        cur += sizeof(uint16_t);
    }
    else
    {
        SetFlag(kContinueFlag);
    }

    segmentRemaining = aMtu - (cur - GetFrameStart());
    aPayloadLength   = aBufferLength - aBufferOffset;

    if (aPayloadLength > segmentRemaining)
    {
        aPayloadLength = segmentRemaining;
    }

    OT_ASSERT(aPayloadLength <= (kFrameSize - (cur - GetFrameStart())));

    memcpy(cur, aBuffer + aBufferOffset, aPayloadLength);
    cur += aPayloadLength;
    aBufferOffset += aPayloadLength;

    if (aBufferOffset >= aBufferLength)
    {
        SetFlag(kEndFlag);
    }

    aIterator = static_cast<Iterator>(cur - GetFrameStart());
}
} // namespace Toble
} // namespace ot

#endif // OPENTHREAD_CONFIG_TOBLE_ENABLE
