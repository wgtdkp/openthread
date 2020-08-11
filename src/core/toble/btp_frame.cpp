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
#include "common/logging.hpp"
#include "toble/btp_frame.hpp"

#if OPENTHREAD_CONFIG_TOBLE_ENABLE

namespace ot {
namespace Toble {

using ot::Encoding::LittleEndian::ReadUint16;
using ot::Encoding::LittleEndian::WriteUint16;
uint16_t DataFrame::GetDataLength(void) const
{
    return mLength;
}

uint16_t DataFrame::Init(const uint8_t *aBuffer,
                         uint16_t       aOffset,
                         uint16_t       aLength,
                         uint16_t       aMtu,
                         uint8_t        aSeqNum,
                         bool           aAck,
                         uint8_t        aAckNum)
{
    uint8_t *cur           = &mFrame[0];
    uint16_t segmentLength = 0;
    uint16_t segmentRemaining;

    mFlags = 0;

    if (aAck)
    {
        mFlags |= Frame::kAckFlag;
        otLogNoteBtp("  ack=%u", aAckNum);

        *cur++ = aAckNum;
    }

    *cur++ = aSeqNum;

    if (aBuffer != NULL)
    {
        if (aOffset == 0)
        {
            mFlags |= Frame::kBeginFlag;
            otLogNoteBtp("  start ------------>");
            otLogNoteBtp("  len=%u", aLength);

            WriteUint16(aLength, cur);
            cur += sizeof(uint16_t);
        }
        else
        {
            mFlags |= Frame::kContinueFlag;
            otLogNoteBtp("  off=%u", aOffset);
        }

        segmentRemaining = aMtu - (cur - mFrame + sizeof(mFlags));
        segmentLength    = aLength - aOffset;

        if (segmentLength > segmentRemaining)
        {
            segmentLength = segmentRemaining;
        }

        otLogNoteBtp("  segLen=%u", segmentLength);

        memcpy(cur, aBuffer + aOffset, segmentLength);
        cur += segmentLength;
        aOffset += segmentLength;

        if (aOffset >= aLength)
        {
            mFlags |= Frame::kEndFlag;
            otLogNoteBtp("  end -------------->");
        }
    }

    mLength = cur - mFrame + sizeof(mFlags);

    otLogNoteBtp("BtpFrameLength=%d", mLength);

    return segmentLength;
}

} // namespace Toble
} // namespace ot

#endif // OPENTHREAD_CONFIG_TOBLE_ENABLE
