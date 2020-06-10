/*
 *  Copyright (c) 2016, The OpenThread Authors.
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
 *   This file implements common methods for manipulating MLE TLVs.
 */

#include "tlvs.hpp"

#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "common/message.hpp"

namespace ot {

otError Ltv::GetOffset(const Message &aMessage, uint8_t aType, uint16_t &aOffset)
{
    return Find(aMessage, aType, &aOffset, NULL, NULL);
}

otError Ltv::GetValueOffset(const Message &aMessage, uint8_t aType, uint16_t &aValueOffset, uint16_t &aLength)
{
    otError  error;
    uint16_t offset;
    uint16_t size;
    bool     isExtendedLtv;

    SuccessOrExit(error = Find(aMessage, aType, &offset, &size, &isExtendedLtv));

    if (!isExtendedLtv)
    {
        aValueOffset = offset + sizeof(Ltv);
        aLength      = size - sizeof(Ltv);
    }
    else
    {
        aValueOffset = offset + sizeof(ExtendedLtv);
        aLength      = size - sizeof(ExtendedLtv);
    }

exit:
    return error;
}

otError Ltv::Find(const Message &aMessage, uint8_t aType, uint16_t *aOffset, uint16_t *aSize, bool *aIsExtendedLtv)
{
    otError  error        = OT_ERROR_NOT_FOUND;
    uint16_t offset       = aMessage.GetOffset();
    uint16_t remainingLen = aMessage.GetLength();
    Ltv      tlv;
    uint32_t size;

    VerifyOrExit(offset <= remainingLen, OT_NOOP);
    remainingLen -= offset;

    while (true)
    {
        VerifyOrExit(sizeof(Ltv) <= remainingLen, OT_NOOP);
        aMessage.Read(offset, sizeof(Ltv), &tlv);

        if (tlv.mLength != kExtendedLength)
        {
            size = tlv.GetSize();
        }
        else
        {
            ExtendedLtv extLtv;

            VerifyOrExit(sizeof(ExtendedLtv) <= remainingLen, OT_NOOP);
            aMessage.Read(offset, sizeof(ExtendedLtv), &extLtv);

            VerifyOrExit(extLtv.GetLength() <= (remainingLen - sizeof(ExtendedLtv)), OT_NOOP);
            size = extLtv.GetSize();
        }

        VerifyOrExit(size <= remainingLen, OT_NOOP);

        if (tlv.GetType() == aType)
        {
            if (aOffset != NULL)
            {
                *aOffset = offset;
            }

            if (aSize != NULL)
            {
                *aSize = static_cast<uint16_t>(size);
            }

            if (aIsExtendedLtv != NULL)
            {
                *aIsExtendedLtv = (tlv.mLength == kExtendedLength);
            }

            error = OT_ERROR_NONE;
            ExitNow();
        }

        offset += size;
        remainingLen -= size;
    }

exit:
    return error;
}

otError Ltv::ReadUint8Ltv(const Message &aMessage, uint8_t aType, uint8_t &aValue)
{
    otError  error = OT_ERROR_NONE;
    LtvUint8 tlv8;

    SuccessOrExit(error = Get(aMessage, aType, sizeof(tlv8), tlv8));
    VerifyOrExit(tlv8.IsValid(), error = OT_ERROR_PARSE);
    aValue = tlv8.GetUint8Value();

exit:
    return error;
}

otError Ltv::ReadUint16Ltv(const Message &aMessage, uint8_t aType, uint16_t &aValue)
{
    otError   error = OT_ERROR_NONE;
    LtvUint16 tlv16;

    SuccessOrExit(error = Get(aMessage, aType, sizeof(tlv16), tlv16));
    VerifyOrExit(tlv16.IsValid(), error = OT_ERROR_PARSE);
    aValue = tlv16.GetUint16Value();

exit:
    return error;
}

otError Ltv::ReadUint32Ltv(const Message &aMessage, uint8_t aType, uint32_t &aValue)
{
    otError   error = OT_ERROR_NONE;
    LtvUint32 tlv32;

    SuccessOrExit(error = Get(aMessage, aType, sizeof(tlv32), tlv32));
    VerifyOrExit(tlv32.IsValid(), error = OT_ERROR_PARSE);
    aValue = tlv32.GetUint32Value();

exit:
    return error;
}

otError Ltv::ReadLtv(const Message &aMessage, uint8_t aType, void *aValue, uint8_t aLength)
{
    otError  error;
    uint16_t offset;
    uint16_t length;

    SuccessOrExit(error = GetValueOffset(aMessage, aType, offset, length));
    VerifyOrExit(length >= aLength, error = OT_ERROR_PARSE);
    aMessage.Read(offset, aLength, static_cast<uint8_t *>(aValue));

exit:
    return error;
}

otError Ltv::AppendUint8Ltv(Message &aMessage, uint8_t aType, uint8_t aValue)
{
    LtvUint8 tlv8;

    tlv8.Init(aType);
    tlv8.SetUint8Value(aValue);

    return tlv8.AppendTo(aMessage);
}

otError Ltv::AppendUint16Ltv(Message &aMessage, uint8_t aType, uint16_t aValue)
{
    LtvUint16 tlv16;

    tlv16.Init(aType);
    tlv16.SetUint16Value(aValue);

    return tlv16.AppendTo(aMessage);
}

otError Ltv::AppendUint32Ltv(Message &aMessage, uint8_t aType, uint32_t aValue)
{
    LtvUint32 tlv32;

    tlv32.Init(aType);
    tlv32.SetUint32Value(aValue);

    return tlv32.AppendTo(aMessage);
}

otError Ltv::AppendLtv(Message &aMessage, uint8_t aType, const uint8_t *aValue, uint8_t aLength)
{
    otError error = OT_ERROR_NONE;
    Ltv     tlv;

    OT_ASSERT(aLength <= Ltv::kBaseLtvMaxLength);

    tlv.SetType(aType);
    tlv.SetLength(aLength);
    SuccessOrExit(error = aMessage.Append(&tlv, sizeof(tlv)));

    VerifyOrExit(aLength > 0, OT_NOOP);
    error = aMessage.Append(aValue, aLength);

exit:
    return error;
}

} // namespace ot
