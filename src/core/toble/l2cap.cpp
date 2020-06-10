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
 *   This file contains the implementation for ToBLE L2CAP transport.
 */

#include "l2cap.hpp"

#include <openthread/platform/toble.h>

#include "common/code_utils.hpp"
#include "common/locator-getters.hpp"
#include "utils/wrap_string.h"

namespace ot {
namespace Toble {

#if OPENTHREAD_CONFIG_ENABLE_TOBLE && OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE

L2cap::L2cap(Instance &aInstance)
    : InstanceLocator(aInstance)
{
}

void L2cap::Start(Connection &aConn)
{
    OT_UNUSED_VARIABLE(aConn);
}

void L2cap::Stop(Connection &aConn)
{
    OT_UNUSED_VARIABLE(aConn);
}

void L2cap::Send(Connection &aConn, const uint8_t *aBuf, uint16_t aLength)
{
    OT_UNUSED_VARIABLE(aConn);
    OT_UNUSED_VARIABLE(aBuf);
    OT_UNUSED_VARIABLE(aLength);
}

// Use the following callbacks to notify next layers of tx/rx done:
//
// Get<Transport>().HandleSendDone()
// Get<Transport>().HandleReceiveDone()

#endif // #if OPENTHREAD_CONFIG_ENABLE_TOBLE & OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE

} // namespace Toble
} // namespace ot
