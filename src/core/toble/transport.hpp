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
 *   This file contains the definition for ToBLE transport abstraction.
 */

#ifndef TOBLE_TRANSPORT_HPP_
#define TOBLE_TRANSPORT_HPP_

#include "openthread-core-config.h"

#include "common/locator.hpp"
#include "toble/btp.hpp"
#include "toble/l2cap.hpp"
#include "utils/wrap_stdint.h"

namespace ot {
namespace Toble {

#if OPENTHREAD_CONFIG_ENABLE_TOBLE

class Connection;

class Transport : public InstanceLocator
{
    friend class ot::Instance;

public:
    enum Type
    {
        kUnspecified, // Unspecified transport type (should be used ONLY on peripheral)
        kBtp,
#if OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
        kL2cap
#endif
    };

    explicit Transport(Instance &aInstance);

    void Start(Connection &aConn);
    void Stop(Connection &aConn);
    void Send(Connection &aConn, const uint8_t *aBuf, uint16_t aLength);

    // Callbacks from BTP or L2CAP transports
    void HandleSendDone(Connection &aConn, otError aError);
    void HandleReceiveDone(Connection &aConn, uint8_t *aFrame, uint16_t aLength, otError aError);

private:
    Btp mBtp;

#if OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
    L2cap mL2cap;
#endif
};

#endif // #if OPENTHREAD_CONFIG_ENABLE_TOBLE

} // namespace Toble
} // namespace ot

#endif // TOBLE_TRANSPORT_HPP_
