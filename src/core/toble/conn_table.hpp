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
 *   This file contains definition of ToBLE connection table.
 */

#ifndef TOBLE_CONNECTION_TABLE_HPP_
#define TOBLE_CONNECTION_TABLE_HPP_

#include "openthread-core-config.h"

#include "mac/mac_frame.hpp"
#include "toble/platform.hpp"
#include "toble/transport.hpp"

#if OPENTHREAD_CONFIG_TOBLE_ENABLE

namespace ot {
namespace Toble {

class Connection
{
public:
    bool IsInUse(void) const { return (mPlatConn != OT_TOBLE_CONNECTION_ID_INVALID); }

    Platform::ConnectionId mPlatConn;
    Transport::Type        mTransport;

#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
    enum
    {
        kInfoStringSize = 120,
    };

    typedef String<kInfoStringSize> InfoString;
    InfoString                      ToString(void) const;

    Mac::ShortAddress mShortAddr;
    Mac::ExtAddress   mExtAddr;
    TimeMilli         mDisconnectTime;
    int8_t            mRssi;
    enum
    {
        kConnecting,
        kConnected,
        kSending,
    } mState;

#if OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
    uint8_t mL2capPsm;
#endif // OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
#endif // OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE

    Btp::Session mSession;
};

class ConnectionTable
{
public:
    enum
    {
#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE && OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
        kMaxConnections = 12,
#elif OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
        kMaxConnections = 10,
#else
        kMaxConnections = 2,
#endif
    };

    ConnectionTable(void);

    // Iterate through all in-use `Connection` elements in the table.
    Connection *GetFirst(void) { return Iterate(NULL); }
    Connection *GetNext(Connection *aPrev) { return Iterate(aPrev); }

    Connection *Find(Platform::ConnectionId aPlatConn);
    Connection *GetNew(void);
    void        Remove(Connection &aConn) { aConn.mPlatConn = OT_TOBLE_CONNECTION_ID_INVALID; }

#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
    Connection *Find(const Mac::Address &aAddress);
    Connection *FindEarliestDisconnectTime(void);
#endif

private:
    Connection *Iterate(Connection *aPrev);

    Connection mConnArray[kMaxConnections];
};

} // namespace Toble
} // namespace ot

#endif // OPENTHREAD_CONFIG_TOBLE_ENABLE

#endif // TOBLE_CONNECTION_TABLE_HPP_
