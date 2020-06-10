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
 *   This file implements ToBLE connection table.
 */

#include "conn_table.hpp"

#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "common/encoding.hpp"
#include "common/instance.hpp"
#include "common/locator-getters.hpp"
#include "common/logging.hpp"
#include "common/random.hpp"
#include "mac/mac_frame.hpp"

#if OPENTHREAD_CONFIG_ENABLE_TOBLE

namespace ot {
namespace Toble {

//----------------------------------------------------------------------------------------------------------------------
// Connection

#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE

Connection::InfoString Connection::ToString(void) const
{
    InfoString str;

    str.Append("%s, ", mExtAddr.ToString().AsCString());

    if (mShortAddr != Mac::kShortAddrInvalid)
    {
        str.Append("0x%04x, ", mShortAddr);
    }

    str.Append("");

    switch (mState)
    {
    case kConnecting:
        str.Append("connecting, dt:%d", mDisconnectTime - TimerMilli::GetNow());
        break;

    case kConnected:
        str.Append("connected, dt:%d", mDisconnectTime - TimerMilli::GetNow());
        break;

    case kSending:
        str.Append("sending");
        break;
    }

    str.Append(", rssi:%d", mRssi);

#if OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
    str.Append(", transport:");

    switch (mTransport)
    {
    case Transport::kBtp:
        str.Append("btp");
        break;
    case Transport::kL2cap:
        str.Append("l2cap (psm:0x%02x)", mL2capPsm);
        break;
    case Transport::kUnspecified:
        str.Append("unspecified");
        break;
    }
#endif

    return str;
}
#endif // #if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE

//----------------------------------------------------------------------------------------------------------------------
// ConnectionTable

ConnectionTable::ConnectionTable(void)
{
    for (Connection *conn = &mConnArray[0]; conn < OT_ARRAY_END(mConnArray); conn++)
    {
        conn->mPlatConn = NULL;
    }
}

Connection *ConnectionTable::Iterate(Connection *aPrev)
{
    Connection *next;

    for (next = ((aPrev == NULL) ? &mConnArray[0] : (aPrev + 1)); next < OT_ARRAY_END(mConnArray); next++)
    {
        VerifyOrExit(!next->IsInUse());
    }

    next = NULL;

exit:
    return next;
}

Connection *ConnectionTable::Find(Platform::Connection *aPlatConn)
{
    Connection *conn;

    for (conn = &mConnArray[0]; conn < OT_ARRAY_END(mConnArray); conn++)
    {
        VerifyOrExit(conn->mPlatConn != aPlatConn);
    }

    conn = NULL;

exit:
    return conn;
}

Connection *ConnectionTable::GetNew(void)
{
    // Find an element with `mPlatConn` being NULL.
    Connection *conn = Find(NULL);

    if (conn != NULL)
    {
        conn->mTransport = Transport::kUnspecified;
    }

    return conn;
}

#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE

Connection *ConnectionTable::Find(const Mac::Address &aAddress)
{
    Connection *conn = NULL;

    if (aAddress.IsShort())
    {
        for (conn = GetFirst(); conn != NULL; conn = GetNext(conn))
        {
            VerifyOrExit(conn->mShortAddr != aAddress.GetShort());
        }
    }
    else if (aAddress.IsExtended())
    {
        for (conn = GetFirst(); conn != NULL; conn = GetNext(conn))
        {
            VerifyOrExit(conn->mExtAddr != aAddress.GetExtended());
        }
    }

exit:
    return conn;
}

Connection *ConnectionTable::FindEarliestDisconnectTime(void)
{
    Connection *earlist = NULL;

    for (Connection *conn = GetFirst(); conn != NULL; conn = GetNext(conn))
    {
        if ((earlist == NULL) || TimerScheduler::IsStrictlyBefore(conn->mDisconnectTime, earlist->mDisconnectTime))
        {
            earlist = conn;
        }
    }

    return earlist;
}

#endif

} // namespace Toble
} // namespace ot

#endif // OPENTHREAD_CONFIG_ENABLE_TOBLE
