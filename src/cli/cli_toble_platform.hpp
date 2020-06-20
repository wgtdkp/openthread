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
 *   This file contains definitions for a simple CLI for the ToBLE platform service.
 */

#ifndef CLI_TOBLE_PLATFORM_HPP_
#define CLI_TOBLE_PLATFORM_HPP_

#include "openthread-core-config.h"

#include <openthread/platform/toble.h>

namespace ot {
namespace Cli {

class Interpreter;

/**
 * This class implements a CLI for ToBLE platform API.
 *
 */
class ToblePlatform
{
public:
    /**
     * Constructor
     *
     * @param[in]  aInterpreter  The CLI interpreter.
     *
     */
    explicit ToblePlatform(Interpreter &aInterpreter);

    /**
     * This method interprets a list of CLI arguments.
     *
     * @param[in]  aArgsLength  The number of elements in @p aArgs.
     * @param[in]  aArgs        An array of command line arguments.
     *
     */
    otError Process(uint8_t aArgsLength, char *aArgs[]);

    void HandleConnected(otTobleConnection *aConn);
    void HandleDisconnected(otTobleConnection *aConn);

private:
    struct Command
    {
        const char *mName;
        otError (ToblePlatform::*mCommand)(uint8_t aArgsLength, char *aArgs[]);
    };

    typedef enum
    {
        kStateFree          = 0,
        kStateConnecting    = 1,
        kStateConnected     = 2,
        kStateReady         = 3,
        kStateDisconnecting = 4,
    } State;

    typedef struct Connection
    {
        State              mState;
        otTobleAddress     mAddress;
        otTobleConnection *mPlatConn;
    } Connection;

    enum
    {
        kNumConnections      = 4,
        kInvalidConnectionId = 0xff,
    };

    Connection *FindConnection(otTobleConnection *aConn);
    Connection *GetNewConnection(void);
    Connection *GetConnection(otTobleConnection *aConn);
    Connection *GetConnection(uint8_t aConnId);
    uint8_t     GetConnectionId(Connection *aConn);

    otError ProcessHelp(uint8_t aArgsLength, char *aArgs[]);
    otError ProcessAdv(uint8_t aArgsLength, char *aArgs[]);
    otError ProcessScan(uint8_t aArgsLength, char *aArgs[]);
    otError ProcessConnect(uint8_t aArgsLength, char *aArgs[]);
    otError ProcessSend(uint8_t aArgsLength, char *aArgs[]);
    otError ProcessShow(uint8_t aArgsLength, char *aArgs[]);

    void        ReverseBuf(uint8_t *aBuffer, uint8_t aLength);
    const char *GetStateString(State aState);

    static const Command sCommands[];
    Interpreter &        mInterpreter;

    Connection                mConns[kNumConnections];
    otTobleConnectionLinkType mLinkType;
    otTobleRole               mRole;
};

} // namespace Cli
} // namespace ot

#endif // CLI_TOBLE_PLATFORM_HPP_
