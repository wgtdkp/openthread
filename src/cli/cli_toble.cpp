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
 *   This file implements a simple CLI for the ToBLE.
 */

#include "cli_toble.hpp"

#include "cli/cli.hpp"
#include "cli/cli_server.hpp"

#if OPENTHREAD_CONFIG_TOBLE_ENABLE

namespace ot {
namespace Cli {

const struct Toble::Command Toble::sCommands[] = {{"help", &Toble::ProcessHelp},
                                                  {"linkmode", &Toble::ProcessLinkMode},
                                                  {"test", &Toble::ProcessTest}};

Toble::Toble(Interpreter &aInterpreter)
    : mInterpreter(aInterpreter)
{
}

otError Toble::ProcessHelp(uint8_t aArgsLength, char *aArgs[])
{
    OT_UNUSED_VARIABLE(aArgsLength);
    OT_UNUSED_VARIABLE(aArgs);

    for (size_t i = 0; i < OT_ARRAY_LENGTH(sCommands); i++)
    {
        mInterpreter.mServer->OutputFormat("%s\r\n", sCommands[i].mName);
    }

    return OT_ERROR_NONE;
}

otError Toble::ProcessLinkMode(uint8_t aArgsLength, char *aArgs[])
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(aArgsLength == 1, error = OT_ERROR_INVALID_ARGS);

    if (strcmp(aArgs[0], "peripheral") == 0)
    {
        otTobleSetLinkMode(mInterpreter.mInstance, OT_TOBLE_LINK_MODE_PERIPHERAL);
    }
    else if (strcmp(aArgs[0], "central") == 0)
    {
        otTobleSetLinkMode(mInterpreter.mInstance, OT_TOBLE_LINK_MODE_CENTRAL);
    }
    else
    {
        mInterpreter.mServer->OutputFormat(
            "%s", otTobleGetLinkMode(mInterpreter.mInstance) == OT_TOBLE_LINK_MODE_CENTRAL ? "peripheral" : "central");
    }

exit:
    return error;
}

otError Toble::ProcessTest(uint8_t aArgsLength, char *aArgs[])
{
    OT_UNUSED_VARIABLE(aArgsLength);
    OT_UNUSED_VARIABLE(aArgs);
    otTobleTest(mInterpreter.mInstance);
    return OT_ERROR_NONE;
}

otError Toble::Process(uint8_t aArgsLength, char *aArgs[])
{
    otError error = OT_ERROR_INVALID_COMMAND;

    if (aArgsLength < 1)
    {
        IgnoreError(ProcessHelp(0, NULL));
        error = OT_ERROR_INVALID_ARGS;
    }
    else
    {
        for (size_t i = 0; i < OT_ARRAY_LENGTH(sCommands); i++)
        {
            if (strcmp(aArgs[0], sCommands[i].mName) == 0)
            {
                error = (this->*sCommands[i].mCommand)(aArgsLength - 1, aArgs + 1);
                break;
            }
        }
    }
    return error;
}

} // namespace Cli
} // namespace ot

#endif // OPENTHREAD_CONFIG_TOBLE_ENABLE
