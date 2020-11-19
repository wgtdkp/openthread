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
 *   This file implements a simple CLI for the SRP client.
 */

#include "cli_srp_client.hpp"

#ifdef OPENTHREAD_CONFIG_SRP_CLIENT_ENABLE

#include <openthread/srp.h>

#include "cli/cli.hpp"
#include "common/code_utils.hpp"

namespace ot {
namespace Cli {

const struct SrpClient::Command SrpClient::sCommands[] = {
    {"help", &SrpClient::ProcessHelp},
    {"start", &SrpClient::ProcessStart},
    {"stop", &SrpClient::ProcessStop},
    {"register", &SrpClient::ProcessRegister},
    {"deregister", &SrpClient::ProcessDeregister},
};

otError SrpClient::Process(uint8_t aArgsLength, char *aArgs[])
{
    otError error = OT_ERROR_INVALID_COMMAND;

    if (aArgsLength < 1)
    {
        IgnoreError(ProcessHelp(0, nullptr));
    }
    else
    {
        for (const Command &command : sCommands)
        {
            if (strcmp(aArgs[0], command.mName) == 0)
            {
                //assert(this->mInterpreter.mInstance != nullptr);
                error = (this->*command.mCommand)(aArgsLength, aArgs);
                break;
            }
        }
    }

    return error;
}

otError SrpClient::ProcessHelp(uint8_t aArgsLength, char *aArgs[])
{
    OT_UNUSED_VARIABLE(aArgsLength);
    OT_UNUSED_VARIABLE(aArgs);

    for (const Command &command : sCommands)
    {
        mInterpreter.OutputLine("%s", command.mName);
    }

    return OT_ERROR_NONE;
}

otError SrpClient::ProcessStart(uint8_t aArgsLength, char *aArgs[])
{
    OT_UNUSED_VARIABLE(aArgsLength);
    OT_UNUSED_VARIABLE(aArgs);

    // assert(mInterpreter.mInstance != nullptr);
    return otSrpClientStart(mInterpreter.mInstance);
}

otError SrpClient::ProcessStop(uint8_t aArgsLength, char *aArgs[])
{
    OT_UNUSED_VARIABLE(aArgsLength);
    OT_UNUSED_VARIABLE(aArgs);

    otSrpClientStop(mInterpreter.mInstance);

    return OT_ERROR_NONE;
}

// srpclient register <name> <type> <domain> <host> <port>
otError SrpClient::ProcessRegister(uint8_t aArgsLength, char *aArgs[])
{
    otError error = OT_ERROR_NONE;

    unsigned long port;

    VerifyOrExit(aArgsLength > 5, error = OT_ERROR_INVALID_ARGS);
    SuccessOrExit(error = mInterpreter.ParseUnsignedLong(aArgs[5], port));

    error = otSrpClientRegister(mInterpreter.mInstance, aArgs[1], aArgs[2], aArgs[3], aArgs[4], port);

exit:
    return error;
}

otError SrpClient::ProcessDeregister(uint8_t aArgsLength, char *aArgs[])
{
    OT_UNUSED_VARIABLE(aArgsLength);
    OT_UNUSED_VARIABLE(aArgs);

    return otSrpClientDeregister(mInterpreter.mInstance);
}

} // namespace Cli
} // namespace ot

#endif // OPENTHREAD_CONFIG_SRP_CLIENT_ENABLE
