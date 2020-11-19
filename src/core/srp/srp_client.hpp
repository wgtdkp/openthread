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
 *   This file includes definitions for the SRP client.
 */

#ifndef SRP_CLIENT_HPP_
#define SRP_CLIENT_HPP_

#include "openthread-core-config.h"

#if OPENTHREAD_CONFIG_SRP_CLIENT_ENABLE

#include <openthread/error.h>

#include "common/locator.hpp"
#include "common/timer.hpp"

namespace ot {

class SrpClient : public InstanceLocator
{
public:

    typedef void (*WakeupCallback)(void*);

    /**
     * This constructor initializes the SRP client object.
     *
     * @param[in]  aInstance     A reference to the OpenThread instance.
     *
     */
    explicit SrpClient(Instance &aInstance)
        : InstanceLocator(aInstance)
        , mTimer(aInstance, HandleTimer, this)
        , mWakeupCallback(nullptr)
    {
    }

    otError Start(void);

    otError Stop(void);

    // TODO: support TXT records.
    otError Register(const char *aName, const char *aType, const char *aDomain,
                     const char *aHost, uint16_t aPort);

    // Deregister all services.
    otError Deregister(void);

    void SetTimerHandler(WakeupCallback aWakeupCallback) { mWakeupCallback = aWakeupCallback; }

    TimerMilli *GetTimer() { return &mTimer; }

private:
    static void HandleTimer(Timer &aTimer);

    TimerMilli mTimer;
    WakeupCallback mWakeupCallback;
};

} // namespace ot

#endif // OPENTHREAD_CONFIG_SRP_CLIENT_ENABLE

#endif // SRP_CLIENT_HPP_
