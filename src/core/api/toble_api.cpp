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
 *   This file implements the OpenThread ToBLE APIs
 */

#include "openthread-core-config.h"

#if OPENTHREAD_CONFIG_TOBLE_ENABLE

#include <openthread/toble.h>

#include "common/locator-getters.hpp"
#include "toble/toble.hpp"

using namespace ot;

#if !OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE && !OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
#error "ToBLE config error - enable either CONFIG_TOBLE_CENTRAL_ENABLE or CONFIG_TOBLE_PERIPHERAL_ENABLE (or both)."
#endif

#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE && OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
otError otTobleSetLinkMode(otInstance *aInstance, otTobleLinkMode aLinkMode)
{
    Instance &instance = *static_cast<Instance *>(aInstance);

    return instance.Get<Toble::Toble>().SetMode(aLinkMode);
}
#endif // OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE && OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE

otTobleLinkMode otTobleGetLinkMode(otInstance *aInstance)
{
    otTobleLinkMode mode;

    OT_UNUSED_VARIABLE(aInstance);

#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE && OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
    mode = static_cast<Instance *>(aInstance)->Get<Toble::Toble>().GetMode();
#elif OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
    mode = OT_TOBLE_LINK_MODE_CENTRAL;
#else
    mode = OT_TOBLE_LINK_MODE_PERIPHERAL;
#endif

    return mode;
}

void otTobleTest(otInstance *aInstance)
{
    static_cast<Instance *>(aInstance)->Get<Toble::Toble>().Test();
}
#endif // #if OPENTHREAD_CONFIG_TOBLE_ENABLE