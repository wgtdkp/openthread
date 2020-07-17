/*
 *  Copyright (c) 2017, The OpenThread Authors.
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
 * @brief
 *   This file includes the OpenThread API for ToBLE feature.
 */

#ifndef OPENTHREAD_TOBLE_H_
#define OPENTHREAD_TOBLE_H_

#include <openthread/error.h>
#include <openthread/instance.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup api-toble
 *
 * @brief
 *   This module includes functions for Thread over BLE (ToBLE).
 *
 * @{
 *
 */

/**
 * This enumeration represents the ToBLE link mode.
 *
 */
typedef enum otTobleLinkMode
{
    OT_TOBLE_LINK_MODE_PERIPHERAL = 0, ///< ToBLE link is in peripheral mode.
    OT_TOBLE_LINK_MODE_CENTRAL    = 1, ///< ToBLE link is in central mode.
} otTobleLinkMode;

/**
 * This function sets the ToBLE link mode.
 *
 * This function is available and can only be used when BOTH peripheral and central modes are enabled, i.e., OpenThread
 * configurations sets both `OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE` and `OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE` to
 * one (non-zero). In addition, this function succeeds only when Thread protocols are disabled and interface is down.
 *
 *
 * @param[in]  aInstance          A pointer to an OpenThread instance.
 * @param[in]  aLinkMode          The ToBLE link mode to set (central or peripheral).
 *
 * @retval OT_ERROR_NONE          Successfully enabled BLER role.
 * @retval OT_ERROR_INVALID_STATE Device is in an invalid state.
 *
 */
otError otTobleSetLinkMode(otInstance *aInstance, otTobleLinkMode aLinkMode);

/**
 * This function gets the current ToBLE link mode.
 *
 * @param[in]  aInstance          A pointer to an OpenThread instance.
 *
 * @retval The current ToBLE link mode.
 *
 */
otTobleLinkMode otTobleGetLinkMode(otInstance *aInstance);

void otTobleTest(otInstance *aInstance);

/**
 * @}
 *
 */

#ifdef __cplusplus
} // extern "C"
#endif

#endif // OPENTHREAD_TOBLE_H_
