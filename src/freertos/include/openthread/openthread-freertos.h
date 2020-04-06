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
 * @brief
 *  This file defines the OpenThread FreeRTOS Instance API.
 */

#ifndef OT_RTOS_SYSTEM_H_
#define OT_RTOS_SYSTEM_H_

#include <ot_rtos/netif.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * This structure represents the OpenThread instance structure.
 *
 * otRtosInstance is a wrapper of otInstance, with additional
 * synchronization and network interface structures.
 *
 */
typedef struct otRtosInstance otRtosInstance;

/**
 * This function returns the lwip netif bound to the ot-rtos instance.
 *
 */
otRtosNetIf *otRtosGetNetIf(otRtosInstance *aOtRtosInstance);

/**
 * This function initializes the static single instance of the OpenThread FreeRTOS library.
 *
 * This function initializes OpenThread and prepares it for subsequent OpenThread API calls. This function must be
 * called before any other calls to OpenThread.
 *
 * This function is available and can only be used when support for multiple OpenThread FreeRTOS instances is disabled.
 *
 * @returns A pointer to the single OpenThread FreeRTOS instance.
 *
 */
otRtosInstance *otRtosInit(void);

/**
 * This function finalizes the static single instance of the OpenThread FreeRTOS library.
 *
 * No OpenThread API should be called after this function.
 *
 */
void otRtosFinalize(otRtosInstance *aOtRtosInstance);

/**
 * This function starts OpenThread FreeRTOS task.
 *
 */
void otRtosStart(otRtosInstance *aOtRtosInstance);

/**
 * This function stops OpenThread FreeRTOS task.
 *
 */
void otRtosStop(otRtosInstance *aOtRtosInstance);

/**
 * This function locks OpenThread APIs.
 *
 * Should be called before invoking a native openthread API.
 *
 */
void otRtosApiLock(otRtosInstance *aOtRtosInstance);

/**
 * This function unlocks OpenThread APIs.
 *
 * Should be called after invoking a native openthread API.
 *
 */
void otRtosApiUnlock(otRtosInstance *aOtRtosInstance);

/**
 * This function notifies (wakes) the ot-rtos task.
 *
 */
void otRtosNotify(otRtosInstance *aOtRtosInstance);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // OT_RTOS_SYSTEM_H_
