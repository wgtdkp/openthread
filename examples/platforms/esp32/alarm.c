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
 * This file implements the alarm driver.
 *
 */

#include "platform-esp32.h"

#include <stdbool.h>
#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_timer.h>

#include <openthread/config.h>
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/diag.h>
#include <openthread/platform/time.h>

#include "error_handling.h"

static uint32_t sAlarmT0   = 0;
static uint32_t sAlarmDt   = 0;
static bool     sIsRunning = false;

void otPlatAlarmMilliStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
    (void)aInstance;

    sAlarmT0             = aT0;
    sAlarmDt             = aDt;
    sIsRunning = true;

    //printf("alarm start running, t0=%u, dt=%u\n", sAlarmT0, sAlarmDt);
}

void otPlatAlarmMilliStop(otInstance *aInstance)
{
    (void)aInstance;

    sIsRunning = false;
}

uint32_t otPlatAlarmMilliGetNow(void)
{
    return esp_timer_get_time() / 1000;
}

void platformAlarmUpdate(otSysMainloopContext *aMainloop)
{
    struct timeval *timeout = &aMainloop->mTimeout;
    uint32_t now = otPlatAlarmMilliGetNow();

    if (!sIsRunning) {
        timeout->tv_sec = INT32_MAX;
        timeout->tv_usec = 0;
    } else if (sAlarmDt + sAlarmT0 > now) {
        uint32_t remaining = sAlarmDt + sAlarmT0 - now;
        timeout->tv_sec = remaining / 1000;
        timeout->tv_usec = (remaining % 1000) * 1000;
    }
    else {
        timeout->tv_sec = 0;
        timeout->tv_usec = 0;
    }
}

void platformAlarmProcess(otInstance *aInstance, const otSysMainloopContext *aMainloop)
{
    (void)aMainloop;

    if (sIsRunning)
    {
        if (sAlarmT0 + sAlarmDt <= otPlatAlarmMilliGetNow())
        {
            //printf("sIsRunning set to false\n");
            sIsRunning = false;

#if OPENTHREAD_CONFIG_DIAG_ENABLE

            if (otPlatDiagModeGet())
            {
                otPlatDiagAlarmFired(aInstance);
            }
            else
#endif
            {
                otPlatAlarmMilliFired(aInstance);
            }
        }
    }
}
