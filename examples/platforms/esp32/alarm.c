#include <stdbool.h>
#include <stdint.h>

#include <openthread/config.h>
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/diag.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_timer.h>

#include "esp32_platform.h"

#include <esp_log.h>

static uint32_t sAlarmT0   = 0;
static uint32_t sAlarmDt   = 0;
static bool     sIsRunning = false;

void otPlatAlarmMilliStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
    (void)aInstance;

    sAlarmT0   = aT0;
    sAlarmDt   = aDt;
    sIsRunning = true;
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

void esp32AlarmUpdateTimeout(uint32_t *aWaitTime)
{
    if (sIsRunning)
    {
        uint32_t now            = otPlatAlarmMilliGetNow();
        uint32_t alarmExpires   = sAlarmT0 + sAlarmDt;
        uint32_t timeoutExpires = now + (*aWaitTime);
        bool     isSameSide     = !((alarmExpires > sAlarmT0) ^ (timeoutExpires > sAlarmT0));
        if ((alarmExpires < timeoutExpires && isSameSide) || (alarmExpires > timeoutExpires && !isSameSide))
        {
            uint32_t timeoutMs = sAlarmT0 + sAlarmDt - now;
            *aWaitTime         = timeoutMs;
        }
    }
}

void esp32AlarmProcess(otInstance *aInstance)
{
    uint32_t expires;
    uint32_t now  = otPlatAlarmMilliGetNow();
    bool     fire = false;

    if (sIsRunning)
    {
        expires = sAlarmT0 + sAlarmDt;

        if (sAlarmT0 <= now)
        {
            if (expires >= sAlarmT0 && expires <= now)
            {
                fire = true;
            }
        }
        else
        {
            if (expires >= sAlarmT0 || expires <= now)
            {
                fire = true;
            }
        }

        if (fire)
        {
            sIsRunning = false;

#if OPENTHREAD_ENABLE_DIAG

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
