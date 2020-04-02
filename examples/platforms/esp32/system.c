#include <openthread-system.h>
#include <openthread/tasklet.h>
#include <openthread/platform/alarm-milli.h>

#include <esp_log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "esp32_platform.h"
#include "event_queue_api.h"
#include "ot_api_call.h"
#include "uart_multiplexer.h"

extern bool gPlatformPseudoResetWasRequested;

void otSysInit(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    if (gPlatformPseudoResetWasRequested)
    {
        gPlatformPseudoResetWasRequested = false;
        return;
    }
    esp32UartInit();
    platformRadioUartInit();
    startUartMultiPlexing();
    platformRadioInit();
}

void otSysDeinit(void)
{
}

bool otSysPseudoResetWasRequested(void)
{
    return gPlatformPseudoResetWasRequested;
}

void otSysProcessDrivers(otInstance *aInstance)
{
    uint32_t timeout   = 1000;
    int      eventType = EVENT_ANY;
    void *   eventData = NULL;

    esp32AlarmUpdateTimeout(&timeout);
    esp32UartUpdateTimeout(&timeout);
    platformRadioUpdateTimeout(&timeout);

    if (otTaskletsArePending(aInstance))
    {
        timeout = 0;
    }

    otxAPIUnlock();
    otEventQueuePop(aInstance, &eventType, &eventData, timeout);
    otxAPILock();

//    ESP_LOGD(PLAT_LOG_TAG, "otEvent: Type=0x%08X Data=%p", eventType, eventData);

    //esp32UartProcess(eventType, eventData);
    platformRadioProcess(eventType, eventData);
    esp32AlarmProcess(aInstance);
#if OPENTHREAD_CONFIG_PLATFORM_NETIF_ENABLE
    esp32NetifProcess(aInstance, eventType, eventData);
#endif
#if OPENTHREAD_CONFIG_PLATFORM_UDP_ENABLE
    esp32UdpProcess(aInstance, eventType, eventData);
#endif

    if (eventData != NULL)
    {
        free(eventData);
    }
}
