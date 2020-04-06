#ifndef OT_EVENT_QUEUE_API_H_
#define OT_EVENT_QUEUE_API_H_

#include <openthread/error.h>
#include <openthread/instance.h>

#ifdef __cplusplus
extern "C" {
#endif

enum EventType
{
    EVENT_ANY     = -1,
    EVENT_UART_IO = 0,
    EVENT_UART_SPINEL,
    EVENT_NETIF_CHANGED,
    EVENT_NETIF_OUTPUT,
    EVENT_UDP_RECEIVE,
    EVENT_QUEUE_YIELD,
    EVENT_CNT,
};

otError otEventQueuePush(otInstance *aInstance, int aType, void *aData, uint32_t aTimeoutMilliSec);

// not to be used outside openthread task
otError otEventQueuePop(otInstance *aInstance, int *aType, void **aData, uint32_t aTimeoutMilliSec);

#ifdef __cplusplus
}
#endif

#endif
