#include "event_queue_api.h"
#include "event_queue_impl.hpp"

static EventQueueMux<EVENT_CNT, 10> sEventQueueMux;

otError otEventQueuePush(otInstance *aInstance, int aType, void *aData, uint32_t aTimeoutMilliSec)
{
    Event event;
    event.type = static_cast<EventType>(aType);
    event.data = aData;

    return sEventQueueMux.pushEvent(event, aTimeoutMilliSec);

    // for now we don't support multi instance
    (void)aInstance;
}

otError otEventQueuePop(otInstance *aInstance, int *aType, void **aData, uint32_t aTimeoutMilliSec)
{
    otError error;
    Event   event;
    event.type = static_cast<EventType>(*aType);
    event.data = NULL;
    if (event.type == EVENT_ANY)
    {
        SuccessOrExit(error = sEventQueueMux.popEventAnyType(event, aTimeoutMilliSec));
    }
    else
    {
        SuccessOrExit(error = sEventQueueMux.popEvent(event.type, event, aTimeoutMilliSec));
    }
    *aType = event.type;
    *aData = event.data;
exit:
    return error;

    // for now we don't support multi instance
    (void)aInstance;
}
