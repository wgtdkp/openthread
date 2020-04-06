#ifndef OT_EVENT_QUEUE_IMPL_HPP_
#define OT_EVENT_QUEUE_IMPL_HPP_

#include <stddef.h>
#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "error_handling.h"
#include "event_queue_api.h"
#include "openthread/error.h"

struct Event {
    EventType type;
    void *data;
};

template <size_t nMux, size_t depth>
class EventQueueMux {
   public:
    EventQueueMux() {
        mAnyCounter = xSemaphoreCreateCounting(depth * nMux, 0);
        for (size_t i = 0; i < nMux; i++) {
            mQueues[i] = xQueueCreate(depth, sizeof(Event));
            mQueueCounters[i] = xSemaphoreCreateCounting(depth, 0);
        }
        mMutex = xSemaphoreCreateMutex();
    }

    otError pushEvent(const Event &aEvent, uint32_t aTimeoutMilliSec) {
        otError error = OT_ERROR_NONE;

        VerifyOrExit(aEvent.type < nMux, error = OT_ERROR_FAILED);
        VerifyOrExit(
            xQueueSend(mQueues[aEvent.type], &aEvent,
                       aTimeoutMilliSec / portTICK_PERIOD_MS) == pdTRUE,
            error = OT_ERROR_BUSY);

        xSemaphoreTake(mMutex, portMAX_DELAY);
        xSemaphoreGive(mAnyCounter);
        xSemaphoreGive(mQueueCounters[aEvent.type]);
        xSemaphoreGive(mMutex);
    exit:
        return error;
    }

    // we assume pop will be called only in the openthread task
    otError popEvent(EventType aType, Event &aEvent,
                     uint32_t aTimeoutMilliSec) {
        otError error = OT_ERROR_NONE;

        VerifyOrExit(aType < nMux, error = OT_ERROR_FAILED);
        VerifyOrExit(
            xSemaphoreTake(mQueueCounters[aEvent.type],
                           aTimeoutMilliSec / portTICK_PERIOD_MS) == pdTRUE,
            error = OT_ERROR_BUSY);
        xSemaphoreTake(mMutex, portMAX_DELAY);
        xSemaphoreTake(mAnyCounter, 0);
        xQueueReceive(mQueues[aEvent.type], &aEvent, 0);
        xSemaphoreGive(mMutex);
    exit:
        return error;
    }

    // we assume pop will be called only in the openthread task
    otError popEventAnyType(Event &aEvent, uint32_t aTimeoutMilliSec) {
        otError error = OT_ERROR_NONE;

        VerifyOrExit(
            xSemaphoreTake(mAnyCounter,
                           aTimeoutMilliSec / portTICK_PERIOD_MS) == pdTRUE,
            error = OT_ERROR_BUSY);
        xSemaphoreTake(mMutex, portMAX_DELAY);
        for (size_t i = 0; i < nMux; i++) {
            if (xSemaphoreTake(mQueueCounters[i], 0) == pdTRUE) {
                xQueueReceive(mQueues[i], &aEvent, 0);
                break;
            }
        }
        xSemaphoreGive(mMutex);
    exit:
        return error;
    }

   private:
    SemaphoreHandle_t mAnyCounter;

    QueueHandle_t mQueues[nMux];
    SemaphoreHandle_t mQueueCounters[nMux];

    SemaphoreHandle_t mMutex;
};

#endif
