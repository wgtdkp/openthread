#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "esp_log.h"

#include "error_handling.h"
#include "event_queue_api.h"
#include "uart_multiplexer.h"

void uartMultiPlexingTask(void *p) {
    (void)p;

    QueueSetHandle_t queueSet =
        xQueueCreateSet(OT_IO_UART_QUEUE_SIZE + OT_SPINEL_UART_QUEUE_SIZE);
    // assert(xQueueAddToSet(esp32UartGetQueue(), queueSet) == pdPASS);
    assert(xQueueAddToSet(platformRadioGetQueue(), queueSet) == pdPASS);
    while (true) {
        uart_event_t event;
        QueueHandle_t activeMember =
            xQueueSelectFromSet(queueSet, portMAX_DELAY);
        /* if (activeMember == esp32UartGetQueue()) {
            xQueueReceive(esp32UartGetQueue(), &event, 0);
            if (event.type == UART_DATA) {
                if (otEventQueuePush(NULL, EVENT_UART_IO, NULL, 0) !=
                    OT_ERROR_NONE) {
                    ESP_LOGW(PLAT_LOG_TAG, "Uart event lost");
                }
            }
        } else */ if (activeMember == platformRadioGetQueue()) {
            xQueueReceive(platformRadioGetQueue(), &event, 0);
            if (event.type == UART_DATA) {
                if (otEventQueuePush(NULL, EVENT_UART_SPINEL, NULL, 0) !=
                    OT_ERROR_NONE) {
                    ESP_LOGW(PLAT_LOG_TAG, "spinel event lost");
                }
            }
        } else {
            ESP_LOGE(PLAT_LOG_TAG, "Unkown uart multiplex error");
        }
    }
}

void startUartMultiPlexing() {
    xTaskCreate(uartMultiPlexingTask, "uart_mux", 4096, NULL, 5, NULL);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}
