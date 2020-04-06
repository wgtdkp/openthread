#ifndef OT_ESP32_PLATFORM_H_
#define OT_ESP32_PLATFORM_H_

#include <openthread/instance.h>

#include <driver/uart.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <lwip/netif.h>

#define PLAT_LOG_TAG "OT_ESP32_PLAT"
#define OT_IO_UART_QUEUE_SIZE 20
#define OT_SPINEL_UART_QUEUE_SIZE 20

#ifdef __cplusplus
extern "C" {
#endif

void espAlarmProcess(otInstance *aInstance);

void platformRadioInit();

void platformRadioUartInit(void);

QueueHandle_t platformRadioGetQueue(void);

void platformRadioDeinit(void);

void platformRadioUpdateTimeout(uint32_t *aWaitTime);

void platformRadioProcess(int aEventType, void *aEventData);

void esp32AlarmUpdateTimeout(uint32_t *aWaitTime);

void esp32AlarmProcess(otInstance *aInstance);

void esp32UartUpdateTimeout(uint32_t *aWaitTime);

void esp32UartInit();

QueueHandle_t esp32UartGetQueue(void);

void esp32UartProcess(int aEventType, void *aEventData);
void esp32NetifProcess(otInstance *aInstance, int aEventType, const void *aEventData);
void esp32UdpProcess(otInstance *aInstance, int aEventType, const void *aEventData);

/**
 * This function gets the LwIP netif for OpenThread.
 *
 * @returns A pointer to the LwIP netif for OpenThread.
 *
 */
struct netif *otxGetNetif(void);

#ifdef __cplusplus
}
#endif
#endif // OTBR_ESP32_PLATFORM_H_
