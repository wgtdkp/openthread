#include <openthread/platform/uart.h>

#include <driver/uart.h>

#include <esp_log.h>

#include "error_handling.h"
#include "esp32_platform.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

//#include "event_queue_api.h"

#define OT_IO_UART_NUM UART_NUM_0
#define OT_RX_BUF_SIZE (UART_FIFO_LEN * 2)

static const uint8_t *sIOTransmitBuffer = NULL;
static uint16_t       sIOTransmitLength = 0;
static uint8_t        sIORxBuffer[OT_RX_BUF_SIZE];
static QueueHandle_t  sUartQueue = NULL;

void esp32UartInit()
{
    sIOTransmitLength = 0;
    sIOTransmitBuffer = NULL;

    uart_config_t uart_config = {.baud_rate           = 115200,
                                 .data_bits           = UART_DATA_8_BITS,
                                 .parity              = UART_PARITY_DISABLE,
                                 .stop_bits           = UART_STOP_BITS_1,
                                 .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
                                 .rx_flow_ctrl_thresh = 0,
                                 .use_ref_tick        = false};
    uart_param_config(OT_IO_UART_NUM, &uart_config);
    uart_driver_install(OT_IO_UART_NUM, UART_FIFO_LEN * 2, 0, OT_IO_UART_QUEUE_SIZE, &sUartQueue, 0);
}

QueueHandle_t esp32UartGetQueue(void)
{
    return sUartQueue;
}

otError otPlatUartEnable(void)
{
    return OT_ERROR_NONE;
}

otError otPlatUartDisable(void)
{
    return OT_ERROR_NONE;
}

otError otPlatUartFlush(void)
{
    return OT_ERROR_NONE;
}

otError otPlatUartSend(const uint8_t *aBuf, uint16_t aBufLength)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(sIOTransmitLength == 0, error = OT_ERROR_BUSY);

    sIOTransmitBuffer = aBuf;
    sIOTransmitLength = aBufLength;

exit:
    return error;
}

void esp32UartUpdateTimeout(uint32_t *aWaitTime)
{
    if (sIOTransmitLength > 0)
    {
        // selecting write fds until ready is not implemented in esp-idf
        // so we fall back to loop polling
        *aWaitTime = 0;
    }
}

void esp32UartProcess(int aEventType, void *aEventData)
{
    if (sIOTransmitLength > 0)
    {
        int writeSize = uart_tx_chars(OT_IO_UART_NUM, (const char *)sIOTransmitBuffer, sIOTransmitLength);
        sIOTransmitLength -= writeSize;
        sIOTransmitBuffer += writeSize;
        if (sIOTransmitLength == 0)
        {
            sIOTransmitBuffer = NULL;
            otPlatUartSendDone();
        }
    }
    if (aEventType == EVENT_UART_IO)
    {
        int readSize = uart_read_bytes(0, (uint8_t *)sIORxBuffer, sizeof(sIORxBuffer), 0);
        if (readSize > 0)
        {
            otPlatUartReceived(sIORxBuffer, (uint16_t)readSize);
        }
    }
}
