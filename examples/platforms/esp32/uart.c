#include "platform-esp32.h"

#include <errno.h>
#include <fcntl.h>
#include <sys/select.h>

#include <openthread/platform/uart.h>

#include <driver/uart.h>
#include <esp_log.h>
#include <esp_vfs_dev.h>

#include "openthread/openthread-system.h"
#include "error_handling.h"

static uint8_t sCliUartRxBuffer[OT_UART_RX_BUF_SIZE];

static int sCliUartFd;

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

    int rval = uart_write_bytes(OT_CLI_UART_NUM, aBuf, aBufLength);

    VerifyOrExit(rval == (int)aBufLength, error = OT_ERROR_FAILED);

    otPlatUartSendDone();

exit:
    return error;
    return rval == aBufLength ? OT_ERROR_NONE : OT_ERROR_FAILED;
}

void platformCliUartInit()
{
    uart_config_t uart_config = {.baud_rate           = 115200,
                                 .data_bits           = UART_DATA_8_BITS,
                                 .parity              = UART_PARITY_DISABLE,
                                 .stop_bits           = UART_STOP_BITS_1,
                                 .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
                                 .rx_flow_ctrl_thresh = 0,
                                 .use_ref_tick        = false};
    ESP_ERROR_CHECK(uart_param_config(OT_CLI_UART_NUM, &uart_config));

    // Disable IO buffer.
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    // Install UART driver for interrupt-driven reads and writes.
    ESP_ERROR_CHECK(uart_driver_install(OT_CLI_UART_NUM, OT_UART_RX_BUF_SIZE, 0, 0, NULL, 0));

    // Tell VFS to use UART driver.
    esp_vfs_dev_uart_use_driver(OT_CLI_UART_NUM);

    esp_vfs_dev_uart_set_rx_line_endings(ESP_LINE_ENDINGS_LF);
    esp_vfs_dev_uart_set_tx_line_endings(ESP_LINE_ENDINGS_LF);

    // Open the CLI UART as a unix file.
    // TODO(wgtdkp): remove magic string.
    sCliUartFd = open("/dev/uart/0", O_RDWR | O_NONBLOCK);
    if (sCliUartFd == -1)
    {
        ESP_LOGE(OT_PLAT_LOG_TAG, "cannot open CLI UART");
        abort();
    }
}

void platformCliUartDeinit(void)
{
    if (sCliUartFd != -1)
    {
        close(sCliUartFd);
        sCliUartFd = -1;
    }
    uart_driver_delete(OT_CLI_UART_NUM);
}

void platformCliUartUpdate(otSysMainloopContext *aMainloop)
{
    FD_SET(sCliUartFd, &aMainloop->mReadFdSet);
    if (sCliUartFd > aMainloop->mMaxFd)
    {
        aMainloop->mMaxFd = sCliUartFd;
    }
}

void platformCliUartProcess(otInstance *aInstance, const otSysMainloopContext *aMainloop)
{
    (void)aInstance;

    if (FD_ISSET(sCliUartFd, &aMainloop->mReadFdSet))
    {
        uint8_t buffer[256];

        int rval = read(sCliUartFd, buffer, sizeof(buffer));

        if (rval > 0)
        {
            otPlatUartReceived(buffer, (uint16_t)rval);
        }
        else if ((rval < 0) && (errno != EAGAIN) && (errno != EINTR))
        {
            abort();
        }
    }
}
