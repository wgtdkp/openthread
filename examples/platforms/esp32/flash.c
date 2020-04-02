#include <utils/flash.h>

#include <esp_partition.h>
#include <esp_spi_flash.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "error_handling.h"
#include "openthread-core-esp32-config.h"

#define OT_FLASH_PARTITION_NAME "ot_storage"

static const esp_partition_t *sOtDataPartition = NULL;
static struct EraseTask {
    const esp_partition_t *mPartition;
    uint32_t mAddress;
    uint32_t mEraseSize;
} sEraseTask;
static xQueueHandle xEraseQueue;

otError utilsFlashInit(void) {
    // esp32 startup code automatically call spi_flash_init();
    otError error = OT_ERROR_NONE;
    sOtDataPartition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                                ESP_PARTITION_SUBTYPE_DATA_FAT,
                                                OT_FLASH_PARTITION_NAME);
    assert(sOtDataPartition != NULL);
    VerifyOrExit(sOtDataPartition != NULL, error = OT_ERROR_NOT_FOUND);
    xEraseQueue = xQueueCreate(1, sizeof(otError));
    VerifyOrExit(xEraseQueue != NULL, error = OT_ERROR_FAILED);
exit:
    return error;
}

uint32_t utilsFlashGetSize(void) {
    return SETTINGS_CONFIG_PAGE_NUM * SETTINGS_CONFIG_PAGE_SIZE;
}

void FlashEraseTask(void *aParameters) {
    struct EraseTask *task = (struct EraseTask *)aParameters;
    assert(task->mPartition != NULL);
    esp_err_t rval = esp_partition_erase_range(task->mPartition, task->mAddress,
                                               task->mEraseSize);

    xQueueSend(xEraseQueue, &rval, portMAX_DELAY);
    vTaskDelete(NULL);
}

otError utilsFlashErasePage(uint32_t aAddress) {
    otError error = OT_ERROR_NONE;

    VerifyOrExit(sOtDataPartition != NULL, error = OT_ERROR_FAILED);
    VerifyOrExit(xEraseQueue != NULL, error = OT_ERROR_FAILED);
    VerifyOrExit(aAddress + SETTINGS_CONFIG_PAGE_SIZE < sOtDataPartition->size,
                 error = OT_ERROR_INVALID_ARGS);
    aAddress -= (aAddress & (SETTINGS_CONFIG_PAGE_SIZE - 1));
    sEraseTask.mPartition = sOtDataPartition;
    sEraseTask.mAddress = aAddress;
    sEraseTask.mEraseSize = SETTINGS_CONFIG_PAGE_SIZE;
    VerifyOrExit(xTaskCreate(FlashEraseTask, "ot_flash_erase", 2048, &sEraseTask, 5,
                             NULL) == pdPASS,
                 error = OT_ERROR_FAILED);
exit:
    return error;
}

otError utilsFlashStatusWait(uint32_t aTimeout) {
    otError error = OT_ERROR_NONE;
    uint32_t timeoutTick = aTimeout / portTICK_PERIOD_MS;
    esp_err_t rval;

    VerifyOrExit(xEraseQueue != NULL, error = OT_ERROR_FAILED);
    if (xQueueReceive(xEraseQueue, &rval, timeoutTick)) {
        VerifyOrExit(rval == ESP_OK, error = OT_ERROR_FAILED);
    } else {
        error = OT_ERROR_BUSY;
    }
exit:
    return error;
}

uint32_t utilsFlashWrite(uint32_t aAddress, uint8_t *aData, uint32_t aSize) {
    uint32_t writeSize = aSize;

    VerifyOrExit(sOtDataPartition != NULL, writeSize = 0);
    VerifyOrExit(aAddress + aSize < sOtDataPartition->size, writeSize = 0);

    VerifyOrExit(
        esp_partition_write(sOtDataPartition, aAddress, aData, aSize) == ESP_OK,
        writeSize = 0);
exit:
    return writeSize;
}

uint32_t utilsFlashRead(uint32_t aAddress, uint8_t *aData, uint32_t aSize) {
    uint32_t readSize = aSize;

    VerifyOrExit(sOtDataPartition != NULL, readSize = 0);
    VerifyOrExit(aAddress + aSize < sOtDataPartition->size, readSize = 0);

    VerifyOrExit(
        esp_partition_read(sOtDataPartition, aAddress, aData, aSize) == ESP_OK,
        readSize = 0);
exit:
    return readSize;
}
