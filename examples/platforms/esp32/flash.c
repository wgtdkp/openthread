#include "platform-esp32.h"

#include <openthread/error.h>
#include <openthread/instance.h>
#include <openthread/platform/flash.h>
#include <openthread/platform/settings.h>

#include <esp_log.h>
#include <esp_partition.h>
#include <esp_spi_flash.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "error_handling.h"

static const esp_partition_t *sOtDataPartition = NULL;

static struct EraseTask
{
    const esp_partition_t *mPartition;
    uint32_t mAddress;
    uint32_t mEraseSize;
} sEraseTask;

static xQueueHandle sEraseQueue;

// Workaround, used by the radio_spinel library.
void otPlatSettingsInit(otInstance *aInstance)
{
    (void)aInstance;
}

// Workaround, used by the radio_spinel library.
void otPlatSettingsDeinit(otInstance *aInstance)
{
    (void)aInstance;
}

void otPlatFlashInit(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    // TODO(wgtdkp):
    // esp32 startup code automatically call spi_flash_init();

    if (!heap_caps_check_integrity_all(true)) {
        ESP_LOGE(OT_PLAT_LOG_TAG, "heap corrupted");
    }

    sOtDataPartition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                                ESP_PARTITION_SUBTYPE_DATA_FAT,
                                                OT_FLASH_PARTITION_NAME);
    assert(sOtDataPartition != NULL);
    sEraseQueue = xQueueCreate(1, sizeof(otError));
    assert(sEraseQueue != NULL);
}

uint32_t otPlatFlashGetSwapSize(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return SETTINGS_CONFIG_PAGE_NUM * SETTINGS_CONFIG_PAGE_SIZE;
}

static void FlashEraseTask(void *aParameters) {
    struct EraseTask *task = (struct EraseTask *)aParameters;
    assert(task->mPartition != NULL);
    esp_err_t rval = esp_partition_erase_range(task->mPartition, task->mAddress,
                                               task->mEraseSize);

    xQueueSend(sEraseQueue, &rval, portMAX_DELAY);
    vTaskDelete(NULL);
}

void otPlatFlashErase(otInstance *aInstance, uint8_t aSwapIndex)
{
    OT_UNUSED_VARIABLE(aInstance);

    assert(sOtDataPartition != NULL);
    assert(sEraseQueue != NULL);

    sEraseTask.mPartition = sOtDataPartition;
    sEraseTask.mAddress = SETTINGS_CONFIG_PAGE_SIZE * (aSwapIndex != 0);
    sEraseTask.mEraseSize = SETTINGS_CONFIG_PAGE_SIZE;

    int ret = xTaskCreate(FlashEraseTask, "ot_flash_erase", 2048, &sEraseTask, 5, NULL);
    assert(ret == pdPASS);
}

void otPlatFlashRead(otInstance *aInstance, uint8_t aSwapIndex, uint32_t aOffset, void *aData, uint32_t aSize) {
    OT_UNUSED_VARIABLE(aInstance);

    esp_err_t error = ESP_FAIL;

    VerifyOrExit(sOtDataPartition != NULL);

    aOffset += SETTINGS_CONFIG_PAGE_SIZE * (aSwapIndex != 0);

    SuccessOrExit(error = esp_partition_read(sOtDataPartition, aOffset, aData, aSize));

exit:
    if (error != ESP_OK)
    {
        ESP_LOGE(OT_PLAT_LOG_TAG, "failed to read flash, swapIndes=%d, offset=%u, size=%u", aSwapIndex, aOffset, aSize);
        abort();
    }
    return;
}

void otPlatFlashWrite(otInstance *aInstance, uint8_t aSwapIndex, uint32_t aOffset, const void *aData, uint32_t aSize) {
    OT_UNUSED_VARIABLE(aInstance);

    esp_err_t error = ESP_FAIL;

    VerifyOrExit(sOtDataPartition != NULL);

    aOffset += SETTINGS_CONFIG_PAGE_SIZE * (aSwapIndex != 0);

    SuccessOrExit(error = esp_partition_write(sOtDataPartition, aOffset, aData, aSize));

exit:
    if (error != ESP_OK)
    {
        ESP_LOGE(OT_PLAT_LOG_TAG, "failed to write flash, swapIndes=%d, offset=%u, size=%u, error=%s", aSwapIndex, aOffset, aSize, esp_err_to_name(error));
        abort();
    }
    return;
}
