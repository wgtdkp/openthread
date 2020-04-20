#ifndef OPENTHREAD_ESP32_PLATFORM_ESP32_H_
#define OPENTHREAD_ESP32_PLATFORM_ESP32_H_

#include <stdint.h>
#include <time.h>

#include <driver/gpio.h>

#include <openthread/instance.h>

#include <openthread/openthread-system.h>

#include "openthread-core-esp32-config.h"

/**
 * The default SPI flash partition used by OpenThread settings.
 *
 */
#define OT_FLASH_PARTITION_NAME "ot_storage"

#define SETTINGS_CONFIG_PAGE_NUM 2

#define SETTINGS_CONFIG_PAGE_SIZE 4096

/**
 * The default platform logging tag.
 *
 */
#define OT_PLAT_LOG_TAG "OT_ESP32_PLAT"

/**
 * The default TXD pin of the radio uart.
 *
 * Specific to ESP32-WROVER-B DEV-KIT.
 *
 */
#define OT_RADIO_UART_TXD (GPIO_NUM_4)

/**
 * The default RXD pin of the radio uart.
 *
 * Specific to ESP32-WROVER-B DEV-KIT.
 *
 */
#define OT_RADIO_UART_RXD (GPIO_NUM_5)

/**
 * The uart used by radio spinel.
 *
 */
#define OT_RADIO_UART_NUM UART_NUM_1

/**
 * The uart used by OpenThread CLI.
 *
 */
#define OT_CLI_UART_NUM UART_NUM_0

/**
 * The uart receive buffer size for both CLI uart and radio uart.
 *
 */
#define OT_UART_RX_BUF_SIZE (UART_FIFO_LEN * 2)

#ifndef MS_PER_S
#define MS_PER_S 1000
#endif
#ifndef US_PER_MS
#define US_PER_MS 1000
#endif
#ifndef US_PER_S
#define US_PER_S (MS_PER_S * US_PER_MS)
#endif
#ifndef NS_PER_US
#define NS_PER_US 1000
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * This function updates OpenThread alarm events to the mainloop context.
 *
 * @param[inout]  aMainloop  The mainloop context;
 *
 */
void platformAlarmUpdate(otSysMainloopContext *aMainloop);

/**
 * This function process alarm events.
 *
 * @param[in] aInstance  The OpenThread instance.
 * @param[in] aMainloop  The mainloop context.
 *
 */
void platformAlarmProcess(otInstance *aInstance, const otSysMainloopContext *aMainloop);

/**
 * This function initialize the CLI UART driver.
 *
 */
void platformCliUartInit(void);

/**
 * This function deinitialize the CLI UART driver.
 *
 */
void platformCliUartDeinit(void);

/**
 * This function updates CLI UART events to the mainloop context.
 *
 * param[inout] aMainloop  The mainloop context.
 *
 */
void platformCliUartUpdate(otSysMainloopContext *aMainloop);

/**
 * This function process CLI UART events.
 *
 * @param[in] aInstance  The OpenThread instance.
 * @param[in] aMainloop  The mainloop context.
 *
 */
void platformCliUartProcess(otInstance *aInstance, const otSysMainloopContext *aMainloop);

/**
 * This function initializes the radio transceiver.
 *
 * @param[in]  aResetRadio            TRUE to reset on init, FALSE to not reset on init.
 * @param[in]  aRestoreDatasetFromNcp TRUE to restore dataset to host from non-volatile memory
 *                                    (only used when attempts to upgrade from NCP to RCP mode),
 *                                    FALSE otherwise.
 *
 */
void platformRadioInit(bool aResetRadio, bool aRestoreDataSetFromNcp);

/**
 * This function deinitialize the radio driver.
 *
 */
void platformRadioDeinit(void);

/**
 * This function updates spinel radio events to the mainloop context.
 *
 * param[inout] aMainloop  The mainloop context.
 *
 */
void platformRadioUpdate(otSysMainloopContext *aMainloop);

/**
 * This function process radio events.
 *
 * @param[in] aInstance  The OpenThread instance.
 * @param[in] aMainloop  The mainloop context.
 *
 */
void platformRadioProcess(otInstance *aInstance, const otSysMainloopContext *aMainloop);

void platformApiLockInit(void);
void platformApiLockDeinit(void);

#ifdef __cplusplus
}
#endif

#endif // OPENTHREAD_ESP32_PLATFORM_ESP32_H_
