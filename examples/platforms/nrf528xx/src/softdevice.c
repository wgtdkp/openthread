/*
 *  Copyright (c) 2017, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file implements the OpenThread softdevice helper functions.
 *
 */

#include <openthread-core-config.h>
#include <openthread/config.h>

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#if SOFTDEVICE_OT_MANAGED
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#endif

#include "platform-nrf5.h"
#include "platform-softdevice.h"
#include "softdevice.h"

#include <nrf_raal_softdevice.h>

otError nrf5SdErrorToOtError(uint32_t aSdError)
{
    switch (aSdError)
    {
    case NRF_SUCCESS:
        return OT_ERROR_NONE;
        break;

    case NRF_ERROR_INVALID_STATE:
    case NRF_ERROR_BUSY:
        return OT_ERROR_INVALID_STATE;
        break;

    case NRF_ERROR_INVALID_PARAM:
    case NRF_ERROR_INVALID_ADDR:
        return OT_ERROR_INVALID_ARGS;
        break;

    case NRF_ERROR_NO_MEM:
        return OT_ERROR_NO_BUFS;
        break;

    case NRF_ERROR_NOT_FOUND:
        return OT_ERROR_NOT_FOUND;
        break;

    case NRF_ERROR_NOT_SUPPORTED:
        return OT_ERROR_NOT_IMPLEMENTED;
        break;

    default:
        return OT_ERROR_FAILED;
        break;
    }
}

void otSysSoftdeviceSocEvtHandler(uint32_t aEvtId)
{
    nrf5SdSocFlashProcess(aEvtId);
    nrf_raal_softdevice_soc_evt_handler(aEvtId);
}

void otSysSoftdeviceRaalConfig(const otSysSoftdeviceRaalConfigParams *aConfig)
{
    nrf_raal_softdevice_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    cfg.timeslot_length      = aConfig->timeslotLength;
    cfg.timeslot_timeout     = aConfig->timeslotTimeout;
    cfg.timeslot_max_length  = aConfig->timeslotMaxLength;
    cfg.timeslot_alloc_iters = aConfig->timeslotAllocIters;
    cfg.timeslot_safe_margin = aConfig->timeslotSafeMargin;
    cfg.lf_clk_accuracy_ppm  = aConfig->lfClkAccuracyPpm;

    nrf_raal_softdevice_config(&cfg);
}

#if SOFTDEVICE_OT_MANAGED

#if MONITOR_MODE_DEBUG
extern void DebugMonitor_Init(void);
#endif

void otSysSoftdeviceInit(void)
{
    ble_cfg_t bleCfg;
    uint32_t  error;
    uint32_t  ramStart = 0;

#if MONITOR_MODE_DEBUG
    NVIC_SetPriority(DebugMonitor_IRQn, APP_IRQ_PRIORITY_MID);
    DebugMonitor_Init();
#endif

    error = nrf_sdh_enable_request();
    assert(error == NRF_SUCCESS);

    error = nrf_sdh_ble_default_cfg_set(BLE_CFG_TAG, &ramStart);
    assert(error == NRF_SUCCESS);

    // The L2CAP configuration is not handled by the @s nrf_sdh_ble_default_cfg_set.
    memset(&bleCfg, 0, sizeof(bleCfg));
    bleCfg.conn_cfg.conn_cfg_tag                        = BLE_CFG_TAG;
    bleCfg.conn_cfg.params.l2cap_conn_cfg.rx_mps        = BLE_DEFAULT_L2CAP_MPS_SIZE;
    bleCfg.conn_cfg.params.l2cap_conn_cfg.rx_queue_size = 1;
    bleCfg.conn_cfg.params.l2cap_conn_cfg.tx_mps        = BLE_DEFAULT_L2CAP_MPS_SIZE;
    bleCfg.conn_cfg.params.l2cap_conn_cfg.tx_queue_size = 20;
    bleCfg.conn_cfg.params.l2cap_conn_cfg.ch_count      = NRF_SDH_BLE_CENTRAL_LINK_COUNT;

    error = sd_ble_cfg_set(BLE_CONN_CFG_L2CAP, &bleCfg, ramStart);
    assert(error == NRF_SUCCESS);

    // Global GAP configurations.
    memset(&bleCfg, 0, sizeof(bleCfg));
    bleCfg.gap_cfg.role_count_cfg.adv_set_count                     = BLE_GAP_ADV_SET_COUNT_DEFAULT;
    bleCfg.gap_cfg.role_count_cfg.periph_role_count                 = BLE_GAP_ROLE_COUNT_PERIPHERAL;
    bleCfg.gap_cfg.role_count_cfg.central_role_count                = BLE_GAP_ROLE_COUNT_CENTRAL;
    bleCfg.gap_cfg.role_count_cfg.central_sec_count                 = BLE_GAP_ROLE_COUNT_CENTRAL_SEC_DEFAULT;
    bleCfg.gap_cfg.role_count_cfg.qos_channel_survey_role_available = 1;

    error = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &bleCfg, ramStart);
    assert(error == NRF_SUCCESS);

    // BLE GAP connection configuration parameters.
    memset(&bleCfg, 0, sizeof(bleCfg));
    bleCfg.conn_cfg.conn_cfg_tag                     = BLE_CFG_TAG;
    bleCfg.conn_cfg.params.gap_conn_cfg.conn_count   = BLE_GAP_ROLE_COUNT_PERIPHERAL + BLE_GAP_ROLE_COUNT_CENTRAL;
    bleCfg.conn_cfg.params.gap_conn_cfg.event_length = BLE_GAP_EVENT_LENGTH_DEFAULT;

    error = sd_ble_cfg_set(BLE_CONN_CFG_GAP, &bleCfg, ramStart);
    assert(error == NRF_SUCCESS);

    // BLE GATTC connection configuration parameters.
    memset(&bleCfg, 0, sizeof(bleCfg));
    bleCfg.conn_cfg.conn_cfg_tag                                  = BLE_CFG_TAG;
    bleCfg.conn_cfg.params.gattc_conn_cfg.write_cmd_tx_queue_size = BLE_GATT_TX_QUEUE_SIZE;

    error = sd_ble_cfg_set(BLE_CONN_CFG_GATTC, &bleCfg, ramStart);
    assert(error == NRF_SUCCESS);

    // BLE GATTS connection configuration parameters.
    memset(&bleCfg, 0, sizeof(bleCfg));
    bleCfg.conn_cfg.conn_cfg_tag                            = BLE_CFG_TAG;
    bleCfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = BLE_GATT_TX_QUEUE_SIZE;

    error = sd_ble_cfg_set(BLE_CONN_CFG_GATTS, &bleCfg, ramStart);
    assert(error == NRF_SUCCESS);

    // BLE GATT connection configuration parameters.
    memset(&bleCfg, 0, sizeof(bleCfg));
    bleCfg.conn_cfg.conn_cfg_tag                 = BLE_CFG_TAG;
    bleCfg.conn_cfg.params.gatt_conn_cfg.att_mtu = BLE_GATT_ATT_MTU;

    error = sd_ble_cfg_set(BLE_CONN_CFG_GATT, &bleCfg, ramStart);
    assert(error == NRF_SUCCESS);

    error = nrf_sdh_ble_enable(&ramStart);
    assert(error == NRF_SUCCESS);
}

void otSysSoftdeviceProcess(void)
{
    nrf_sdh_evts_poll();
}

static void soc_evt_handler(uint32_t aEvtId, void *aContext)
{
    (void)aContext;

    otSysSoftdeviceSocEvtHandler(aEvtId);
}

void SD_EVT_IRQHandler(void)
{
    // Empty implementation.
}

// Register a handler for SOC events.
NRF_SDH_SOC_OBSERVER(m_ot_soc_observer, 0, soc_evt_handler, NULL);

#endif // SOFTDEVICE_OT_MANAGED
