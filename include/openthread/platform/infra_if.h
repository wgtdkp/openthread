/*
 *  Copyright (c) 2020, The OpenThread Authors.
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
 * @brief
 *   This file includes the platform abstraction for the infrastructure interface.
 */

#ifndef OPENTHREAD_PLATFORM_INFRA_IF_H_
#define OPENTHREAD_PLATFORM_INFRA_IF_H_

#include <stdint.h>

#include <openthread/error.h>
#include <openthread/instance.h>
#include <openthread/ip6.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 *
 * This method sends an ICMPv6 message on given infrastructure interface.
 *
 * @param[in]  aInfraIfIndex  The index of the infrastructure interface.
 * @param[in]  aDestAddress   The destination address this message is sent to.
 * @param[in]  aBuffer        The ICMPv6 message buffer.
 * @param[in]  aBufferLength  The length of the message buffer.
 *
 * @retval OT_ERROR_NONE    Successfully sent the ICMPv6 message.
 * @retval OT_ERROR_FAILED  Failed to send the ICMPv6 message.
 *
 */
otError otPlatInfraIfSendIcmp6(uint32_t            aInfraIfIndex,
                               const otIp6Address *aDestAddress,
                               const uint8_t *     aBuffer,
                               uint16_t            aBufferLength);

/**
 *
 * The infra interface driver calls this method to notify OpenThread that an ICMPv6 message is received.
 *
 * @param[in]  aInstance      The OpenThread instance structure.
 * @param[in]  aInfraIfIndex  The index of the infrastructure interface on which the ICMPv6 message is received.
 * @param[in]  aSrcAddress    The source address this message is received from.
 * @param[in]  aBuffer        The ICMPv6 message buffer.
 * @param[in]  aBufferLength  The length of the ICMPv6 message buffer.
 *
 * @note  ICMPv6 message sent from current infra interface should not to passed in.
 *
 */
extern void otPlatInfraIfRecvIcmp6(otInstance *        aInstance,
                                   uint32_t            aInfraIfIndex,
                                   const otIp6Address *aSrcAddress,
                                   const uint8_t *     aBuffer,
                                   uint16_t            aBufferLength);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // OPENTHREAD_PLATFORM_INFRA_IF_H_
