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
 *   This file includes compile-time configurations for the ToBLE feature.
 *
 */

#ifndef CONFIG_TOBLE_H_
#define CONFIG_TOBLE_H_

/**
 * @def OPENTHREAD_CONFIG_TOBLE_ENABLE
 *
 * Define to 1 to enable ToBLE.
 *
 */
#ifndef OPENTHREAD_CONFIG_TOBLE_ENABLE
#define OPENTHREAD_CONFIG_TOBLE_ENABLE 0
#endif

/**
 * @def OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
 *
 * Enables ToBLE operation as a central,
 *
 * This compiles in all the central related code. When ToBLE is enabled `OPENTHREAD_CONFIG_ENABLE_TOBLE` we need either
 * central or peripheral (or both) enabled.
 *
 * In the case where both central and peripheral modes are enabled, OpenThread API otTobleSetMode() is provided to
 * allow user to select the mode at run-time.
 *
 */
#ifndef OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
#define OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE 1
#endif

/**
 * Enabled ToBLE operation as a peripheral.
 *
 * This compiles in all the peripheral related code. When ToBLE is enabled `OPENTHREAD_CONFIG_ENABLE_TOBLE` we need
 * either central or peripheral (or both) enabled.
 *
 * In the case where both central and peripheral modes are enabled, OpenThread API otTobleSetMode() is provided to
 * allow user to select the mode at run-time.
 *
 */
#ifndef OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
#define OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE 1
#endif

/**
 * @def OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
 *
 * Define to 1 to support ToBLE L2CAP transport.
 *
 */
#ifndef OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
#define OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE 0
#endif

/**
 * @def OPENTHREAD_CONFIG_TOBLE_154_ATTACH_ATTEMPT_RATIO
 *
 * Specifies the default radio (how many attempts) to try to attach using ToBLE vs 802.15.4 when device is detached.
 *
 * This configuration is applicable when `OPENTHREAD_CONFIG_TOBLE_MULTI_RADIO_ENABLE` is enabled.
 *
 * The ratio value indicates the number of attach attempts on 802.15.4 link before an attempt using ToBLE link is
 * performed (while device is detached). When ratio values is larger than one, it would first perform `value - 1`
 * attach attempts on IEEE 802.15.4 link and if all fail, then a single attach attempt on ToBLE link is performed.
 * Afterwards, the same cycle repeats (e.g., value of two would instruct the device to alternate between two links).
 * If the ratio value is set to one, the device will only use ToBLE link, and if it is set to zero, the device will
 * always try to attach using IEEE 802.15.4 link.
 *
 */
#ifndef OPENTHREAD_CONFIG_TOBLE_154_ATTACH_ATTEMPT_RATIO
#define OPENTHREAD_CONFIG_TOBLE_154_ATTACH_ATTEMPT_RATIO 2
#endif

/**
 * @def OPENTHREAD_CONFIG_TOBLE_ATT_MTU_MAX
 *
 * Specifies the maximum BTP segment size.
 *
 */
#ifndef OPENTHREAD_CONFIG_TOBLE_BTP_MAX_SEGMENT_SIZE
#define OPENTHREAD_CONFIG_TOBLE_BTP_MAX_SEGMENT_SIZE 256
#endif

#endif // CONFIG_TOBLE_H_
