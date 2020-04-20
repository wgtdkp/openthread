/*
 *  Copyright (c) 2018, The OpenThread Authors.
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

#ifndef OPENTHREAD_ESP32_OPENTHREAD_CORE_ESP32_CONFIG_H_
#define OPENTHREAD_ESP32_OPENTHREAD_CORE_ESP32_CONFIG_H_

/**
 * @def OPENTHREAD_CONFIG_PLATFORM_INFO
 *
 * The platform-specific string to insert into the OpenThread version string.
 *
 */
#define OPENTHREAD_CONFIG_PLATFORM_INFO "ESP32"

/**
 * @def OPENTHREAD_CONFIG_PLATFORM_FLASH_API_ENABLE
 *
 * Define to 1 to enable otPlatFlash* APIs to support non-volatile storage.
 *
 * When defined to 1, the platform MUST implement the otPlatFlash* APIs instead of the otPlatSettings* APIs.
 *
 */
#define OPENTHREAD_CONFIG_PLATFORM_FLASH_API_ENABLE 1

#define OPENTHREAD_CONFIG_LOG_OUTPUT OPENTHREAD_CONFIG_LOG_OUTPUT_PLATFORM_DEFINED

#define OPENTHREAD_CONFIG_LOG_LEVEL OT_LOG_LEVEL_INFO

#define OPENTHREAD_CONFIG_MLE_STEERING_DATA_SET_OOB_ENABLE 1

#define OPENTHREAD_CONFIG_NUM_MESSAGE_BUFFERS 50

#define OPENTHREAD_CONFIG_PLATFORM_NETIF_ENABLE 1

/* Define to 1 if you want to enable CoAP to an application. */
#define OPENTHREAD_CONFIG_COAP_API_ENABLE 1

/* Define to 1 to enable the border agent feature. */
#define OPENTHREAD_CONFIG_BORDER_AGENT_ENABLE 1

/* Define to 1 if you want to enable Border Router */
#define OPENTHREAD_CONFIG_BORDER_ROUTER_ENABLE 1

/* Define to 1 if you want to enable log for certification test */
#define OPENTHREAD_CONFIG_REFERENCE_DEVICE_ENABLE 1

/* Define to 1 if you want to enable channel manager feature */
#define OPENTHREAD_CONFIG_CHANNEL_MANAGER_ENABLE 0

/* Define to 1 if you want to use channel monitor feature */
#define OPENTHREAD_CONFIG_CHANNEL_MONITOR_ENABLE 0

/* Define to 1 if you want to use child supervision feature */
#define OPENTHREAD_CONFIG_CHILD_SUPERVISION_ENABLE 1

/* Define to 1 to enable the commissioner role. */
#define OPENTHREAD_CONFIG_COMMISSIONER_ENABLE 1

/* Define to 1 if you want to enable DHCPv6 Client */
#define OPENTHREAD_CONFIG_DHCP6_CLIENT_ENABLE 1

/* Define to 1 if you want to enable DHCPv6 Server */
#define OPENTHREAD_CONFIG_DHCP6_SERVER_ENABLE 1

/* Define to 1 if you want to use diagnostics module */
#define OPENTHREAD_CONFIG_DIAG_ENABLE 0

/* Define to 1 if you want to enable DNS Client */
#define OPENTHREAD_CONFIG_DNS_CLIENT_ENABLE 1

/* Define to 1 to enable dtls support. */
#define OPENTHREAD_CONFIG_DTLS_ENABLE 1

/* Define to 1 if you want to use jam detection feature */
#define OPENTHREAD_CONFIG_JAM_DETECTION_ENABLE 1

/* Define to 1 to enable the joiner role. */
#define OPENTHREAD_CONFIG_JOINER_ENABLE 1

/* Define to 1 if you want to use legacy network support */
#define OPENTHREAD_CONFIG_LEGACY_ENABLE 1

/* Define to 1 if you want to use MAC filter feature */
#define OPENTHREAD_CONFIG_MAC_FILTER_ENABLE 1

/* Define to 1 to enable network diagnostic for MTD. */
#define OPENTHREAD_CONFIG_TMF_NETWORK_DIAG_MTD_ENABLE 0

/* Define to 1 if you want to enable support for multiple OpenThread
   instances. */
#define OPENTHREAD_CONFIG_MULTIPLE_INSTANCE_ENABLE 0

/* Enable the external heap. */
#define OPENTHREAD_CONFIG_HEAP_EXTERNAL_ENABLE 0

/* Define to 1 to enable the NCP SPI interface. */
#define OPENTHREAD_CONFIG_NCP_SPI_ENABLE 0

/* Define to 1 if using NCP Spinel Encrypter */
#define OPENTHREAD_ENABLE_NCP_SPINEL_ENCRYPTER 0

/* Define to 1 to enable the NCP UART interface. */
#define OPENTHREAD_CONFIG_NCP_UART_ENABLE 1

/* Define to 1 if using NCP vendor hook */
#define OPENTHREAD_ENABLE_NCP_VENDOR_HOOK 0

/* Define to 1 if you want to enable raw link-layer API */
#define OPENTHREAD_CONFIG_LINK_RAW_ENABLE 0

/* Define to 1 if you want to enable Service */
#define OPENTHREAD_CONFIG_TMF_NETDATA_SERVICE_ENABLE 1

/* Define to 1 to enable the UDP proxy feature. */
#define OPENTHREAD_ENABLE_UDP_PROXY 1

/* OpenThread examples */
#define OPENTHREAD_EXAMPLES none

/* Name of package */
#define PACKAGE "openthread"

/* Define to the address where bug reports for this package should be sent. */
#define PACKAGE_BUGREPORT "openthread-devel@googlegroups.com"

/* Define to the full name of this package. */
#define PACKAGE_NAME "OPENTHREAD"

/* Define to the full name and version of this package. */
#define PACKAGE_STRING "OPENTHREAD gcf8d4ec"

/* Define to the one symbol short name of this package. */
#define PACKAGE_TARNAME "openthread"

/* Define to the home page for this package. */
#define PACKAGE_URL "http://github.com/openthread/openthread"

#endif // OPENTHREAD_ESP32_OPENTHREAD_CORE_ESP32_CONFIG_H_
