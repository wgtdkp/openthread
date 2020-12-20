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
 *  This file defines the API for Service Registration Protocol (SRP).
 */

#ifndef OPENTHREAD_SRP_H_
#define OPENTHREAD_SRP_H_

#include <stdint.h>

#include <openthread/instance.h>
#include <openthread/ip6.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup api-srp
 *
 * @brief
 *  This module includes functions to control service registrations.
 *
 * @{
 *
 */

/**
 * This structure represents a SRP service host.
 *
 */
typedef void otSrpHost;

/**
 * This structure represents a SRP service.
 *
 */
struct otSrpService
{
    char *        mFullName;  ///< The full service name.
    uint16_t      mPort;      ///< The service port.
    otSrpHost *   mHost;      ///< The host of this service.
    uint8_t *     mTxtData;   ///< The standard DNS TXT record data.
    uint16_t      mTxtLength; ///< The TXT record length.
    bool          mDeleted;
    otSrpService *mNext; ///< The next linked service within the same host.
};

enum
{
    OT_SRP_SERVER_EVENT_REGISTER_SERVICE   = 0x01, ///< The services are registered on the SRP server.
    OT_SRP_SERVER_EVENT_REGISTER_HOST      = 0x02, ///< The host is registered on the SRP server.
    OT_SRP_SERVER_EVENT_UNREGISTER_SERVICE = 0x04, ///< The services are unregistered from the SRP server.
    OT_SRP_SERVER_EVENT_UNREGISTER_HOST    = 0x08, ///< A host is unregistered from the SRP server.
};

/**
 * This method enables/disables the SRP server.
 *
 * @param[in]  aInstance  A pointer to an OpenThread instance.
 * @param[in]  aEnabled   A boolean to enable/disable the SRP server.
 *
 */
void otSrpServerSetEnabled(otInstance *aInstance, bool aEnabled);

/**
 * This method sets LEASE & KEY-LEASE range that is acceptable by the SRP server.
 *
 */
otError otSrpServerSetLeaseRange(otInstance *aInstance,
                                 uint32_t    aMinLease,
                                 uint32_t    aMaxLease,
                                 uint32_t    aMinKeyLease,
                                 uint32_t    aMaxKeyLease);

/**
 * This method handles SRP service events.
 *
 * This function is designed to be called by the SRP server to delegate
 * service operations (e.g. registration, deregistration...) to
 * Advertising Proxy and Discovery Proxy to help advertise and discover
 * the given service on multicast-capable links. The proxies should call
 * otSrpServiceEventResult to return the result.
 *
 * @param[in]  aHost     The services & host to be updated. The event handler should get
 *                       the services with otSrpServerHostGetServices.
 * @param[in]  aEvents   The SRP server events.
 *                       OT_SRP_SERVER_EVENT_REGISTER_SERVICE:
 *                           The event handler should remove any resources already associated
 *                           to the services before adding new resources.
 *                       OT_SRP_SERVER_EVENT_REGISTER_HOST:
 *                           The event handler should remove any remove any resources already
 *                           associated to the host before adding new resources.
 *                       OT_SRP_SERVER_EVENT_UNREGISTER_SERVICE:
 *                           The event handler should remove the service instances.
 *                       OT_SRP_SERVER_EVENT_UNREGISTER_HOST:
 *                           The event handler should remove the host instance.
 *
 * @param[in]  aTimeout  The maximum time in milliseconds for the handler
 *                       to process the service event.
 * @param[in]  aContext  The associated context. Never be NULL.
 *
 * @sa otSrpServerSetServiceHandler
 * @sa otSrpServiceEventResult
 *
 */
typedef void (*otSrpServiceHandler)(const otSrpHost *aHost, int aEvents, uint32_t aTimeout, void *aContext);

/**
 * This method sets the service handler on SRP server.
 *
 * @param[in]  aInstance        A pointer to an OpenThread instance.
 * @param[in]  aServiceHandler  A pointer to a service handler.
 * @param[in]  aContext         A pointer to arbitrary context information.
 *                              May be NULL if not used.
 *
 */
void otSrpServerSetServiceHandler(otInstance *aInstance, otSrpServiceHandler aServiceHandler, void *aContext);

/**
 * This method reports the result of handling the service events.
 *
 * The Advertising and Discovery Proxy should call this function
 * to return the service handling result to the SRP server.
 *
 * @param[in]  aInstance  A pointer to an OpenThread instance.
 * @param[in]  aError     An error to be returned to the SRP server.
 * @param[in]  aContext   The context passed in otSrpServiceHandler.
 *
 */
void otSrpServiceEventResult(otInstance *aInstance, otError aError, void *aContext);

/**
 * This method returns the next registered host on the SRP server.
 *
 * @param[in]  aInstance  A pointer to an OpenThread instance.
 * @param[in]  aHost      A pointer to current host. Use NULL to get the first host.
 *
 * @retval  A pointer to the registered host. nullptr, if no more hosts can be found.
 *
 */
const otSrpHost *otSrpServerGetNextHost(otInstance *aInstance, const otSrpHost *aHost);

/**
 * This method returns the full name of a given host.
 *
 * @param[in]  aHost  A SRP service host.
 *
 * @retval  A pointer to the host.
 *
 */
const char *otSrpServerHostGetFullName(const otSrpHost *aHost);

/**
 * This method returns the addresses associated to given host.
 *
 * @param[in]   aHost          The host we are requesting.
 * @param[out]  aAddressesNum  A pointer to where we should output the address number to.
 *
 * @retval  A pointer to the list of IPv6 Address.
 *
 */
const otIp6Address *otSrpServerHostGetAddresses(const otSrpHost *aHost, uint8_t *aAddressesNum);

/**
 * This method returns the list of services associated to given host.
 *
 * @param[in]  aHost  A pointer to the SRP service host.
 *
 * @retval  A pointer to the head of the SRP service list.
 *
 */
const otSrpService *otSrpServerHostGetServices(const otSrpHost *aHost);

/**
 * @}
 *
 */

#ifdef __cplusplus
} // extern "C"
#endif

#endif // OPENTHREAD_SRP_H_
