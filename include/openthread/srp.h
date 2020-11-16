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
 *  This file defines the SRP Server & Client API.
 *
 * See https://tools.ietf.org/html/draft-ietf-dnssd-srp-05
 * for more informations about the Server Registration Protocol.
 *
 */

#ifndef OPENTHREAD_SRP_H_
#define OPENTHREAD_SRP_H_

#include <openthread/error.h>
#include <openthread/instance.h>
#include <openthread/ip6.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup api-srp
 *
 * @brief
 *  This module includes functions to register/discover services via DNS-SD SRP.
 *
 * @{
 *
 */

/**
 * This structure represents a SRP service.
 *
 */
struct otSrpService
{
    const char *        mName;           ///< The service name.
    const char *        mType;           ///< The service protocol type.
    const char *        mDomain;         ///< The domain to register the service in.
    uint16_t            mPort;           ///< The service port.
    const char *        mHostname;       ///< The host name.
    const otIp6Address *mAddresses;      ///< The address list to be advertised.
    uint16_t            mAddressesCount; ///< The number of addresses.
    const uint8_t *     mTxtRecord;      ///< The standard DNS TXT record.
    uint16_t            mTxtLength;      ///< The TXT record length.
};

enum
{
    OT_SRP_SERVICE_EVENT_REGISTER,   ///< Register the service.
    OT_SRP_SERVICE_EVENT_DEREGISTER, ///< Deregister the service.
    OT_SRP_SERVICE_EVENT_UPDATE,     ///< Update the service.
    OT_SRP_SERVICE_EVENT_BROWSE,     ///< Browse the service.
    OT_SRP_SERVICE_EVENT_RESOLVE,    ///< Resolve the service.
};

/**
 * Server side API.
 *
 */

/**
 * Starts the SRP server.
 *
 */
otError otSrpServerStart(otInstance *aInstance);

/**
 * Stops the SRP server.
 *
 */
void otSrpServerStop(otInstance *aInstance);

/**
 * Handlers SRP service events.
 *
 * This function is designed to be called by the SRP server to delegate
 * service operations (e.g. registration, deregistration...) to
 * Advertising Proxy and Discovery Proxy to help advertise and discover
 * the given service on multicast-capable links. The proxies should call
 * otSrpServiceReportResult to return the handling result.
 *
 * @param[in]  aService  The given service.
 * @param[in]  aEvent    The service event.
 * @param[in]  aTimeout  The maximum time in Milliseconds for the handler
 *                       to process the service event.
 * @param[in]  aContext  The associated context. Never be NULL.
 *
 * @see otSrpServerSetServiceHandler
 * @see otSrpServiceReportResult
 *
 */
typedef void (*otSrpServiceHandler)(
    const otSrpService *aService,
    int aEvent,
    uint32_t aTimeout,
    void *aContext);

/**
 * Sets the service handler on SRP server.
 *
 * Call this function to handle service events. Otherwise, the
 * SRP server will depend on itself.
 *
 * @param[in]  aInstance        A pointer to an OpenThread instance.
 * @param[in]  aServiceHandler  A pointer to a service handler.
 * @param[in]  aContext         A pointer to arbitrary context information.
 *                              May be NULL if not used.
 *
 */
void otSrpServerSetServiceHandler(
    otInstance *aInstance,
    otSrpServiceHandler aServiceHandler,
    void *aContext);

/**
 * Reports the result of the servide handler.
 *
 * The Advertising and Discovery Proxy should call this function
 * to return the servic handling result to the SRP server.
 *
 * @param[in]  aInstance  A pointer to an OpenThread instance.
 * @param[in]  aError     An error to be returned to the SRP server.
 * @param[in]  aService   A SRP service (or a portion of it). This field may be NULL
 *                        and available only when @p aError is OT_ERROR_NONE.
 * @param[in]  aEvent     A service event.
 * @param[in]  aContext   The context passed in otSrpServiceHandler.
 *
 */
void otSrpServiceReportResult(
    otInstance *aInstance,
    otError aError,
    const otSrpService *aService,
    int aEvent,
    void *aContext);

/**
 * Client side API.
 *
 */

/**
 * Sets the default host name for the SRP client.
 *
 * In case the `mHostname` field of otSrpService is not provided,
 * this value is used as the default host name.
 *
 * @param[in]  aHostname  The default host name.
 *
 */
otError otSrpClientSetDefaultHostname(const char *aHostname);

/**
 * The function is called to handle the service registration result.
 *
 * @param[in]  aError    The error indicates the registration result.
 * @param[in]  aContext  A pointer to arbitrary context information.
 *                       May be NULL if not used.
 *
 */
typedef void (*otSrpClientRegistrationCallback)(
    otError aError,
    void *aContext);

/**
 * Registers a service to the SRP server.
 *
 */
otError otSrpClientRegister(
    otInstance *aInstance,
    const otSrpService *aService,
    otSrpClientRegistrationCallback aCallback,
    void *aContext);

/**
 * The function is called to handle the service deregistration result.
 *
 * @param[in]  aError    The error indicates the deregistration result.
 * @param[in]  aContext  A pointer to arbitrary context information.
 *                       May be NULL if not used.
 *
 */
typedef void (*otSrpClientDeregistrationCallback)(
    otError aError,
    void *aContext);

/**
 * Deregisters a service on the SRP server.
 *
 */
otError otSrpClientDeregister(
    otInstance *aInstance,
    const otSrpService *aService,
    otSrpClientDeregistrationCallback aCallback,
    void *aContext);

/**
 * The function is called to handle the found services.
 *
 * @param[in]  aError    The error indicates the browsing result.
 * @param[in]  aService  The service found. Available only when
 *                       @p aError is OT_ERROR_NONE.
 * @param[in]  aContext  The context passed in otSrpClientBrowse.
 *
 * @note This function will be called once for each found service.
 *       @p aError will be set to OT_ERROR_NOT_FOUND to indicate
 *       that there will be no more invocation of this function.
 *
 * @see otSrpClientBrowse
 *
 */
typedef void (*otSrpClientBrowseCallback)(
    otError aError,
    const otSrpService *aService,
    void *aContext);

/**
 * Discovers specific services.
 *
 */
otError otSrpClientBrowse(
    otInstance *aInstance,
    const char *aType,
    const char *aDomain,
    otSrpClientBrowseCallback aCallback,
    void *aContext);

/**
 * The function is called to handle the found services.
 *
 * @param[in]  aError    The error indicates the browsing result.
 * @param[in]  aService  The service found. Available only when
 *                       @p aError is OT_ERROR_NONE.
 * @param[in]  aContext  The context passed in otSrpClientResolve.
 *
 * @see otSrpClientResolve
 *
 */
typedef void (*otSrpClientResolveCallback)(
    otError aError,
    const otSrpService *aService,
    void *aContext);

/**
 * Resolves a specific service.
 *
 */
otError otSrpClientResolve(
    otInstance *aInstance,
    const otSrpService *aService,
    otSrpClientResolveCallback aCallback,
    void *aContext);

/**
 * @}
 *
 */

#ifdef __cplusplus
} // extern "C"
#endif

#endif // OPENTHREAD_SRP_H_
